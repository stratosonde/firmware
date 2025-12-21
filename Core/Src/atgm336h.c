/**
  ******************************************************************************
  * @file    atgm336h.c
  * @brief   ATGM336H-5N31 GNSS Module Driver Implementation
  ******************************************************************************
  * @attention
  *
  * This driver provides interface to the ATGM336H-5N31 GNSS module
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "atgm336h.h"
#include "SEGGER_RTT.h"
#include "usart_if.h"
#include "stm32_lpm.h"
#include "utilities_def.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GNSS_HandleTypeDef *pHgnss = NULL;  // For DMA ISR callback access

/* Private function prototypes -----------------------------------------------*/
static float GNSS_ConvertToDecimalDegrees(float raw_degrees);
static bool GNSS_GetToken(const char *sentence, int index, char *buffer, int max_len);
static int GNSS_ParseGGA(GNSS_HandleTypeDef *hgnss, const char *sentence);
static int GNSS_ParseRMC(GNSS_HandleTypeDef *hgnss, const char *sentence);
static int GNSS_ParseGSV(GNSS_HandleTypeDef *hgnss, const char *sentence);
static bool GNSS_VerifyChecksum(const char *sentence);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_Init(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL)
  {
    return GNSS_ERROR;
  }

  /* Save handle for ISR callback */
  pHgnss = hgnss;

  /* Initialize data structure */
  memset(&hgnss->data, 0, sizeof(GNSS_Data_t));
  hgnss->is_initialized = false;
  hgnss->is_powered = false;
  
  /* Initialize DMA circular buffer */
  memset(hgnss->dma_buffer, 0, sizeof(hgnss->dma_buffer));
  hgnss->dma_head = 0;
  hgnss->dma_tail = 0;
  hgnss->dma_data_ready = false;
  
  /* Initialize NMEA sentence processing */
  memset(hgnss->nmea_sentence, 0, sizeof(hgnss->nmea_sentence));
  hgnss->nmea_length = 0;

  /* Configure power control pin (PB10) as output */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = hgnss->pwr_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(hgnss->pwr_port, &GPIO_InitStruct);

  /* Configure enable pin (PB5) as output */
  GPIO_InitStruct.Pin = hgnss->en_pin;
  HAL_GPIO_Init(hgnss->en_port, &GPIO_InitStruct);

  /* Ensure module is powered off initially */
  /* PB10 (power): ACTIVE HIGH, so RESET=OFF */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_RESET);
  /* PB5 (enable): ACTIVE HIGH, so RESET=OFF */
  HAL_GPIO_WritePin(hgnss->en_port, hgnss->en_pin, GPIO_PIN_RESET);
  
  SEGGER_RTT_WriteString(0, "GNSS_Init: PB10(pwr)=LOW(off), PB5(en)=LOW(off)\r\n");

  hgnss->is_initialized = true;

  return GNSS_OK;
}

/**
  * @brief  Power on GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_PowerOn(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL || !hgnss->is_initialized)
  {
    return GNSS_ERROR;
  }

  if (hgnss->is_powered)
  {
    return GNSS_OK; // Already powered on
  }

  /* Power on the module - Control BOTH pins */
  SEGGER_RTT_WriteString(0, "GNSS_PowerOn: PB10(pwr)=HIGH(on), PB5(en)=HIGH(on)\r\n");
  
  /* PB10 (power): ACTIVE HIGH, so SET=ON */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_SET);
  
  /* PB5 (enable): ACTIVE HIGH, so SET=ON */
  HAL_GPIO_WritePin(hgnss->en_port, hgnss->en_pin, GPIO_PIN_SET);
  
  hgnss->is_powered = true;

  /* Wait for module to stabilize */
  HAL_Delay(GNSS_POWER_ON_DELAY);

  /* Reset DMA circular buffer pointers */
  hgnss->dma_head = 0;
  hgnss->dma_tail = 0;
  hgnss->dma_data_ready = false;
  memset(hgnss->dma_buffer, 0, sizeof(hgnss->dma_buffer));
  
  /* Reset NMEA sentence processing */
  hgnss->nmea_length = 0;
  memset(hgnss->nmea_sentence, 0, sizeof(hgnss->nmea_sentence));

  /* Start DMA circular buffer reception - PRODUCTION SAFE */
  /* No ISR conflicts, no vcom callback overwrite, minimal interrupt processing */
  SEGGER_RTT_WriteString(0, "GNSS_PowerOn: Starting DMA circular buffer reception...\r\n");
  HAL_StatusTypeDef dma_status = HAL_UART_Receive_DMA(hgnss->huart, hgnss->dma_buffer, GNSS_DMA_BUFFER_SIZE);
  
  if (dma_status != HAL_OK)
  {
    SEGGER_RTT_WriteString(0, "GNSS_PowerOn: ERROR - DMA start failed\r\n");
    return GNSS_ERROR;
  }

  /* Disable STOP mode while GNSS is receiving to prevent UART interrupts from being disabled */
  /* UART wakeup from STOP mode doesn't work reliably with continuous NMEA data stream */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_DISABLE);
  SEGGER_RTT_WriteString(0, "GNSS_PowerOn: DMA started, STOP mode disabled for UART RX\r\n");

  return GNSS_OK;
}

/**
  * @brief  Power off GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_PowerOff(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL || !hgnss->is_initialized)
  {
    return GNSS_ERROR;
  }

  /* CRITICAL: Abort DMA transfer before powering off */
  if (hgnss->huart != NULL && hgnss->huart->hdmarx != NULL)
  {
    HAL_UART_AbortReceive(hgnss->huart);
    SEGGER_RTT_WriteString(0, "GNSS_PowerOff: DMA receive aborted\r\n");
  }

  /* Re-enable STOP mode now that GNSS is no longer receiving */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_ENABLE);
  SEGGER_RTT_WriteString(0, "GNSS_PowerOff: Re-enabled STOP mode\r\n");

  /* Power off the module - Control BOTH pins */
  SEGGER_RTT_WriteString(0, "GNSS_PowerOff: PB10(pwr)=LOW(off), PB5(en)=LOW(off)\r\n");
  
  /* PB10 (power): ACTIVE HIGH, so RESET=OFF */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_RESET);
  
  /* PB5 (enable): ACTIVE HIGH, so RESET=OFF */
  HAL_GPIO_WritePin(hgnss->en_port, hgnss->en_pin, GPIO_PIN_RESET);
  
  hgnss->is_powered = false;

  return GNSS_OK;
}

/**
  * @brief  Configure GNSS module with initialization commands
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_Configure(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL || !hgnss->is_powered)
  {
    return GNSS_ERROR;
  }

  SEGGER_RTT_WriteString(0, "\r\n=== Configuring ATGM336H GNSS Module ===\r\n");
  
  /* Send NMEA output configuration (GGA + RMC only for efficiency) */
  SEGGER_RTT_WriteString(0, "Sending: NMEA config (GGA+RMC only)...\r\n");
  if (GNSS_SendCommand(hgnss, GNSS_CMD_NMEA_CONFIG) != GNSS_OK)
  {
    SEGGER_RTT_WriteString(0, "WARNING: Failed to send NMEA config\r\n");
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */
  
  /* Send HIGH ALTITUDE MODE command - CRITICAL for balloon operation! */
  SEGGER_RTT_WriteString(0, "Sending: HIGH ALTITUDE MODE (defeats 18km limit)...\r\n");
  if (GNSS_SendCommand(hgnss, GNSS_CMD_HIGH_ALT_MODE) != GNSS_OK)
  {
    SEGGER_RTT_WriteString(0, "WARNING: Failed to send high altitude mode\r\n");
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */
  
  /* Send update rate configuration (1 Hz) */
  SEGGER_RTT_WriteString(0, "Sending: Update rate (1 Hz)...\r\n");
  if (GNSS_SendCommand(hgnss, GNSS_CMD_UPDATE_RATE) != GNSS_OK)
  {
    SEGGER_RTT_WriteString(0, "WARNING: Failed to send update rate\r\n");
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */
  
  /* Send satellite system configuration (GPS + BeiDou) */
  SEGGER_RTT_WriteString(0, "Sending: Satellite systems (GPS+BeiDou)...\r\n");
  if (GNSS_SendCommand(hgnss, GNSS_CMD_SATELLITE_SYS) != GNSS_OK)
  {
    SEGGER_RTT_WriteString(0, "WARNING: Failed to send satellite config\r\n");
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */
  
  /* Send fix mode configuration (Auto 2D/3D) */
  SEGGER_RTT_WriteString(0, "Sending: Fix mode (Auto 2D/3D)...\r\n");
  if (GNSS_SendCommand(hgnss, GNSS_CMD_FIX_MODE) != GNSS_OK)
  {
    SEGGER_RTT_WriteString(0, "WARNING: Failed to send fix mode\r\n");
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */
  
  SEGGER_RTT_WriteString(0, "=== GNSS Configuration Complete ===\r\n\r\n");
  
  return GNSS_OK;
}

/**
  * @brief  Get position data from GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  timeout: Timeout in milliseconds
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_GetPosition(GNSS_HandleTypeDef *hgnss, uint32_t timeout)
{
  if (hgnss == NULL || !hgnss->is_initialized)
  {
    return GNSS_ERROR;
  }

  if (!hgnss->is_powered)
  {
    return GNSS_ERROR;
  }

  uint32_t start_tick = HAL_GetTick();
  
  /* Clear previous data validity */
  hgnss->data.valid = false;

  SEGGER_RTT_WriteString(0, "GNSS_GetPosition: Waiting for fix (DMA circular buffer)...\r\n");

  /* Wait for valid fix or timeout */
  /* DMA fills buffer in background, we process it in main loop context */
  uint32_t loop_count = 0;
  
  while ((HAL_GetTick() - start_tick) < timeout)
  {
    /* Process DMA buffer data - safe for main loop (no ISR conflicts) */
    GNSS_ProcessDMABuffer(hgnss);
      
    /* Check if we have a valid fix now */
    if (hgnss->data.valid && hgnss->data.fix_quality != GNSS_FIX_INVALID)
    {
      SEGGER_RTT_WriteString(0, "GNSS: VALID FIX!\r\n");
      return GNSS_OK;
    }
    
    /* Sleep/Yield to allow background tasks */
    HAL_Delay(10);
    
    /* Print status every 5 loops (~50ms) for short timeouts */
    loop_count++;
    if (loop_count % 5 == 0)
    {
      SEGGER_RTT_WriteString(0, "[GNSS: Waiting for fix...]\r\n");
    }
  }

  SEGGER_RTT_WriteString(0, "\r\nGNSS_GetPosition: TIMEOUT - No valid fix obtained\r\n");

  return GNSS_TIMEOUT;
}

/**
  * @brief  Process received UART data byte by byte (LEGACY - kept for compatibility)
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  data: Received byte
  * @retval GNSS status
  * @note   This function is no longer used in DMA mode but kept for legacy support
  */
GNSS_StatusTypeDef GNSS_ProcessByte(GNSS_HandleTypeDef *hgnss, uint8_t data)
{
  if (hgnss == NULL)
  {
    return GNSS_ERROR;
  }

  /* This function is deprecated in favor of DMA circular buffer processing */
  /* See GNSS_ProcessDMABuffer() for the new implementation */
  
  return GNSS_OK;
}

/**
  * @brief  Parse NMEA sentence
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  sentence: NMEA sentence string
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_ParseNMEA(GNSS_HandleTypeDef *hgnss, const char *sentence)
{
  if (hgnss == NULL || sentence == NULL)
  {
    return GNSS_ERROR;
  }

  /* Log raw NMEA sentence - DISABLED to reduce output */
  // SEGGER_RTT_WriteString(0, "[NMEA] ");
  // SEGGER_RTT_WriteString(0, sentence);
  // SEGGER_RTT_WriteString(0, "\r\n");

  /* Verify checksum if present */
  if (!GNSS_VerifyChecksum(sentence))
  {
    SEGGER_RTT_WriteString(0, "[NMEA] Checksum FAILED\r\n");
    return GNSS_INVALID;
  }

  /* Parse GGA sentence (position and altitude) */
  /* Support both GPS-only ($GPGGA) and multi-GNSS ($GNGGA) formats */
  if (strncmp(sentence, NMEA_GGA, strlen(NMEA_GGA)) == 0 ||
      strncmp(sentence, "$GNGGA", 6) == 0)
  {
    GNSS_ParseGGA(hgnss, sentence);
  }
  /* Parse RMC sentence (speed and course) */
  /* Support both GPS-only ($GPRMC) and multi-GNSS ($GNRMC) formats */
  else if (strncmp(sentence, NMEA_RMC, strlen(NMEA_RMC)) == 0 ||
           strncmp(sentence, "$GNRMC", 6) == 0)
  {
    GNSS_ParseRMC(hgnss, sentence);
  }
  /* Parse GSV sentence (satellites in view) */
  /* Support GPS ($GPGSV), GLONASS ($GLGSV), BeiDou ($BDGSV), and combined ($GNGSV) */
  else if (strncmp(sentence, NMEA_GSV, strlen(NMEA_GSV)) == 0 ||
           strncmp(sentence, "$GNGSV", 6) == 0 ||
           strncmp(sentence, "$GLGSV", 6) == 0 ||
           strncmp(sentence, "$BDGSV", 6) == 0)
  {
    GNSS_ParseGSV(hgnss, sentence);
  }

  return GNSS_OK;
}

/**
  * @brief  Check if fix is valid
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval true if valid fix, false otherwise
  */
bool GNSS_IsFixValid(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL)
  {
    return false;
  }

  return (hgnss->data.valid && hgnss->data.fix_quality != GNSS_FIX_INVALID);
}

/**
  * @brief  Check if fix meets quality thresholds for production use
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval true if fix is good quality, false otherwise
  * @note   Good quality defined as: valid, 3D fix, 4+ satellites, HDOP <= 5.0
  */
bool GNSS_IsFixGoodQuality(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL)
  {
    return false;
  }

  return (hgnss->data.valid &&
          hgnss->data.fix_quality != GNSS_FIX_INVALID &&
          hgnss->data.satellites >= 4 &&
          hgnss->data.hdop <= 5.0f &&
          GNSS_ValidateCoordinates(hgnss->data.latitude, hgnss->data.longitude));
}

/**
  * @brief  Validate GPS coordinates are within valid ranges
  * @param  lat: Latitude in decimal degrees
  * @param  lon: Longitude in decimal degrees
  * @retval true if coordinates are valid, false otherwise
  */
bool GNSS_ValidateCoordinates(float lat, float lon)
{
  return (lat >= -90.0f && lat <= 90.0f && 
          lon >= -180.0f && lon <= 180.0f &&
          (lat != 0.0f || lon != 0.0f)); // Reject null island (0,0)
}

/**
  * @brief  Send command to GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  cmd: Command string
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_SendCommand(GNSS_HandleTypeDef *hgnss, const char *cmd)
{
  if (hgnss == NULL || cmd == NULL || !hgnss->is_powered)
  {
    return GNSS_ERROR;
  }

  HAL_StatusTypeDef status;
  status = HAL_UART_Transmit(hgnss->huart, (uint8_t *)cmd, strlen(cmd), GNSS_UART_TIMEOUT);

  if (status != HAL_OK)
  {
    return GNSS_ERROR;
  }

  return GNSS_OK;
}

/**
  * @brief  Process DMA circular buffer data (called from main loop)
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_ProcessDMABuffer(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL || !hgnss->is_powered)
  {
    return GNSS_ERROR;
  }

  /* Get current DMA head position (hardware write position) */
  uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(hgnss->huart->hdmarx);
  hgnss->dma_head = GNSS_DMA_BUFFER_SIZE - dma_remaining;

  /* GPS status monitoring - print summary every 10 seconds */
  static uint32_t last_debug_time = 0;
  uint32_t now = HAL_GetTick();
  
  /* Print GPS summary every 10 seconds */
  if ((now - last_debug_time > 10000))
  {
    char summary[200];
    
    if (hgnss->data.valid && hgnss->data.fix_quality != GNSS_FIX_INVALID)
    {
      /* Valid fix - show full details */
      snprintf(summary, sizeof(summary),
               "[GPS] FIX | Sats:%d HDOP:%.1f | Lat:%.6f Lon:%.6f Alt:%.1fm | Speed:%.1fkm/h\r\n",
               hgnss->data.satellites,
               hgnss->data.hdop,
               hgnss->data.latitude,
               hgnss->data.longitude,
               hgnss->data.altitude,
               hgnss->data.speed);
    }
    else
    {
      /* No fix - show basic status */
      snprintf(summary, sizeof(summary),
               "[GPS] Searching... | Sats visible:%d | HDOP:%.1f | Status:%s\r\n",
               hgnss->data.satellites_in_view,
               hgnss->data.hdop,
               (hgnss->data.fix_quality == GNSS_FIX_INVALID) ? "No Fix" : "Acquiring");
    }
    
    SEGGER_RTT_WriteString(0, summary);
    last_debug_time = now;
  }

  /* Process all available bytes between tail and head */
  while (hgnss->dma_tail != hgnss->dma_head)
  {
    uint8_t byte = hgnss->dma_buffer[hgnss->dma_tail];
    
    /* Start of NMEA sentence */
    if (byte == '$')
    {
      hgnss->nmea_length = 0;
      hgnss->nmea_sentence[hgnss->nmea_length++] = (char)byte;
    }
    /* End of NMEA sentence */
    else if (byte == '\n' || byte == '\r')
    {
      if (hgnss->nmea_length > 0)
      {
        hgnss->nmea_sentence[hgnss->nmea_length] = '\0';
        
        /* Parse complete NMEA sentence in main loop context (safe) */
        GNSS_ParseNMEA(hgnss, hgnss->nmea_sentence);
        
        hgnss->nmea_length = 0;
      }
    }
    /* Middle of NMEA sentence */
    else if (hgnss->nmea_length > 0 && hgnss->nmea_length < (GNSS_NMEA_MAX_LENGTH - 1))
    {
      hgnss->nmea_sentence[hgnss->nmea_length++] = (char)byte;
    }
    /* Buffer overflow protection */
    else if (hgnss->nmea_length >= (GNSS_NMEA_MAX_LENGTH - 1))
    {
      hgnss->nmea_length = 0; /* Discard sentence and reset */
    }

    /* Advance tail with wraparound */
    hgnss->dma_tail = (hgnss->dma_tail + 1) % GNSS_DMA_BUFFER_SIZE;
  }

  return GNSS_OK;
}

/**
  * @brief  Calculate NMEA checksum
  * @param  sentence: NMEA sentence (without $ and checksum)
  * @retval Calculated checksum
  */
uint8_t GNSS_CalculateChecksum(const char *sentence)
{
  uint8_t checksum = 0;
  const char *p = sentence;

  /* Skip the $ character if present */
  if (*p == '$')
  {
    p++;
  }

  /* Calculate XOR of all characters until * or end */
  while (*p != '\0' && *p != '*')
  {
    checksum ^= *p;
    p++;
  }

  return checksum;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  DMA callback access for external ISR handling
  * @param  huart: UART handle pointer
  * @retval None
  * @note   These callbacks are handled in usart_if.c to avoid duplicate definitions
  *         The ISR will call GNSS_DMA_RxCallback() functions when appropriate
  */
void GNSS_DMA_RxHalfCallback(UART_HandleTypeDef *huart)
{
  /* Called from usart_if.c HAL_UART_RxHalfCpltCallback */
  if (huart->Instance == USART1 && pHgnss != NULL)
  {
    /* Minimal ISR - just set data ready flag */
    pHgnss->dma_data_ready = true;
  }
}

void GNSS_DMA_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Called from usart_if.c HAL_UART_RxCpltCallback */
  if (huart->Instance == USART1 && pHgnss != NULL)
  {
    /* Minimal ISR - just set data ready flag */
    pHgnss->dma_data_ready = true;
  }
}

/**
  * @brief  Convert NMEA coordinate format to decimal degrees
  * @param  raw_degrees: Raw coordinate (DDMM.MMMM format)
  * @retval Decimal degrees
  */
static float GNSS_ConvertToDecimalDegrees(float raw_degrees)
{
  int degrees = (int)(raw_degrees / 100.0f);
  float minutes = raw_degrees - (degrees * 100.0f);
  return degrees + (minutes / 60.0f);
}

/** 
  * @brief  Extract a token from NMEA sentence by index
  * @param  sentence: The source NMEA sentence
  * @param  index: The CSV field index (0 = Msg ID, 1 = First Data, etc.)
  * @param  buffer: Destination buffer
  * @param  max_len: Max size of destination buffer
  * @retval true if found, false if not found
  */
static bool GNSS_GetToken(const char *sentence, int index, char *buffer, int max_len)
{
  const char *start = sentence;
  const char *end;
  int current_idx = 0;

  /* Basic validation */
  if (sentence == NULL || buffer == NULL || max_len <= 0) return false;

  /* Find start of desired field */
  while (current_idx < index)
  {
    start = strchr(start, ',');
    if (start == NULL) return false; /* Not enough fields */
    start++; /* Skip the comma */
    current_idx++;
  }

  /* Find end of field (comma or checksum mark '*') */
  end = strchr(start, ',');
  if (end == NULL)
  {
    end = strchr(start, '*');
    if (end == NULL)
    {
       /* Handle end of string if no checksum marker */
       end = start + strlen(start);
       /* Trim CR/LF if present at end */
       while (end > start && (*(end-1) == '\r' || *(end-1) == '\n')) end--;
    }
  }

  /* Copy data */
  int len = (int)(end - start);
  if (len < 0) len = 0;
  if (len >= max_len) len = max_len - 1;

  memcpy(buffer, start, len);
  buffer[len] = '\0';

  return true;
}

/**
  * @brief  Parse GGA NMEA sentence
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  sentence: GGA sentence
  * @retval 0 on success, -1 on error
  */
static int GNSS_ParseGGA(GNSS_HandleTypeDef *hgnss, const char *sentence)
{
  char token[32];
  int field;
  float lat_raw = 0, lon_raw = 0;
  char lat_dir = 'N', lon_dir = 'E';

  for (field = 1; field < 15; field++)
  {
    if (!GNSS_GetToken(sentence, field, token, sizeof(token))) break;

    switch (field)
    {
      case 1: /* UTC time */
        if (strlen(token) >= 6) hgnss->data.timestamp = atoi(token);
        break;

      case 2: /* Latitude */
        if (strlen(token) > 0) lat_raw = atof(token);
        break;

      case 3: /* Latitude direction */
        if (strlen(token) > 0) lat_dir = token[0];
        break;

      case 4: /* Longitude */
        if (strlen(token) > 0) lon_raw = atof(token);
        break;

      case 5: /* Longitude direction */
        if (strlen(token) > 0) lon_dir = token[0];
        break;

      case 6: /* Fix quality */
        if (strlen(token) > 0) hgnss->data.fix_quality = (GNSS_FixQuality_t)atoi(token);
        break;

      case 7: /* Number of satellites */
        if (strlen(token) > 0) hgnss->data.satellites = atoi(token);
        break;

      case 8: /* HDOP */
        if (strlen(token) > 0) hgnss->data.hdop = atof(token);
        break;

      case 9: /* Altitude */
        if (strlen(token) > 0) hgnss->data.altitude = atof(token);
        break;
    }
  }

  /* Convert coordinates to decimal degrees */
  if (lat_raw > 0)
  {
    hgnss->data.latitude = GNSS_ConvertToDecimalDegrees(lat_raw);
    if (lat_dir == 'S') hgnss->data.latitude = -hgnss->data.latitude;
  }

  if (lon_raw > 0)
  {
    hgnss->data.longitude = GNSS_ConvertToDecimalDegrees(lon_raw);
    if (lon_dir == 'W') hgnss->data.longitude = -hgnss->data.longitude;
  }

  /* Validate coordinates */
  if (lat_raw == 0 && lon_raw == 0) return -1; /* No fix info yet */
  
  if (!GNSS_ValidateCoordinates(hgnss->data.latitude, hgnss->data.longitude))
  {
    /* Invalid coordinates */
    return -1;
  }

  /* Mark data as valid if we have a fix and coordinates are valid */
  if (hgnss->data.fix_quality != GNSS_FIX_INVALID && hgnss->data.satellites > 0)
  {
    hgnss->data.valid = true;
  }

  return 0;
}

/**
  * @brief  Parse RMC NMEA sentence
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  sentence: RMC sentence
  * @retval 0 on success, -1 on error
  */
static int GNSS_ParseRMC(GNSS_HandleTypeDef *hgnss, const char *sentence)
{
  char token[32];
  int field;
  char status = 'V';

  for (field = 1; field < 13; field++)
  {
    if (!GNSS_GetToken(sentence, field, token, sizeof(token))) break;

    switch (field)
    {
      case 1: /* UTC time */
        if (strlen(token) >= 6) hgnss->data.timestamp = atoi(token);
        break;

      case 2: /* Status */
        if (strlen(token) > 0) status = token[0];
        break;

      case 7: /* Speed */
        if (strlen(token) > 0) hgnss->data.speed = atof(token) * 1.852f;
        break;

      case 8: /* Track angle */
        if (strlen(token) > 0) hgnss->data.course = atof(token);
        break;

      case 9: /* Date */
        if (strlen(token) >= 6) hgnss->data.date = atoi(token);
        break;
    }
  }

  if (status == 'A')
  {
    hgnss->data.valid = true;
  }

  return 0;
}

/**
  * @brief  Parse GSV NMEA sentence (satellites in view)
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  sentence: GSV sentence
  * @retval 0 on success, -1 on error
  * @note   GSV format: $GPGSV,<num_msg>,<msg_num>,<sats_in_view>,...
  */
static int GNSS_ParseGSV(GNSS_HandleTypeDef *hgnss, const char *sentence)
{
  char token[16];
  /* Field 3 is Sats in View in GSV */
  if (GNSS_GetToken(sentence, 3, token, sizeof(token)))
  {
     if (strlen(token) > 0)
     {
        hgnss->data.satellites_in_view = atoi(token);
        // Disabled to prevent RTT buffer overflow
        // SEGGER_RTT_printf(0, "[GSV] Satellites in view: %d\r\n", hgnss->data.satellites_in_view);
     }
  }
  return 0;
}

/**
  * @brief  Verify NMEA checksum
  * @param  sentence: NMEA sentence with checksum
  * @retval true if valid, false otherwise
  */
static bool GNSS_VerifyChecksum(const char *sentence)
{
  const char *checksum_ptr = strchr(sentence, '*');

  /* No checksum present - assume valid */
  if (checksum_ptr == NULL)
  {
    return true;
  }

  /* Calculate checksum */
  uint8_t calculated = GNSS_CalculateChecksum(sentence);

  /* Extract checksum from sentence - manual hex parsing for reliability */
  const char *hex_str = checksum_ptr + 1;
  
  /* Need at least 2 hex digits */
  if (hex_str[0] == '\0' || hex_str[1] == '\0')
  {
    return false;
  }
  
  /* Parse first hex digit */
  uint8_t provided = 0;
  char c1 = hex_str[0];
  if (c1 >= '0' && c1 <= '9')
    provided = (c1 - '0') << 4;
  else if (c1 >= 'A' && c1 <= 'F')
    provided = (c1 - 'A' + 10) << 4;
  else if (c1 >= 'a' && c1 <= 'f')
    provided = (c1 - 'a' + 10) << 4;
  else
    return false; /* Invalid hex character */
  
  /* Parse second hex digit */
  char c2 = hex_str[1];
  if (c2 >= '0' && c2 <= '9')
    provided |= (c2 - '0');
  else if (c2 >= 'A' && c2 <= 'F')
    provided |= (c2 - 'A' + 10);
  else if (c2 >= 'a' && c2 <= 'f')
    provided |= (c2 - 'a' + 10);
  else
    return false; /* Invalid hex character */

  return (calculated == provided);
}
