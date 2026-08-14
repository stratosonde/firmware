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
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */
#include "usart_if.h"
#include "stm32_lpm.h"
#include "utilities_def.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GNSS_HandleTypeDef *pHgnss = NULL;  // For DMA ISR callback access

/* Private function prototypes -----------------------------------------------*/
static double GNSS_ConvertToDecimalDegrees(double raw_degrees);
static bool GNSS_GetToken(const char *sentence, int index, char *buffer, int max_len);
static int GNSS_ParseGGA(GNSS_HandleTypeDef *hgnss, const char *sentence);
static int GNSS_ParseRMC(GNSS_HandleTypeDef *hgnss, const char *sentence);
static int GNSS_ParseGSV(GNSS_HandleTypeDef *hgnss, const char *sentence);
static int GNSS_ParseVTG(GNSS_HandleTypeDef *hgnss, const char *sentence);
static void GNSS_UpdateVerticalSpeed(GNSS_HandleTypeDef *hgnss);
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
  memset(&hgnss->extended, 0, sizeof(GNSS_ExtendedData_t));
  hgnss->is_initialized = false;
  hgnss->is_powered = false;
  
  /* Initialize DMA circular buffer */
  memset(hgnss->dma_buffer, 0, sizeof(hgnss->dma_buffer));
  hgnss->dma_head = 0;
  hgnss->dma_tail = 0;
  hgnss->dma_data_ready = false;
  hgnss->dma_produced_total = 0;
  hgnss->dma_consumed_total = 0;
  hgnss->dma_overrun_count = 0;
  hgnss->rx_dma_active = false;      /* SP-01 (#244): no stream expected yet */
  hgnss->uart_error_count = 0;

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

  /* FULL POWER-OFF MODE: PB10=LOW, PB5=LOW (0µA at startup) */
  /* GPS module is completely off - no ephemeris retention */
  /* First fix will be cold-start (~30-60 seconds), but ensures clean UART/DMA startup */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_RESET);   // PB10 LOW (no power)
  HAL_GPIO_WritePin(hgnss->en_port, hgnss->en_pin, GPIO_PIN_RESET);     // PB5 LOW (disabled)

  SONDE_LOG_STR("GNSS_Init: PB10=LOW, PB5=LOW (fully powered off, 0µA at startup)\r\n");
  
  /* NOTE: UART pins (PB6/PB7) left in UART mode (configured by main.c) */
  /* No parasitic power issue since GPS is fully off (PB10=LOW, PB5=LOW) */
  /* UART pins will only be forced to OUTPUT-LOW in GNSS_EnterStandby() */
  SONDE_LOG_STR("GNSS_Init: UART pins left in AF mode - ready for first wake\r\n");

  hgnss->is_initialized = true;

  return GNSS_OK;
}

/**
  * @brief  Power on GNSS module (commissioning path)
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  * @note   F2 (#168): CANONICAL power-up. GNSS_Init drives PB10/PB5 LOW
  *         (module fully off), so this function MUST assert them — the old
  *         "pins stay HIGH permanently" comment was stale and commissioning
  *         could configure a powered-off receiver.
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

  /* F2 (#168): assert power + enable BEFORE any UART traffic */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_SET);   /* PB10 HIGH */
  HAL_GPIO_WritePin(hgnss->en_port,  hgnss->en_pin,  GPIO_PIN_SET);   /* PB5 HIGH */
  HAL_Delay(100);  /* module power/enable stabilization */
  SONDE_LOG_STR("GNSS_PowerOn: PWR/EN asserted, waking GPS via UART...\r\n");
  
  /* Disable STOP mode while GNSS is active */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_DISABLE);

  /* Reset DMA circular buffer pointers (and the F-011 absolute counters) */
  hgnss->dma_head = 0;
  hgnss->dma_tail = 0;
  hgnss->dma_data_ready = false;
  hgnss->dma_produced_total = 0;
  hgnss->dma_consumed_total = 0;
  memset(hgnss->dma_buffer, 0, sizeof(hgnss->dma_buffer));
  
  /* Reset NMEA sentence processing */
  hgnss->nmea_length = 0;
  memset(hgnss->nmea_sentence, 0, sizeof(hgnss->nmea_sentence));

  /* DR-10: vertical-speed state must not survive the power cycle - the first
   * GGA of the new acquisition would otherwise compute dalt over the whole
   * sleep interval and publish it as an instantaneous vertical speed. */
  hgnss->extended.has_prev_altitude = false;

  /* Start DMA circular buffer reception */
  HAL_StatusTypeDef dma_status = HAL_UART_Receive_DMA(hgnss->huart, hgnss->dma_buffer, GNSS_DMA_BUFFER_SIZE);
  
  if (dma_status != HAL_OK)
  {
    /* R9 (#194): transactional startup - EVERY failure from STARTING routes
     * through one cleanup that returns all hardware to OFF. Previously this
     * path left the module electrically powered (~25-30 mA) and STOP mode
     * disabled while is_powered stayed false, so nothing later cleaned up
     * and physical/software state disagreed. */
    UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_ENABLE);
    HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hgnss->en_port,  hgnss->en_pin,  GPIO_PIN_RESET);
    hgnss->is_powered = false;
    SONDE_LOG_STR("GNSS_PowerOn: DMA start FAILED - rolled back to OFF (pins LOW, STOP re-enabled)\r\n");
    return GNSS_ERROR;
  }

  /* SP-01 (#244): the circular stream is expected live from here — the UART
   * error callback may now re-arm it after an FE/NE glitch. */
  hgnss->rx_dma_active = true;

  /* Send wake character to exit standby mode */
  if (HAL_UART_Transmit(hgnss->huart, (uint8_t*)GNSS_WAKE_CHAR, 1, 100) != HAL_OK)
  {
    /* R9 (#194): same transactional cleanup - DMA was started, so abort it
     * too on the way back to OFF. */
    hgnss->rx_dma_active = false;  /* SP-01 (#244): not live anymore */
    HAL_UART_AbortReceive(hgnss->huart);
    UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_ENABLE);
    HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hgnss->en_port,  hgnss->en_pin,  GPIO_PIN_RESET);
    hgnss->is_powered = false;
    SONDE_LOG_STR("GNSS_PowerOn: wake TX FAILED - rolled back to OFF\r\n");
    return GNSS_ERROR;
  }
  HAL_Delay(100);  // GPS takes ~100ms to wake and start NMEA output
  
  hgnss->is_powered = true;
  SONDE_LOG_STR("GNSS_PowerOn: DMA started, GPS woken from standby\r\n");

  return GNSS_OK;
}

/**
  * @brief  Power off GNSS module (sends standby command, re-enables MCU sleep)
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  * @note   Now uses PCAS12 standby command (CASIC protocol) instead of toggling pins
  */
GNSS_StatusTypeDef GNSS_PowerOff(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL || !hgnss->is_initialized)
  {
    return GNSS_ERROR;
  }

  /* OPTIONAL: Send standby command (commented out - using EN/PWR pins instead) */
  /* GNSS_StatusTypeDef cmd_status = GNSS_SendCommandBody(hgnss, GNSS_CMD_BODY_STANDBY);
  
  if (cmd_status == GNSS_OK)
  {
    SONDE_LOG_STR("[GPS STANDBY] Standby command sent successfully (PCAS12)\r\n");
  }
  else
  {
    SONDE_LOG_STR("[GPS STANDBY] ERROR - Standby command TX failed!\r\n");
  }
  
  HAL_Delay(200); */

  /* SP-01 (#244): the stream is no longer expected live BEFORE the abort —
   * a late error callback must not re-arm a stream we are tearing down. */
  hgnss->rx_dma_active = false;

  /* Abort DMA reception */
  if (hgnss->huart != NULL && hgnss->huart->hdmarx != NULL && hgnss->is_powered)
  {
    HAL_UART_AbortReceive(hgnss->huart);
    SONDE_LOG_STR("GNSS_PowerOff: DMA receive aborted\r\n");
  }

  /* CRITICAL: Re-enable STOP mode so MCU can sleep */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_ENABLE);
  SONDE_LOG_STR("GNSS_PowerOff: MCU STOP mode re-enabled\r\n");

  /* FW-8: this function does NOT touch PB10/PB5 — they keep whatever state
   * the caller established (GNSS_EnterStandby() drives both LOW, 0µA). */
  SONDE_LOG_STR("GNSS_PowerOff: DMA aborted, pins unchanged, MCU can now sleep\r\n");
  
  hgnss->is_powered = false;

  return GNSS_OK;
}

/**
  * @brief  Configure GNSS module with initialization commands
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
/* R3-06 (#220): receiver-side behavioral verification. PCAS writes have no
 * query/ACK, so the only in-band evidence that the NMEA mask took effect is
 * the OUTPUT stream changing: GGA keeps flowing while GSV stops. Streams the
 * DMA buffer for window_ms with a rolling 4-char matcher (split-sentence
 * safe), counting whole "GGA,"/"GSV," sentence IDs. */
static void GNSS_SampleSentenceMix(GNSS_HandleTypeDef *hgnss, uint32_t window_ms,
                                   uint32_t *gga_count, uint32_t *gsv_count)
{
  *gga_count = 0;
  *gsv_count = 0;
  if (hgnss == NULL || hgnss->huart == NULL || hgnss->huart->hdmarx == NULL)
  {
    return;
  }

  uint16_t prev = (uint16_t)((GNSS_DMA_BUFFER_SIZE -
      __HAL_DMA_GET_COUNTER(hgnss->huart->hdmarx)) % GNSS_DMA_BUFFER_SIZE);
  char shift[4];
  uint8_t shift_len = 0;
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < window_ms)
  {
    uint16_t head = (uint16_t)((GNSS_DMA_BUFFER_SIZE -
        __HAL_DMA_GET_COUNTER(hgnss->huart->hdmarx)) % GNSS_DMA_BUFFER_SIZE);
    while (prev != head)
    {
      char c = (char)hgnss->dma_buffer[prev];
      prev = (uint16_t)((prev + 1U) % GNSS_DMA_BUFFER_SIZE);
      if (shift_len < 4) { shift[shift_len++] = c; }
      else { memmove(shift, shift + 1, 3); shift[3] = c; }
      if (shift_len == 4)
      {
        if (memcmp(shift, "GGA,", 4) == 0) { (*gga_count)++; }
        else if (memcmp(shift, "GSV,", 4) == 0) { (*gsv_count)++; }
      }
    }
    HAL_Delay(20);
  }
}

GNSS_StatusTypeDef GNSS_Configure(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL || !hgnss->is_powered)
  {
    return GNSS_ERROR;
  }

  SONDE_LOG_STR("\r\n=== Configuring ATGM336H GNSS Module ===\r\n");

  /* F2 (#168): every command send is accounted — a failure must surface in
   * the return value, not just a warning line. */
  uint8_t cfg_failures = 0;

  /* R26: flight mask is GGA+RMC+VTG with GSV off (bandwidth); the old
   * "GGA+RMC only" log string was never true. */
  SONDE_LOG_STR("Sending: NMEA config (GGA+RMC+VTG, GSV off)...\r\n");
  if (GNSS_SendCommandBody(hgnss, GNSS_CMD_BODY_NMEA_CONFIG) != GNSS_OK)
  {
    SONDE_LOG_STR("WARNING: Failed to send NMEA config\r\n");
    cfg_failures++;
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */

  /* Send constellation selection command (GPS+GLONASS) */
  SONDE_LOG_STR("Sending: Constellation select (GPS+GLONASS)...\r\n");
  if (GNSS_SendCommandBody(hgnss, GNSS_CMD_BODY_CONSTELLATION) != GNSS_OK)
  {
    SONDE_LOG_STR("WARNING: Failed to send constellation config\r\n");
    cfg_failures++;
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */

  /* CRITICAL: Send airborne dynamic model command (defeats 18km CoCom limit) */
  SONDE_LOG_STR("Sending: AIRBORNE dynamic model (defeats 18km CoCom limit)...\r\n");
  if (GNSS_SendCommandBody(hgnss, GNSS_CMD_BODY_AIRBORNE_MODE) != GNSS_OK)
  {
    SONDE_LOG_STR("WARNING: Failed to send airborne mode - GPS may lose fix above 18km!\r\n");
    cfg_failures++;
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */

  /* Send update rate configuration (1 Hz) */
  SONDE_LOG_STR("Sending: Update rate (1 Hz)...\r\n");
  if (GNSS_SendCommandBody(hgnss, GNSS_CMD_BODY_UPDATE_RATE) != GNSS_OK)
  {
    SONDE_LOG_STR("WARNING: Failed to send update rate\r\n");
    cfg_failures++;
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */

  /* Send satellite system configuration (GPS + BeiDou + GLONASS) */
  SONDE_LOG_STR("Sending: Satellite systems (GPS+BeiDou+GLONASS)...\r\n");
  if (GNSS_SendCommandBody(hgnss, GNSS_CMD_BODY_SATELLITE_SYS) != GNSS_OK)
  {
    SONDE_LOG_STR("WARNING: Failed to send satellite config\r\n");
    cfg_failures++;
  }
  HAL_Delay(10);  /* Minimal 10ms delay for GNSS module to process command */
  
  /* R24: PCAS11,2 ("fix mode") send DELETED. PCAS11 is one dynamic-model
   * setting — a second write overwrites the airborne model set above and
   * re-enables the 18 km CoCom limit. Airborne must be the LAST PCAS11 write. */
  
  /* Save all configuration to GPS internal flash (PCAS00) */
  SONDE_LOG_STR("Sending: Save configuration to flash...\r\n");
  if (GNSS_SendCommandBody(hgnss, GNSS_CMD_BODY_SAVE_CONFIG) != GNSS_OK)
  {
    SONDE_LOG_STR("WARNING: Failed to save configuration\r\n");
    cfg_failures++;
  }
  HAL_Delay(100);  /* Give GPS time to save to flash */

  /* R3-06 (#220): receiver-side verification of the NMEA mask. The PCAS
   * write protocol has no query/ACK, so the only in-band evidence is the
   * output stream: after the mask command, GGA must keep flowing and GSV
   * must stop. ~3 s of 1 Hz output is enough for both. */
  uint32_t gga_seen = 0, gsv_seen = 0;
  GNSS_SampleSentenceMix(hgnss, 3000, &gga_seen, &gsv_seen);
  bool nmea_verified = (gga_seen > 0) && (gsv_seen == 0);
  SONDE_LOG("NMEA mask verification: GGA sentences=%lu GSV sentences=%lu -> %s\r\n",
            (unsigned long)gga_seen, (unsigned long)gsv_seen,
            nmea_verified ? "VERIFIED receiver-side" : "NO receiver-side evidence");

  /* F2 (#168): never claim success the receiver did not confirm */
  if (cfg_failures > 0)
  {
    SONDE_LOG("=== GNSS Configuration FAILED (%u command(s) not sent) ===\r\n\r\n",
              cfg_failures);
    return GNSS_ERROR;
  }

  /* R3-06 (#220): precise claims. KNOWN: every command was handed to the
   * UART. VERIFIED (receiver-side): the NMEA output mask. NOT verifiable
   * in-band (no PCAS query): the airborne dynamic model and the flash save -
   * those need the bench commissioning procedure (full power removal, cold
   * restart, controlled acquisition check). */
  if (nmea_verified)
  {
    SONDE_LOG_STR("=== GNSS config transmitted; NMEA mask VERIFIED receiver-side ===\r\n");
    SONDE_LOG_STR("    (airborne model + flash save: transmitted, not receiver-verifiable - bench commissioning check required)\r\n\r\n");
  }
  else
  {
    SONDE_LOG_STR("=== GNSS config transmitted; UNVERIFIED - no receiver-side evidence (see bench commissioning procedure) ===\r\n\r\n");
  }

  return GNSS_OK;
}

/* F-023/D12 (#59): GNSS_GetPosition DELETED — dead blocking API (no callers,
 * HAL_Delay spin with no IWDG refresh). Don't leave brick functions behind. */

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
  // SONDE_LOG_STR("[NMEA] ");
  // SONDE_LOG_STR(sentence);
  // SONDE_LOG_STR("\r\n");

  /* Verify checksum if present */
  if (!GNSS_VerifyChecksum(sentence))
  {
    SONDE_LOG_STR("[NMEA] Checksum FAILED\r\n");
    return GNSS_INVALID;
  }

  /* Parse GGA sentence (position and altitude) */
  /* Support both GPS-only ($GPGGA) and multi-GNSS ($GNGGA) formats */
  if (strncmp(sentence, NMEA_GGA, strlen(NMEA_GGA)) == 0 ||
      strncmp(sentence, "$GNGGA", 6) == 0)
  {
    GNSS_ParseGGA(hgnss, sentence);
    /* Update vertical speed after new altitude reading */
    GNSS_UpdateVerticalSpeed(hgnss);
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
  /* Parse VTG sentence (track and ground speed) */
  /* Support GPS ($GPVTG) and multi-GNSS ($GNVTG) formats */
  else if (strncmp(sentence, NMEA_VTG, strlen(NMEA_VTG)) == 0 ||
           strncmp(sentence, "$GNVTG", 6) == 0)
  {
    GNSS_ParseVTG(hgnss, sentence);
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

/* R2-16 (#120): position trust gate for LastPos_Store. data.valid alone is
 * not enough: RMC 'A' sets it without any lat/lon tokens, and a partial GGA
 * leaves (0,0) standing. */
bool GNSS_HasPosition(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL)
  {
    return false;
  }

  /* DR-02 (#237): the persistence trust gate must be at least as strict as
   * the quality gate - range-check too, so an out-of-range coordinate with
   * presence+valid can never become last-known-good. */
  return (hgnss->data.valid && hgnss->data.position_present &&
          GNSS_ValidateCoordinates(hgnss->data.latitude, hgnss->data.longitude));
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

  /* STAB-P1#1 (#237): position_present is part of the good-quality invariant.
   * Coordinate ABSENCE is represented by position_present, not by an invalid
   * coordinate sentinel - ValidateCoordinates deliberately accepts (0,0)
   * (R32/#57), so a position-less GGA (valid fix metrics, empty lat/lon) plus
   * an RMC 'A' otherwise passed as a fresh good fix at Null Island. */
  return (hgnss->data.valid &&
          hgnss->data.position_present &&
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
  /* R32 (#57): range-only. (0,0) is a legitimate Gulf of Guinea fix; a no-fix
   * state is detected by token presence in the parser (have_lat/have_lon) and
   * by the data.valid flag - never by a magic coordinate value. */
  return (lat >= -90.0f && lat <= 90.0f && 
          lon >= -180.0f && lon <= 180.0f);
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

#ifndef SONDE_FLIGHT_BUILD
  /* DEBUG: Log command string */
  SONDE_LOG_STR("[GPS CMD] Sending: ");
  SONDE_LOG_STR(cmd);

  /* DEBUG: Log hex bytes */
  SONDE_LOG_STR("[GPS CMD] Hex: ");
  for(size_t i = 0; i < strlen(cmd); i++)
  {
    char hex_buf[8];
    snprintf(hex_buf, sizeof(hex_buf), "%02X ", (uint8_t)cmd[i]);
    SONDE_LOG_STR(hex_buf);
  }
  SONDE_LOG_STR("\r\n");
#endif /* SONDE_FLIGHT_BUILD */

  HAL_StatusTypeDef status;
  status = HAL_UART_Transmit(hgnss->huart, (uint8_t *)cmd, strlen(cmd), GNSS_UART_TIMEOUT);

  if (status != HAL_OK)
  {
    SONDE_LOG_STR("[GPS CMD] UART Transmit FAILED!\r\n");
    return GNSS_ERROR;
  }
  
  SONDE_LOG_STR("[GPS CMD] UART Transmit OK\r\n");

  return GNSS_OK;
}

GNSS_StatusTypeDef GNSS_SendCommandBody(GNSS_HandleTypeDef *hgnss, const char *body)
{
  if (hgnss == NULL || body == NULL)
  {
    return GNSS_ERROR;
  }

  /* R23 (#22): checksum computed at runtime — can never drift from the body */
  char cmd[96];
  uint8_t cs = GNSS_CalculateChecksum(body);
  int len = snprintf(cmd, sizeof(cmd), "$%s*%02X\r\n", body, cs);
  if (len <= 0 || len >= (int)sizeof(cmd))
  {
    return GNSS_ERROR;  /* body too long — refuse to truncate a command */
  }

  return GNSS_SendCommand(hgnss, cmd);
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

  /* Get current DMA head position (hardware write position).
   * F-011: when the DMA counter reads 0 (buffer exactly full), the raw
   * subtraction yields head == GNSS_DMA_BUFFER_SIZE, which dma_tail (always
   * 0..SIZE-1) can never equal — the consume loop below never terminates.
   * Wrap the head into [0, SIZE-1]; head == tail then correctly means
   * "no new bytes" (one byte of buffer capacity is sacrificed, as is
   * standard for lock-free circular buffers). */
  uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(hgnss->huart->hdmarx);
  hgnss->dma_head = (uint16_t)((GNSS_DMA_BUFFER_SIZE - dma_remaining) % GNSS_DMA_BUFFER_SIZE);

#ifndef SONDE_FLIGHT_BUILD
  /* R2-15 (#119) sweep: the 10-second GPS summary is a 200-byte stack buffer +
   * two snprintf paths that execute in flight builds for output compiled away.
   * Gate the whole block. */
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
      /* FW-16: integer-only print (float printf support is not linked) */
      int32_t hdop_d = (int32_t)(hgnss->data.hdop * 10.0f);
      int32_t lat_u  = (int32_t)(hgnss->data.latitude * 1000000.0);
      int32_t lon_u  = (int32_t)(hgnss->data.longitude * 1000000.0);
      int32_t alt_d  = (int32_t)(hgnss->data.altitude * 10.0f);
      int32_t spd_d  = (int32_t)(hgnss->data.speed * 10.0f);
      snprintf(summary, sizeof(summary),
               "[GPS] FIX | Sats:%d HDOP:%d.%d | Lat:%d.%06d Lon:%d.%06d Alt:%d.%dm | Speed:%d.%dkm/h\r\n",
               hgnss->data.satellites,
               (int)(hdop_d / 10), (int)((hdop_d < 0 ? -hdop_d : hdop_d) % 10),
               (int)(lat_u / 1000000), (int)((lat_u < 0 ? -lat_u : lat_u) % 1000000),
               (int)(lon_u / 1000000), (int)((lon_u < 0 ? -lon_u : lon_u) % 1000000),
               (int)(alt_d / 10), (int)((alt_d < 0 ? -alt_d : alt_d) % 10),
               (int)(spd_d / 10), (int)((spd_d < 0 ? -spd_d : spd_d) % 10));
    }
    else
    {
      /* No fix - show basic status */
      /* FW-16: integer-only print (float printf support is not linked) */
      int32_t hdop_d2 = (int32_t)(hgnss->data.hdop * 10.0f);
      snprintf(summary, sizeof(summary),
               "[GPS] Searching... | Sats visible:%d | HDOP:%d.%d | Status:%s\r\n",
               hgnss->data.satellites_in_view,
               (int)(hdop_d2 / 10), (int)((hdop_d2 < 0 ? -hdop_d2 : hdop_d2) % 10),
               (hgnss->data.fix_quality == GNSS_FIX_INVALID) ? "No Fix" : "Acquiring");
    }

    SONDE_LOG_STR(summary);
    last_debug_time = now;
  }
#endif /* SONDE_FLIGHT_BUILD */

  /* F-011 (#25): overrun detection on the absolute counters. If the DMA
   * producer (half/full callbacks, 256-granular) has lapped the consumer,
   * unparsed bytes are already destroyed — count it, drop to the head, and
   * resync: the parser restarts cleanly at the next '$' (DDR-0003: the gap
   * is surfaced, never silent). */
  /* S-01 (#225): the R2-19 (#123) compare was quanta-INCONSISTENT.
   * dma_produced_total advances in 256-byte callback quanta while
   * dma_consumed_total advances one byte at a time, so the raw unsigned
   * difference mismeasures the true backlog both ways:
   *  - STARTUP BURST: at wake the receiver dumps its buffered output while
   *    the parser has not drained; (produced - consumed) legitimately
   *    exceeded SIZE - 256 with every byte still buffered -> false overrun,
   *    resync shredded the first ~270 ms of every acquisition.
   *  - STEADY-STATE INVERSION: the byte-exact consumer routinely runs up to
   *    255 bytes AHEAD of the 256-quantum producer counter; the unsigned
   *    subtraction then wraps to ~2^32 and the compare misfires.
   * Quanta-consistent form: fire only when a byte is PROVABLY destroyed -
   * the producer counter is more than a FULL buffer ahead of the consumer
   * (hardware must have lapped). A full buffer of slack makes the detector
   * immune to startup bursts by construction; clamping the inverted case
   * (consumer ahead = counter lag, not loss) kills the wrap misfire. */
  uint32_t dma_backlog = (hgnss->dma_produced_total >= hgnss->dma_consumed_total)
                         ? (hgnss->dma_produced_total - hgnss->dma_consumed_total)
                         : 0U;
  /* R2-19 (#123) survives in narrowed form: at EXACTLY one buffer-full the
   * data is not yet destroyed (loss begins at SIZE + 1 written) - but when
   * head == tail the F-011 wrap makes the full buffer positionally
   * unreachable, so the resync is still the only way forward. */
  if (dma_backlog > GNSS_DMA_BUFFER_SIZE ||
      (dma_backlog == GNSS_DMA_BUFFER_SIZE && hgnss->dma_tail == hgnss->dma_head))
  {
    hgnss->dma_overrun_count++;
    SONDE_LOG("[GPS DMA] OVERRUN #%lu - producer lapped consumer, resyncing\r\n",
                      (unsigned long)hgnss->dma_overrun_count);
    hgnss->dma_tail = hgnss->dma_head;
    hgnss->dma_consumed_total = hgnss->dma_produced_total;
    hgnss->nmea_length = 0;  /* discard any partial sentence */
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

    /* Advance tail with wraparound (+ absolute consumer counter, F-011) */
    hgnss->dma_tail = (hgnss->dma_tail + 1) % GNSS_DMA_BUFFER_SIZE;
    hgnss->dma_consumed_total++;
  }

  return GNSS_OK;
}

/**
  * @brief  Parse GSV NMEA sentence (satellites in view with detailed PRN tracking)
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  sentence: GSV sentence
  * @retval 0 on success, -1 on error
  * @note   GSV format: $GPGSV,<num_msg>,<msg_num>,<sats_in_view>,<prn>,<elev>,<azim>,<snr>,...
  * @note   Supports GPS ($GPGSV), GLONASS ($GLGSV), BeiDou ($BDGSV)
  */
static int GNSS_ParseGSV(GNSS_HandleTypeDef *hgnss, const char *sentence)
{
  char token[16];
  int msg_num = 0;
  
  /* Determine constellation type from sentence ID */
  GNSS_SatelliteInfo_t *sat_array = NULL;
  uint8_t *sat_count = NULL;
  uint8_t max_sats = GNSS_MAX_SATS_PER_CONSTELLATION;
  
  if (strncmp(sentence, "$GPGSV", 6) == 0)
  {
    sat_array = hgnss->extended.gps_sats;
    sat_count = &hgnss->extended.gps_count;
  }
  else if (strncmp(sentence, "$GLGSV", 6) == 0)
  {
    sat_array = hgnss->extended.glonass_sats;
    sat_count = &hgnss->extended.glonass_count;
  }
  else if (strncmp(sentence, "$BDGSV", 6) == 0)
  {
    sat_array = hgnss->extended.beidou_sats;
    sat_count = &hgnss->extended.beidou_count;
  }
  else
  {
    /* GNGSV or unknown - update total count only */
    if (GNSS_GetToken(sentence, 3, token, sizeof(token)))
    {
      if (strlen(token) > 0)
        hgnss->data.satellites_in_view = atoi(token);
    }
    return 0;
  }
  
  /* Get message number (field 2) - if it's message 1, reset the count */
  if (GNSS_GetToken(sentence, 2, token, sizeof(token)))
  {
    if (strlen(token) > 0)
      msg_num = atoi(token);
  }
  
  if (msg_num == 1)
  {
    *sat_count = 0;  /* Reset count for new GSV sequence */
  }
  
  /* Field 3 is total satellites in view */
  if (GNSS_GetToken(sentence, 3, token, sizeof(token)))
  {
    if (strlen(token) > 0)
      hgnss->data.satellites_in_view = atoi(token);
  }
  
  /* Parse up to 4 satellites per GSV message (fields 4-19) */
  for (int sat_idx = 0; sat_idx < 4; sat_idx++)
  {
    int base_field = 4 + (sat_idx * 4);  /* Each satellite has 4 fields */
    
    /* Check if we have room in the array */
    if (*sat_count >= max_sats)
      break;
    
    /* Field: PRN */
    if (!GNSS_GetToken(sentence, base_field, token, sizeof(token)))
      break;
    if (strlen(token) == 0)
      break;  /* No more satellites in this message */
      
    uint8_t prn = atoi(token);
    if (prn == 0)
      break;
    
    /* Store satellite info */
    sat_array[*sat_count].prn = prn;
    
    /* Field: Elevation */
    if (GNSS_GetToken(sentence, base_field + 1, token, sizeof(token)) && strlen(token) > 0)
      sat_array[*sat_count].elevation = atoi(token);
    else
      sat_array[*sat_count].elevation = 0;
    
    /* Field: Azimuth */
    if (GNSS_GetToken(sentence, base_field + 2, token, sizeof(token)) && strlen(token) > 0)
      sat_array[*sat_count].azimuth = atoi(token);
    else
      sat_array[*sat_count].azimuth = 0;
    
    /* Field: SNR */
    if (GNSS_GetToken(sentence, base_field + 3, token, sizeof(token)) && strlen(token) > 0)
      sat_array[*sat_count].snr = atoi(token);
    else
      sat_array[*sat_count].snr = 0;
    
    (*sat_count)++;
  }
  
  return 0;
}

/**
  * @brief  Parse VTG NMEA sentence (track and ground speed)
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  sentence: VTG sentence
  * @retval 0 on success, -1 on error
  * @note   VTG format: $GPVTG,<track_true>,T,<track_mag>,M,<speed_knots>,N,<speed_kmh>,K,<mode>
  */
static int GNSS_ParseVTG(GNSS_HandleTypeDef *hgnss, const char *sentence)
{
  char token[16];
  
  /* Field 1: Track made good (true north) */
  if (GNSS_GetToken(sentence, 1, token, sizeof(token)) && strlen(token) > 0)
  {
    hgnss->extended.track_true = atof(token);
  }
  
  /* Field 3: Track made good (magnetic north) */
  if (GNSS_GetToken(sentence, 3, token, sizeof(token)) && strlen(token) > 0)
  {
    hgnss->extended.track_magnetic = atof(token);
  }
  
  /* Field 7: Ground speed (km/h) - more accurate than RMC */
  if (GNSS_GetToken(sentence, 7, token, sizeof(token)) && strlen(token) > 0)
  {
    hgnss->extended.ground_speed_kmh = atof(token);
  }
  
  return 0;
}

/**
  * @brief  Calculate vertical speed from altitude changes
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval None
  * @note   Must be called after altitude update (from GGA)
  */
static void GNSS_UpdateVerticalSpeed(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL)
    return;
  
  uint32_t current_time = HAL_GetTick();
  float current_altitude = hgnss->data.altitude;
  
  /* Need at least one previous reading to calculate speed */
  if (hgnss->extended.has_prev_altitude)
  {
    /* Calculate time delta in seconds */
    float time_delta_s = (current_time - hgnss->extended.prev_timestamp) / 1000.0f;
    
    /* Minimum time delta to avoid division by zero */
    if (time_delta_s > 0.1f)  /* At least 100ms between readings */
    {
      /* Calculate vertical speed in m/s */
      float altitude_change = current_altitude - hgnss->extended.prev_altitude;
      hgnss->extended.vertical_speed_ms = altitude_change / time_delta_s;
      
      /* Calculate 3D speed (pythagorean theorem) */
      /* Convert ground speed from km/h to m/s for calculation */
      float ground_speed_ms = hgnss->extended.ground_speed_kmh / 3.6f;
      
      /* 3D speed = sqrt(horizontal^2 + vertical^2) */
      float speed_3d_ms = sqrtf((ground_speed_ms * ground_speed_ms) + 
                                (hgnss->extended.vertical_speed_ms * hgnss->extended.vertical_speed_ms));
      
      /* Convert back to km/h */
      hgnss->extended.speed_3d_kmh = speed_3d_ms * 3.6f;
    }
  }
  
  /* Store current values for next calculation */
  hgnss->extended.prev_altitude = current_altitude;
  hgnss->extended.prev_timestamp = current_time;
  hgnss->extended.has_prev_altitude = true;
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
    /* Minimal ISR - set data ready + advance absolute producer (F-011, #25) */
    pHgnss->dma_data_ready = true;
    pHgnss->dma_produced_total += GNSS_DMA_BUFFER_SIZE / 2;
  }
}

void GNSS_DMA_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Called from usart_if.c HAL_UART_RxCpltCallback */
  if (huart->Instance == USART1 && pHgnss != NULL)
  {
    /* Minimal ISR - set data ready + advance absolute producer (F-011, #25) */
    pHgnss->dma_data_ready = true;
    pHgnss->dma_produced_total += GNSS_DMA_BUFFER_SIZE / 2;
  }
}

uint32_t GNSS_GetDmaOverrunCount(const GNSS_HandleTypeDef *hgnss)
{
  return (hgnss != NULL) ? hgnss->dma_overrun_count : 0;
}

uint32_t GNSS_GetUartErrorCount(const GNSS_HandleTypeDef *hgnss)
{
  return (hgnss != NULL) ? hgnss->uart_error_count : 0;
}

/* SP-01 (#244): recover the circular GNSS stream after a UART error.
 *
 * The vendored HAL treats ANY error (FE/NE/ORE/PE) as BLOCKING while
 * CR3.DMAR is set (stm32wlxx_hal_uart.c: HAL_UART_IRQHandler runs
 * UART_EndRxTransfer() + HAL_DMA_Abort_IT()) and NEVER consults the
 * AdvancedInit DMADisableonRxError bit S-C (#212) set — that bit only relaxes
 * the hardware DMA-request disable. Without this callback, a single framing
 * or noise glitch (expected at GPS wake: PB6 is driven LOW and PB7 is analog
 * during standby, so the first character after wake is easily partial) killed
 * reception for the whole acquisition window, looking exactly like "no bytes".
 *
 * Called via HAL_UART_ErrorCallback (usart_if.c) AFTER the HAL has fully
 * quiesced the DMA abort; the hardware error flags were already cleared by
 * the ISR. We count the event, drop the torn partial sentence (the NMEA
 * checksum would reject it anyway), restart the ring bookkeeping, and re-arm
 * the circular DMA. This is NOT a fresh acquisition: the overrun counter and
 * the cross-sleep vertical-speed reference (DR-10) both survive. A
 * teardown-time error (rx_dma_active already false) is ignored. */
void GNSS_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart->Instance != USART1 || pHgnss == NULL)
  {
    return;
  }
  if (!pHgnss->rx_dma_active)
  {
    return;  /* teardown/dormant: nothing to recover */
  }

  pHgnss->uart_error_count++;
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  /* The DMA restarts at buffer index 0, so the ring indices and the F-011
   * absolute producer/consumer counters must restart too. */
  pHgnss->dma_head = 0;
  pHgnss->dma_tail = 0;
  pHgnss->dma_data_ready = false;
  pHgnss->dma_produced_total = 0;
  pHgnss->dma_consumed_total = 0;
  pHgnss->nmea_length = 0;

  if (HAL_UART_Receive_DMA(huart, pHgnss->dma_buffer, GNSS_DMA_BUFFER_SIZE) != HAL_OK)
  {
    /* Re-arm refused (e.g. HAL lock contention against an interrupted
     * task-level UART op): the stream stays dead for this window. Honest
     * disengage — the GNSS timeout path ends the acquisition and the next
     * wake re-arms from scratch. The event itself is counted above. */
    pHgnss->rx_dma_active = false;
  }
}

/**
  * @brief  Convert NMEA coordinate format to decimal degrees
  * @param  raw_degrees: Raw coordinate (DDMM.MMMM format)
  * @retval Decimal degrees
  * @note   R34 (#57): double math end-to-end — the archive container resolves
  *         1e-7 deg (~1 cm); float conversion quantized to ~1 m.
  */
static double GNSS_ConvertToDecimalDegrees(double raw_degrees)
{
  int degrees = (int)(raw_degrees / 100.0);
  double minutes = raw_degrees - ((double)degrees * 100.0);
  return degrees + (minutes / 60.0);
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
  double lat_raw = 0, lon_raw = 0;   /* R34 (#57): double until the store */
  bool have_lat = false, have_lon = false;  /* R32 (#57): token PRESENCE, not magnitude */
  /* F-11 (#186): no N/E default - the hemisphere letter gives the coordinate
   * its meaning; a missing/garbled direction REJECTS the fix below. */
  char lat_dir = '\0', lon_dir = '\0';
  /* DR-01 (#236): parse into LOCALS ONLY. hgnss->data is committed in one
   * block after the last guard, so a rejected sentence leaves the shared
   * state byte-for-byte unchanged - the caller discards the return value,
   * so a pre-guard write (even one later rejected by F-11 or the range
   * check) would otherwise stand and be consumed as a real position. */
  uint32_t timestamp = 0;       bool have_timestamp = false;
  GNSS_FixQuality_t fix_quality = GNSS_FIX_INVALID; bool have_fix_quality = false;
  uint8_t satellites = 0;       bool have_satellites = false;
  float hdop = 0.0f;            bool have_hdop = false;
  float altitude = 0.0f;        bool have_altitude = false;

  for (field = 1; field < 15; field++)
  {
    if (!GNSS_GetToken(sentence, field, token, sizeof(token))) break;

    switch (field)
    {
      case 1: /* UTC time */
        if (strlen(token) >= 6)
        {
          /* F-10 (#185): range-check HHMMSS before storing - a garbled but
           * checksum-valid time (e.g. 995910 -> hours=99) must never reach
           * SysTimeSyncFromGnss; it would discipline SysTime by ~4 days.
           * 60 s permits a leap second. */
          uint32_t hhmmss = strtoul(token, NULL, 10);
          uint32_t hh = hhmmss / 10000u, mm = (hhmmss / 100u) % 100u, ss = hhmmss % 100u;
          if (hh <= 23u && mm <= 59u && ss <= 60u) { timestamp = hhmmss; have_timestamp = true; }
        }
        break;

      case 2: /* Latitude */
        if (strlen(token) > 0) { lat_raw = atof(token); have_lat = true; }
        break;

      case 3: /* Latitude direction */
        if (strlen(token) > 0) lat_dir = token[0];
        break;

      case 4: /* Longitude */
        if (strlen(token) > 0) { lon_raw = atof(token); have_lon = true; }
        break;

      case 5: /* Longitude direction */
        if (strlen(token) > 0) lon_dir = token[0];
        break;

      case 6: /* Fix quality */
        if (strlen(token) > 0) { fix_quality = (GNSS_FixQuality_t)atoi(token); have_fix_quality = true; }
        break;

      case 7: /* Number of satellites */
        if (strlen(token) > 0) { satellites = (uint8_t)atoi(token); have_satellites = true; }
        break;

      case 8: /* HDOP */
        if (strlen(token) > 0) { hdop = (float)atof(token); have_hdop = true; }
        break;

      case 9: /* Altitude */
        if (strlen(token) > 0) { altitude = (float)atof(token); have_altitude = true; }
        break;
    }
  }

  /* Validate coordinates */
  if (!have_lat || !have_lon) return -1; /* No fix info yet (empty fields) */

  /* F-11 (#186): the hemisphere letter is the qualifier that gives the
   * coordinate its sign. A missing or non-{N,S}/{E,W} direction (truncated
   * sentence whose '*' landed after the numeric field still passes checksum)
   * REJECTS the fix - it must never default to N/E. A dropped 'W' otherwise
   * moved a Calgary launch (51N 114W) to +114E with no downstream flag. */
  if ((lat_dir != 'N' && lat_dir != 'S') || (lon_dir != 'E' && lon_dir != 'W'))
  {
    return -1;
  }

  /* Convert coordinates to decimal degrees (R32/#57: presence-tracked — a real
   * 0.0 coordinate on the equator / prime meridian is valid data, not absence) */
  double latitude = GNSS_ConvertToDecimalDegrees(lat_raw);
  if (lat_dir == 'S') latitude = -latitude;
  double longitude = GNSS_ConvertToDecimalDegrees(lon_raw);
  if (lon_dir == 'W') longitude = -longitude;

  if (!GNSS_ValidateCoordinates((float)latitude, (float)longitude))
  {
    /* Invalid coordinates */
    return -1;
  }

  /* DR-01 (#236): every guard has passed - commit the whole sentence in one
   * block. R2-16 (#120): position_present=true is only ever committed here,
   * from a sentence that carried real lat/lon tokens AND passed validation. */
  if (have_timestamp)    hgnss->data.timestamp = timestamp;
  if (have_fix_quality)  hgnss->data.fix_quality = fix_quality;
  if (have_satellites)   hgnss->data.satellites = satellites;
  if (have_hdop)         hgnss->data.hdop = hdop;
  if (have_altitude)     hgnss->data.altitude = altitude;
  hgnss->data.latitude = latitude;
  hgnss->data.longitude = longitude;
  hgnss->data.position_present = true;

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
  /* F-10 (#185): time/date are HELD until the status field is known - a 'V'
   * (void) sentence must never rewrite the SysTime discipline inputs, and the
   * time is range-checked before it is stored (same HHMMSS rule as GGA). */
  uint32_t rmc_time = 0;
  uint32_t rmc_date = 0;
  bool have_time = false, have_date = false;

  for (field = 1; field < 13; field++)
  {
    if (!GNSS_GetToken(sentence, field, token, sizeof(token))) break;

    switch (field)
    {
      case 1: /* UTC time */
        if (strlen(token) >= 6) { rmc_time = strtoul(token, NULL, 10); have_time = true; }
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
        if (strlen(token) >= 6) { rmc_date = strtoul(token, NULL, 10); have_date = true; }
        break;
    }
  }

  /* F-10 (#185): only an ACTIVE sentence stores clock inputs, and only if the
   * time is in range (hh <= 23, mm <= 59, ss <= 60 - 60 permits a leap
   * second). A garbled or void sentence leaves the held time/date untouched. */
  if (status == 'A')
  {
    if (have_time)
    {
      uint32_t hh = rmc_time / 10000u, mm = (rmc_time / 100u) % 100u, ss = rmc_time % 100u;
      if (hh <= 23u && mm <= 59u && ss <= 60u) hgnss->data.timestamp = rmc_time;
    }
    if (have_date) hgnss->data.date = rmc_date;
  }

  /* R2-30 (#130): the RMC status field is authoritative for fix validity -
   * 'V' (void) must CLEAR a previously latched valid, not just fail to set
   * it. Otherwise a fix lost mid-window kept reading as held until reboot. */
  hgnss->data.valid = (status == 'A');

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

  /* R33 (#57): no checksum present -> REJECT. Every ATGM336H sentence carries
   * *XX; a missing one means a DMA-truncated/corrupt sentence, not a valid one. */
  if (checksum_ptr == NULL)
  {
    return false;
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
/**
  * @brief  Enter GPS standby mode (FW-8: FULL power-off, 0µA — PCAS12 saves
  *         ephemeris to GPS internal flash, then BOTH PB10 and PB5 are cut.
  *         There is no live backup rail; hot-start relies on the saved
  *         ephemeris in flash, not on retained power. The ~15µA figure in
  *         older comments described a PB10-HIGH backup mode that is NOT
  *         what this function does.)
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_EnterStandby(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL)
  {
    return GNSS_ERROR;
  }
  
  /* CRITICAL: Send PCAS12 standby command FIRST (while UART active).
   * FW-8: PCAS12 is the standby-entry command — it is what makes the module
   * persist ephemeris to its internal flash (PCAS00 is the flash-save command
   * used at commissioning). The 100 ms below covers that internal save before
   * the full power cut. */
  GNSS_StatusTypeDef cmd_status = GNSS_SendCommandBody(hgnss, GNSS_CMD_BODY_STANDBY);

  if (cmd_status == GNSS_OK)
  {
    SONDE_LOG_STR("[GPS STANDBY] PCAS12 command sent - GPS saving ephemeris...\r\n");
  }
  else
  {
    SONDE_LOG_STR("[GPS STANDBY] WARNING - PCAS12 TX failed!\r\n");
  }

  /* Wait 100ms for the PCAS12-triggered ephemeris save before power cut */
  HAL_Delay(100);
  
  /* Abort DMA first - stop receiving data */
  if (hgnss->huart != NULL && hgnss->huart->hdmarx != NULL && hgnss->is_powered)
  {
    HAL_UART_AbortReceive(hgnss->huart);
    SONDE_LOG_STR("[GPS STANDBY] DMA aborted\r\n");
  }
  
  /* Flush UART hardware FIFO to discard any stale data */
  if (hgnss->huart != NULL)
  {
    __HAL_UART_FLUSH_DRREGISTER(hgnss->huart);
    __HAL_UART_CLEAR_FLAG(hgnss->huart, UART_CLEAR_PEF);   // Parity error
    __HAL_UART_CLEAR_FLAG(hgnss->huart, UART_CLEAR_FEF);   // Framing error
    __HAL_UART_CLEAR_FLAG(hgnss->huart, UART_CLEAR_NEF);   // Noise error
    __HAL_UART_CLEAR_FLAG(hgnss->huart, UART_CLEAR_OREF);  // Overrun error
    SONDE_LOG_STR("[GPS STANDBY] UART hardware FIFO flushed\r\n");
  }
  
  /* Clear software buffers and reset state */
  hgnss->dma_head = 0;
  hgnss->dma_tail = 0;
  hgnss->dma_data_ready = false;
  hgnss->nmea_length = 0;
  memset(hgnss->dma_buffer, 0, sizeof(hgnss->dma_buffer));
  memset(hgnss->nmea_sentence, 0, sizeof(hgnss->nmea_sentence));
  SONDE_LOG_STR("[GPS STANDBY] Software buffers cleared\r\n");
  
  /* Configure UART pins for minimal power (full power-off follows below) */
  if (hgnss->huart != NULL)
  {
    /* SP-01 (#244): stream no longer expected live before DeInit tears it
     * down — a late error callback must not re-arm here. */
    hgnss->rx_dma_active = false;
    HAL_UART_DeInit(hgnss->huart);
    SONDE_LOG_STR("[GPS STANDBY] UART deinitialized\r\n");

    /* SP-09 (#249): pin sleep policy is owned by GNSS_UARTPins_SleepSafe() -
     * the one definition every teardown path shares. */
    GNSS_UARTPins_SleepSafe();

    SONDE_LOG_STR("[GPS STANDBY] PB6=OUTPUT-LOW, PB7=ANALOG (hi-Z)\r\n");
  }

  /* FULL POWER-OFF MODE: Both PB10=LOW and PB5=LOW after PCAS12 */
  /* GPS has saved ephemeris to internal flash (via PCAS12 command) */
  /* Power consumption: 0µA during sleep; hot-start on wake uses the
   * flash-persisted ephemeris — no live backup rail exists. */
  
  /* Cut all power to GPS module */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_RESET);  // PB10 LOW (no power)
  HAL_GPIO_WritePin(hgnss->en_port, hgnss->en_pin, GPIO_PIN_RESET);    // PB5 LOW (disabled)

  SONDE_LOG_STR("[GPS STANDBY] PB10=LOW, PB5=LOW (full power-off, 0µA) - ephemeris saved\r\n");

  /* CRITICAL: Re-enable MCU STOP mode */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_ENABLE);
  SONDE_LOG_STR("[GPS STANDBY] MCU STOP mode re-enabled\r\n");

  SONDE_LOG_STR("[GPS STANDBY] Complete - GPS fully off (0µA), ephemeris in flash\r\n");
  
  hgnss->is_powered = false;
  return GNSS_OK;
}

/* SP-09 (#249): the SINGLE owner of the UART pin sleep state. Pre-fix, PB6
 * had three owners with two contradictory policies: the CubeMX MSP (AF on
 * every HAL_UART_Init), EnterStandby's anti-parasitic PB6-LOW, and
 * EnterStopMode's blanket ANALOG which silently undid the LOW drive every
 * sleep cycle. Every teardown site now ends here:
 *   PB6 (MCU TX -> GPS RX) OUTPUT-LOW  - no ESD-clamp leakage into the
 *                                        depowered module's RX pin
 *   PB7 (GPS TX -> MCU RX) ANALOG/Hi-Z - it's the GPS module's OUTPUT;
 *                                        NEVER drive this low.
 * NOTE: HAL_UART_MspDeInit de-configures the pins, so sites that call
 * HAL_UART_DeInit must call this AFTER the DeInit. */
void GNSS_UARTPins_SleepSafe(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  Wake GPS from standby mode
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  * @note   IMPROVED SEQUENCE: Initialize UART/DMA BEFORE enabling GPS (eliminates FIFO corruption)
  */
GNSS_StatusTypeDef GNSS_WakeFromStandby(GNSS_HandleTypeDef *hgnss)
{
  if (hgnss == NULL || !hgnss->is_initialized)
  {
    return GNSS_ERROR;
  }
  
  /* Disable MCU STOP mode during GPS operation */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_DISABLE);
  SONDE_LOG_STR("[GPS WAKE] MCU STOP mode disabled\r\n");
  
  /* STEP 1: Initialize UART peripheral FIRST (before GPS transmits) */
  if (hgnss->huart != NULL)
  {
    /* Reinitialize UART peripheral - restores PB6/PB7 to UART function */
    /* R3-06 (#220): check the init result - a failed UART restore must not
     * continue blindly (the DMA would start on a dead peripheral). Roll back
     * the LPM lock exactly like the DMA-failure path below. */
    if (HAL_UART_Init(hgnss->huart) != HAL_OK)
    {
      SONDE_LOG_STR("[GPS WAKE] ERROR - UART reinit failed\r\n");
      UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_ENABLE);
      return GNSS_ERROR;
    }
    SONDE_LOG_STR("[GPS WAKE] UART reinitialized\r\n");
    
    /* CRITICAL: Flush UART hardware FIFO to ensure clean start */
    /* This prevents reading any stale/corrupt data from previous cycle */
    __HAL_UART_FLUSH_DRREGISTER(hgnss->huart);
    __HAL_UART_CLEAR_FLAG(hgnss->huart, UART_CLEAR_PEF);   // Parity error
    __HAL_UART_CLEAR_FLAG(hgnss->huart, UART_CLEAR_FEF);   // Framing error
    __HAL_UART_CLEAR_FLAG(hgnss->huart, UART_CLEAR_NEF);   // Noise error
    __HAL_UART_CLEAR_FLAG(hgnss->huart, UART_CLEAR_OREF);  // Overrun error
    SONDE_LOG_STR("[GPS WAKE] UART hardware FIFO flushed\r\n");
  }
  
  /* STEP 2: Clear software buffers (and the F-011 absolute counters) */
  hgnss->dma_head = 0;
  hgnss->dma_tail = 0;
  hgnss->dma_data_ready = false;
  hgnss->dma_produced_total = 0;
  hgnss->dma_consumed_total = 0;
  hgnss->nmea_length = 0;
  memset(hgnss->dma_buffer, 0, sizeof(hgnss->dma_buffer));
  memset(hgnss->nmea_sentence, 0, sizeof(hgnss->nmea_sentence));
  /* DR-10: drop the cross-sleep vertical-speed reference too (see GNSS_PowerOn). */
  hgnss->extended.has_prev_altitude = false;
  SONDE_LOG_STR("[GPS WAKE] Software buffers cleared\r\n");
  
  /* STEP 3: Start DMA reception (UART is ready and listening) */
  HAL_StatusTypeDef dma_status = HAL_UART_Receive_DMA(hgnss->huart, hgnss->dma_buffer, GNSS_DMA_BUFFER_SIZE);
  if (dma_status != HAL_OK)
  {
    SONDE_LOG_STR("[GPS WAKE] ERROR - DMA start failed\r\n");
    /* BUG 2.4 FIX: Re-enable STOP mode on error path to prevent permanent ~mA Sleep-only */
    UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_ENABLE);
    return GNSS_ERROR;
  }
  SONDE_LOG_STR("[GPS WAKE] DMA started - ready to receive\r\n");

  /* SP-01 (#244): the circular stream is expected live from here — BEFORE the
   * module is enabled (next step), so an early glitched first character can
   * still recover via the error callback instead of killing the whole window. */
  hgnss->rx_dma_active = true;
  
  /* STEP 4: Ensure PB10 is HIGH (main power) - may have been affected by sleep mode */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_SET);   // PB10 HIGH (main power)
  SONDE_LOG_STR("[GPS WAKE] PB10 HIGH - main power confirmed\r\n");
  
  /* STEP 5: Enable GPS via PB5 - UART/DMA infrastructure is ready to capture from first byte */
  HAL_GPIO_WritePin(hgnss->en_port, hgnss->en_pin, GPIO_PIN_SET);     // PB5 HIGH (enable)
  SONDE_LOG_STR("[GPS WAKE] PB5 HIGH - GPS enabled\r\n");
  
#ifndef SONDE_FLIGHT_BUILD
  /* R2-15 (#119): this debug block must not exist in flight builds — the
   * buffers, snprintf calls, and float/int conversions execute even though
   * SONDE_LOG_STR compiles the output away. Gate the whole thing, not just
   * the log macro (same pattern as FR-16/#97). */
  /* DEBUG: Read back GPIO states to verify pins are actually set */
  GPIO_PinState pb10_state = HAL_GPIO_ReadPin(hgnss->pwr_port, hgnss->pwr_pin);
  GPIO_PinState pb5_state = HAL_GPIO_ReadPin(hgnss->en_port, hgnss->en_pin);
  char pin_buf[80];
  snprintf(pin_buf, sizeof(pin_buf), "[GPS WAKE DEBUG] Pin readback: PB10=%d PB5=%d (expect 1,1)\r\n",
           pb10_state, pb5_state);
  SONDE_LOG_STR(pin_buf);

  /* DEBUG: Check UART error flags */
  uint32_t uart_errors = hgnss->huart->ErrorCode;
  if (uart_errors != HAL_UART_ERROR_NONE) {
    char err_buf[80];
    snprintf(err_buf, sizeof(err_buf), "[GPS WAKE DEBUG] UART errors: 0x%08lX\r\n",
             (unsigned long)uart_errors);
    SONDE_LOG_STR(err_buf);
  } else {
    SONDE_LOG_STR("[GPS WAKE DEBUG] UART no errors\r\n");
  }
#endif /* SONDE_FLIGHT_BUILD */
  
  /* No delay needed - 40 second polling loop will wait for GPS boot and satellite acquisition */
  hgnss->is_powered = true;
  SONDE_LOG_STR("[GPS WAKE] Complete - UART/DMA ready for GPS transmission\r\n");
  
  return GNSS_OK;
}
