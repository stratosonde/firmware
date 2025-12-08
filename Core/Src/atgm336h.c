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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static float GNSS_ConvertToDecimalDegrees(float raw_degrees);
static int GNSS_ParseGGA(GNSS_HandleTypeDef *hgnss, const char *sentence);
static int GNSS_ParseRMC(GNSS_HandleTypeDef *hgnss, const char *sentence);
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

  /* Initialize data structure */
  memset(&hgnss->data, 0, sizeof(GNSS_Data_t));
  hgnss->nmea_index = 0;
  hgnss->is_initialized = false;
  hgnss->is_powered = false;

  /* Configure power control pin as output */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = hgnss->pwr_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(hgnss->pwr_port, &GPIO_InitStruct);

  /* Ensure module is powered off initially */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_RESET);

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

  /* Power on the module */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_SET);
  hgnss->is_powered = true;

  /* Wait for module to stabilize */
  HAL_Delay(GNSS_POWER_ON_DELAY);

  /* Clear NMEA buffer */
  hgnss->nmea_index = 0;
  memset(hgnss->nmea_buffer, 0, sizeof(hgnss->nmea_buffer));

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

  /* Power off the module */
  HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_RESET);
  hgnss->is_powered = false;

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
  uint8_t rx_byte;
  HAL_StatusTypeDef uart_status;

  /* Clear previous data validity */
  hgnss->data.valid = false;

  /* Read UART data until we get a valid fix or timeout */
  while ((HAL_GetTick() - start_tick) < timeout)
  {
    /* Try to receive one byte */
    uart_status = HAL_UART_Receive(hgnss->huart, &rx_byte, 1, 100);

    if (uart_status == HAL_OK)
    {
      /* Process the received byte */
      GNSS_ProcessByte(hgnss, rx_byte);

      /* Check if we have a valid fix */
      if (hgnss->data.valid && hgnss->data.fix_quality != GNSS_FIX_INVALID)
      {
        return GNSS_OK;
      }
    }
  }

  return GNSS_TIMEOUT;
}

/**
  * @brief  Process received UART data byte by byte
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  data: Received byte
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_ProcessByte(GNSS_HandleTypeDef *hgnss, uint8_t data)
{
  if (hgnss == NULL)
  {
    return GNSS_ERROR;
  }

  /* Start of NMEA sentence */
  if (data == '$')
  {
    hgnss->nmea_index = 0;
    hgnss->nmea_buffer[hgnss->nmea_index++] = data;
  }
  /* End of NMEA sentence */
  else if (data == '\n' || data == '\r')
  {
    if (hgnss->nmea_index > 0)
    {
      hgnss->nmea_buffer[hgnss->nmea_index] = '\0';

      /* Parse the complete NMEA sentence */
      GNSS_ParseNMEA(hgnss, (const char *)hgnss->nmea_buffer);

      hgnss->nmea_index = 0;
    }
  }
  /* Middle of NMEA sentence */
  else if (hgnss->nmea_index > 0 && hgnss->nmea_index < sizeof(hgnss->nmea_buffer) - 1)
  {
    hgnss->nmea_buffer[hgnss->nmea_index++] = data;
  }

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

  /* Verify checksum if present */
  if (!GNSS_VerifyChecksum(sentence))
  {
    return GNSS_INVALID;
  }

  /* Parse GGA sentence (position and altitude) */
  if (strncmp(sentence, NMEA_GGA, strlen(NMEA_GGA)) == 0)
  {
    GNSS_ParseGGA(hgnss, sentence);
  }
  /* Parse RMC sentence (speed and course) */
  else if (strncmp(sentence, NMEA_RMC, strlen(NMEA_RMC)) == 0)
  {
    GNSS_ParseRMC(hgnss, sentence);
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
  * @brief  Parse GGA NMEA sentence
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  sentence: GGA sentence
  * @retval 0 on success, -1 on error
  */
static int GNSS_ParseGGA(GNSS_HandleTypeDef *hgnss, const char *sentence)
{
  char *token;
  char *saveptr;
  char buffer[128];
  int field = 0;
  float lat_raw = 0, lon_raw = 0;
  char lat_dir = 'N', lon_dir = 'E';

  /* Make a copy since strtok_r modifies the string */
  strncpy(buffer, sentence, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  /* Parse fields */
  token = strtok_r(buffer, ",", &saveptr);

  while (token != NULL && field < 15)
  {
    switch (field)
    {
      case 1: /* UTC time */
        if (strlen(token) >= 6)
        {
          hgnss->data.timestamp = atoi(token);
        }
        break;

      case 2: /* Latitude */
        if (strlen(token) > 0)
        {
          lat_raw = atof(token);
        }
        break;

      case 3: /* Latitude direction */
        if (strlen(token) > 0)
        {
          lat_dir = token[0];
        }
        break;

      case 4: /* Longitude */
        if (strlen(token) > 0)
        {
          lon_raw = atof(token);
        }
        break;

      case 5: /* Longitude direction */
        if (strlen(token) > 0)
        {
          lon_dir = token[0];
        }
        break;

      case 6: /* Fix quality */
        if (strlen(token) > 0)
        {
          hgnss->data.fix_quality = (GNSS_FixQuality_t)atoi(token);
        }
        break;

      case 7: /* Number of satellites */
        if (strlen(token) > 0)
        {
          hgnss->data.satellites = atoi(token);
        }
        break;

      case 8: /* HDOP */
        if (strlen(token) > 0)
        {
          hgnss->data.hdop = atof(token);
        }
        break;

      case 9: /* Altitude */
        if (strlen(token) > 0)
        {
          hgnss->data.altitude = atof(token);
        }
        break;
    }

    token = strtok_r(NULL, ",", &saveptr);
    field++;
  }

  /* Convert coordinates to decimal degrees */
  if (lat_raw > 0)
  {
    hgnss->data.latitude = GNSS_ConvertToDecimalDegrees(lat_raw);
    if (lat_dir == 'S')
    {
      hgnss->data.latitude = -hgnss->data.latitude;
    }
  }

  if (lon_raw > 0)
  {
    hgnss->data.longitude = GNSS_ConvertToDecimalDegrees(lon_raw);
    if (lon_dir == 'W')
    {
      hgnss->data.longitude = -hgnss->data.longitude;
    }
  }

  /* Mark data as valid if we have a fix */
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
  char *token;
  char *saveptr;
  char buffer[128];
  int field = 0;
  char status = 'V';

  /* Make a copy since strtok_r modifies the string */
  strncpy(buffer, sentence, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  /* Parse fields */
  token = strtok_r(buffer, ",", &saveptr);

  while (token != NULL && field < 13)
  {
    switch (field)
    {
      case 1: /* UTC time */
        if (strlen(token) >= 6)
        {
          hgnss->data.timestamp = atoi(token);
        }
        break;

      case 2: /* Status A=active, V=void */
        if (strlen(token) > 0)
        {
          status = token[0];
        }
        break;

      case 7: /* Speed over ground (knots) */
        if (strlen(token) > 0)
        {
          hgnss->data.speed = atof(token) * 1.852f; // Convert knots to km/h
        }
        break;

      case 8: /* Track angle (degrees) */
        if (strlen(token) > 0)
        {
          hgnss->data.course = atof(token);
        }
        break;

      case 9: /* Date */
        if (strlen(token) >= 6)
        {
          hgnss->data.date = atoi(token);
        }
        break;
    }

    token = strtok_r(NULL, ",", &saveptr);
    field++;
  }

  /* Update validity based on RMC status */
  if (status == 'A')
  {
    hgnss->data.valid = true;
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

  /* Extract checksum from sentence */
  uint8_t provided = 0;
  if (sscanf(checksum_ptr + 1, "%2hhx", &provided) != 1)
  {
    return false;
  }

  return (calculated == provided);
}
