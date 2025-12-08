/**
  ******************************************************************************
  * @file    atgm336h.h
  * @brief   ATGM336H-5N31 GNSS Module Driver Header
  ******************************************************************************
  * @attention
  *
  * This driver provides interface to the ATGM336H-5N31 GNSS module
  * Features:
  * - UART communication
  * - NMEA sentence parsing (GGA, RMC)
  * - Power control
  * - High altitude mode configuration
  *
  ******************************************************************************
  */

#ifndef __ATGM336H_H
#define __ATGM336H_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief GNSS Status enumeration
  */
typedef enum
{
  GNSS_OK       = 0,
  GNSS_ERROR    = 1,
  GNSS_TIMEOUT  = 2,
  GNSS_NO_FIX   = 3,
  GNSS_INVALID  = 4
} GNSS_StatusTypeDef;

/**
  * @brief GNSS Fix Quality enumeration
  */
typedef enum
{
  GNSS_FIX_INVALID = 0,
  GNSS_FIX_GPS     = 1,
  GNSS_FIX_DGPS    = 2
} GNSS_FixQuality_t;

/**
  * @brief GNSS Data Structure
  */
typedef struct
{
  float latitude;           // Latitude in decimal degrees (positive = North)
  float longitude;          // Longitude in decimal degrees (positive = East)
  float altitude;           // Altitude in meters above sea level
  float speed;              // Ground speed in km/h
  float course;             // Course in degrees from true north
  uint8_t satellites;       // Number of satellites used for fix
  float hdop;               // Horizontal dilution of precision
  GNSS_FixQuality_t fix_quality;  // Fix quality
  uint32_t timestamp;       // UTC timestamp (HHMMSS format)
  uint32_t date;            // UTC date (DDMMYY format)
  bool valid;               // Flag indicating if data is valid
} GNSS_Data_t;

/**
  * @brief GNSS Handle Structure
  */
typedef struct
{
  UART_HandleTypeDef *huart;          // UART handle
  GPIO_TypeDef *pwr_port;              // Power control GPIO port
  uint16_t pwr_pin;                    // Power control GPIO pin
  bool is_powered;                     // Power state flag
  bool is_initialized;                 // Initialization flag
  uint8_t rx_buffer[256];              // RX buffer for UART DMA
  uint8_t nmea_buffer[128];            // Buffer for NMEA sentence parsing
  uint8_t nmea_index;                  // Current index in NMEA buffer
  GNSS_Data_t data;                    // Latest GNSS data
} GNSS_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/
#define GNSS_UART_BAUDRATE          9600
#define GNSS_UART_TIMEOUT           1000
#define GNSS_POWER_ON_DELAY         1000  // ms
#define GNSS_FIX_TIMEOUT            60000 // ms
#define GNSS_MAX_RETRIES            3

/* NMEA Sentence identifiers */
#define NMEA_GGA                    "$GPGGA"
#define NMEA_RMC                    "$GPRMC"
#define NMEA_GSA                    "$GPGSA"
#define NMEA_GSV                    "$GPGSV"
#define NMEA_VTG                    "$GPVTG"
#define NMEA_GLL                    "$GPGLL"

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_Init(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Power on GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_PowerOn(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Power off GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_PowerOff(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Get position data from GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  timeout: Timeout in milliseconds
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_GetPosition(GNSS_HandleTypeDef *hgnss, uint32_t timeout);

/**
  * @brief  Parse NMEA sentence
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  sentence: NMEA sentence string
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_ParseNMEA(GNSS_HandleTypeDef *hgnss, const char *sentence);

/**
  * @brief  Check if fix is valid
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval true if valid fix, false otherwise
  */
bool GNSS_IsFixValid(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Process received UART data
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  data: Received byte
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_ProcessByte(GNSS_HandleTypeDef *hgnss, uint8_t data);

/**
  * @brief  Send command to GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  cmd: Command string
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_SendCommand(GNSS_HandleTypeDef *hgnss, const char *cmd);

/**
  * @brief  Calculate NMEA checksum
  * @param  sentence: NMEA sentence (without $ and checksum)
  * @retval Calculated checksum
  */
uint8_t GNSS_CalculateChecksum(const char *sentence);

#ifdef __cplusplus
}
#endif

#endif /* __ATGM336H_H */
