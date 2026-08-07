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

/* Exported constants --------------------------------------------------------*/
#define GNSS_UART_BAUDRATE          9600
#define GNSS_UART_TIMEOUT           1000
#define GNSS_POWER_ON_DELAY         1000  // ms
#define GNSS_FIX_TIMEOUT            60000 // ms
#define GNSS_MAX_RETRIES            3

/* DMA Circular Buffer Configuration */
#define GNSS_DMA_BUFFER_SIZE        512   // Circular buffer for DMA reception
#define GNSS_NMEA_MAX_LENGTH        128   // Maximum NMEA sentence length

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
  * @brief Maximum satellites per constellation
  */
#define GNSS_MAX_SATS_PER_CONSTELLATION 20

/**
  * @brief Individual satellite information
  */
typedef struct
{
  uint8_t prn;           // Satellite PRN/ID number
  uint8_t elevation;     // Elevation angle (degrees, 0-90)
  uint16_t azimuth;      // Azimuth angle (degrees, 0-359)
  uint8_t snr;           // Signal-to-noise ratio (dBHz)
} GNSS_SatelliteInfo_t;

/**
  * @brief Extended GNSS data for detailed satellite tracking and 3D speed
  */
typedef struct
{
  // Per-constellation satellite lists
  GNSS_SatelliteInfo_t gps_sats[GNSS_MAX_SATS_PER_CONSTELLATION];
  uint8_t gps_count;
  
  GNSS_SatelliteInfo_t glonass_sats[GNSS_MAX_SATS_PER_CONSTELLATION];
  uint8_t glonass_count;
  
  GNSS_SatelliteInfo_t beidou_sats[GNSS_MAX_SATS_PER_CONSTELLATION];
  uint8_t beidou_count;
  
  // Speed data
  float ground_speed_kmh;    // From VTG sentence (more accurate than RMC)
  float vertical_speed_ms;   // Calculated from altitude changes (m/s)
  float speed_3d_kmh;        // Computed 3D speed
  float track_true;          // Course over ground (true north, degrees)
  float track_magnetic;      // Course over ground (magnetic, degrees)
  
  // For vertical speed calculation
  float prev_altitude;       // Previous altitude reading (meters)
  uint32_t prev_timestamp;   // Previous timestamp (HAL_GetTick())
  bool has_prev_altitude;    // Flag to indicate if we have previous data
  
} GNSS_ExtendedData_t;

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
  uint8_t satellites_in_view; // Number of satellites in view (from GSV)
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
  GPIO_TypeDef *pwr_port;              // Power control GPIO port (PB10)
  uint16_t pwr_pin;                    // Power control GPIO pin (active low)
  GPIO_TypeDef *en_port;               // Enable GPIO port (PB5)
  uint16_t en_pin;                     // Enable GPIO pin
  bool is_powered;                     // Power state flag
  bool is_initialized;                 // Initialization flag
  
  /* DMA Circular Buffer Management */
  uint8_t dma_buffer[GNSS_DMA_BUFFER_SIZE]; // DMA circular buffer
  volatile uint16_t dma_head;          // DMA write position (updated by hardware)
  uint16_t dma_tail;                   // Application read position
  volatile bool dma_data_ready;        // Flag set by DMA interrupts
  /* F-011 (#25): absolute producer/consumer counters. Producer advances
   * SIZE/2 per half/full DMA callback; consumer advances per byte read in
   * GNSS_ProcessDMABuffer. producer - consumer > SIZE = DMA lapped the
   * consumer = overrun (counted, resynced, surfaced). */
  volatile uint32_t dma_produced_total;   // Bytes produced by DMA (256-granular)
  uint32_t dma_consumed_total;            // Bytes consumed by the parser
  uint32_t dma_overrun_count;             // Cumulative overrun events (health metric)
  
  /* NMEA Sentence Processing */
  char nmea_sentence[GNSS_NMEA_MAX_LENGTH];  // Current NMEA sentence being built
  uint16_t nmea_length;                // Current sentence length
  
  GNSS_Data_t data;                    // Latest GNSS data
  GNSS_ExtendedData_t extended;        // Extended satellite tracking and 3D speed data
} GNSS_HandleTypeDef;

/* NMEA Sentence identifiers */
#define NMEA_GGA                    "$GPGGA"
#define NMEA_RMC                    "$GPRMC"
#define NMEA_GSA                    "$GPGSA"
#define NMEA_GSV                    "$GPGSV"
#define NMEA_VTG                    "$GPVTG"
#define NMEA_GLL                    "$GPGLL"

/* ATGM336H PCAS Configuration Command BODIES (no '$', no '*CS\r\n').
 * R23 (#22): sentences are built at runtime by GNSS_SendCommandBody() —
 * snprintf("$%s*%02X\r\n", body, GNSS_CalculateChecksum(body)) — so a
 * hand-computed checksum can never silently drift from the body again.
 * (History: three baked checksums were wrong and silently rejected by the
 * module: PCAS02, PCAS04,7, PCAS12. R24: PCAS11,2 pedestrian DELETED —
 * PCAS11 is single-setting last-write-wins; ,2 re-enabled the 18km CoCom
 * limit.) Expected checksums are pinned by the host test suite
 * (tests/host/test_main.c) as the pre-commit guard. */
#define GNSS_CMD_BODY_NMEA_CONFIG       "PCAS03,1,0,0,0,1,1,0,0"  // R26 flight mask: GGA+RMC+VTG, GSV OFF (*03)
#define GNSS_CMD_BODY_NMEA_CONFIG_DEBUG "PCAS03,1,0,0,1,1,1,0,0"  // Commissioning/bench variant, GSV on (*02)
#define GNSS_CMD_BODY_CONSTELLATION     "PCAS04,5"                // GPS+GLONASS (*1C)
#define GNSS_CMD_BODY_AIRBORNE_MODE     "PCAS11,5"                // Airborne dynamic model — defeats 18km CoCom limit (*18)
#define GNSS_CMD_BODY_UPDATE_RATE       "PCAS02,1000"             // 1 Hz (*2E)
#define GNSS_CMD_BODY_SATELLITE_SYS     "PCAS04,7"                // GPS+BeiDou+GLONASS (*1E)
#define GNSS_CMD_BODY_SAVE_CONFIG       "PCAS00"                  // Save configuration to flash (*01)
#define GNSS_CMD_BODY_STANDBY           "PCAS12,0"                // Standby ~15µA, permanent (*1E)

#define GNSS_WAKE_CHAR           "a"                               // Any char wakes from standby

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
  * @brief  Power off GNSS module (sends standby command)
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_PowerOff(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Enter GPS standby mode (low power ~15µA)
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_EnterStandby(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Wake GPS from standby mode
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_WakeFromStandby(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Configure GNSS module with initialization commands
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_Configure(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Get position data from GNSS module
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  timeout: Timeout in milliseconds
  * @retval GNSS status
  */
/* F-023/D12 (#59): GNSS_GetPosition deleted — dead blocking API. */

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
  * @brief  Check if fix meets quality thresholds for production use
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval true if fix is good quality, false otherwise
  */
bool GNSS_IsFixGoodQuality(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Validate GPS coordinates are within valid ranges
  * @param  lat: Latitude in decimal degrees
  * @param  lon: Longitude in decimal degrees
  * @retval true if coordinates are valid, false otherwise
  */
bool GNSS_ValidateCoordinates(float lat, float lon);

/**
  * @brief  Process DMA circular buffer data (called from main loop)
  * @param  hgnss: Pointer to GNSS handle structure
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_ProcessDMABuffer(GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Process received UART data (legacy byte-by-byte processing)
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
  * @brief  Send a PCAS command given its BODY only — the '$', checksum and
  *         CRLF are computed at runtime (R23, #22). Baked-checksum strings
  *         can drift from their bodies; computed ones cannot.
  * @param  hgnss: Pointer to GNSS handle structure
  * @param  body: Command body, e.g. "PCAS03,1,0,0,0,1,1,0,0"
  * @retval GNSS status
  */
GNSS_StatusTypeDef GNSS_SendCommandBody(GNSS_HandleTypeDef *hgnss, const char *body);

/**
  * @brief  F-011 (#25): cumulative DMA overrun count (health metric).
  *         Nonzero means the NMEA producer lapped the consumer at least once.
  */
uint32_t GNSS_GetDmaOverrunCount(const GNSS_HandleTypeDef *hgnss);

/**
  * @brief  Calculate NMEA checksum
  * @param  sentence: NMEA sentence (without $ and checksum)
  * @retval Calculated checksum
  */
uint8_t GNSS_CalculateChecksum(const char *sentence);

/**
  * @brief  DMA callbacks for external ISR handling (called by usart_if.c)
  */
void GNSS_DMA_RxHalfCallback(UART_HandleTypeDef *huart);
void GNSS_DMA_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __ATGM336H_H */
