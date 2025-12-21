/**
  ******************************************************************************
  * @file    flash_log.h
  * @brief   Power-Safe Flash Data Logging Header
  * @details High-level API for logging sensor data to external SPI flash
  *          with power-failure safety and LIFO retrieval support.
  *
  * Design Principles:
  *   1. Power-safe: Uses ping-pong headers and append-only writes
  *   2. Self-describing: Each record has magic number and CRC32
  *   3. LIFO retrieval: Most recent data first for downlink efficiency
  *   4. Expandable: 64-byte records with reserved space for future fields
  *
  * Memory Layout (2MB W25Q16JV):
  *   Sector 0 (4KB):    Header A + Header B (ping-pong for power safety)
  *   Sectors 1-511:     Data records (circular buffer, ~32,000 records)
  *
  * Power-Failure Recovery:
  *   - On init, read both headers, use valid one with higher sequence
  *   - Data is written first, header updated second
  *   - If power fails during data write: lost record, no corruption
  *   - If power fails during header write: old header still valid
  *
  ******************************************************************************
  */

#ifndef __FLASH_LOG_H
#define __FLASH_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "sys_sensors.h"
#include "w25q16jv.h"

/* Exported defines ----------------------------------------------------------*/

/** @brief Record magic number for validity check */
#define FLASH_LOG_RECORD_MAGIC    0xFEEDDA7A

/** @brief Header magic number */
#define FLASH_LOG_HEADER_MAGIC    0xF1A5DEAD

/** @brief Header version (increment if structure changes) */
#define FLASH_LOG_HEADER_VERSION  1

/** @brief Record size in bytes (must be power of 2 for efficiency) */
#define FLASH_LOG_RECORD_SIZE     64

/** @brief Data area start address (after header sector) */
#define FLASH_LOG_DATA_START      W25Q_SECTOR_SIZE  /* 0x1000 = 4KB */

/** @brief Data area end address */
#define FLASH_LOG_DATA_END        W25Q_FLASH_SIZE

/** @brief Maximum number of records that fit in flash */
#define FLASH_LOG_MAX_RECORDS     ((FLASH_LOG_DATA_END - FLASH_LOG_DATA_START) / FLASH_LOG_RECORD_SIZE)

/** @brief Records per sector */
#define FLASH_LOG_RECORDS_PER_SECTOR  (W25Q_SECTOR_SIZE / FLASH_LOG_RECORD_SIZE)

/* Exported types ------------------------------------------------------------*/

/**
  * @brief Flash log status codes
  */
typedef enum {
    FLASH_LOG_OK = 0,           /**< Operation successful */
    FLASH_LOG_ERROR,            /**< General error */
    FLASH_LOG_ERROR_INIT,       /**< Not initialized */
    FLASH_LOG_ERROR_FLASH,      /**< Flash driver error */
    FLASH_LOG_ERROR_FULL,       /**< Log is full (wrap disabled) */
    FLASH_LOG_ERROR_EMPTY,      /**< No records to read */
    FLASH_LOG_ERROR_CRC,        /**< CRC validation failed */
    FLASH_LOG_ERROR_PARAM       /**< Invalid parameter */
} FlashLog_StatusTypeDef;

/**
  * @brief Flash log record structure (64 bytes, power of 2 for alignment)
  * @note  All multi-byte fields are little-endian
  * @note  Structure is packed to ensure consistent size across compilers
  */
typedef struct __attribute__((packed)) {
    /* Header (8 bytes) */
    uint32_t magic;             /**< Record magic: 0xFEEDDA7A */
    uint32_t sequence;          /**< Monotonic sequence number (never wraps in practice) */
    
    /* Timestamp (4 bytes) */
    uint32_t timestamp;         /**< RTC timestamp (seconds since device start or epoch) */
    
    /* Environmental sensors (12 bytes) */
    float pressure;             /**< Barometric pressure in mbar */
    float temperature;          /**< Temperature in degC */
    float humidity;             /**< Relative humidity in % */
    
    /* GNSS data (18 bytes) */
    int32_t latitude;           /**< Latitude in binary format (scaled by 8388607/90) */
    int32_t longitude;          /**< Longitude in binary format (scaled by 8388607/180) */
    int16_t altitude_gps;       /**< GPS altitude in meters */
    int16_t altitude_bar;       /**< Barometric altitude in meters * 10 */
    uint8_t satellites;         /**< Number of satellites */
    uint8_t gnss_fix_quality;   /**< Fix quality (0=none, 1=GPS, 2=DGPS) */
    uint8_t gnss_hdop_x10;      /**< HDOP * 10 (0-255 = 0.0-25.5) */
    uint8_t gnss_valid;         /**< GNSS valid flag */
    uint8_t reserved1;          /**< Reserved for alignment */
    uint8_t reserved2;          /**< Reserved for alignment */
    
    /* Power and status (4 bytes) */
    uint16_t battery_mv;        /**< Battery voltage in millivolts */
    uint8_t flags;              /**< Status flags (reserved) */
    uint8_t reserved3;          /**< Reserved for future use */
    
    /* Reserved for expansion (14 bytes) */
    uint8_t reserved[14];       /**< Future expansion space */
    
    /* Integrity (4 bytes) */
    uint32_t crc32;             /**< CRC32 of all preceding bytes (60 bytes) */
    
} FlashLog_Record_t;

/* Compile-time size check */
_Static_assert(sizeof(FlashLog_Record_t) == FLASH_LOG_RECORD_SIZE, 
               "FlashLog_Record_t must be exactly 64 bytes");

/**
  * @brief Flash log header structure (stored in sector 0)
  * @note  Two copies stored for ping-pong power safety
  */
typedef struct __attribute__((packed)) {
    uint32_t magic;             /**< Header magic: 0xF1A5DEAD */
    uint32_t version;           /**< Header version */
    uint32_t write_addr;        /**< Next write address */
    uint32_t record_count;      /**< Total records written (may exceed MAX if wrapped) */
    uint32_t sequence;          /**< Header update sequence (for ping-pong selection) */
    uint32_t oldest_addr;       /**< Address of oldest valid record */
    uint32_t flags;             /**< Status flags */
    uint32_t reserved[3];       /**< Reserved for future use */
    uint32_t crc32;             /**< CRC32 of preceding bytes */
} FlashLog_Header_t;

/**
  * @brief Flash log handle structure
  */
typedef struct {
    W25Q_HandleTypeDef *hw25q;  /**< Pointer to W25Q flash handle */
    bool initialized;           /**< Initialization flag */
    uint32_t write_addr;        /**< Cached next write address */
    uint32_t oldest_addr;       /**< Cached oldest record address */
    uint32_t record_count;      /**< Cached total record count */
    uint32_t next_sequence;     /**< Next record sequence number */
    uint8_t active_header;      /**< Active header slot (0 or 1) */
} FlashLog_HandleTypeDef;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize the flash logging system
  * @param  hlog: Pointer to flash log handle
  * @param  hw25q: Pointer to initialized W25Q flash handle
  * @retval FlashLog_StatusTypeDef
  * @note   Recovers state from flash headers if present
  */
FlashLog_StatusTypeDef FlashLog_Init(FlashLog_HandleTypeDef *hlog, W25Q_HandleTypeDef *hw25q);

/**
  * @brief  De-initialize the flash logging system
  * @param  hlog: Pointer to flash log handle
  * @retval FlashLog_StatusTypeDef
  */
FlashLog_StatusTypeDef FlashLog_DeInit(FlashLog_HandleTypeDef *hlog);

/**
  * @brief  Write a sensor data record to flash
  * @param  hlog: Pointer to flash log handle
  * @param  sensor_data: Pointer to sensor data structure
  * @param  timestamp: RTC timestamp for this record
  * @retval FlashLog_StatusTypeDef
  * @note   Automatically handles sector wraparound
  */
FlashLog_StatusTypeDef FlashLog_WriteRecord(FlashLog_HandleTypeDef *hlog, 
                                            const sensor_t *sensor_data,
                                            uint32_t timestamp);

/**
  * @brief  Read the most recent record from flash (LIFO)
  * @param  hlog: Pointer to flash log handle
  * @param  record: Pointer to store the record
  * @param  offset: Number of records back from most recent (0 = newest)
  * @retval FlashLog_StatusTypeDef
  */
FlashLog_StatusTypeDef FlashLog_ReadRecord(FlashLog_HandleTypeDef *hlog,
                                           FlashLog_Record_t *record,
                                           uint32_t offset);

/**
  * @brief  Read multiple records starting from most recent (LIFO batch read)
  * @param  hlog: Pointer to flash log handle
  * @param  records: Array to store records
  * @param  max_count: Maximum records to read
  * @param  actual_count: Pointer to store actual number read
  * @param  start_offset: Start offset from most recent (0 = newest)
  * @retval FlashLog_StatusTypeDef
  */
FlashLog_StatusTypeDef FlashLog_ReadRecords(FlashLog_HandleTypeDef *hlog,
                                            FlashLog_Record_t *records,
                                            uint32_t max_count,
                                            uint32_t *actual_count,
                                            uint32_t start_offset);

/**
  * @brief  Get the total number of valid records in flash
  * @param  hlog: Pointer to flash log handle
  * @retval Number of records (0 if empty or error)
  */
uint32_t FlashLog_GetRecordCount(FlashLog_HandleTypeDef *hlog);

/**
  * @brief  Get the number of available records (considering wrap)
  * @param  hlog: Pointer to flash log handle
  * @retval Number of available records
  */
uint32_t FlashLog_GetAvailableRecords(FlashLog_HandleTypeDef *hlog);

/**
  * @brief  Check if the log has wrapped around
  * @param  hlog: Pointer to flash log handle
  * @retval true if wrapped, false otherwise
  */
bool FlashLog_HasWrapped(FlashLog_HandleTypeDef *hlog);

/**
  * @brief  Erase all log data (factory reset)
  * @param  hlog: Pointer to flash log handle
  * @retval FlashLog_StatusTypeDef
  * @warning This erases ALL logged data and cannot be undone!
  */
FlashLog_StatusTypeDef FlashLog_EraseAll(FlashLog_HandleTypeDef *hlog);

/**
  * @brief  Sync header to flash (force header update)
  * @param  hlog: Pointer to flash log handle
  * @retval FlashLog_StatusTypeDef
  * @note   Normally called automatically, but can force sync if needed
  */
FlashLog_StatusTypeDef FlashLog_SyncHeader(FlashLog_HandleTypeDef *hlog);

/**
  * @brief  Verify integrity of a specific record
  * @param  record: Pointer to record to verify
  * @retval true if valid, false if corrupt
  */
bool FlashLog_VerifyRecord(const FlashLog_Record_t *record);

/**
  * @brief  Get statistics about flash usage
  * @param  hlog: Pointer to flash log handle
  * @param  total_capacity: Pointer to store max records capacity
  * @param  used_records: Pointer to store used record count
  * @param  free_records: Pointer to store free record count
  * @retval FlashLog_StatusTypeDef
  */
FlashLog_StatusTypeDef FlashLog_GetStats(FlashLog_HandleTypeDef *hlog,
                                         uint32_t *total_capacity,
                                         uint32_t *used_records,
                                         uint32_t *free_records);

/* CRC32 utility -------------------------------------------------------------*/

/**
  * @brief  Calculate CRC32 for data integrity
  * @param  data: Pointer to data
  * @param  len: Length in bytes
  * @retval CRC32 value
  */
uint32_t FlashLog_CRC32(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_LOG_H */
