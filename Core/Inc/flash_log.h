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
 *   3. One-pass recovery (R3-04, #218 / DDR-0005 BR-TX-008..011): newest
 *      records are sent FIRST; the recovery watermark advances AT SEND TIME
 *      (never on ACK); the walker never autonomously resends a record.
 *      Backend owns gap repair (BR-TX-012). Supersedes the legacy FIFO drain
 *      / commit-on-ACK watermark (C4/F15);
 *      see the v5 migration note at FLASH_LOG_HEADER_VERSION.
 *   4. Expandable: 64-byte records with reserved space for future fields
 *
 * Memory Layout (2MB W25Q16JV), T4 / FlashStorageNotes.md:
 *   Sector 0 (4KB):    Header A (ping-pong copy 1)
 *   Sector 1 (4KB):    Header B (ping-pong copy 2 — separate sector so one
 *                      erase can never kill both copies)
 *   Sectors 2-511:     Data records (circular buffer, ~32,000 records)
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
#include "sys_sensors.h"
#include "w25q16jv.h"
#include <stdbool.h>
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

/** @brief Record magic number for validity check */
#define FLASH_LOG_RECORD_MAGIC 0xFEEDDA7A

/** @brief Header magic number */
#define FLASH_LOG_HEADER_MAGIC 0xF1A5DEAD

/** @brief Header version (increment if structure changes)
 *  v3 (T4): headers moved to separate sectors 0/1, data starts at sector 2.
 *  v4 (D5/#35): record layout — altitude_gps int32, altitude_bar deleted,
 *  solar_mv/voltage_slope/power_mode added. Old v3 headers fail validation
 *  -> clean init (acceptable pre-launch).
 *  v5 (R3-04/#218, DDR-0005 one-pass recovery): SEMANTICS of two existing
 *  fields change — last_transmitted_seq becomes tx_high_water (highest seq
 *  ever sent, monotonic up) and reserved[0] becomes recovery_frontier
 *  (walker visited every seq >= frontier, monotonic down). Same struct
 *  layout/size; v4 headers are ACCEPTED and migrated (frontier =
 *  next_sequence: the retained archive is re-walked once, newest-first;
 *  the backend dedupes by sequence). */
#define FLASH_LOG_HEADER_VERSION 5
#define FLASH_LOG_HEADER_VERSION_LEGACY_FIFO 4

/** @brief Record size in bytes (must be power of 2 for efficiency) */
#define FLASH_LOG_RECORD_SIZE 64

/** @brief Data area start address (after BOTH header sectors — T4: header A
 *  in sector 0, header B in sector 1, records from sector 2 onward) */
#define FLASH_LOG_DATA_START (2 * W25Q_SECTOR_SIZE) /* 0x2000 = 8KB */

/** @brief Data area end address */
#define FLASH_LOG_DATA_END W25Q_FLASH_SIZE

/** @brief Maximum number of records that fit in flash */
#define FLASH_LOG_MAX_RECORDS ((FLASH_LOG_DATA_END - FLASH_LOG_DATA_START) / FLASH_LOG_RECORD_SIZE)

/** @brief Records per sector */
#define FLASH_LOG_RECORDS_PER_SECTOR (W25Q_SECTOR_SIZE / FLASH_LOG_RECORD_SIZE)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Flash log status codes
 */
typedef enum {
  FLASH_LOG_OK = 0,      /**< Operation successful */
  FLASH_LOG_ERROR,       /**< General error */
  FLASH_LOG_ERROR_INIT,  /**< Not initialized */
  FLASH_LOG_ERROR_FLASH, /**< Flash driver error */
  FLASH_LOG_ERROR_FULL,  /**< Log is full (wrap disabled) */
  FLASH_LOG_ERROR_EMPTY, /**< No records to read */
  FLASH_LOG_ERROR_CRC,   /**< CRC validation failed */
  FLASH_LOG_ERROR_PARAM  /**< Invalid parameter */
} FlashLog_StatusTypeDef;

/**
 * @brief Flash log record structure (64 bytes, power of 2 for alignment)
 * @note  All multi-byte fields are little-endian
 * @note  Structure is packed to ensure consistent size across compilers
 */
typedef struct __attribute__((packed)) {
  /* Header (8 bytes) */
  uint32_t magic;    /**< Record magic: 0xFEEDDA7A */
  uint32_t sequence; /**< Monotonic sequence number (never wraps in practice) */

  /* Timestamp (4 bytes) */
  uint32_t timestamp; /**< UTC epoch seconds (R45: SysTime, GPS-disciplined); boot-relative only before first fix */

  /* Environmental sensors (12 bytes) */
  float pressure;    /**< Barometric pressure in mbar */
  float temperature; /**< Temperature in degC */
  float humidity;    /**< Relative humidity in % */

  /* GNSS data (16 bytes) */
  int32_t latitude;     /**< Latitude in binary format (scaled by 8388607/90) */
  int32_t longitude;    /**< Longitude in binary format (scaled by 8388607/180) */
  int32_t altitude_gps; /**< GPS altitude in meters (D5/#35: int32; >32767 m float altitudes fit) */
  /* altitude_bar DELETED (D5/#35): never assigned; ground computes it */
  uint8_t satellites;       /**< Number of satellites */
  uint8_t gnss_fix_quality; /**< Fix quality (0=none, 1=GPS, 2=DGPS) */
  uint8_t gnss_hdop_x10;    /**< HDOP * 10 (0-255 = 0.0-25.5) */
  uint8_t gnss_valid;       /**< GNSS valid flag */

  /* Power and status (8 bytes) */
  uint16_t battery_mv;   /**< Battery voltage in millivolts */
  uint16_t solar_mv;     /**< Solar panel voltage in millivolts (D5/F-025: was never archived) */
  int16_t voltage_slope; /**< Battery slope mV/hour at write time (D5: honest history) */
  uint8_t power_mode;    /**< Operating mode enum at write time (D5) */
  uint8_t flags;         /**< Data-honesty flags (FW-7): b0 press_stale, b1 temp_stale, b2 hum_stale, b3 gnss_stale, b4 batt_stale (#136), b5-b7 TransmitVeto_t (2026-08-11 §6a: record WHY, DDR-0003) */

  /* Reserved for expansion (12 bytes) */
  uint8_t reserved[12]; /**< Future expansion space */

  /* Integrity (4 bytes) */
  uint32_t crc32; /**< CRC32 of all preceding bytes (60 bytes) */

} FlashLog_Record_t;

/* Compile-time size check */
_Static_assert(sizeof(FlashLog_Record_t) == FLASH_LOG_RECORD_SIZE,
               "FlashLog_Record_t must be exactly 64 bytes");

/**
 * @brief Flash log header structure (copies in sectors 0 and 1 — T4)
 * @note  Two copies in SEPARATE sectors for ping-pong power safety
 */
typedef struct __attribute__((packed)) {
  uint32_t magic;                /**< Header magic: 0xF1A5DEAD */
  uint32_t version;              /**< Header version */
  uint32_t write_addr;           /**< Next write address */
  uint32_t record_count;         /**< Total records written (may exceed MAX if wrapped) */
  uint32_t sequence;             /**< Header update sequence (for ping-pong selection) */
  uint32_t oldest_addr;          /**< Address of oldest valid record */
  uint32_t flags;                /**< Status flags */
  uint32_t last_transmitted_seq; /**< v5: TX HIGH WATER - highest sequence ever
                                      handed to the radio (monotonic up).
                                      (v4: rising FIFO commit watermark) */
  uint32_t reserved[2];          /**< reserved[0] v5: RECOVERY FRONTIER - the
                                         one-pass walker has visited every
                                         sequence >= frontier (monotonic
                                         down); reserved[1] free */
  uint32_t crc32;                /**< CRC32 of preceding bytes */
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
  uint32_t tx_high_water;     /**< v5: highest sequence ever handed to the radio (monotonic up) */
  uint32_t recovery_frontier; /**< v5: one-pass walker watermark — every seq >= frontier visited (monotonic down) */
  uint32_t header_generation; /**< F-007/R12 (#50): monotonic header generation, incremented every write */
  uint8_t active_header;      /**< Active header slot (0 or 1) */
  uint8_t sync_deferred;      /**< Finding #8: MarkRecoverySent skips SyncHeader while set (bulk burst) */
  uint8_t header_dirty;       /**< Finding #8: watermark advanced past the last persisted header */
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
                                            uint32_t timestamp,
                                            int16_t voltage_slope,
                                            uint8_t power_mode,
                                            uint8_t veto);

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

/* FlashLog_EraseAll DELETED (D11, #52): grep-verified uncalled, and dangerous
 * by construction — 512 sector erases with no IWDG service and no watermark
 * reset. Field recovery uses the debugger; no brick functions in the tree. */

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

/**
 * @brief  Check if there are unsent records available for bulk transmission
 * @param  hlog: Pointer to flash log handle
 * @retval true if unsent records exist, false otherwise
 */
bool FlashLog_HasUnsentData(FlashLog_HandleTypeDef *hlog);

/**
 * @brief  R3-04 (#218, DDR-0005 BR-TX-008): get records for one-pass
 *         archive recovery, NEWEST FIRST. The sendable set is the union of
 *         pending-live records (seq > tx_high_water, not yet sent by any
 *         path) and walker-eligible records (oldest_seq <= seq <
 *         recovery_frontier). Corrupt/torn records are skipped and counted;
 *         a contiguous corrupt run at the leading edge is retired
 *         immediately (never wedge, T4/RV-01 discipline).
 * @param  hlog: Pointer to flash log handle
 * @param  records: Array to store records (descending sequence order)
 * @param  max_count: Maximum records to read
 * @param  actual_count: Pointer to store actual number read
 * @param  skipped_count: Pointer to store corrupt-skip count (may be NULL)
 * @retval FlashLog_StatusTypeDef (FLASH_LOG_ERROR_EMPTY when nothing sendable)
 */
FlashLog_StatusTypeDef FlashLog_GetRecoveryRecords(FlashLog_HandleTypeDef *hlog,
                                                   FlashLog_Record_t *records,
                                                   uint32_t max_count,
                                                   uint32_t *actual_count,
                                                   uint32_t *skipped_count);

/**
 * @brief  R3-04 (#218, DDR-0005 BR-TX-009/010): advance the one-pass
 *         watermark AT SEND TIME for one record. seq > tx_high_water raises
 *         the high water; seq < recovery_frontier lowers the frontier. The
 *         walker never revisits a record autonomously; there is no ACK
 *         retry (BR-TX-011). Respects DeferHeaderSync batching (#8).
 * @param  hlog: Pointer to flash log handle
 * @param  sequence: the sequence just handed to the radio
 * @retval FlashLog_StatusTypeDef
 */
FlashLog_StatusTypeDef FlashLog_MarkRecoverySent(FlashLog_HandleTypeDef *hlog, uint32_t sequence);

/**
 * @brief  Defer header persistence (finding #8, 2026-08-10): while deferred,
 *         FlashLog_MarkRecoverySent advances the RAM watermarks only — no
 *         sector erase per sent packet. Call FlashLog_FlushHeaderSync() once
 *         at burst end. A mid-burst reset replays from the last synced
 *         watermarks (backend dedupes by sequence) — conservative direction
 *         only.
 */
void FlashLog_DeferHeaderSync(FlashLog_HandleTypeDef *hlog);

/**
 * @brief  End deferral and persist the header if the watermark moved.
 *         No-op when nothing is dirty. A failed flush stays dirty so the
 *         next call retries.
 */
FlashLog_StatusTypeDef FlashLog_FlushHeaderSync(FlashLog_HandleTypeDef *hlog);

/**
 * @brief  Get count of unsent records
 * @param  hlog: Pointer to flash log handle
 * @retval Number of unsent records
 */
uint32_t FlashLog_GetUnsentCount(FlashLog_HandleTypeDef *hlog);

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
