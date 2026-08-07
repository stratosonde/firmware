/**
  ******************************************************************************
  * @file    flash_log.c
  * @brief   Power-Safe Flash Data Logging Implementation
  * @details Production-grade logging system with power-failure recovery
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_log.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
/* T4 FIX (DDR-0004): the two ping-pong headers MUST live in DIFFERENT sectors.
 * Previously A=0x0000 and B=0x0100 were both inside sector 0 — one sector
 * erase killed both copies, and every header rewrite after the first two was
 * a non-erased rewrite (NOR flash cannot flip 0->1 without erase), so the
 * ping-pong was fake. Now: sector 0 = header A, sector 1 = header B,
 * data starts at sector 2 (FLASH_LOG_DATA_START in flash_log.h). */
#define HEADER_A_ADDR     0x0000             /* Sector 0 */
#define HEADER_B_ADDR     W25Q_SECTOR_SIZE   /* Sector 1 */
#define HEADER_UPDATE_INTERVAL  10  /* Update header every N records */

/* CRC32 polynomial (IEEE 802.3) */
#define CRC32_POLYNOMIAL  0xEDB88320

/* Private variables ---------------------------------------------------------*/
static uint32_t crc32_table[256];
static bool crc32_table_initialized = false;

/* Private function prototypes -----------------------------------------------*/
static void FlashLog_InitCRC32Table(void);
static FlashLog_StatusTypeDef FlashLog_ReadHeader(FlashLog_HandleTypeDef *hlog, uint32_t addr, FlashLog_Header_t *header);
static FlashLog_StatusTypeDef FlashLog_WriteHeader(FlashLog_HandleTypeDef *hlog);
static bool FlashLog_ValidateHeader(const FlashLog_Header_t *header);
static uint32_t FlashLog_GetRecordAddress(FlashLog_HandleTypeDef *hlog, uint32_t record_index);
static FlashLog_StatusTypeDef FlashLog_EraseSectorIfNeeded(FlashLog_HandleTypeDef *hlog, uint32_t addr);
static FlashLog_StatusTypeDef FlashLog_FrontierScan(FlashLog_HandleTypeDef *hlog);

/* CRC32 Implementation ------------------------------------------------------*/

/**
  * @brief  Initialize CRC32 lookup table (called once)
  */
static void FlashLog_InitCRC32Table(void)
{
    uint32_t i, j, crc;
    
    if (crc32_table_initialized) {
        return;
    }
    
    for (i = 0; i < 256; i++) {
        crc = i;
        for (j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
        crc32_table[i] = crc;
    }
    
    crc32_table_initialized = true;
}

uint32_t FlashLog_CRC32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t i;
    
    FlashLog_InitCRC32Table();
    
    for (i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc32_table[(crc ^ data[i]) & 0xFF];
    }
    
    return ~crc;
}

/* Private Functions ---------------------------------------------------------*/

/**
  * @brief  Read header from flash and validate
  */
static FlashLog_StatusTypeDef FlashLog_ReadHeader(FlashLog_HandleTypeDef *hlog, 
                                                   uint32_t addr, 
                                                   FlashLog_Header_t *header)
{
    W25Q_StatusTypeDef status;
    
    status = W25Q_Read(hlog->hw25q, addr, (uint8_t *)header, sizeof(FlashLog_Header_t));
    if (status != W25Q_OK) {
        return FLASH_LOG_ERROR_FLASH;
    }
    
    return FLASH_LOG_OK;
}

bool FlashLog_HasUnsentData(FlashLog_HandleTypeDef *hlog)
{
    if (hlog == NULL || !hlog->initialized) {
        return false;
    }
    
    return (hlog->next_sequence > hlog->last_transmitted_sequence);
}

FlashLog_StatusTypeDef FlashLog_GetUnsentRecordsFIFO(FlashLog_HandleTypeDef *hlog,
                                                     FlashLog_Record_t *records,
                                                     uint32_t max_count,
                                                     uint32_t *actual_count,
                                                     uint32_t *skipped_count)
{
    FlashLog_StatusTypeDef status;
    uint32_t unsent_count, i;
    uint32_t sequence_to_read;

    /* R13 (#51): bound the scan — a long corrupt run must not spin thousands
     * of SPI reads in one call. The watermark persists, so the next call
     * resumes exactly where this one stopped. */
    #define FLASH_LOG_MAX_PROBES_PER_CALL  256U
    uint32_t probes = 0;

    if (hlog == NULL || !hlog->initialized || records == NULL || actual_count == NULL) {
        return FLASH_LOG_ERROR_PARAM;
    }

    *actual_count = 0;
    if (skipped_count != NULL) {
        *skipped_count = 0;
    }
    
    /* BUG 1.7 FIX: Clamp watermark after ring-buffer wraparound.
     * If unsent backlog exceeds capacity (e.g. long ocean gap), the oldest-unsent
     * sequence maps to an overwritten record. Skip to the oldest record that still
     * exists in flash to prevent permanent bulk-transfer wedge. */
    uint32_t available = FlashLog_GetAvailableRecords(hlog);
    if (hlog->next_sequence > available &&
        hlog->last_transmitted_sequence < (hlog->next_sequence - available)) {
        hlog->last_transmitted_sequence = hlog->next_sequence - available;
    }
    
    /* Calculate how many unsent records we have */
    unsent_count = FlashLog_GetUnsentCount(hlog);
    
    if (unsent_count == 0) {
        return FLASH_LOG_ERROR_EMPTY;
    }
    
    /* Limit to requested count */
    if (max_count > unsent_count) {
        max_count = unsent_count;
    }
    
    /* C4 FIX: Read FIFO (oldest unsent first) so MarkRecordsTransmitted
     * correctly advances last_transmitted_sequence from the old end.
     *
     * T4 FIX: torn-write tolerance. A power loss mid-record leaves garbage that
     * fails CRC; previously ANY read error aborted the batch and wedged bulk
     * transfer forever. Now: skip the corrupt record by advancing the
     * last_transmitted_sequence watermark past it (it is unrecoverable), and
     * keep packing the remaining good records. The caller marks only the
     * records actually sent, so the watermark lands exactly past every
     * consumed sequence (good + skipped). */
    sequence_to_read = hlog->last_transmitted_sequence;
    while ((*actual_count) < max_count &&
           sequence_to_read < hlog->next_sequence &&
           probes < FLASH_LOG_MAX_PROBES_PER_CALL) {
        /* Convert sequence to ReadRecord offset (newest-relative) */
        uint32_t offset = (hlog->next_sequence - 1) - sequence_to_read;
        probes++;

        status = FlashLog_ReadRecord(hlog, &records[*actual_count], offset);
        if (status != FLASH_LOG_OK) {
            /* F-006/R13 (#51): corrupt/torn record — count it, but do NOT
             * advance the watermark here. Skip + good reads compose with the
             * caller's count-based mark ONLY if marking happens from the
             * entry watermark: caller marks (packed + skipped) after TX
             * confirm, landing the watermark exactly past every consumed
             * sequence. Never silent (DDR-0007): the count is returned. */
            if (skipped_count != NULL) {
                (*skipped_count)++;
            }
            sequence_to_read++;
            continue;
        }

        (*actual_count)++;
        sequence_to_read++;
    }
    (void)i;

    return FLASH_LOG_OK;
}

FlashLog_StatusTypeDef FlashLog_MarkRecordsTransmitted(FlashLog_HandleTypeDef *hlog, uint32_t count)
{
    if (hlog == NULL || !hlog->initialized) {
        return FLASH_LOG_ERROR_PARAM;
    }
    
    if (count == 0) {
        return FLASH_LOG_OK; /* Nothing to mark */
    }
    
    /* Update last transmitted sequence to mark these records as sent */
    uint32_t new_last_transmitted = hlog->last_transmitted_sequence + count;

    /* F15 FIX: off-by-one. "All caught up" means last_transmitted ==
     * next_sequence; clamping to next_sequence-1 left one phantom unsent
     * record forever (GetUnsentCount never reached 0) and wedged bulk
     * transfer on the last record. Clamp to next_sequence itself. */
    if (new_last_transmitted > hlog->next_sequence) {
        new_last_transmitted = hlog->next_sequence;
    }
    
    hlog->last_transmitted_sequence = new_last_transmitted;
    
    /* Update header to persist transmission tracking */
    return FlashLog_SyncHeader(hlog);
}

uint32_t FlashLog_GetUnsentCount(FlashLog_HandleTypeDef *hlog)
{
    if (hlog == NULL || !hlog->initialized) {
        return 0;
    }
    
    if (hlog->next_sequence > hlog->last_transmitted_sequence) {
        return hlog->next_sequence - hlog->last_transmitted_sequence;
    }
    
    return 0;
}

/**
  * @brief  Validate header magic, version, and CRC
  */
static bool FlashLog_ValidateHeader(const FlashLog_Header_t *header)
{
    uint32_t calculated_crc;
    
    /* Check magic number */
    if (header->magic != FLASH_LOG_HEADER_MAGIC) {
        return false;
    }
    
    /* Check version */
    if (header->version != FLASH_LOG_HEADER_VERSION) {
        return false;
    }
    
    /* F-021 (#55): beyond magic+CRC, header field CONTENTS must be plausible
     * before a header is trusted — a CRC-valid but semantically impossible
     * header must not steer the log. */
    if (header->write_addr < FLASH_LOG_DATA_START || header->write_addr >= FLASH_LOG_DATA_END ||
        (header->write_addr % FLASH_LOG_RECORD_SIZE) != 0) {
        return false;  /* write frontier outside the data region or unaligned */
    }
    if (header->oldest_addr < FLASH_LOG_DATA_START || header->oldest_addr >= FLASH_LOG_DATA_END ||
        (header->oldest_addr % FLASH_LOG_RECORD_SIZE) != 0) {
        return false;
    }
    if (header->record_count > FLASH_LOG_MAX_RECORDS) {
        return false;  /* more records than the ring can hold */
    }
    if (header->last_transmitted_seq > header->record_count) {
        return false;  /* watermark ahead of the frontier is impossible */
    }

    /* Validate CRC32 */
    calculated_crc = FlashLog_CRC32((const uint8_t *)header, 
                                    sizeof(FlashLog_Header_t) - sizeof(uint32_t));
    
    return (calculated_crc == header->crc32);
}

/**
  * @brief  Write current header to flash (ping-pong update)
  */
static FlashLog_StatusTypeDef FlashLog_WriteHeader(FlashLog_HandleTypeDef *hlog)
{
    FlashLog_Header_t header;
    W25Q_StatusTypeDef status;
    uint32_t write_addr;
    
    /* Prepare header */
    memset(&header, 0, sizeof(header));
    header.magic = FLASH_LOG_HEADER_MAGIC;
    header.version = FLASH_LOG_HEADER_VERSION;
    header.write_addr = hlog->write_addr;
    header.record_count = hlog->record_count;
    /* F-007/R12 (#50): real monotonic generation — record_count was NOT a
     * freshness counter (a watermark-only SyncHeader left two valid headers
     * with equal "freshness" and different watermarks; the tie-break could
     * resurrect an older watermark → duplicate retransmission). */
    header.sequence = hlog->header_generation;
    header.oldest_addr = hlog->oldest_addr;
    header.flags = 0;
    header.last_transmitted_seq = hlog->last_transmitted_sequence;
    
    /* Calculate CRC32 (all fields except crc32 itself) */
    header.crc32 = FlashLog_CRC32((const uint8_t *)&header, 
                                  sizeof(FlashLog_Header_t) - sizeof(uint32_t));
    
    /* Toggle between header A and B for wear leveling */
    hlog->active_header = (hlog->active_header == 0) ? 1 : 0;
    write_addr = (hlog->active_header == 0) ? HEADER_A_ADDR : HEADER_B_ADDR;

    /* T4 FIX (DDR-0004): erase-before-write invariant. Each header lives in its
     * own sector, so erasing it can only destroy the STALE copy — the other
     * (current) header survives in its own sector. Without this erase, NOR
     * flash rewrites silently corrupt (bits only flip 1->0). */
    status = W25Q_EraseSector(hlog->hw25q, write_addr);
    if (status != W25Q_OK) {
        return FLASH_LOG_ERROR_FLASH;
    }

    status = W25Q_Write(hlog->hw25q, write_addr, (const uint8_t *)&header, sizeof(header));
    if (status != W25Q_OK) {
        return FLASH_LOG_ERROR_FLASH;
    }

    hlog->header_generation++;  /* F-007/R12 (#50): monotonic per successful write */

    return FLASH_LOG_OK;
}

/**
  * @brief  Calculate record address from index
  */
static uint32_t FlashLog_GetRecordAddress(FlashLog_HandleTypeDef *hlog, uint32_t record_index)
{
    uint32_t offset = (record_index % FLASH_LOG_MAX_RECORDS) * FLASH_LOG_RECORD_SIZE;
    return FLASH_LOG_DATA_START + offset;
}

/**
  * @brief  Erase sector if we're about to overwrite old data
  */
static FlashLog_StatusTypeDef FlashLog_EraseSectorIfNeeded(FlashLog_HandleTypeDef *hlog, uint32_t addr)
{
    W25Q_StatusTypeDef status;
    uint32_t sector_addr = (addr / W25Q_SECTOR_SIZE) * W25Q_SECTOR_SIZE;
    
    /* Only erase if this is the start of a new sector */
    if ((addr % W25Q_SECTOR_SIZE) == 0) {
        status = W25Q_EraseSector(hlog->hw25q, sector_addr);
        if (status != W25Q_OK) {
            return FLASH_LOG_ERROR_FLASH;
        }
    }
    
    return FLASH_LOG_OK;
}

/* Public Functions ----------------------------------------------------------*/

FlashLog_StatusTypeDef FlashLog_Init(FlashLog_HandleTypeDef *hlog, W25Q_HandleTypeDef *hw25q)
{
    FlashLog_Header_t header_a, header_b;
    FlashLog_StatusTypeDef status;
    bool valid_a, valid_b;
    
    if (hlog == NULL || hw25q == NULL) {
        return FLASH_LOG_ERROR_PARAM;
    }
    
    /* Clear handle */
    memset(hlog, 0, sizeof(FlashLog_HandleTypeDef));
    hlog->hw25q = hw25q;
    
    /* Initialize CRC32 table */
    FlashLog_InitCRC32Table();
    
    /* Try to read both headers */
    status = FlashLog_ReadHeader(hlog, HEADER_A_ADDR, &header_a);
    if (status != FLASH_LOG_OK) {
        return status;
    }
    
    status = FlashLog_ReadHeader(hlog, HEADER_B_ADDR, &header_b);
    if (status != FLASH_LOG_OK) {
        return status;
    }
    
    /* Validate headers */
    valid_a = FlashLog_ValidateHeader(&header_a);
    valid_b = FlashLog_ValidateHeader(&header_b);
    
    if (valid_a && valid_b) {
        /* Both valid — F-007/R12 (#50): sequence is now a real generation
         * counter; pick the newest with a wrap-safe compare. */
        bool a_newer = ((int32_t)(header_a.sequence - header_b.sequence) > 0);
        if (a_newer) {
            hlog->write_addr = header_a.write_addr;
            hlog->record_count = header_a.record_count;
            hlog->oldest_addr = header_a.oldest_addr;
            hlog->last_transmitted_sequence = header_a.last_transmitted_seq;
            hlog->header_generation = header_a.sequence + 1;
            hlog->active_header = 0;
        } else {
            hlog->write_addr = header_b.write_addr;
            hlog->record_count = header_b.record_count;
            hlog->oldest_addr = header_b.oldest_addr;
            hlog->last_transmitted_sequence = header_b.last_transmitted_seq;
            hlog->header_generation = header_b.sequence + 1;
            hlog->active_header = 1;
        }
    } else if (valid_a) {
        /* Only A valid */
        hlog->write_addr = header_a.write_addr;
        hlog->record_count = header_a.record_count;
        hlog->oldest_addr = header_a.oldest_addr;
        hlog->last_transmitted_sequence = header_a.last_transmitted_seq;
        hlog->header_generation = header_a.sequence + 1;
        hlog->active_header = 0;
    } else if (valid_b) {
        /* Only B valid */
        hlog->write_addr = header_b.write_addr;
        hlog->record_count = header_b.record_count;
        hlog->oldest_addr = header_b.oldest_addr;
        hlog->last_transmitted_sequence = header_b.last_transmitted_seq;
        hlog->header_generation = header_b.sequence + 1;
        hlog->active_header = 1;
    } else {
        /* No valid headers - initialize fresh */
        hlog->write_addr = FLASH_LOG_DATA_START;
        hlog->record_count = 0;
        hlog->oldest_addr = FLASH_LOG_DATA_START;
        hlog->active_header = 0;

        /* Erase BOTH header sectors to start fresh (T4: separate sectors) */
        W25Q_EraseSector(hw25q, HEADER_A_ADDR);
        W25Q_EraseSector(hw25q, HEADER_B_ADDR);
        
        /* Write initial header */
        status = FlashLog_WriteHeader(hlog);
        if (status != FLASH_LOG_OK) {
            return status;
        }
    }
    
    /* FW-12 (DDR-0004): the header checkpoints every HEADER_UPDATE_INTERVAL
     * records; recover the uncheckpointed tail before trusting the frontier. */
    if (valid_a || valid_b) {
        status = FlashLog_FrontierScan(hlog);
        if (status != FLASH_LOG_OK) {
            return status;
        }
    }

    hlog->next_sequence = hlog->record_count;
    hlog->initialized = true;
    
    return FLASH_LOG_OK;
}

/**
  * @brief  FW-12 (DDR-0004): frontier scan. The header is only persisted every
  *         HEADER_UPDATE_INTERVAL records, so up to that many records can be
  *         written-but-uncheckpointed when power is cut. Trusting the stale
  *         header blindly would reuse those sequence numbers and wedge
  *         MarkRecordsTransmitted. Scan forward from the header write_addr
  *         across the possible uncheckpointed window (wrap-aware); a record
  *         whose magic+CRC verifies extends the recovered frontier.
  * @note   Erase-ahead means the next-record slot reads 0xFF (magic check
  *         fails fast); a CRC failure here is a torn record at the true
  *         frontier, not an error.
  */
static FlashLog_StatusTypeDef FlashLog_FrontierScan(FlashLog_HandleTypeDef *hlog)
{
    FlashLog_Record_t probe;

    /* F-008 (#49): no early return on record_count == 0 — a fresh count-0
     * header followed by 1-9 uncheckpointed records and a power cut would
     * orphan them (write_addr reused, records overwritten). The scan from
     * DATA_START is bounded (<= HEADER_UPDATE_INTERVAL probes) and safe. */

    for (uint32_t i = 0; i < HEADER_UPDATE_INTERVAL; i++) {
        if (W25Q_Read(hlog->hw25q, hlog->write_addr,
                      (uint8_t *)&probe, sizeof(probe)) != W25Q_OK) {
            return FLASH_LOG_ERROR_FLASH;  /* real hardware error */
        }
        if (!FlashLog_VerifyRecord(&probe)) {
            break;  /* erased or torn slot: frontier found */
        }
        hlog->write_addr += FLASH_LOG_RECORD_SIZE;
        if (hlog->write_addr >= FLASH_LOG_DATA_END) {
            hlog->write_addr = FLASH_LOG_DATA_START;
        }
        hlog->record_count++;
        if (hlog->record_count > FLASH_LOG_MAX_RECORDS) {
            hlog->oldest_addr = hlog->write_addr;
        }
    }
    return FLASH_LOG_OK;
}

FlashLog_StatusTypeDef FlashLog_DeInit(FlashLog_HandleTypeDef *hlog)
{
    if (hlog == NULL) {
        return FLASH_LOG_ERROR_PARAM;
    }
    
    /* Sync header before deinit */
    FlashLog_SyncHeader(hlog);
    
    hlog->initialized = false;
    hlog->hw25q = NULL;
    
    return FLASH_LOG_OK;
}

FlashLog_StatusTypeDef FlashLog_WriteRecord(FlashLog_HandleTypeDef *hlog,
                                            const sensor_t *sensor_data,
                                            uint32_t timestamp,
                                            int16_t voltage_slope,
                                            uint8_t power_mode)
{
    FlashLog_Record_t record;
    W25Q_StatusTypeDef status;
    FlashLog_StatusTypeDef log_status;
    
    if (hlog == NULL || !hlog->initialized || sensor_data == NULL) {
        return FLASH_LOG_ERROR_PARAM;
    }
    
    /* Erase sector if starting new sector */
    log_status = FlashLog_EraseSectorIfNeeded(hlog, hlog->write_addr);
    if (log_status != FLASH_LOG_OK) {
        return log_status;
    }
    
    /* Build record */
    memset(&record, 0, sizeof(record));
    
    /* Header */
    record.magic = FLASH_LOG_RECORD_MAGIC;
    record.sequence = hlog->next_sequence;  /* F-010 (#52): incremented only after write success below */
    record.timestamp = timestamp;
    
    /* Environmental sensors */
    record.pressure = sensor_data->pressure;
    record.temperature = sensor_data->temperature;
    record.humidity = sensor_data->humidity;
    
    /* GNSS data */
    record.latitude = sensor_data->latitude;
    record.longitude = sensor_data->longitude;
    record.altitude_gps = sensor_data->altitudeGps;  /* D5/#35: int32 */
    /* altitude_bar deleted (D5/#35) — ground computes it */
    record.satellites = sensor_data->satellites;
    record.gnss_fix_quality = sensor_data->gnss_fix_quality;
    record.gnss_hdop_x10 = (uint8_t)(sensor_data->gnss_hdop * 10.0f);
    record.gnss_valid = sensor_data->gnss_valid ? 1 : 0;

    /* Power + status (D5/#35: solar/slope/mode archived at write time — F-025/R19) */
    record.battery_mv = (uint16_t)(sensor_data->battery_voltage * 1000.0f);
    {
        float solar_mv_f = sensor_data->solar_voltage * 1000.0f;
        if (solar_mv_f < 0.0f) solar_mv_f = 0.0f;
        if (solar_mv_f > 65535.0f) solar_mv_f = 65535.0f;
        record.solar_mv = (uint16_t)(solar_mv_f + 0.5f);
    }
    record.voltage_slope = voltage_slope;
    record.power_mode = power_mode;

    /* FW-7 (DDR-0007): archive carries the reading's own freshness */
    record.flags = (sensor_data->press_stale ? 0x01 : 0)
                 | (sensor_data->temp_stale  ? 0x02 : 0)
                 | (sensor_data->hum_stale   ? 0x04 : 0)
                 | (sensor_data->gnss_stale  ? 0x08 : 0);
    
    /* Calculate CRC32 (all fields except crc32) */
    record.crc32 = FlashLog_CRC32((const uint8_t *)&record, 
                                  sizeof(FlashLog_Record_t) - sizeof(uint32_t));
    
    /* Write record to flash */
    status = W25Q_Write(hlog->hw25q, hlog->write_addr, 
                        (const uint8_t *)&record, sizeof(record));
    if (status != W25Q_OK) {
        return FLASH_LOG_ERROR_FLASH;
    }
    
    /* Update write pointer (F-010: sequence consumed only now, after success) */
    hlog->next_sequence++;
    hlog->write_addr += FLASH_LOG_RECORD_SIZE;
    hlog->record_count++;
    
    /* Handle wraparound */
    if (hlog->write_addr >= FLASH_LOG_DATA_END) {
        hlog->write_addr = FLASH_LOG_DATA_START;
    }
    
    /* Update oldest address if we wrapped */
    if (hlog->record_count > FLASH_LOG_MAX_RECORDS) {
        hlog->oldest_addr = hlog->write_addr;
    }
    
    /* Periodically update header to flash */
    if ((hlog->record_count % HEADER_UPDATE_INTERVAL) == 0) {
        log_status = FlashLog_WriteHeader(hlog);
        if (log_status != FLASH_LOG_OK) {
            return log_status;
        }
    }
    
    return FLASH_LOG_OK;
}

FlashLog_StatusTypeDef FlashLog_ReadRecord(FlashLog_HandleTypeDef *hlog,
                                           FlashLog_Record_t *record,
                                           uint32_t offset)
{
    W25Q_StatusTypeDef status;
    uint32_t available_records;
    uint32_t read_addr;
    uint32_t record_index;
    
    if (hlog == NULL || !hlog->initialized || record == NULL) {
        return FLASH_LOG_ERROR_PARAM;
    }
    
    available_records = FlashLog_GetAvailableRecords(hlog);
    
    if (offset >= available_records) {
        return FLASH_LOG_ERROR_EMPTY;
    }
    
    /* Calculate address: most recent is one record before write_addr */
    /* For LIFO: record 0 = newest, record N = oldest */
    record_index = hlog->next_sequence - 1 - offset;
    read_addr = FlashLog_GetRecordAddress(hlog, record_index);
    
    /* Read record */
    status = W25Q_Read(hlog->hw25q, read_addr, (uint8_t *)record, sizeof(FlashLog_Record_t));
    if (status != W25Q_OK) {
        return FLASH_LOG_ERROR_FLASH;
    }
    
    /* Validate record */
    if (!FlashLog_VerifyRecord(record)) {
        return FLASH_LOG_ERROR_CRC;
    }
    
    return FLASH_LOG_OK;
}

FlashLog_StatusTypeDef FlashLog_ReadRecords(FlashLog_HandleTypeDef *hlog,
                                            FlashLog_Record_t *records,
                                            uint32_t max_count,
                                            uint32_t *actual_count,
                                            uint32_t start_offset)
{
    FlashLog_StatusTypeDef status;
    uint32_t i;
    uint32_t available;
    
    if (hlog == NULL || !hlog->initialized || records == NULL || actual_count == NULL) {
        return FLASH_LOG_ERROR_PARAM;
    }
    
    *actual_count = 0;
    available = FlashLog_GetAvailableRecords(hlog);
    
    if (start_offset >= available) {
        return FLASH_LOG_ERROR_EMPTY;
    }
    
    /* Read up to max_count records or until we run out */
    for (i = 0; i < max_count; i++) {
        uint32_t offset = start_offset + i;
        
        if (offset >= available) {
            break;  /* No more records */
        }
        
        status = FlashLog_ReadRecord(hlog, &records[i], offset);
        if (status != FLASH_LOG_OK) {
            return status;  /* Stop on error */
        }
        
        (*actual_count)++;
    }
    
    return FLASH_LOG_OK;
}

uint32_t FlashLog_GetRecordCount(FlashLog_HandleTypeDef *hlog)
{
    if (hlog == NULL || !hlog->initialized) {
        return 0;
    }
    
    return hlog->record_count;
}

uint32_t FlashLog_GetAvailableRecords(FlashLog_HandleTypeDef *hlog)
{
    if (hlog == NULL || !hlog->initialized) {
        return 0;
    }
    
    /* If we haven't wrapped yet, all records are available */
    if (hlog->record_count <= FLASH_LOG_MAX_RECORDS) {
        return hlog->record_count;
    }
    
    /* After wrap, only FLASH_LOG_MAX_RECORDS are available */
    return FLASH_LOG_MAX_RECORDS;
}

bool FlashLog_HasWrapped(FlashLog_HandleTypeDef *hlog)
{
    if (hlog == NULL || !hlog->initialized) {
        return false;
    }
    
    return (hlog->record_count > FLASH_LOG_MAX_RECORDS);
}

FlashLog_StatusTypeDef FlashLog_SyncHeader(FlashLog_HandleTypeDef *hlog)
{
    if (hlog == NULL || !hlog->initialized) {
        return FLASH_LOG_ERROR_PARAM;
    }
    
    return FlashLog_WriteHeader(hlog);
}

bool FlashLog_VerifyRecord(const FlashLog_Record_t *record)
{
    uint32_t calculated_crc;
    
    if (record == NULL) {
        return false;
    }
    
    /* Check magic number */
    if (record->magic != FLASH_LOG_RECORD_MAGIC) {
        return false;
    }
    
    /* Verify CRC32 */
    calculated_crc = FlashLog_CRC32((const uint8_t *)record, 
                                    sizeof(FlashLog_Record_t) - sizeof(uint32_t));
    
    return (calculated_crc == record->crc32);
}

FlashLog_StatusTypeDef FlashLog_GetStats(FlashLog_HandleTypeDef *hlog,
                                         uint32_t *total_capacity,
                                         uint32_t *used_records,
                                         uint32_t *free_records)
{
    uint32_t available;
    
    if (hlog == NULL || !hlog->initialized) {
        return FLASH_LOG_ERROR_PARAM;
    }
    
    available = FlashLog_GetAvailableRecords(hlog);
    
    if (total_capacity != NULL) {
        *total_capacity = FLASH_LOG_MAX_RECORDS;
    }
    
    if (used_records != NULL) {
        *used_records = available;
    }
    
    if (free_records != NULL) {
        if (hlog->record_count < FLASH_LOG_MAX_RECORDS) {
            *free_records = FLASH_LOG_MAX_RECORDS - hlog->record_count;
        } else {
            *free_records = 0;  /* Circular buffer is full, will overwrite */
        }
    }
    
    return FLASH_LOG_OK;
}
