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
#define HEADER_A_ADDR     0x0000  /* First 256 bytes of sector 0 */
#define HEADER_B_ADDR     0x0100  /* Second 256 bytes of sector 0 */
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
    header.sequence = hlog->record_count;  /* Use record count as sequence */
    header.oldest_addr = hlog->oldest_addr;
    header.flags = 0;
    
    /* Calculate CRC32 (all fields except crc32 itself) */
    header.crc32 = FlashLog_CRC32((const uint8_t *)&header, 
                                  sizeof(FlashLog_Header_t) - sizeof(uint32_t));
    
    /* Toggle between header A and B for wear leveling */
    hlog->active_header = (hlog->active_header == 0) ? 1 : 0;
    write_addr = (hlog->active_header == 0) ? HEADER_A_ADDR : HEADER_B_ADDR;
    
    /* Write to flash (assumes header area is erased or using page program */
    status = W25Q_Write(hlog->hw25q, write_addr, (const uint8_t *)&header, sizeof(header));
    if (status != W25Q_OK) {
        return FLASH_LOG_ERROR_FLASH;
    }
    
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
        /* Both valid - use the one with higher sequence */
        if (header_a.sequence > header_b.sequence) {
            hlog->write_addr = header_a.write_addr;
            hlog->record_count = header_a.record_count;
            hlog->oldest_addr = header_a.oldest_addr;
            hlog->active_header = 0;
        } else {
            hlog->write_addr = header_b.write_addr;
            hlog->record_count = header_b.record_count;
            hlog->oldest_addr = header_b.oldest_addr;
            hlog->active_header = 1;
        }
    } else if (valid_a) {
        /* Only A valid */
        hlog->write_addr = header_a.write_addr;
        hlog->record_count = header_a.record_count;
        hlog->oldest_addr = header_a.oldest_addr;
        hlog->active_header = 0;
    } else if (valid_b) {
        /* Only B valid */
        hlog->write_addr = header_b.write_addr;
        hlog->record_count = header_b.record_count;
        hlog->oldest_addr = header_b.oldest_addr;
        hlog->active_header = 1;
    } else {
        /* No valid headers - initialize fresh */
        hlog->write_addr = FLASH_LOG_DATA_START;
        hlog->record_count = 0;
        hlog->oldest_addr = FLASH_LOG_DATA_START;
        hlog->active_header = 0;
        
        /* Erase header sector to start fresh */
        W25Q_EraseSector(hw25q, 0);
        
        /* Write initial header */
        status = FlashLog_WriteHeader(hlog);
        if (status != FLASH_LOG_OK) {
            return status;
        }
    }
    
    hlog->next_sequence = hlog->record_count;
    hlog->initialized = true;
    
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
                                            uint32_t timestamp)
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
    record.sequence = hlog->next_sequence++;
    record.timestamp = timestamp;
    
    /* Environmental sensors */
    record.pressure = sensor_data->pressure;
    record.temperature = sensor_data->temperature;
    record.humidity = sensor_data->humidity;
    
    /* GNSS data */
    record.latitude = sensor_data->latitude;
    record.longitude = sensor_data->longitude;
    record.altitude_gps = sensor_data->altitudeGps;
    record.altitude_bar = sensor_data->altitudeBar;
    record.satellites = sensor_data->satellites;
    record.gnss_fix_quality = sensor_data->gnss_fix_quality;
    record.gnss_hdop_x10 = (uint8_t)(sensor_data->gnss_hdop * 10.0f);
    record.gnss_valid = sensor_data->gnss_valid ? 1 : 0;
    
    /* Battery */
    record.battery_mv = (uint16_t)(sensor_data->battery_voltage * 1000.0f);
    
    /* Calculate CRC32 (all fields except crc32) */
    record.crc32 = FlashLog_CRC32((const uint8_t *)&record, 
                                  sizeof(FlashLog_Record_t) - sizeof(uint32_t));
    
    /* Write record to flash */
    status = W25Q_Write(hlog->hw25q, hlog->write_addr, 
                        (const uint8_t *)&record, sizeof(record));
    if (status != W25Q_OK) {
        return FLASH_LOG_ERROR_FLASH;
    }
    
    /* Update write pointer */
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

FlashLog_StatusTypeDef FlashLog_EraseAll(FlashLog_HandleTypeDef *hlog)
{
    W25Q_StatusTypeDef status;
    uint32_t sector;
    
    if (hlog == NULL || !hlog->initialized) {
        return FLASH_LOG_ERROR_PARAM;
    }
    
    /* Erase all sectors (this takes time!) */
    for (sector = 0; sector < W25Q_SECTOR_COUNT; sector++) {
        uint32_t addr = sector * W25Q_SECTOR_SIZE;
        status = W25Q_EraseSector(hlog->hw25q, addr);
        if (status != W25Q_OK) {
            return FLASH_LOG_ERROR_FLASH;
        }
    }
    
    /* Reset state */
    hlog->write_addr = FLASH_LOG_DATA_START;
    hlog->record_count = 0;
    hlog->oldest_addr = FLASH_LOG_DATA_START;
    hlog->next_sequence = 0;
    hlog->active_header = 0;
    
    /* Write fresh header */
    return FlashLog_WriteHeader(hlog);
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
