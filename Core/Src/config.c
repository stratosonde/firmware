/**
  ******************************************************************************
  * @file    config.c
  * @brief   Centralized configuration system implementation
  ******************************************************************************
  * @attention
  *
  * This module provides flash-based configuration persistence with validation
  * and default fallback for all system parameters.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "flash_if.h"
#include "SEGGER_RTT.h"
#include "LmHandler.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define CRC32_POLYNOMIAL  0xEDB88320

/* Private variables ---------------------------------------------------------*/
static SystemConfig_t g_config;
static bool g_config_initialized = false;
static uint32_t g_flash_read_count = 0;
static uint32_t g_flash_write_count = 0;
static uint32_t g_crc_failure_count = 0;
static uint8_t g_config_buffer[CONFIG_FLASH_SIZE] __attribute__((aligned(8), unused));

/* Private function prototypes -----------------------------------------------*/
static uint32_t Config_CRC32(const uint8_t *data, uint32_t length);
static ConfigStatus_t Config_FlashRead(void);
static ConfigStatus_t Config_FlashWrite(void);
static ConfigStatus_t Config_WriteInternal(void);  /* F-002 (#53) */

/* Exported functions --------------------------------------------------------*/

ConfigStatus_t Config_Init(void)
{
    ConfigStatus_t status;
    
    if (g_config_initialized) {
        return CONFIG_OK;  // Already initialized
    }
    
    SEGGER_RTT_WriteString(0, "\r\n=== Config_Init ===\r\n");
    
    // Try to load from flash
    status = Config_Load();
    
    if (status == CONFIG_OK) {
        SEGGER_RTT_WriteString(0, "Configuration loaded from flash successfully\r\n");
    } else {
        SEGGER_RTT_printf(0, "Flash config invalid (status: %d), loading defaults\r\n", status);
        status = Config_LoadDefaults();
        if (status != CONFIG_OK) {
            SEGGER_RTT_printf(0, "ERROR: Failed to load default config (status: %d)\r\n", status);
            return status;
        }
        
        // Save defaults to flash (F-002: internal path — the public flag isn't set yet)
        status = Config_WriteInternal();
        if (status != CONFIG_OK) {
            SEGGER_RTT_printf(0, "WARNING: Failed to save default config to flash (status: %d)\r\n", status);
            // Continue anyway - we have valid defaults in RAM
        }
    }
    
    g_config_initialized = true;
    SEGGER_RTT_WriteString(0, "Configuration system initialized\r\n");
    
    return CONFIG_OK;
}

const SystemConfig_t* Config_Get(void)
{
    if (!g_config_initialized) {
        return NULL;
    }
    return &g_config;
}

ConfigStatus_t Config_Load(void)
{
    ConfigStatus_t status;
    
    SEGGER_RTT_WriteString(0, "Loading configuration from flash...\r\n");
    
    // Read from flash
    status = Config_FlashRead();
    if (status != CONFIG_OK) {
        g_flash_read_count++;
        return status;
    }
    
    // Validate loaded configuration
    status = Config_Validate(&g_config);
    if (status != CONFIG_OK) {
        SEGGER_RTT_printf(0, "Config validation failed (status: %d)\r\n", status);
        return status;
    }
    
    g_flash_read_count++;
    SEGGER_RTT_WriteString(0, "Configuration loaded and validated successfully\r\n");
    
    return CONFIG_OK;
}

/* F-002 (#53): internal write path NOT gated on g_config_initialized —
 * Config_Init must be able to persist first-boot defaults before the flag
 * is set. Config_Save (public) keeps the gate. */
static ConfigStatus_t Config_WriteInternal(void)
{
    ConfigStatus_t status;

    // Validate before saving
    status = Config_Validate(&g_config);
    if (status != CONFIG_OK) {
        SEGGER_RTT_printf(0, "Config validation failed before save (status: %d)\r\n", status);
        return status;
    }

    // Update CRC32
    g_config.crc32 = Config_CalculateCRC32(&g_config);

    // Write to flash
    status = Config_FlashWrite();
    if (status != CONFIG_OK) {
        SEGGER_RTT_printf(0, "Flash write failed (status: %d)\r\n", status);
        return status;
    }

    g_flash_write_count++;
    return CONFIG_OK;
}

ConfigStatus_t Config_Save(void)
{
    if (!g_config_initialized) {
        return CONFIG_ERROR_PARAM;
    }

    SEGGER_RTT_WriteString(0, "Saving configuration to flash...\r\n");

    ConfigStatus_t status = Config_WriteInternal();
    if (status == CONFIG_OK) {
        SEGGER_RTT_WriteString(0, "Configuration saved to flash successfully\r\n");
    }
    return status;
}

ConfigStatus_t Config_Validate(const SystemConfig_t *config)
{
    if (config == NULL) {
        return CONFIG_ERROR_PARAM;
    }
    
    // Check magic number
    if (config->magic != CONFIG_MAGIC_NUMBER) {
        g_crc_failure_count++;
        return CONFIG_ERROR_MAGIC;
    }
    
    // Check version
    if (config->version > CONFIG_VERSION) {
        return CONFIG_ERROR_VERSION;
    }
    
    // Check size
    if (config->size != sizeof(SystemConfig_t)) {
        return CONFIG_ERROR_SIZE;
    }
    
    // Validate CRC32 (F-003/R35: full zeroed-copy struct, matching CalculateCRC32)
    SystemConfig_t temp_config = *config;
    temp_config.crc32 = 0;
    uint32_t calculated_crc = Config_CRC32((const uint8_t*)&temp_config,
                                           sizeof(SystemConfig_t));
    
    if (calculated_crc != config->crc32) {
        g_crc_failure_count++;
        return CONFIG_ERROR_CRC;
    }
    
    // Validate ranges (add basic sanity checks)
    if (config->tx_interval_normal < 60000 || config->tx_interval_normal > 3600000) {
        return CONFIG_ERROR_RANGE;  // 1 minute to 1 hour
    }
    
    if (config->battery_low_threshold > config->bulk_battery_min_mv) {
        return CONFIG_ERROR_RANGE;  // Bulk threshold must be higher than low
    }
    
    if (config->link_margin_threshold > 63) {
        return CONFIG_ERROR_RANGE;  // LoRaWAN margin is 6-bit field
    }

    /* F-018 (#53): the remaining semantic checks — interval hierarchy and
     * the power/GPS knobs the flight actually depends on. */
    if (!(config->tx_interval_normal <= config->tx_interval_conservative &&
          config->tx_interval_conservative <= config->tx_interval_reduced &&
          config->tx_interval_reduced <= config->tx_interval_recovery &&
          config->tx_interval_recovery <= config->tx_interval_survival)) {
        return CONFIG_ERROR_RANGE;  // mode hierarchy must be monotonic
    }
    if (config->tx_interval_survival > 7200000) {
        return CONFIG_ERROR_RANGE;  // >2h breaks the deadman assumption
    }
    if (config->gps_temperature_lockout < -80 || config->gps_temperature_lockout > 0) {
        return CONFIG_ERROR_RANGE;
    }
    if (!(config->battery_critical_threshold < config->battery_low_threshold &&
          config->battery_low_threshold < config->bulk_battery_min_mv)) {
        return CONFIG_ERROR_RANGE;  // critical < low < bulk-min ordering
    }

    return CONFIG_OK;
}

ConfigStatus_t Config_LoadDefaults(void)
{
    SEGGER_RTT_WriteString(0, "Loading factory default configuration...\r\n");
    
    // Clear structure
    memset(&g_config, 0, sizeof(SystemConfig_t));
    
    // Header
    g_config.magic = CONFIG_MAGIC_NUMBER;
    g_config.version = CONFIG_VERSION;
    g_config.size = sizeof(SystemConfig_t);
    
    // Transmission settings (from current lora_app.h values)
    g_config.tx_interval_normal = 300000;      // 5 minutes 
    g_config.tx_interval_conservative = 600000; // 10 minutes
    g_config.tx_interval_reduced = 900000;     // 15 minutes  
    g_config.tx_interval_recovery = 1800000;   // 30 minutes
    g_config.tx_interval_survival = 3600000;   // 60 minutes
    
    // LoRaWAN parameters
    g_config.lorawan_datarate = 0;             // DR_0 (SF10)
    g_config.lorawan_txpower = 0;              // TX_POWER_0 
    g_config.lorawan_adr_enabled = 0;          // ADR disabled for balloon
    g_config.lorawan_confirmed = 0;            // Unconfirmed messages
    g_config.lorawan_class_b_timeout = 8000;   // 8 second timeout
    
    // Power management thresholds
    g_config.battery_low_threshold = 4500;     // 4.5V low threshold
    g_config.battery_critical_threshold = 4000; // 4.0V critical
    g_config.bulk_battery_min_mv = 5000;       // 5.0V for bulk transfer
    g_config.gps_temperature_lockout = -55;    // -55°C GPS lockout
    g_config.power_mode_hysteresis = 10;       // 10% hysteresis
    /* F19 FIX: solar_charging_threshold default removed (field deleted -
     * 6000 mV could never trip on the ~1.1 V two-wafer panel; zero consumers) */
    
    // GPS settings
    g_config.gps_timeout_normal = 60;          // 60 second timeout
    g_config.gps_timeout_conservative = 60;    // 60 second timeout
    g_config.gps_min_satellites = 4;           // 4 satellites minimum
    g_config.gps_max_hdop_x10 = 25;            // 2.5 max HDOP
    g_config.gps_standby_power_ua = 15;        // 15µA standby current
    
    // Adaptive transmission thresholds  
    g_config.link_margin_threshold = 15;       // 15dB margin
    g_config.gateway_count_threshold = 2;      // 2 gateways
    g_config.max_bulk_packets = 20;            // 20 packets max per cycle
    g_config.frame_counter_save_interval = 10; // Save every 10 TX
    g_config.bulk_timeout_ms = 60000;          // 1 minute bulk timeout
    
    // Flash logging settings
    g_config.flash_log_interval = 1;           // Log every sensor read
    g_config.flash_log_enabled = 1;            // Logging enabled
    g_config.flash_log_compression = 0;        // No compression
    g_config.flash_log_retention_days = 30;    // 30 day retention
    
    // Debug settings
    g_config.debug_lpp_enabled = 1;            // LPP debug enabled
    g_config.debug_gnss_detail_enabled = 1;    // GNSS debug enabled
    g_config.debug_lpp_interval = 5;           // Every 5th transmission
    g_config.debug_rtt_level = 2;              // Moderate debug level
    g_config.debug_flags = 0x00000001;         // Basic debug flags
    
    // Calculate CRC32
    g_config.crc32 = Config_CalculateCRC32(&g_config);
    
    SEGGER_RTT_WriteString(0, "Factory defaults loaded successfully\r\n");
    
    return CONFIG_OK;
}

ConfigStatus_t Config_GetStats(uint32_t *flash_reads, 
                               uint32_t *flash_writes, 
                               uint32_t *crc_failures)
{
    if (flash_reads) {
        *flash_reads = g_flash_read_count;
    }
    if (flash_writes) {
        *flash_writes = g_flash_write_count;
    }
    if (crc_failures) {
        *crc_failures = g_crc_failure_count;
    }
    
    return CONFIG_OK;
}

ConfigStatus_t Config_PrintCurrent(void)
{
    if (!g_config_initialized) {
        SEGGER_RTT_WriteString(0, "Configuration not initialized\r\n");
        return CONFIG_ERROR_PARAM;
    }
    
    SEGGER_RTT_WriteString(0, "\r\n=== CURRENT CONFIGURATION ===\r\n");
    
    // Header
    SEGGER_RTT_printf(0, "Magic: 0x%08lX, Version: %d, Size: %d\r\n",
                      g_config.magic, g_config.version, g_config.size);
    
    // Transmission settings
    SEGGER_RTT_printf(0, "TX Intervals: Normal=%lums Conservative=%lums Reduced=%lums\r\n",
                      g_config.tx_interval_normal, g_config.tx_interval_conservative, 
                      g_config.tx_interval_reduced);
    SEGGER_RTT_printf(0, "              Recovery=%lums Survival=%lums\r\n",
                      g_config.tx_interval_recovery, g_config.tx_interval_survival);
    
    // LoRaWAN
    SEGGER_RTT_printf(0, "LoRaWAN: DR=%d TxPower=%d ADR=%s Confirmed=%s\r\n",
                      g_config.lorawan_datarate, g_config.lorawan_txpower,
                      g_config.lorawan_adr_enabled ? "ON" : "OFF",
                      g_config.lorawan_confirmed ? "ON" : "OFF");
    
    // Power management
    SEGGER_RTT_printf(0, "Battery: Low=%dmV Critical=%dmV Bulk=%dmV GPS_Lockout=%d°C\r\n",
                      g_config.battery_low_threshold, g_config.battery_critical_threshold,
                      g_config.bulk_battery_min_mv, g_config.gps_temperature_lockout);
    
    // Adaptive transmission
    SEGGER_RTT_printf(0, "Adaptive: Margin=%ddB Gateways=%d MaxBulk=%d FCntSave=%d\r\n",
                      g_config.link_margin_threshold, g_config.gateway_count_threshold,
                      g_config.max_bulk_packets, g_config.frame_counter_save_interval);
    
    // Debug settings
    SEGGER_RTT_printf(0, "Debug: LPP=%s GNSS=%s Interval=%d Level=%d\r\n",
                      g_config.debug_lpp_enabled ? "ON" : "OFF",
                      g_config.debug_gnss_detail_enabled ? "ON" : "OFF",
                      g_config.debug_lpp_interval, g_config.debug_rtt_level);
    
    SEGGER_RTT_printf(0, "CRC32: 0x%08lX\r\n", g_config.crc32);
    SEGGER_RTT_WriteString(0, "=== END CONFIGURATION ===\r\n\r\n");
    
    return CONFIG_OK;
}

uint32_t Config_CalculateCRC32(const SystemConfig_t *config)
{
    if (config == NULL) {
        return 0;
    }
    
    // Create temporary copy with CRC32 field zeroed
    SystemConfig_t temp_config = *config;
    temp_config.crc32 = 0;

    /* F-003/R35 (#53): hash the FULL zeroed-copy struct. crc32 is at offset
     * 8, so hashing sizeof-4 left the struct tail unprotected — latent until
     * any tail field changed. Both sides bumped together (CONFIG_VERSION 2). */
    return Config_CRC32((const uint8_t*)&temp_config, sizeof(SystemConfig_t));
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Calculate CRC32 using IEEE 802.3 polynomial
 */
static uint32_t Config_CRC32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x00000001) {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return ~crc;
}

/**
 * @brief Read configuration from flash
 */
static ConfigStatus_t Config_FlashRead(void)
{
    FLASH_IF_StatusTypedef flash_status;
    
    // Read configuration from flash
    flash_status = FLASH_IF_Read(&g_config, (void*)CONFIG_FLASH_ADDRESS, sizeof(SystemConfig_t));
    
    if (flash_status != FLASH_IF_OK) {
        SEGGER_RTT_printf(0, "Config flash read failed (status: %d)\r\n", flash_status);
        return CONFIG_ERROR_FLASH;
    }
    
    return CONFIG_OK;
}

/**
 * @brief Write configuration to flash
 */
static ConfigStatus_t Config_FlashWrite(void)
{
    FLASH_IF_StatusTypedef flash_status;
    
    // Erase flash page first
    flash_status = FLASH_IF_Erase((void*)CONFIG_FLASH_ADDRESS, CONFIG_FLASH_SIZE);
    if (flash_status != FLASH_IF_OK) {
        SEGGER_RTT_printf(0, "Config flash erase failed (status: %d)\r\n", flash_status);
        return CONFIG_ERROR_FLASH;
    }
    
    // Write configuration
    flash_status = FLASH_IF_Write((void*)CONFIG_FLASH_ADDRESS, &g_config, sizeof(SystemConfig_t));
    if (flash_status != FLASH_IF_OK) {
        SEGGER_RTT_printf(0, "Config flash write failed (status: %d)\r\n", flash_status);
        return CONFIG_ERROR_FLASH;
    }
    
    return CONFIG_OK;
}
