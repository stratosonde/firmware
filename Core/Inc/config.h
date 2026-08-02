/**
  ******************************************************************************
  * @file    config.h
  * @brief   Centralized configuration system for Stratosonde firmware
  ******************************************************************************
  * @attention
  *
  * This module centralizes all configuration parameters that are currently
  * scattered as #defines across multiple files. Provides flash-based 
  * persistence with validation and default fallback.
  *
  ******************************************************************************
  */

#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/** @brief Configuration magic number for validation */
#define CONFIG_MAGIC_NUMBER           0xC0FF33C0

/** @brief Configuration version (increment when structure changes) */
#define CONFIG_VERSION                1

/**
  * @brief Configuration flash address
  *
  * @warning The STM32WLE5 erases internal flash in 2KB pages. This region MUST
  *          therefore occupy a page of its own, otherwise Config_Save() would
  *          erase a neighbouring page and destroy its contents.
  *
  * Internal flash page map (256KB device, 2KB pages):
  *   0x0803C000  page 120 - Tier-1 credentials copy A  (FW-1/ADR-0006)
  *   0x0803C800  page 121 - Tier-1 credentials copy B
  *   0x0803D000  page 122 - Tier-1 credentials copy C
  *   0x0803D800  page 123 - Tier-2 counters slot A (ping-pong)
  *   0x0803E000  page 124 - Tier-2 counters slot B (ping-pong)
  *   0x0803E800  page 125 - System configuration     (this module)
  *   0x0803F000  page 126 - LoRaWAN NVM context      (LORAWAN_NVM_BASE_ADDRESS)
  *   0x0803F800  page 127 - legacy multi-region page (retired, reserved)
  *
  * Previously this was 0x0803FC00, which sits *inside* page 127. Saving the
  * configuration erased the whole page and wiped every stored region context
  * (DevAddr, session keys and frame counters), and vice-versa.
  */
#define CONFIG_FLASH_ADDRESS          (0x0803E800UL)  // Page 125, dedicated

/** @brief Configuration flash region size (one full 2KB erase page) */
#define CONFIG_FLASH_SIZE             2048

/* Exported types ------------------------------------------------------------*/

/**
 * @brief System configuration structure
 * @note All settings centralized here to replace scattered #defines
 * @note Flash-persistent with validation and defaults
 */
typedef struct __attribute__((packed)) {
    /* Header and validation (12 bytes) */
    uint32_t magic;                    // CONFIG_MAGIC_NUMBER for validation
    uint16_t version;                  // Configuration version  
    uint16_t size;                     // Structure size for validation
    uint32_t crc32;                    // CRC32 of configuration data
    
    /* Transmission settings (16 bytes) */
    uint32_t tx_interval_normal;       // Normal mode interval (ms) - default 300000 (5 min)
    uint32_t tx_interval_conservative; // Conservative interval (ms) - default 600000 (10 min)
    uint32_t tx_interval_reduced;      // Reduced interval (ms) - default 900000 (15 min)
    uint32_t tx_interval_recovery;     // Recovery interval (ms) - default 1800000 (30 min)
    uint32_t tx_interval_survival;     // Survival interval (ms) - default 3600000 (60 min)
    
    /* LoRaWAN parameters (8 bytes) */
    uint8_t  lorawan_datarate;         // Default datarate (DR0-DR5) - default DR_0
    uint8_t  lorawan_txpower;          // TX power (dBm) - default TX_POWER_0
    uint8_t  lorawan_adr_enabled;      // ADR enable flag - default 0 (disabled)
    uint8_t  lorawan_confirmed;        // Confirmed messages flag - default 0
    uint32_t lorawan_class_b_timeout;  // Class B/C timeout (ms) - default 8000
    
    /* Power management thresholds (12 bytes) */
    uint16_t battery_low_threshold;    // Low battery threshold (mV) - default 4500
    uint16_t battery_critical_threshold; // Critical threshold (mV) - default 4000  
    uint16_t bulk_battery_min_mv;      // Min battery for bulk transfer (mV) - default 5000
    int8_t   gps_temperature_lockout;  // GPS lockout temperature (°C) - default -55
    uint8_t  power_mode_hysteresis;    // Mode change hysteresis (%) - default 10
    /* F19 FIX: solar_charging_threshold deleted. The 6000 mV default could
     * never trip on the real ~1.1 V two-wafer panel, and the field had zero
     * consumers (verified by grep) — a decorative knob. Kept as reserved to
     * preserve the flash-stored struct layout. Raw solar_mv telemetry flows
     * regardless; re-derive a threshold from bench data if ever needed. */
    uint16_t reserved_solar;           // Was solar_charging_threshold (deleted, F19)
    uint8_t  reserved_power;           // Reserved for alignment
    
    /* GPS settings (8 bytes) */
    uint8_t  gps_timeout_normal;       // GPS timeout normal mode (s) - default 60
    uint8_t  gps_timeout_conservative; // GPS timeout conservative (s) - default 60  
    uint8_t  gps_min_satellites;       // Min satellites for good fix - default 4
    uint8_t  gps_max_hdop_x10;         // Max HDOP * 10 for good fix - default 25 (2.5)
    uint32_t gps_standby_power_ua;     // GPS standby current (µA) - default 15
    
    /* Adaptive transmission thresholds (8 bytes) */
    uint8_t  link_margin_threshold;    // Min demod margin for SF7 (dB) - default 15
    uint8_t  gateway_count_threshold;  // Min gateway count for SF7 - default 2
    uint8_t  max_bulk_packets;         // Max bulk packets per cycle - default 20
    uint8_t  frame_counter_save_interval; // Frame counter save interval - default 10
    uint32_t bulk_timeout_ms;          // Bulk transfer timeout (ms) - default 60000
    
    /* Flash logging settings (8 bytes) */
    uint16_t flash_log_interval;       // Log interval (records) - default 1 (every read)
    uint8_t  flash_log_enabled;        // Flash logging enable - default 1
    uint8_t  flash_log_compression;    // Compression enable - default 0
    uint32_t flash_log_retention_days; // Data retention (days) - default 30
    
    /* Debug settings (8 bytes) */
    uint8_t  debug_lpp_enabled;        // CayenneLPP debug enable - default 1
    uint8_t  debug_gnss_detail_enabled; // GNSS detail debug enable - default 1  
    uint8_t  debug_lpp_interval;       // Debug packet interval - default 5
    uint8_t  debug_rtt_level;          // RTT debug level (0-3) - default 2
    uint32_t debug_flags;              // Debug feature flags - default 0x00000001
    
    /* Reserved for future expansion (16 bytes) */
    uint8_t  reserved[16];             // Future configuration fields
    
} SystemConfig_t;

/* Compile-time size check */
_Static_assert(sizeof(SystemConfig_t) <= CONFIG_FLASH_SIZE, 
               "SystemConfig_t must fit in allocated flash space");

/**
 * @brief Configuration validation result
 */
typedef enum {
    CONFIG_OK = 0,              // Configuration valid
    CONFIG_ERROR_MAGIC,         // Invalid magic number
    CONFIG_ERROR_VERSION,       // Unsupported version
    CONFIG_ERROR_SIZE,          // Invalid size
    CONFIG_ERROR_CRC,           // CRC validation failed
    CONFIG_ERROR_FLASH,         // Flash read/write error
    CONFIG_ERROR_PARAM,         // Invalid parameter
    CONFIG_ERROR_RANGE          // Value out of valid range
} ConfigStatus_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize configuration system
 * @retval ConfigStatus_t: Initialization status
 * @note Loads config from flash or uses defaults if invalid
 */
ConfigStatus_t Config_Init(void);

/**
 * @brief Get current configuration  
 * @retval SystemConfig_t*: Pointer to current configuration (read-only)
 * @note Returns NULL if not initialized
 */
const SystemConfig_t* Config_Get(void);

/**
 * @brief Load configuration from flash
 * @retval ConfigStatus_t: Load status
 * @note Falls back to defaults if flash data invalid
 */
ConfigStatus_t Config_Load(void);

/**
 * @brief Save current configuration to flash
 * @retval ConfigStatus_t: Save status
 */
ConfigStatus_t Config_Save(void);

/**
 * @brief Validate configuration values
 * @param config: Configuration to validate
 * @retval ConfigStatus_t: Validation status
 * @note Checks ranges and dependencies
 */
ConfigStatus_t Config_Validate(const SystemConfig_t *config);

/**
 * @brief Reset to factory defaults
 * @retval ConfigStatus_t: Reset status
 * @note Does not save to flash (call Config_Save separately)
 */
ConfigStatus_t Config_LoadDefaults(void);

/**
 * @brief Update a specific configuration parameter
 * @param param_name: Parameter name (for debugging)
 * @param new_value: New value to set
 * @param param_ptr: Pointer to parameter in config structure
 * @param param_size: Size of parameter in bytes
 * @retval ConfigStatus_t: Update status
 * @note Validates range and updates CRC
 */
ConfigStatus_t Config_UpdateParameter(const char *param_name, 
                                      uint32_t new_value,
                                      void *param_ptr, 
                                      uint8_t param_size);

/**
 * @brief Get configuration statistics
 * @param flash_reads: Pointer to store flash read count (optional)
 * @param flash_writes: Pointer to store flash write count (optional)
 * @param crc_failures: Pointer to store CRC failure count (optional)
 * @retval ConfigStatus_t: Status
 */
ConfigStatus_t Config_GetStats(uint32_t *flash_reads, 
                               uint32_t *flash_writes, 
                               uint32_t *crc_failures);

/**
 * @brief Print current configuration to RTT for debugging
 * @retval ConfigStatus_t: Status
 */
ConfigStatus_t Config_PrintCurrent(void);

/**
 * @brief Calculate CRC32 for configuration structure
 * @param config: Configuration structure  
 * @retval uint32_t: CRC32 value
 * @note Used internally for validation
 */
uint32_t Config_CalculateCRC32(const SystemConfig_t *config);

/* Convenience macros for accessing configuration values --------------------- */

#define CFG_TX_INTERVAL_NORMAL       (Config_Get()->tx_interval_normal)
#define CFG_TX_INTERVAL_CONSERVATIVE (Config_Get()->tx_interval_conservative)
#define CFG_TX_INTERVAL_REDUCED      (Config_Get()->tx_interval_reduced)
#define CFG_TX_INTERVAL_RECOVERY     (Config_Get()->tx_interval_recovery)
#define CFG_TX_INTERVAL_SURVIVAL     (Config_Get()->tx_interval_survival)

#define CFG_LORAWAN_DATARATE         (Config_Get()->lorawan_datarate)
#define CFG_LORAWAN_TXPOWER          (Config_Get()->lorawan_txpower)
#define CFG_LORAWAN_ADR_ENABLED      (Config_Get()->lorawan_adr_enabled)

#define CFG_BATTERY_LOW_THRESHOLD    (Config_Get()->battery_low_threshold)
#define CFG_BATTERY_CRITICAL_THRESHOLD (Config_Get()->battery_critical_threshold)
#define CFG_GPS_TEMPERATURE_LOCKOUT  (Config_Get()->gps_temperature_lockout)

#define CFG_LINK_MARGIN_THRESHOLD    (Config_Get()->link_margin_threshold)
#define CFG_GATEWAY_COUNT_THRESHOLD  (Config_Get()->gateway_count_threshold)
#define CFG_MAX_BULK_PACKETS         (Config_Get()->max_bulk_packets)

#define CFG_FRAME_COUNTER_SAVE_INTERVAL (Config_Get()->frame_counter_save_interval)

#define CFG_DEBUG_LPP_ENABLED        (Config_Get()->debug_lpp_enabled)
#define CFG_DEBUG_GNSS_DETAIL_ENABLED (Config_Get()->debug_gnss_detail_enabled)
#define CFG_DEBUG_LPP_INTERVAL       (Config_Get()->debug_lpp_interval)

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */
