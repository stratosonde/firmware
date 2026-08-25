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
#include "LmHandler.h"
#include "SEGGER_RTT.h"
#include "flash_if.h"
#include "sonde_log.h" /* R50 (#47): compile-time log gate */
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define CRC32_POLYNOMIAL 0xEDB88320

/* Private variables ---------------------------------------------------------*/
/* g_config MUST be 8-byte aligned: SystemConfig_t is __attribute__((packed))
 * (alignment 1), so without this attribute the linker may place it at any
 * byte boundary. FLASH_IF_INT_Write reads the source buffer with 64-bit
 * loads (LDRD); an unaligned LDRD faults on Cortex-M4 -> HardFault ->
 * reset -> boot loop, self-sustaining because the erase has already
 * blanked the page so every boot retries the save. Observed on target:
 * g_config at 0x20000BD1 (odd), fault inside Config_Init's defaults save. */
static SystemConfig_t g_config __attribute__((aligned(8)));
static bool g_config_initialized = false;
static uint32_t g_flash_read_count = 0;
static uint32_t g_flash_write_count = 0;
static uint32_t g_crc_failure_count = 0;
static uint8_t g_config_buffer[CONFIG_FLASH_SIZE] __attribute__((aligned(8), unused));

/* Private function prototypes -----------------------------------------------*/
static uint32_t Config_CRC32(const uint8_t *data, uint32_t length);
static ConfigStatus_t Config_FlashRead(void);
static ConfigStatus_t Config_FlashWrite(void);
static ConfigStatus_t Config_WriteInternal(void); /* F-002 (#53) */

/* Exported functions --------------------------------------------------------*/

ConfigStatus_t Config_Init(void) {
  ConfigStatus_t status;

  if (g_config_initialized) {
    return CONFIG_OK; // Already initialized
  }

  SONDE_LOG_STR("\r\n=== Config_Init ===\r\n");

  // Try to load from flash
  status = Config_Load();

  if (status == CONFIG_OK) {
    SONDE_LOG_STR("Configuration loaded from flash successfully\r\n");
  } else {
    SONDE_LOG("Flash config invalid (status: %d), loading defaults\r\n", status);
    status = Config_LoadDefaults();
    if (status != CONFIG_OK) {
      SONDE_LOG("ERROR: Failed to load default config (status: %d)\r\n", status);
      return status;
    }

    // Save defaults to flash (F-002: internal path — the public flag isn't set yet)
    status = Config_WriteInternal();
    if (status != CONFIG_OK) {
      SONDE_LOG("WARNING: Failed to save default config to flash (status: %d)\r\n", status);
      // Continue anyway - we have valid defaults in RAM
    }
  }

  g_config_initialized = true;
  SONDE_LOG_STR("Configuration system initialized\r\n");

  return CONFIG_OK;
}

const SystemConfig_t *Config_Get(void) {
  if (!g_config_initialized) {
    return NULL;
  }
  return &g_config;
}

ConfigStatus_t Config_Load(void) {
  ConfigStatus_t status;

  SONDE_LOG_STR("Loading configuration from flash...\r\n");

  // Read from flash
  status = Config_FlashRead();
  if (status != CONFIG_OK) {
    g_flash_read_count++;
    return status;
  }

  // Validate loaded configuration
  status = Config_Validate(&g_config);
  if (status != CONFIG_OK) {
    SONDE_LOG("Config validation failed (status: %d)\r\n", status);
    return status;
  }

  g_flash_read_count++;
  SONDE_LOG_STR("Configuration loaded and validated successfully\r\n");

  return CONFIG_OK;
}

/* F-002 (#53): internal write path NOT gated on g_config_initialized —
 * Config_Init must be able to persist first-boot defaults before the flag
 * is set. Config_Save (public) keeps the gate. */
static ConfigStatus_t Config_WriteInternal(void) {
  ConfigStatus_t status;

  // Validate before saving
  status = Config_Validate(&g_config);
  if (status != CONFIG_OK) {
    SONDE_LOG("Config validation failed before save (status: %d)\r\n", status);
    return status;
  }

  // Update CRC32
  g_config.crc32 = Config_CalculateCRC32(&g_config);

  // Write to flash
  status = Config_FlashWrite();
  if (status != CONFIG_OK) {
    SONDE_LOG("Flash write failed (status: %d)\r\n", status);
    return status;
  }

  g_flash_write_count++;
  return CONFIG_OK;
}

/* R14 (#198) IMMUTABILITY ASSUMPTION, documented per the 2026-08-11
 * stability review's decision point: configuration is WRITE-ONCE AT
 * COMMISSIONING (on external power) and NEVER written in flight. Today
 * Config_Save has no callers anywhere in the tree - the only writes are
 * Config_Init persisting first-boot defaults. The single-slot
 * erase-then-write in Config_FlashWrite is acceptable ONLY under that
 * assumption (a brownout mid-write loses the config and boot falls back to
 * defaults, which is recoverable on the bench, not in flight). ANY future
 * in-flight caller breaks the assumption and must first implement the
 * two-slot-with-generation scheme tracked in #78 (A-003) - the host-suite
 * R14 scan goes red if a caller outside this file ever appears. */
ConfigStatus_t Config_Save(void) {
  if (!g_config_initialized) {
    return CONFIG_ERROR_PARAM;
  }

  SONDE_LOG_STR("Saving configuration to flash...\r\n");

  ConfigStatus_t status = Config_WriteInternal();
  if (status == CONFIG_OK) {
    SONDE_LOG_STR("Configuration saved to flash successfully\r\n");
  }
  return status;
}

/* S-04 (#228): deadman timeout derived from a survival cadence - see
 * ConfigGetDeadmanTimeoutS. */
static uint32_t DeadmanTimeoutForSurvival(uint32_t survival_ms) {
  uint32_t survival_s = survival_ms / 1000U;
  uint32_t t = survival_s * 3U;
  return (t < CONFIG_DEADMAN_FLOOR_S) ? CONFIG_DEADMAN_FLOOR_S : t;
}

uint32_t ConfigGetDeadmanTimeoutS(void) {
  const SystemConfig_t *c = Config_Get();
  return DeadmanTimeoutForSurvival((c != NULL) ? c->tx_interval_survival
                                               : 3600000UL);
}

uint16_t ConfigGetFirstFlightBatteryMinMv(void) {
  const SystemConfig_t *c = Config_Get();
  uint16_t configured = (c != NULL) ? c->battery_critical_threshold
                                    : CONFIG_FIRST_FLIGHT_BATTERY_FLOOR_MV;
  /* PWR-SIMPLIFY Gate B: the persisted threshold can only be stricter than
   * the 3800 mV bench-derived floor, never weaker. */
  return (configured < CONFIG_FIRST_FLIGHT_BATTERY_FLOOR_MV)
             ? CONFIG_FIRST_FLIGHT_BATTERY_FLOOR_MV
             : configured;
}

ConfigStatus_t Config_Validate(const SystemConfig_t *config) {
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
  uint32_t calculated_crc = Config_CRC32((const uint8_t *)&temp_config,
                                         sizeof(SystemConfig_t));

  if (calculated_crc != config->crc32) {
    g_crc_failure_count++;
    return CONFIG_ERROR_CRC;
  }

  // Validate ranges (add basic sanity checks)
  if (config->tx_interval_normal < 60000 || config->tx_interval_normal > 3600000) {
    return CONFIG_ERROR_RANGE; // 1 minute to 1 hour
  }

  if (config->battery_low_threshold > config->bulk_battery_min_mv) {
    return CONFIG_ERROR_RANGE; // Bulk threshold must be higher than low
  }

  if (config->link_margin_threshold > 63) {
    return CONFIG_ERROR_RANGE; // LoRaWAN margin is 6-bit field
  }

  /* F-018 (#53): the remaining semantic checks — interval hierarchy and
   * the power/GPS knobs the flight actually depends on. */
  if (!(config->tx_interval_normal <= config->tx_interval_conservative &&
        config->tx_interval_conservative <= config->tx_interval_reduced &&
        config->tx_interval_reduced <= config->tx_interval_recovery &&
        config->tx_interval_recovery <= config->tx_interval_survival)) {
    return CONFIG_ERROR_RANGE; // mode hierarchy must be monotonic
  }
  /* DR-04 (#240): the explicit survival ceiling. The S-04 3/4 rule below is
   * structurally always-true above the 3 h floor (tmo = 3s -> s <= 2.25s),
   * so without this check the only live ceiling was the 24 h one and a 5 h
   * cadence validated while stm32_lpm_if.c's MAX_SLEEP_CHUNKS covered ~2 h -
   * chunked sleep aborted every cycle in the mode that exists to save power.
   * stm32_lpm_if.c derives its chunk bound from the SAME constant. */
  if (config->tx_interval_survival > CONFIG_MAX_TX_INTERVAL_MS) {
    return CONFIG_ERROR_RANGE; // survival cadence exceeds the 2 h ceiling
  }
  /* S-04 (#228): the deadman timeout is DERIVED as 3x the survival cadence
   * (ConfigGetDeadmanTimeoutS), so the margin can no longer collapse at the
   * old fixed 3 h timeout. Keep a sanity ceiling anyway: a survival cadence
   * beyond 3/4 of the derived timeout means the integrator set the cadence
   * where the timeout belongs - set it equal to timeout/3 instead. (24 h
   * and up is rejected outright: the sonde would be unreachable for a day.)
   * Computed in 64-bit: the fields are milliseconds up to uint32 range. */
  {
    uint64_t tmo_ms = (uint64_t)DeadmanTimeoutForSurvival(config->tx_interval_survival) * 1000ULL;
    if (config->tx_interval_survival >= 86400000UL ||
        (uint64_t)config->tx_interval_survival > (tmo_ms * 3ULL) / 4ULL) {
      return CONFIG_ERROR_RANGE; // survival cadence must stay <= 3/4 of the deadman timeout; set it equal to timeout/3 instead
    }
  }
  if (config->gps_temperature_lockout < -80 || config->gps_temperature_lockout > 0) {
    return CONFIG_ERROR_RANGE;
  }
  /* S-07 (#229): the GPS acquisition bounds were never validated - a 0 or
   * tiny value (corrupt write, bad provisioning) silently means 'GPS never
   * attempted'. >= 10 s is the shortest sane bound; the uint8 type caps
   * the top end at 255 s inherently. */
  if (config->gps_timeout_normal < 10 || config->gps_timeout_conservative < 10) {
    return CONFIG_ERROR_RANGE;
  }
  if (!(config->battery_critical_threshold < config->battery_low_threshold &&
        config->battery_low_threshold < config->bulk_battery_min_mv)) {
    return CONFIG_ERROR_RANGE; // critical < low < bulk-min
  }
  /* R12 (#197): frame_counter_save_interval is now config-authoritative
   * (CfgFrameCounterSaveInterval in multiregion_context.c) - a 0 would
   * batch forever and a huge value defeats the R3 margin scheme. */
  if (config->frame_counter_save_interval < 1 ||
      config->frame_counter_save_interval > 100) {
    return CONFIG_ERROR_RANGE;
  }

  return CONFIG_OK;
}

ConfigStatus_t Config_LoadDefaults(void) {
  SONDE_LOG_STR("Loading factory default configuration...\r\n");

  // Clear structure
  memset(&g_config, 0, sizeof(SystemConfig_t));

  // Header
  g_config.magic = CONFIG_MAGIC_NUMBER;
  g_config.version = CONFIG_VERSION;
  g_config.size = sizeof(SystemConfig_t);

  // Transmission settings (from current lora_app.h values)
  g_config.tx_interval_normal = 300000;       // 5 minutes
  g_config.tx_interval_conservative = 600000; // 10 minutes
  g_config.tx_interval_reduced = 900000;      // 15 minutes
  g_config.tx_interval_recovery = 1800000;    // 30 minutes
  g_config.tx_interval_survival = 3600000;    // 60 minutes

  // LoRaWAN parameters
  g_config.lorawan_datarate = 0;           // DR_0 (SF10)
  g_config.lorawan_txpower = 0;            // TX_POWER_0
  g_config.lorawan_adr_enabled = 0;        // ADR disabled for balloon
  g_config.lorawan_confirmed = 0;          // Unconfirmed messages
  g_config.lorawan_class_b_timeout = 8000; // 8 second timeout

  // Power management thresholds
  g_config.battery_low_threshold = 4500; // 4.5V low threshold
  g_config.battery_critical_threshold = CONFIG_FIRST_FLIGHT_BATTERY_FLOOR_MV;
  g_config.bulk_battery_min_mv = 5000;    // 5.0V for bulk transfer
  /* PWR-SIMPLIFY Gate A (2026-08-24): -60°C. Corrected bench CSV (the -55/-60
   * GNSS-loaded readings were swapped) shows the knee between -60 (passes,
   * +0.14 V loaded margin, full charge) and -65 (fails). Revisit after the
   * low-SoC sag re-measurement. */
  g_config.gps_temperature_lockout = -60; // -60°C first-flight admission minimum
  g_config.power_mode_hysteresis = 10;    // 10% hysteresis
  /* F19 FIX: solar_charging_threshold default removed (field deleted -
   * 6000 mV could never trip on the ~1.1 V two-wafer panel; zero consumers) */

  // GPS settings
  g_config.gps_timeout_normal = 60;       // 60 second timeout
  g_config.gps_timeout_conservative = 60; // 60 second timeout
  g_config.gps_min_satellites = 4;        // 4 satellites minimum
  g_config.gps_max_hdop_x10 = 25;         // 2.5 max HDOP
  g_config.gps_standby_power_ua = 15;     // 15µA standby current

  // Adaptive transmission thresholds
  g_config.link_margin_threshold = 15;       // 15dB margin
  g_config.gateway_count_threshold = 2;      // 2 gateways
  g_config.max_bulk_packets = 20;            // 20 packets max per cycle
  g_config.frame_counter_save_interval = 10; // Save every 10 TX
  g_config.bulk_timeout_ms = 60000;          // 1 minute bulk timeout

  // Flash logging settings
  g_config.flash_log_interval = 1;        // Log every sensor read
  g_config.flash_log_enabled = 1;         // Logging enabled
  g_config.flash_log_compression = 0;     // No compression
  g_config.flash_log_retention_days = 30; // 30 day retention

  // Debug settings
  g_config.debug_lpp_enabled = 1;         // LPP debug enabled
  g_config.debug_gnss_detail_enabled = 1; // GNSS debug enabled
  g_config.debug_lpp_interval = 5;        // Every 5th transmission
  g_config.debug_rtt_level = 2;           // Moderate debug level
  g_config.debug_flags = 0x00000001;      // Basic debug flags

  // Calculate CRC32
  g_config.crc32 = Config_CalculateCRC32(&g_config);

  SONDE_LOG_STR("Factory defaults loaded successfully\r\n");

  return CONFIG_OK;
}

ConfigStatus_t Config_GetStats(uint32_t *flash_reads,
                               uint32_t *flash_writes,
                               uint32_t *crc_failures) {
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

ConfigStatus_t Config_PrintCurrent(void) {
  if (!g_config_initialized) {
    SONDE_LOG_STR("Configuration not initialized\r\n");
    return CONFIG_ERROR_PARAM;
  }

  SONDE_LOG_STR("\r\n=== CURRENT CONFIGURATION ===\r\n");

  // Header
  SONDE_LOG("Magic: 0x%08lX, Version: %d, Size: %d\r\n",
            g_config.magic, g_config.version, g_config.size);

  // Transmission settings
  SONDE_LOG("TX Intervals: Normal=%lums Conservative=%lums Reduced=%lums\r\n",
            g_config.tx_interval_normal, g_config.tx_interval_conservative,
            g_config.tx_interval_reduced);
  SONDE_LOG("              Recovery=%lums Survival=%lums\r\n",
            g_config.tx_interval_recovery, g_config.tx_interval_survival);

  // LoRaWAN
  SONDE_LOG("LoRaWAN: DR=%d TxPower=%d ADR=%s Confirmed=%s\r\n",
            g_config.lorawan_datarate, g_config.lorawan_txpower,
            g_config.lorawan_adr_enabled ? "ON" : "OFF",
            g_config.lorawan_confirmed ? "ON" : "OFF");

  // Power management
  SONDE_LOG("Battery: Low=%dmV CriticalCfg=%dmV FirstFlightEffective=%dmV "
            "Bulk=%dmV GPS_Lockout=%d°C\r\n",
            g_config.battery_low_threshold, g_config.battery_critical_threshold,
            ConfigGetFirstFlightBatteryMinMv(), g_config.bulk_battery_min_mv,
            g_config.gps_temperature_lockout);

  // Adaptive transmission
  SONDE_LOG("Adaptive: Margin=%ddB Gateways=%d MaxBulk=%d FCntSave=%d\r\n",
            g_config.link_margin_threshold, g_config.gateway_count_threshold,
            g_config.max_bulk_packets, g_config.frame_counter_save_interval);

  // Debug settings
  SONDE_LOG("Debug: LPP=%s GNSS=%s Interval=%d Level=%d\r\n",
            g_config.debug_lpp_enabled ? "ON" : "OFF",
            g_config.debug_gnss_detail_enabled ? "ON" : "OFF",
            g_config.debug_lpp_interval, g_config.debug_rtt_level);

  SONDE_LOG("CRC32: 0x%08lX\r\n", g_config.crc32);
  SONDE_LOG_STR("=== END CONFIGURATION ===\r\n\r\n");

  return CONFIG_OK;
}

uint32_t Config_CalculateCRC32(const SystemConfig_t *config) {
  if (config == NULL) {
    return 0;
  }

  // Create temporary copy with CRC32 field zeroed
  SystemConfig_t temp_config = *config;
  temp_config.crc32 = 0;

  /* F-003/R35 (#53): hash the FULL zeroed-copy struct. crc32 is at offset
   * 8, so hashing sizeof-4 left the struct tail unprotected — latent until
   * any tail field changed. Both sides bumped together (CONFIG_VERSION 2). */
  return Config_CRC32((const uint8_t *)&temp_config, sizeof(SystemConfig_t));
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Calculate CRC32 using IEEE 802.3 polynomial
 */
static uint32_t Config_CRC32(const uint8_t *data, uint32_t length) {
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
static ConfigStatus_t Config_FlashRead(void) {
  FLASH_IF_StatusTypedef flash_status;

  // Read configuration from flash
  flash_status = FLASH_IF_Read(&g_config, (void *)CONFIG_FLASH_ADDRESS, sizeof(SystemConfig_t));

  if (flash_status != FLASH_IF_OK) {
    SONDE_LOG("Config flash read failed (status: %d)\r\n", flash_status);
    return CONFIG_ERROR_FLASH;
  }

  return CONFIG_OK;
}

/**
 * @brief Write configuration to flash
 */
static ConfigStatus_t Config_FlashWrite(void) {
  FLASH_IF_StatusTypedef flash_status;

  /* FR-04 (#81): verify the write preconditions BEFORE erasing, so a
   * parameter failure cannot destroy the stored config (previously the
   * 99-byte struct always failed FLASH_IF_Write's 64-bit alignment check
   * after the erase had already succeeded). */
  if ((sizeof(SystemConfig_t) % 8U) != 0U ||
      (((uint32_t)CONFIG_FLASH_ADDRESS) % 8U) != 0U ||
      (((uint32_t)(uintptr_t)&g_config) % 8U) != 0U) {
    /* The &g_config term turns a source-misalignment HardFault (LDRD in
     * FLASH_IF_INT_Write) into a clean error return - see the comment on
     * g_config's declaration. It should never fire now that g_config is
     * declared __attribute__((aligned(8))); it is a tripwire, not a fix. */
    SONDE_LOG("Config flash precondition failed (size/alignment)\r\n");
    return CONFIG_ERROR_FLASH;
  }

  // Erase flash page first
  flash_status = FLASH_IF_Erase((void *)CONFIG_FLASH_ADDRESS, CONFIG_FLASH_SIZE);
  if (flash_status != FLASH_IF_OK) {
    SONDE_LOG("Config flash erase failed (status: %d)\r\n", flash_status);
    return CONFIG_ERROR_FLASH;
  }

  // Write configuration
  flash_status = FLASH_IF_Write((void *)CONFIG_FLASH_ADDRESS, &g_config, sizeof(SystemConfig_t));
  if (flash_status != FLASH_IF_OK) {
    SONDE_LOG("Config flash write failed (status: %d)\r\n", flash_status);
    return CONFIG_ERROR_FLASH;
  }

  return CONFIG_OK;
}
