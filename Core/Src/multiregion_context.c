/**
 ******************************************************************************
 * @file    multiregion_context.c
 * @brief   Multi-region LoRaWAN context storage and switching implementation
 ******************************************************************************
 * @attention
 *
 * This module provides context save/restore and seamless switching between
 * LoRaWAN regions (US915, EU868, AS923, etc.) without requiring re-joins.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "multiregion_context.h"
#include "LmHandler.h"
#include "LoRaMac.h"
#include "config.h" /* R12 (#197): frame_counter_save_interval is config-authoritative */
#include "flash_if.h"
#include "lora_app.h"
#include "mission_state.h"
#include "multiregion_h3.h"
#include "se-identity-select.h" /* F6: real keys gitignored; zeroed template fallback for CI */
#include "sys_app.h"
#include "sys_sensors.h"

#include "SEGGER_RTT.h"
#include "sonde_log.h" /* R50 (#47): compile-time log gate */
#include "stm32wlxx_hal.h"
#include "sys_caps.h" /* LT-02/H-04 (#272): radio-degraded marking on rollback failure */
#include <stddef.h>   /* FR-18 (#99): offsetof for the CRC span */
#include <stdio.h>
#include <string.h>

/* R12 (#197): the frame-counter cadence is config-authoritative
 * (SystemConfig_t.frame_counter_save_interval); the macro in
 * multiregion_context.h is only the fallback before Config_Init (R47 NULL
 * precedent). ONE accessor so batching, save margin and restore margin
 * can never disagree. */
static uint8_t CfgFrameCounterSaveInterval(void) {
  const SystemConfig_t *c = Config_Get();
  return c ? c->frame_counter_save_interval : FRAME_COUNTER_SAVE_INTERVAL;
}

/* F-R3 (#73): per-region identity + radio defaults as ONE table — replaces
 * the two copy-pasted DevEUI switches (JoinRegion and
 * InitializeRegionFromNetworkServer). H-07 (#274): hoisted above the
 * capture path, which now uses it too - the local switch there omitted
 * RU864, and two lists can never drift again. */
typedef struct {
  LoRaMacRegion_t region;
  uint8_t dev_eui[8];
  uint8_t datarate;
  uint32_t rx2_frequency;
  uint8_t rx2_datarate;
} RegionIdentity_t;

static const RegionIdentity_t kRegionIdentities[] = {
    {LORAMAC_REGION_US915, {LORAWAN_DEVICE_EUI_US915}, DR_2, 923300000, DR_8},
    {LORAMAC_REGION_EU868, {LORAWAN_DEVICE_EUI_EU868}, DR_0, 869525000, DR_0},
    {LORAMAC_REGION_AS923, {LORAWAN_DEVICE_EUI_AS923}, DR_2, 923200000, DR_2},
    {LORAMAC_REGION_AU915, {LORAWAN_DEVICE_EUI_AU915}, DR_2, 923300000, DR_8},
    {LORAMAC_REGION_IN865, {LORAWAN_DEVICE_EUI_IN865}, DR_0, 866550000, DR_2},
    {LORAMAC_REGION_KR920, {LORAWAN_DEVICE_EUI_KR920}, DR_0, 921900000, DR_0},
    /* SP-05 (#246): RU864 in-band (864-868 MHz). EU433/CN470 stay unmapped -
     * outside the radio's 850-950 MHz band (lorawan_conf.h keeps them out). */
    {LORAMAC_REGION_RU864, {LORAWAN_DEVICE_EUI_RU864}, DR_0, 869100000, DR_0},
};

static const RegionIdentity_t *FindRegionIdentity(LoRaMacRegion_t region) {
  for (uint32_t i = 0; i < (sizeof(kRegionIdentities) / sizeof(kRegionIdentities[0])); i++) {
    if (kRegionIdentities[i].region == region) {
      return &kRegionIdentities[i];
    }
  }
  return NULL;
}

/* Private defines -----------------------------------------------------------*/
/* Two-tier session storage (FW-1 / DDR-0018).
 * Flash map (2KB pages; reserved region is now pages 120-127 = 16KB):
 *   0x0803C000  page 120 - Tier-1 credentials copy A
 *   0x0803C800  page 121 - Tier-1 credentials copy B
 *   0x0803D000  page 122 - Tier-1 credentials copy C
 *   0x0803D800  page 123 - Tier-2 counters slot A (ping-pong)
 *   0x0803E000  page 124 - Tier-2 counters slot B (ping-pong)
 *   0x0803E800  page 125 - System configuration     (config module)
 *   0x0803F000  page 126 - LoRaWAN NVM context      (LORAWAN_NVM_BASE_ADDRESS)
 *   0x0803F800  page 127 - LoRaWAN NVM slot B (F-016/#54 ping-pong store)
 *
 * RULE (mirrors backup_regs.h): any new flash-page user must add its
 * allocation to this map AND the mirror in config.h first (FR-21/#102).
 *
 * Tier-1 holds immutable join credentials (DevAddr/DevEUI/session keys) as
 * three redundant, independently CRC'd copies written once at commissioning
 * and never erased in flight. Tier-2 holds only the dynamic frame counters,
 * ping-ponged between two slots with erase-before-write, so a brownout
 * mid-save can never take the credentials with it. */
#define MULTIREGION_FLASH_PAGE_SIZE 2048

#define TIER1_NUM_COPIES 3
#define TIER2_NUM_SLOTS 2

#define TIER1_MAGIC 0x54314D52UL /* 'T1MR' */
#define TIER2_MAGIC 0x54324D52UL /* 'T2MR' */
/* C-01 (#270): PROVISIONED latch magic ('PROV'). A magic, not a bool:
 * distinct from 0 and 0xFFFFFFFF so a zeroed struct and erased flash both
 * read as "not provisioned". */
#define TIER1_PROVISIONED_MAGIC 0x50524F56UL /* 'PROV' */

/* Private types -------------------------------------------------------------*/

/**
 * @brief Tier-1 credential bank: one full copy of all region contexts.
 *        Written once at commissioning to three separate flash pages.
 */
typedef struct {
  uint32_t magic;      // TIER1_MAGIC
  uint16_t version;    // MULTIREGION_VERSION
  uint8_t num_valid;   // How many contexts are joined
  uint8_t active_slot; // Active slot at commissioning time
  uint32_t generation; // R8 (#190): monotonic; newest valid copy wins
  MinimalRegionContext_t contexts[MAX_REGION_CONTEXTS];
  /* C-01 (#270): durable PROVISIONED latch. Before crc32 so the existing
   * whole-bank CRC span covers it; TIER1_PROVISIONED_MAGIC when set. */
  uint32_t provisioned;
  uint32_t crc32; // Whole-bank validation
} __attribute__((aligned(8))) Tier1Bank_t;

/**
 * @brief Tier-2 per-region dynamic entry (frame counters + radio state)
 */
typedef struct {
  uint32_t uplink_counter;
  uint32_t downlink_counter;
  uint32_t last_rx_mic;
  uint32_t last_used;
  uint8_t datarate;
  int8_t tx_power;
  uint8_t adr_enabled;
  uint8_t valid; // Entry carries live counter data
} Tier2Entry_t;

/**
 * @brief Tier-2 counter bank: dynamic state only, ping-ponged between two
 *        flash slots. The monotonic sequence picks the newest valid slot
 *        on restore.
 */
typedef struct {
  uint32_t magic;      // TIER2_MAGIC
  uint16_t version;    // MULTIREGION_VERSION
  uint8_t active_slot; // Most recently active slot
  uint8_t reserved;
  uint32_t sequence; // Monotonic; newest slot wins
  Tier2Entry_t entries[MAX_REGION_CONTEXTS];
  uint32_t crc32; // Whole-bank validation
} __attribute__((aligned(8))) Tier2Bank_t;

/* Compile-time checks: each bank must fit in one 2KB flash page */
_Static_assert(sizeof(Tier1Bank_t) <= MULTIREGION_FLASH_PAGE_SIZE,
               "Tier-1 bank must fit in one flash page");
_Static_assert(sizeof(Tier2Bank_t) <= MULTIREGION_FLASH_PAGE_SIZE,
               "Tier-2 bank must fit in one flash page");
/* FR-04 (#81): both banks go through FLASH_IF_Write, which rejects lengths
 * that are not 64-bit aligned. aligned(8) on the type makes sizeof a
 * multiple of 8 today; pin it so a future field cannot reintroduce the
 * config.c erase-then-fail class of bug. */
_Static_assert(sizeof(Tier1Bank_t) % 8U == 0U,
               "Tier1Bank_t must be 8-byte aligned for FLASH_IF_Write");
_Static_assert(sizeof(Tier2Bank_t) % 8U == 0U,
               "Tier2Bank_t must be 8-byte aligned for FLASH_IF_Write");
/* FR-18 (#99): the context CRC covers offsetof(MinimalRegionContext_t, crc16)
 * bytes. Pin the layout so an added/reordered member or an enum-width change
 * fails at compile time instead of silently shifting the CRC span. */
_Static_assert(offsetof(MinimalRegionContext_t, crc16) == 76,
               "context CRC span assumes crc16 at offset 76");
_Static_assert(sizeof(MinimalRegionContext_t) == 80,
               "MinimalRegionContext_t layout drift");

/* Private variables ---------------------------------------------------------*/
static MultiRegionStorage_t g_storage;
static bool g_initialized = false;
static uint8_t g_flash_buffer[MULTIREGION_FLASH_PAGE_SIZE] __attribute__((aligned(8)));

/* Two-tier persistence state */
static const uint32_t TIER1_ADDRS[TIER1_NUM_COPIES] = {0x0803C000UL, 0x0803C800UL, 0x0803D000UL};
static const uint32_t TIER2_ADDRS[TIER2_NUM_SLOTS] = {0x0803D800UL, 0x0803E000UL};
static uint32_t g_t2_sequence = 0;   // Last Tier-2 sequence written/read
static int8_t g_t2_last_slot = -1;   // Last Tier-2 slot written (-1 = none)
static bool g_tier1_dirty = false;   // Static credentials changed, Tier-1 rewrite needed
static uint32_t g_t1_generation = 0; // R8 (#190): last Tier-1 generation read/written
static uint32_t g_provisioned = 0;   // C-01 (#270): RAM mirror of the Tier-1 PROVISIONED latch

/* F-R3 (#73) / SP-05 (#246) / C-01 (#270): the required-region set, at file
 * scope so the pre-join ceremony and the provisioning-latch checks share
 * one list - they can never drift apart. */
static const LoRaMacRegion_t kPreJoinRegions[] = {
    LORAMAC_REGION_US915,
    LORAMAC_REGION_EU868,
    LORAMAC_REGION_AS923,
    LORAMAC_REGION_AU915,
    LORAMAC_REGION_IN865,
    LORAMAC_REGION_KR920,
    LORAMAC_REGION_RU864,
};
#define NUM_PREJOIN_REGIONS ((uint8_t)(sizeof(kPreJoinRegions) / sizeof(kPreJoinRegions[0])))

/* Batched frame counter save infrastructure */
static uint8_t g_unsaved_tx_count = 0; // Track unsaved successful transmissions

/* External variables --------------------------------------------------------*/
extern LmHandlerParams_t LmHandlerParams;

/* Private function prototypes -----------------------------------------------*/
static uint16_t CalculateCRC16(const uint8_t *data, uint32_t length);
static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length);
static bool ValidateContextCRC(MinimalRegionContext_t *ctx);
static void UpdateContextCRC(MinimalRegionContext_t *ctx);
static bool FlashReadStorage(void);
static bool FlashWriteStorage(void);
static bool FlashReadTier1(Tier1Bank_t *out);
static bool FlashWriteTier1(void);
static bool VerifyAndSetProvisioningLatch(void); /* C-01 (#270) */
static bool FlashReadTier2(Tier2Bank_t *out);
static bool FlashWriteTier2(void);
static int8_t FindContextSlot(LoRaMacRegion_t region);
static bool CaptureCurrentContext(MinimalRegionContext_t *ctx);
static bool ValidateContextSemantics(const MinimalRegionContext_t *ctx); /* R2 (#189) */
const char *RegionToString(LoRaMacRegion_t region);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize the multi-region manager
 */
void MultiRegion_Init(void) {
  if (g_initialized) {
    return;
  }

  SONDE_LOG_STR("\r\n=== MultiRegion_Init ===\r\n");
  APP_LOG(TS_ON, VLEVEL_H, "\r\n=== MultiRegion_Init ===\r\n");

  // Initialize flash interface
  FLASH_IF_Init(g_flash_buffer);

  // Try to restore contexts from flash (Tier-1 credentials + Tier-2 counters)
  if (FlashReadStorage()) {
    APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Restored %d contexts from flash\r\n",
            g_storage.num_valid);
    g_initialized = true;
    return;
  }

  APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: No valid Tier-1 bank, initializing fresh\r\n");

  // Initialize fresh storage
  memset(&g_storage, 0, sizeof(g_storage));
  g_storage.magic = MULTIREGION_MAGIC;
  g_storage.version = MULTIREGION_VERSION;
  g_storage.active_slot = 0xFF; // No active region yet
  g_storage.num_valid = 0;
  g_t2_sequence = 0;
  g_t2_last_slot = -1;
  g_tier1_dirty = false;
  g_provisioned = 0; // C-01 (#270): fresh storage is never provisioned

  // Note: We rely on DevAddr==0 to detect empty slots, not region value
  // memset already zeroed everything, which is perfect for our needs

  g_initialized = true;
  SONDE_LOG_STR("MultiRegion: Initialized with fresh storage (DevAddr-based validation)\r\n");
  APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Initialized with fresh storage\r\n");
}

/**
 * @brief Get current active region
 */
LoRaMacRegion_t MultiRegion_GetActiveRegion(void) {
  if (!g_initialized || g_storage.active_slot >= MAX_REGION_CONTEXTS) {
    return LORAMAC_REGION_US915; // Default fallback
  }

  return g_storage.contexts[g_storage.active_slot].region;
}

/**
 * @brief Check if a region has a valid joined context
 */
bool MultiRegion_IsRegionJoined(LoRaMacRegion_t region) {
  if (!g_initialized) {
    return false;
  }

  int8_t slot = FindContextSlot(region);
  if (slot < 0) {
    return false;
  }

  MinimalRegionContext_t *ctx = &g_storage.contexts[slot];

  // Validate: must have valid DevAddr and pass CRC check
  if (ctx->dev_addr == 0 || ctx->dev_addr == 0xFFFFFFFF) {
    return false;
  }

  /* R2 (#189): CRC-valid is not enough - the context must also be
   * semantically a real session. */
  return ValidateContextCRC(ctx) && ValidateContextSemantics(ctx);
}

/**
 * @brief Save current active context to flash (with batching for flash endurance)
 */
bool MultiRegion_SaveCurrentContext(void) {
  SONDE_LOG_STR("\r\n=== MultiRegion_SaveCurrentContext START (BATCHED) ===\r\n");
  APP_LOG(TS_ON, VLEVEL_M, "\r\n=== MultiRegion_SaveCurrentContext START (BATCHED) ===\r\n");

  if (!g_initialized) {
    SONDE_LOG_STR("ERROR: Not initialized, cannot save\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Not initialized, cannot save\r\n");
    return false;
  }

  // Increment unsaved transmission counter
  g_unsaved_tx_count++;
  SONDE_LOG("Unsaved TX count: %d/%d\r\n", g_unsaved_tx_count, CfgFrameCounterSaveInterval());

  // Check if we should save to flash (every N successful transmissions)
  if (g_unsaved_tx_count < CfgFrameCounterSaveInterval()) {
    SONDE_LOG_STR("Batching: Skipping flash save (interval not reached)\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Batched save %d/%d\r\n",
            g_unsaved_tx_count, CfgFrameCounterSaveInterval());
    return true; // Return success, just didn't write to flash
  }

  // Perform the actual save
  /* FR-15 (#293): the counter reset moved into ForceSave (success only) -
   * clearing it here meant a failed save read as "0 unsaved TXs" and the
   * next checkpoint was a full interval away. */
  SONDE_LOG("Batching: Performing flash save (reached %d TXs)\r\n", CfgFrameCounterSaveInterval());

  return MultiRegion_ForceSaveCurrentContext();
}

/**
 * @brief Force immediate save of current active context to flash (bypasses batching)
 */
bool MultiRegion_ForceSaveCurrentContext(void) {
  SONDE_LOG_STR("\r\n=== MultiRegion_ForceSaveCurrentContext START ===\r\n");
  APP_LOG(TS_ON, VLEVEL_M, "\r\n=== MultiRegion_ForceSaveCurrentContext START ===\r\n");

  if (!g_initialized) {
    SONDE_LOG_STR("ERROR: Not initialized, cannot save\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Not initialized, cannot save\r\n");
    return false;
  }

  SONDE_LOG_STR("Checking network activation status...\r\n");

  // Find or create slot for current region
  MibRequestConfirm_t mib;
  mib.Type = MIB_NETWORK_ACTIVATION;
  LoRaMacMibGetRequestConfirm(&mib);

  if (mib.Param.NetworkActivation == ACTIVATION_TYPE_NONE) {
    SONDE_LOG_STR("ERROR: Not joined, cannot save context\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Not joined, cannot save context\r\n");
    return false;
  }

  SONDE_LOG_STR("Network is activated, proceeding...\r\n");

  LoRaMacRegion_t current_region = LmHandlerParams.ActiveRegion;
  SONDE_LOG("Current region: %s\r\n", RegionToString(current_region));

  int8_t slot = FindContextSlot(current_region);

  if (slot < 0) {
    SONDE_LOG_STR("No existing slot found, searching for empty slot...\r\n");
    // Find empty slot - relies purely on DevAddr (cleaner approach)
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
      // DevAddr==0 means empty - region value doesn't matter
      if (g_storage.contexts[i].dev_addr == 0 ||
          g_storage.contexts[i].dev_addr == 0xFFFFFFFF) {
        slot = (int8_t)i;
        g_storage.num_valid++;
        SONDE_LOG("Found empty slot: %d\r\n", i);
        break;
      }
    }
  } else {
    SONDE_LOG("Using existing slot: %d\r\n", slot);
  }

  if (slot < 0) {
    SONDE_LOG_STR("ERROR: No available slots\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: No available slots\r\n");
    return false;
  }

  // Get context pointer
  MinimalRegionContext_t *ctx = &g_storage.contexts[slot];

  // CRITICAL FIX: If this is a new slot (DevAddr==0), initialize static fields FIRST
  // CaptureCurrentContext() only updates dynamic fields
  if (ctx->dev_addr == 0 || ctx->dev_addr == 0xFFFFFFFF) {
    SONDE_LOG_STR("New slot (DevAddr=0) - initializing static fields from MAC...\r\n");

    // Set region FIRST
    ctx->region = current_region;
    SONDE_LOG("  Set ctx->region = %s\r\n", RegionToString(current_region));

    // Get DevAddr from MAC
    mib.Type = MIB_DEV_ADDR;
    LoRaMacMibGetRequestConfirm(&mib);
    ctx->dev_addr = mib.Param.DevAddr;
    SONDE_LOG("  Set ctx->dev_addr = 0x%08lX\r\n", ctx->dev_addr);

    // Set DevEUI based on region (safer than querying MAC during join)
    /* H-07 (#274): from the identity table - the local switch omitted
     * RU864 (#246), so a session captured on RU864 returned false and
     * that bank could never become usable. */
    const RegionIdentity_t *ident = FindRegionIdentity(current_region);
    if (!ident) {
      SONDE_LOG("  ERROR: region %s has no identity row - cannot capture\r\n",
                RegionToString(current_region));
      return false;
    }
    memcpy(ctx->dev_eui, ident->dev_eui, 8);
    SONDE_LOG_STR("  Set ctx->dev_eui from region identity table\r\n");

    // CRITICAL FIX: Get session keys from ACTIVE crypto context using LmHandler API
    // After OTAA join, keys exist in active session but may not be in NVM yet
    // Reading from NVM was causing zeros because NVM write happens later
    uint8_t temp_key[16];

    if (LmHandlerGetKey(APP_S_KEY, temp_key) == LORAMAC_HANDLER_SUCCESS) {
      memcpy(ctx->app_s_key, temp_key, 16);
      SONDE_LOG_STR("  Set AppSKey from active session (via LmHandler API)\r\n");
    } else {
      SONDE_LOG_STR("  ERROR: Failed to get AppSKey!\r\n");
      ctx->dev_addr = 0; /* F-008 (#205): fail CLOSED - missing key material must never persist (DevAddr==0 is the empty-slot marker) */
      return false;
    }

    if (LmHandlerGetKey(NWK_S_KEY, temp_key) == LORAMAC_HANDLER_SUCCESS) {
      memcpy(ctx->nwk_s_key, temp_key, 16);
      SONDE_LOG_STR("  Set NwkSKey from active session (via LmHandler API)\r\n");
    } else {
      SONDE_LOG_STR("  ERROR: Failed to get NwkSKey!\r\n");
      ctx->dev_addr = 0; /* F-008 (#205): fail CLOSED - missing key material must never persist (DevAddr==0 is the empty-slot marker) */
      return false;
    }

    // FW-1: static credentials changed - Tier-1 bank rewrite required (commissioning-only)
    g_tier1_dirty = true;
  }

  // Now capture dynamic context (frame counters, datarate, etc.)
  SONDE_LOG_STR("Calling CaptureCurrentContext for dynamic fields...\r\n");
  if (!CaptureCurrentContext(ctx)) {
    SONDE_LOG_STR("ERROR: Failed to capture context\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Failed to capture context\r\n");
    return false;
  }

  SONDE_LOG_STR("Context captured successfully\r\n");

  g_storage.active_slot = (uint8_t)slot;

  /*
   * FRAME COUNTER MARGIN (C6 fix) — DR-07 (#239): the margin lives on the
   * RESTORE side only. It used to be applied here too, persisting an
   * already-margined value that the restore then margined AGAIN: every
   * reset burned INTERVAL of FCntUp gap whether or not an uplink occurred,
   * and DDR-0006 forbids the in-flight rejoin that could re-sync. The save
   * persists the TRUE counter; the restore (and the Tier-2-lost degrade
   * path) adds exactly one INTERVAL — the reset creates the exposure, not
   * the save. The live MAC counter is never modified.
   *
   * FR-05 (#82): ctx points into g_storage.contexts[] and the capture above
   * rewrote CRC-covered fields, so restamp before the write — a stale CRC
   * fails ValidateContextCRC() and a later switch-back to that region is
   * silently refused (IsRegionJoined CRC check).
   */
  UpdateContextCRC(ctx);

  // Save to flash
  SONDE_LOG_STR("Calling FlashWriteStorage...\r\n");
  bool result = FlashWriteStorage();

  if (result) {
    /* FR-15 (#293): reset the unsaved-TX count only AFTER a successful
     * write. It used to be cleared BEFORE FlashWriteStorage: a failed
     * save then read as "0 unsaved TXs", the next checkpoint was a full
     * interval away, and a reset in that window restored an FCntUp up to
     * INTERVAL behind the true network counter - the NS rejects reused
     * frames. Keeping the count makes the next SaveCurrentContext retry
     * the save immediately. */
    g_unsaved_tx_count = 0;
    SONDE_LOG_STR("Flash write successful!\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Saved %s context (slot %d)\r\n",
            RegionToString(current_region), slot);
  } else {
    SONDE_LOG_STR("ERROR: Flash write failed!\r\n");
  }

  SONDE_LOG_STR("=== MultiRegion_SaveCurrentContext END ===\r\n\r\n");

  return result;
}

/**
 * @brief Save all contexts to flash
 */
bool MultiRegion_SaveAllContexts(void) {
  if (!g_initialized) {
    return false;
  }

  return FlashWriteStorage();
}

/**
 * @brief Restore all contexts from flash on boot
 */
bool MultiRegion_RestoreContexts(void) {
  if (!g_initialized) {
    MultiRegion_Init();
  }

  return g_storage.num_valid > 0;
}

/* F-R2 (#75): region channel masks as data, not copy-pasted if/else chains. */
typedef struct {
  LoRaMacRegion_t region;
  uint16_t mask[6];
  uint8_t len;      /* number of valid mask words */
  const char *note; /* human-readable description for logs */
} RegionChannelMask_t;

static const RegionChannelMask_t kChannelMasks[] = {
    /* Helium sub-band 2: channels 8-15 + 500kHz channel 64 */
    {LORAMAC_REGION_US915, {0xFF00, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000}, 6, "US915 sub-band 2 (ch 8-15 + ch 64)"},
    /* Standard channels 0-7 */
    {LORAMAC_REGION_EU868, {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}, 1, "EU868 standard channels 0-7"},
};

/**
 * @brief Apply the region's channel mask (and default mask) to the MAC.
 *        During OTAA join the network configures channels; when switching to
 *        ABP we must restore them. Regions without a table entry keep MAC
 *        defaults (same as before F-R2).
 */
/* FR-17 (#294): returns false when a required mask MIB set fails so the
 * restore fails closed (R1/#187 contract) instead of reporting SUCCESS on a
 * stack whose bank masks were never applied. Regions without a table entry
 * keep the MAC defaults by design - their restore needs no bank mask. */
static bool ApplyRegionChannelMask(LoRaMacRegion_t region) {
  for (uint32_t i = 0; i < (sizeof(kChannelMasks) / sizeof(kChannelMasks[0])); i++) {
    if (kChannelMasks[i].region != region) {
      continue;
    }
    MibRequestConfirm_t mib_ch;
    mib_ch.Type = MIB_CHANNELS_MASK;
    mib_ch.Param.ChannelsMask = (uint16_t *)kChannelMasks[i].mask;
    if (LoRaMacMibSetRequestConfirm(&mib_ch) != LORAMAC_STATUS_OK) {
      SONDE_LOG("ERROR: Failed to set channel mask for %s\r\n", RegionToString(region));
      return false;
    }
    mib_ch.Type = MIB_CHANNELS_DEFAULT_MASK;
    mib_ch.Param.ChannelsDefaultMask = (uint16_t *)kChannelMasks[i].mask;
    if (LoRaMacMibSetRequestConfirm(&mib_ch) != LORAMAC_STATUS_OK) {
      SONDE_LOG("ERROR: Failed to set default channel mask for %s\r\n", RegionToString(region));
      return false;
    }
    SONDE_LOG("Channel mask applied: %s\r\n", kChannelMasks[i].note);
    return true;
  }
  return true; /* no table entry: MAC defaults are the intended masks */
}
/* F-R3 (#73) / H-07 (#274): the identity table + lookup live at the top of
 * the file, shared by JoinRegion, InitializeRegionFromNetworkServer AND the
 * capture path. */

/**
 * @brief F-R2 (#75): restore a banked session context into the MAC — the
 *        sequenced ritual formerly inlined in MultiRegion_SwitchToRegion.
 *        Shared with the commissioning join paths (#73).
 *
 *        The fixed HAL_Delay()s are INTENTIONAL: there is no reliable
 *        ready-predicate for stack/radio settling (LoRaMacIsBusy only covers
 *        the post-start settle, which is polled below), and this path runs
 *        mid-flight. Do not convert them to poll loops without a real
 *        predicate. DevAddr is deliberately programmed twice (before and
 *        after LoRaMacStart) because MAC start can overwrite it.
 *
 *        Session key material is never logged (#75, fdb971f).
 *
 * @retval LORAMAC_HANDLER_SUCCESS     session restored and verified
 * @retval LORAMAC_HANDLER_BUSY_ERROR  MAC still busy after the settle loop
 */
static LmHandlerErrorStatus_t RestoreSessionToMac(MinimalRegionContext_t *ctx, LoRaMacRegion_t region) {
  SONDE_LOG("Restoring session context for %s\r\n", RegionToString(region));

  // STEP 1: Reinitialize stack (loads zeros from se-identity.h)
  if (LoRaApp_ReInitStack(region) != LORAMAC_HANDLER_SUCCESS) {
    /* F-01 (#245): fail closed like R1's steps 4-10 */
    SONDE_LOG_STR("  ERROR: stack reinit failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  HAL_Delay(100);

  // STEP 2: Configure the handler (this might load from NVM)
  if (LmHandlerConfigure(&LmHandlerParams) != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("  ERROR: LmHandlerConfigure failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  HAL_Delay(50);

  // STEP 3: NOW set identity and keys AFTER configure (to override NVM restore)
  if (LmHandlerSetDevEUI(ctx->dev_eui) != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("  ERROR: SetDevEUI failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  if (LmHandlerSetKey(APP_S_KEY, (uint8_t *)ctx->app_s_key) != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("  ERROR: SetKey(AppSKey) failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  if (LmHandlerSetKey(NWK_S_KEY, (uint8_t *)ctx->nwk_s_key) != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("  ERROR: SetKey(NwkSKey) failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  /* R1 (#187): FAIL CLOSED. Every required step is checked; any failure
   * aborts the restore with LORAMAC_HANDLER_ERROR so the caller never
   * marks the region active on a dead session (the banked context stays
   * valid - a later cycle can retry). Previously these failures were log
   * lines and the function fell through to SUCCESS. (F-01 #245 extended
   * the same contract to steps 1-3, which were still bare.) */
  // STEP 4: Set DevAddr and activation via MIB before channel mask
  MibRequestConfirm_t mib;
  mib.Type = MIB_DEV_ADDR;
  mib.Param.DevAddr = ctx->dev_addr;
  if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK) {
    SONDE_LOG_STR("  ERROR: DevAddr MIB set failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  mib.Type = MIB_NETWORK_ACTIVATION;
  mib.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
  if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK) {
    SONDE_LOG_STR("  ERROR: activation MIB set failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  // STEP 5: Restore frame counters and keys into NVM
  mib.Type = MIB_NVM_CTXS;
  LoRaMacMibGetRequestConfirm(&mib);
  LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t *)mib.Param.Contexts;
  if (nvm == NULL) {
    SONDE_LOG_STR("  ERROR: NVM contexts unavailable - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  {
    nvm->Crypto.FCntList.FCntUp = ctx->uplink_counter;
    nvm->Crypto.FCntList.NFCntDown = ctx->downlink_counter;
    nvm->MacGroup1.LastRxMic = ctx->last_rx_mic;
    nvm->MacGroup2.NetworkActivation = ACTIVATION_TYPE_ABP;
    /* LmHandlerSetKey() should have done this; belt-and-braces copy kept
     * from the pre-refactor code. Key material is never logged (#75). */
    memcpy(nvm->SecureElement.KeyList[APP_S_KEY].KeyValue, ctx->app_s_key, 16);
    memcpy(nvm->SecureElement.KeyList[NWK_S_KEY].KeyValue, ctx->nwk_s_key, 16);

    SONDE_LOG("Context restored: %s DevAddr=0x%08lX FCntUp=%lu FCntDown=%lu\r\n",
              RegionToString(ctx->region), ctx->dev_addr,
              ctx->uplink_counter, ctx->downlink_counter);
  }

  mib.Type = MIB_NETWORK_ACTIVATION;
  mib.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
  if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK) {
    SONDE_LOG_STR("  ERROR: activation re-set failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  // STEP 6: region channel masks, table-driven (F-R2)
  /* FR-17 (#294): fail closed like every other restore step - a stack on
   * the wrong bank masks is not a successful restore. Feeds the LT-02
   * (#272) rollback path. */
  if (!ApplyRegionChannelMask(region)) {
    SONDE_LOG_STR("  ERROR: channel mask apply failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  // STEP 7: Start MAC and allow state machine to stabilize
  LoRaMacStart();
  HAL_Delay(200);

  // STEP 8: Set DevAddr via MIB AFTER LoRaMacStart() to ensure it persists
  mib.Type = MIB_DEV_ADDR;
  mib.Param.DevAddr = ctx->dev_addr;
  if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK) {
    /* R1 (#187): was a log line, then SUCCESS. Fail closed instead. */
    SONDE_LOG_STR("  ERROR: Failed to set DevAddr - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  // STEP 9: Process MAC events to complete initialization
  for (int i = 0; i < 10; i++) {
    LmHandlerProcess();
    HAL_Delay(10);
  }

  if (LoRaMacIsBusy()) {
    SONDE_LOG_STR("  WARNING: MAC still busy, giving more time...\r\n");
    HAL_Delay(500);
    for (int i = 0; i < 20; i++) {
      LmHandlerProcess();
      HAL_Delay(10);
    }
  }

  if (LoRaMacIsBusy()) {
    SONDE_LOG_STR("  ERROR: MAC is busy after initialization!\r\n");
    return LORAMAC_HANDLER_BUSY_ERROR;
  }

  /* R11 (#196): capture/restore symmetry. CaptureCurrentContext records
   * datarate/tx_power/ADR/RX2 alongside the counters - the restore must
   * recreate them, or a switched/reset session silently runs on default
   * radio params (airtime and link-budget drift). Persistence contract:
   * Model B - Tier-1/2 own credentials + counters + these radio params;
   * everything else is LoRaMac NVM's job. */
  /* H-06 (#272): these four writes ARE the airtime/link-budget contract -
   * a failed write must abort the restore like every other required step,
   * not silently run defaults (the R11/#196 defect class). */
  mib.Type = MIB_CHANNELS_DATARATE;
  mib.Param.ChannelsDatarate = (int8_t)ctx->datarate;
  if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK) {
    SONDE_LOG_STR("  ERROR: MIB_CHANNELS_DATARATE set failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  mib.Type = MIB_CHANNELS_TX_POWER;
  mib.Param.ChannelsTxPower = (int8_t)ctx->tx_power;
  if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK) {
    SONDE_LOG_STR("  ERROR: MIB_CHANNELS_TX_POWER set failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  mib.Type = MIB_ADR;
  mib.Param.AdrEnable = (ctx->adr_enabled != 0);
  if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK) {
    SONDE_LOG_STR("  ERROR: MIB_ADR set failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  mib.Type = MIB_RX2_CHANNEL;
  mib.Param.Rx2Channel.Frequency = ctx->rx2_frequency;
  mib.Param.Rx2Channel.Datarate = ctx->rx2_datarate;
  if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK) {
    SONDE_LOG_STR("  ERROR: MIB_RX2_CHANNEL set failed - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  // STEP 10: Verify DevAddr and session keys landed in the secure element
  MibRequestConfirm_t verify_mib;
  verify_mib.Type = MIB_DEV_ADDR;
  LoRaMacMibGetRequestConfirm(&verify_mib);
  if (verify_mib.Param.DevAddr != ctx->dev_addr) {
    /* R1 (#187): verification failure aborts the restore - never report
     * success on a session that did not land. */
    SONDE_LOG("  ERROR: DevAddr mismatch! MAC=0x%08lX, Expected=0x%08lX - restore aborted\r\n",
              verify_mib.Param.DevAddr, ctx->dev_addr);
    return LORAMAC_HANDLER_ERROR;
  }

  verify_mib.Type = MIB_NVM_CTXS;
  LoRaMacMibGetRequestConfirm(&verify_mib);
  LoRaMacNvmData_t *verify_nvm = (LoRaMacNvmData_t *)verify_mib.Param.Contexts;
  if (verify_nvm == NULL) {
    SONDE_LOG_STR("  ERROR: NVM verify unavailable - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  if (memcmp(verify_nvm->SecureElement.KeyList[APP_S_KEY].KeyValue, ctx->app_s_key, 16) != 0) {
    SONDE_LOG_STR("  ERROR: AppSKey mismatch in secure element - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  if (memcmp(verify_nvm->SecureElement.KeyList[NWK_S_KEY].KeyValue, ctx->nwk_s_key, 16) != 0) {
    SONDE_LOG_STR("  ERROR: NwkSKey mismatch in secure element - restore aborted\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  return LORAMAC_HANDLER_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* LT-02/H-04/H-06 (#272): rollback outcome query for the caller        */
/* ------------------------------------------------------------------ */
static bool g_last_switch_rollback_ok = true; /* no failed switch yet */

bool MultiRegion_LastSwitchRollbackOk(void) {
  return g_last_switch_rollback_ok;
}

/**
 * @brief LT-02/H-04 (#272): roll back to the previously-active region after
 *        a failed switch. Re-runs the restore ritual for the old context;
 *        RestoreSessionToMac's STEP 10 DevAddr/key verification makes the
 *        rollback self-verifying (success means the session really landed).
 */
static LmHandlerErrorStatus_t RollbackToRegion(uint8_t slot, LoRaMacRegion_t region) {
  MinimalRegionContext_t *ctx = &g_storage.contexts[slot];
  SONDE_LOG("Rolling back to previous region %s (slot %d)\r\n",
            RegionToString(region), slot);
  return RestoreSessionToMac(ctx, region);
}

/**
 * @brief Switch to a different region context
 */
LmHandlerErrorStatus_t MultiRegion_SwitchToRegion(LoRaMacRegion_t region) {
  // Debug entry
  SONDE_LOG("\r\n>>> MultiRegion_SwitchToRegion() called for %s\r\n", RegionToString(region));

  if (!g_initialized) {
    SONDE_LOG_STR("ERROR: Not initialized, returning error\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Not initialized\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  // Debug current state
  SONDE_LOG("Current active_slot: %d, Current region: %s\r\n",
            g_storage.active_slot,
            g_storage.active_slot < MAX_REGION_CONTEXTS ? RegionToString(g_storage.contexts[g_storage.active_slot].region) : "NONE");

  // Check if already on this region AND MAC is actually joined
  // On boot, even though we're on the target region, MAC isn't joined yet so we need to restore
  if (g_storage.active_slot < MAX_REGION_CONTEXTS &&
      g_storage.contexts[g_storage.active_slot].region == region &&
      LmHandlerJoinStatus() == LORAMAC_HANDLER_SET) {
    SONDE_LOG_STR("Already on target region with active MAC session, skipping restore\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Already on %s with active session\r\n", RegionToString(region));
    return LORAMAC_HANDLER_SUCCESS;
  }

  // Find target context
  int8_t slot = FindContextSlot(region);
  if (slot < 0) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Region %s not joined\r\n", RegionToString(region));
    return LORAMAC_HANDLER_ERROR;
  }

  MinimalRegionContext_t *ctx = &g_storage.contexts[slot];
  if (!ValidateContextCRC(ctx) || !ValidateContextSemantics(ctx)) { /* R2 (#189) */
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Context validation failed\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  // Check if MAC is busy
  if (LoRaMacIsBusy()) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: MAC busy, cannot switch\r\n");
    return LORAMAC_HANDLER_BUSY_ERROR;
  }

  SONDE_LOG("\r\n=== Switching to %s (slot %d) ===\r\n",
            RegionToString(region), slot);
  APP_LOG(TS_ON, VLEVEL_H, "\r\n=== Switching to %s (slot %d) ===\r\n",
          RegionToString(region), slot);

  /* F-R2 (#75): the restore ritual lives in RestoreSessionToMac() so the
   * commissioning join paths (#73) can share it. Fixed HAL_Delay()s there
   * are intentional — no ready-predicate exists for stack/radio settling. */
  /* LT-02/H-04 (#272): capture the current view BEFORE the restore tears
   * the MAC down (STEP 1's reinit de-initialises the MAC and moves
   * LmHandlerParams.ActiveRegion to the NEW region) - on failure these are
   * what the rollback must restore, and what the two views must be
   * re-synced to. Default the rollback flag true: every early return above
   * leaves the working session untouched. */
  uint8_t old_slot = g_storage.active_slot;
  LoRaMacRegion_t old_region = MultiRegion_GetActiveRegion();
  g_last_switch_rollback_ok = true;

  LmHandlerErrorStatus_t status = RestoreSessionToMac(ctx, region);
  if (status != LORAMAC_HANDLER_SUCCESS) {
    /* The MAC is de-initialised and LmHandlerParams.ActiveRegion names
     * the NEW region while active_slot still names the OLD one. Try to
     * roll back to the previous session. */
    bool rollback_ok = false;
    if (old_slot < MAX_REGION_CONTEXTS &&
        ValidateContextCRC(&g_storage.contexts[old_slot]) &&
        ValidateContextSemantics(&g_storage.contexts[old_slot])) {
      rollback_ok = (RollbackToRegion(old_slot, old_region) == LORAMAC_HANDLER_SUCCESS);
    }
    /* Re-sync the MAC's view either way: rollback success means the old
     * session is really loaded; rollback failure means the name at least
     * matches the bank the policy gates read. */
    LmHandlerParams.ActiveRegion = old_region;
    if (!rollback_ok) {
      /* Sessionless radio: say so where telemetry can see it. Sticky
       * by design (sys_caps never forgets) - a failed rollback means
       * no known-good session exists to transmit on. */
      g_last_switch_rollback_ok = false;
      SysCaps_MarkFailed(SYS_CAP_RADIO);
      SONDE_LOG_STR("SESSION LOST: rollback failed - radio degraded\r\n");
      APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: SESSION LOST - rollback failed\r\n");
    } else {
      SONDE_LOG("Rollback to %s succeeded - previous session restored\r\n",
                RegionToString(old_region));
    }
    return status;
  }
  g_storage.active_slot = (uint8_t)slot;
  ctx->last_used = HAL_GetTick();

  SONDE_LOG("Successfully switched to %s\r\n", RegionToString(region));
  APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Successfully switched to %s\r\n",
          RegionToString(region));

  return LORAMAC_HANDLER_SUCCESS;
}

/**
 * @brief Auto-switch to an already-detected region (policy only, no geofence lookup)
 */
LmHandlerErrorStatus_t MultiRegion_AutoSwitchToRegion(LoRaMacRegion_t target_region) {
// Check if auto-switching is enabled
#if MULTIREGION_AUTO_SWITCH_ENABLED == 0
  // Disabled - just return success
  return LORAMAC_HANDLER_SUCCESS;
#endif

  if (!g_initialized) {
    return LORAMAC_HANDLER_ERROR;
  }

  LoRaMacRegion_t current_region = MultiRegion_GetActiveRegion();

  if (target_region == current_region) {
    return LORAMAC_HANDLER_SUCCESS; // No switch needed
  }

  APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: GPS suggests switch %s -> %s\r\n",
          RegionToString(current_region), RegionToString(target_region));

  // Check if target region is joined
  if (!MultiRegion_IsRegionJoined(target_region)) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Target region not joined, staying on %s\r\n",
            RegionToString(current_region));
    return LORAMAC_HANDLER_SUCCESS; // Not an error, just stay on current
  }

  // Perform the switch
  return MultiRegion_SwitchToRegion(target_region);
}

/**
 * @brief Auto-switch based on GPS location
 */
LmHandlerErrorStatus_t MultiRegion_AutoSwitchForLocation(float lat, float lon) {
/* #77: thin wrapper — the geofence resolution happens here, the policy in
 * MultiRegion_AutoSwitchToRegion. Callers that already resolved the H3
 * region this cycle (SelectRegionAndSession) skip this redundant lookup. */
#if MULTIREGION_AUTO_SWITCH_ENABLED == 0
  return LORAMAC_HANDLER_SUCCESS;
#endif

  return MultiRegion_AutoSwitchToRegion(MultiRegion_DetectFromGPS_H3(lat, lon));
}

/**
 * @brief Get context storage statistics
 */
void MultiRegion_GetStats(uint8_t *total_slots, uint8_t *used_slots) {
  if (total_slots) {
    *total_slots = MAX_REGION_CONTEXTS;
  }
  if (used_slots) {
    *used_slots = g_initialized ? g_storage.num_valid : 0;
  }
}

/**
 * @brief Clear all stored contexts
 */
bool MultiRegion_ClearAllContexts(void) {
  APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Clearing all contexts\r\n");

  memset(&g_storage, 0, sizeof(g_storage));
  g_storage.magic = MULTIREGION_MAGIC;
  g_storage.version = MULTIREGION_VERSION;
  g_storage.active_slot = 0xFF;
  g_storage.num_valid = 0;
  g_t2_sequence = 0;
  g_t2_last_slot = -1;
  g_tier1_dirty = false;
  g_provisioned = 0; // C-01 (#270): erasing the Tier-1 pages kills the latch

  // Erase all tier pages (3x Tier-1 + 2x Tier-2)
  bool ok = true;
  for (uint8_t i = 0; i < TIER1_NUM_COPIES; i++) {
    if (FLASH_IF_Erase((void *)(uintptr_t)TIER1_ADDRS[i], MULTIREGION_FLASH_PAGE_SIZE) != FLASH_IF_OK) {
      ok = false;
    }
  }
  for (uint8_t i = 0; i < TIER2_NUM_SLOTS; i++) {
    if (FLASH_IF_Erase((void *)(uintptr_t)TIER2_ADDRS[i], MULTIREGION_FLASH_PAGE_SIZE) != FLASH_IF_OK) {
      ok = false;
    }
  }
  /* FR-11 (#94): "clear all contexts" must also clear the LoRaWAN NVM
   * slots, or a stale DevEUI/session survives the wipe. */
  if (!LoRaApp_EraseNvmSlots()) {
    ok = false;
  }
  return ok;
}

/**
 * @brief Send post-join data packets to receive MAC commands
 * @param num_packets Number of packets to send (typically 2)
 * @note Uses real sensor data to trigger any channel mask updates from network
 */
static void SendPostJoinDataPackets(uint8_t num_packets) {
  sensor_t sensor_data = {0}; /* #35: zero-init — uninitialized members were archived as authentic */

  SONDE_LOG("\r\n--- Sending %d post-join data packets ---\r\n", num_packets);

  for (uint8_t i = 0; i < num_packets; i++) {
    SONDE_LOG("Packet %d/%d: ", i + 1, num_packets);

    // Read sensor data
    EnvSensors_Read(&sensor_data);

    // Prepare simple LoRaWAN packet with temperature data
    LmHandlerAppData_t appData;
    uint8_t payload[3];
    int16_t temp_int = (int16_t)(sensor_data.temperature * 100);
    payload[0] = 0x01; // Temperature channel
    payload[1] = (temp_int >> 8) & 0xFF;
    payload[2] = temp_int & 0xFF;

    appData.Port = 2;
    appData.BufferSize = 3;
    appData.Buffer = payload;

    // Send unconfirmed message
    LmHandlerErrorStatus_t status = LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0);

    if (status == LORAMAC_HANDLER_SUCCESS) {
      SONDE_LOG_STR("Sent successfully\r\n");

      // Wait for TX complete and any downlinks
      HAL_Delay(2000);

      // Process any received MAC commands
      for (int j = 0; j < 10; j++) {
        LmHandlerProcess();
        HAL_Delay(100);
      }
    } else {
      SONDE_LOG("Send failed (status=%d)\r\n", status);
    }

    // Wait between packets
    if (i < num_packets - 1) {
      HAL_Delay(3000);
    }
  }

  SONDE_LOG_STR("--- Post-join packets complete ---\r\n\r\n");
}

/**
 * @brief Join a specific region and store context
 */
LmHandlerErrorStatus_t MultiRegion_JoinRegion(LoRaMacRegion_t region) {
  // Access external join success flag from lora_app.c
  extern volatile uint8_t g_multiregion_join_success;

  /* F4/T1 (DDR-0018/0008) + FR-12 (#89): joins are COMMISSIONING-ONLY.
   * This guard previously sat AFTER LmHandlerJoin() — "defense in depth"
   * that fired after the join request it exists to block. In FLIGHT it
   * would have transmitted before returning the error. Check FIRST. */
  if (!MissionState_IsCommissioning()) {
    SONDE_LOG_STR("JoinRegion: BLOCKED - joins are commissioning-only (DDR-0018)\r\n");
    return LORAMAC_HANDLER_ERROR;
  }

  if (!g_initialized) {
    MultiRegion_Init();
  }

  SONDE_LOG("\r\n=== Joining region %s ===\r\n", RegionToString(region));
  APP_LOG(TS_ON, VLEVEL_H, "\r\n=== Joining region %s ===\r\n", RegionToString(region));

  // Check if this is a region change (not first join)
  MibRequestConfirm_t check_mib;
  check_mib.Type = MIB_NETWORK_ACTIVATION;
  LoRaMacMibGetRequestConfirm(&check_mib);

  if (check_mib.Param.NetworkActivation != ACTIVATION_TYPE_NONE) {
    // We were already joined to another region - full reset needed
    SONDE_LOG_STR("Previous join detected - performing full stack reset...\r\n");
    if (LoRaApp_ReInitStack(region) != LORAMAC_HANDLER_SUCCESS) {
      /* F-01 (#245): the reset failed - do not join on a torn stack */
      SONDE_LOG_STR("JoinRegion: stack reset FAILED - aborting join\r\n");
      return LORAMAC_HANDLER_ERROR;
    }
    // LoRaApp_ReInitStack sets ActiveRegion but doesn't configure
    // We'll set DevEUI and configure below
  } else {
    // First join - set region but don't configure yet
    SONDE_LOG_STR("First join - setting region parameter...\r\n");
    LmHandlerParams.ActiveRegion = region;

    // CRITICAL: Erase LoRaWAN NVM to prevent DevEUI restoration
    // FR-11 (#94): erase BOTH slots via the owner — page-126-only erase
    // left a valid stale slot B that restore would select.
    SONDE_LOG_STR("Erasing LoRaWAN NVM to ensure clean state...\r\n");
    LoRaApp_EraseNvmSlots();
    HAL_Delay(100);
  }

  /* F-R3 (#73): identity from the shared table. DevEUI is set twice around
   * LmHandlerConfigure because Configure can restore a stale one from NVM. */
  const RegionIdentity_t *ident = FindRegionIdentity(region);
  if (!ident) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Unsupported region %d\r\n", region);
    return LORAMAC_HANDLER_ERROR;
  }

  LmHandlerSetDevEUI((uint8_t *)ident->dev_eui);
  if (LmHandlerConfigure(&LmHandlerParams) != LORAMAC_HANDLER_SUCCESS) {
    /* F-01 (#245): no join against an unconfigured stack */
    SONDE_LOG_STR("JoinRegion: LmHandlerConfigure FAILED - aborting join\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  LmHandlerSetDevEUI((uint8_t *)ident->dev_eui);

  // CRITICAL: Set JoinEUI (AppEUI) - must be set before join
  const uint8_t joineui[] = FORMAT32_KEY(LORAWAN_JOIN_EUI);
  LmHandlerSetAppEUI((uint8_t *)joineui);

  // CRITICAL: Set AppKey and NwkKey - must be set before join
  const uint8_t appkey[] = FORMAT_KEY(LORAWAN_APP_KEY);
  const uint8_t nwkkey[] = FORMAT_KEY(LORAWAN_NWK_KEY);
  LmHandlerSetKey(APP_KEY, (uint8_t *)appkey);
  LmHandlerSetKey(NWK_KEY, (uint8_t *)nwkkey);
  SONDE_LOG("Identity and join credentials set for %s\r\n", RegionToString(region));

  // Reset join success flag before triggering join
  g_multiregion_join_success = false;

  // Trigger join (LmHandlerJoin returns void)
  LmHandlerJoin(ACTIVATION_TYPE_OTAA, true);

/* R30/D6: the wait loop is now BOUNDED. "Infinite retry until success" was
 * acceptable only if the commissioning gate could never fail; a bounded
 * loop is safe under every circumstance (gate bypass, state corruption,
 * bench unit left out of gateway range overnight). On timeout the region
 * is marked failed and PreJoinAllRegions moves on to the next bank. */
#define JOIN_TIMEOUT_MS (5UL * 60UL * 1000UL) /* 5 min per region, then give up */
  uint32_t start_time = HAL_GetTick();
  uint32_t last_join_attempt = HAL_GetTick();
  uint32_t retry_interval = 30000; // Retry every 30 seconds

  SONDE_LOG("Waiting for %s join (max %lus)...\r\n",
            RegionToString(region), (unsigned long)(JOIN_TIMEOUT_MS / 1000UL));

  // Wait for join to complete by checking callback flag
  while (!g_multiregion_join_success) {
    // R30: bounded wait — give up on this region after JOIN_TIMEOUT_MS
    if ((HAL_GetTick() - start_time) > JOIN_TIMEOUT_MS) {
      SONDE_LOG("%s join TIMEOUT after %lus - skipping region\r\n",
                RegionToString(region),
                (unsigned long)((HAL_GetTick() - start_time) / 1000UL));
      return LORAMAC_HANDLER_ERROR;
    }

    // CRITICAL: Process MAC events to handle join accept
    LmHandlerProcess();

    // Check if we need to retry join (every 30 seconds)
    if ((HAL_GetTick() - last_join_attempt) > retry_interval) {
      SONDE_LOG("Retrying %s join...\r\n", RegionToString(region));
      LmHandlerJoin(ACTIVATION_TYPE_OTAA, true);
      last_join_attempt = HAL_GetTick();
    }

    // C5 FIX: Refresh watchdog during join wait to prevent reset.
    // Join can take 5-30+ seconds (RX windows + retries), which can
    // exceed the ~33s IWDG timeout and cause an unexpected reset.
    // BUG 3.1 FIX: Guard against NULL Instance — this code can run before MX_IWDG_Init()
    extern IWDG_HandleTypeDef hiwdg;
    if (hiwdg.Instance != NULL) {
      HAL_IWDG_Refresh(&hiwdg);
    }

    // Delay to prevent tight loop (250ms is sufficient for MAC processing)
    HAL_Delay(250);
  }

  // Join successful
  uint32_t join_time = (HAL_GetTick() - start_time) / 1000;
  (void)join_time; /* FR-19: log-only in flight */
  SONDE_LOG("%s join SUCCESS! (took %lus)\r\n", RegionToString(region), join_time);
  APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Join successful for %s (took %lus)\r\n",
          RegionToString(region), join_time);

  // Save the joined context (force save for critical join operations)
  HAL_Delay(500); // Give MAC time to stabilize
  MultiRegion_ForceSaveCurrentContext();

  // Send 2 post-join data packets to receive MAC commands (channel masks, etc.)
  SendPostJoinDataPackets(2);

  // Save context again after data transmissions (frame counters updated, force save)
  MultiRegion_ForceSaveCurrentContext();

  return LORAMAC_HANDLER_SUCCESS;
}

/**
 * @brief C-01 (#270, DDR-0018): verify the persisted Tier-1 bank region by
 *        region, then set or clear the durable PROVISIONED latch to match.
 *        Runs at the end of MultiRegion_PreJoinAllRegions (commissioning
 *        only). Every region in kPreJoinRegions[] must read back from flash
 *        with a valid CRC, sane semantics and a non-zero DevAddr; only then
 *        is the latch written. Any failure leaves the latch clear.
 */
static bool VerifyAndSetProvisioningLatch(void) {
  Tier1Bank_t bank;
  bool all_ok = true;

  if (!FlashReadTier1(&bank)) {
    SONDE_LOG_STR("PROVISIONING INCOMPLETE: no CRC-valid Tier-1 bank to verify\r\n");
    all_ok = false;
  } else {
    for (uint8_t i = 0; i < NUM_PREJOIN_REGIONS; i++) {
      LoRaMacRegion_t region = kPreJoinRegions[i];
      bool region_ok = false;
      for (uint8_t j = 0; j < MAX_REGION_CONTEXTS; j++) {
        MinimalRegionContext_t *ctx = &bank.contexts[j];
        if (ctx->region == region &&
            ValidateContextCRC(ctx) &&       /* CRC: the bytes survived */
            ValidateContextSemantics(ctx)) { /* non-zero DevAddr, real keys */
          region_ok = true;
          break;
        }
      }
      SONDE_LOG("PROVISIONING verify %s: %s\r\n",
                RegionToString(region), region_ok ? "ok" : "FAIL");
      if (!region_ok) {
        SONDE_LOG("PROVISIONING INCOMPLETE: %s not verified\r\n",
                  RegionToString(region));
        all_ok = false;
      }
    }
  }

  /* Persist the outcome either way: the latch must reflect THIS
   * verification, so a stale magic from an earlier bank cannot survive a
   * failed re-commission. Commissioning-only path - the extra Tier-1 write
   * is irrelevant next to the one-way flight door this latch gates. */
  g_provisioned = all_ok ? TIER1_PROVISIONED_MAGIC : 0;
  if (!FlashWriteTier1()) {
    /* The latch is only as good as its persistence: never report
     * provisioned on a RAM-only flag. */
    g_provisioned = 0;
    SONDE_LOG_STR("PROVISIONING INCOMPLETE: Tier-1 latch write failed\r\n");
    return false;
  }
  if (all_ok) {
    SONDE_LOG_STR("PROVISIONING COMPLETE: all regions verified - latch set (C-01)\r\n");
  }
  return all_ok;
}

/**
 * @brief C-01 (#270, DDR-0018): true only when the CRC-valid Tier-1 bank
 *        carried the PROVISIONED latch at boot AND every required region
 *        still validates. The per-region re-check means a bank that loses a
 *        context after latching still reads as NOT provisioned.
 */
bool MultiRegion_IsProvisioningComplete(void) {
  if (!g_initialized || g_provisioned != TIER1_PROVISIONED_MAGIC) {
    return false;
  }
  for (uint8_t i = 0; i < NUM_PREJOIN_REGIONS; i++) {
    if (!MultiRegion_IsRegionJoined(kPreJoinRegions[i])) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Pre-join all required regions (ground operations)
 */
bool MultiRegion_PreJoinAllRegions(void) {
  /* F4/T3 (DDR-0002): the entire pre-join ceremony is commissioning-only */
  if (!MissionState_IsCommissioning()) {
    SONDE_LOG_STR("PreJoinAllRegions: BLOCKED - commissioning-only (DDR-0002)\r\n");
    return false;
  }

  // Set pre-join flag to prevent TX timer from starting during joins
  extern volatile uint8_t g_multiregion_in_prejoin;
  g_multiregion_in_prejoin = 1;

  SONDE_LOG_STR("\r\n========================================\r\n");
  SONDE_LOG_STR("=== MULTI-REGION PRE-JOIN SEQUENCE ===\r\n");
  SONDE_LOG_STR("========================================\r\n\r\n");
  APP_LOG(TS_ON, VLEVEL_H, "\r\n========================================\r\n");
  APP_LOG(TS_ON, VLEVEL_H, "=== MULTI-REGION PRE-JOIN SEQUENCE ===\r\n");
  APP_LOG(TS_ON, VLEVEL_H, "========================================\r\n\r\n");

  // CRITICAL: Erase LoRaWAN NVM first to prevent old DevEUI restoration
  // FR-11 (#94): erase BOTH slots via the owner — page-126-only erase
  // left a valid stale slot B that restore would select.
  SONDE_LOG_STR("Erasing LoRaWAN NVM for clean multi-region start...\r\n");
  LoRaApp_EraseNvmSlots();
  HAL_Delay(100);

  bool all_success = true;
  uint8_t join_success_count = 0; /* R30/D6: flight entry requires >= 1 */

  /* F-R3 (#73): one loop over the region table. SP-05 (#246): seven banks -
   * IN865/KR920/RU864 join on the bench too (DDR-0018 INV-COMM-001: join
   * every configured region on the ground; the geofence maps all three).
   * C-01 (#270): the table is file scope, shared with the latch checks. */
  const uint8_t num_regions = NUM_PREJOIN_REGIONS;

  for (uint8_t i = 0; i < num_regions; i++) {
    LoRaMacRegion_t region = kPreJoinRegions[i];
    SONDE_LOG("\r\n--- Joining %s ---\r\n", RegionToString(region));
    if (MultiRegion_JoinRegion(region) != LORAMAC_HANDLER_SUCCESS) {
      APP_LOG(TS_ON, VLEVEL_H, "FAILED: %s join\r\n", RegionToString(region));
      all_success = false;
    } else {
      APP_LOG(TS_ON, VLEVEL_H, "SUCCESS: %s joined\r\n", RegionToString(region));
      join_success_count++;
      // Display session keys for copying to Chirpstack
      MultiRegion_DisplaySessionKeys();
    }
    HAL_Delay(5000);
  }
  // Switch back to US915 as starting region
  /* FR-18 (#295): a failed switch-back must block the PROVISIONED latch -
   * otherwise the device claims commissioned while the MAC sits on the
   * wrong session (or none). */
  if (MultiRegion_SwitchToRegion(LORAMAC_REGION_US915) != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("WARNING: switch-back to US915 FAILED - latch NOT set\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "Switch-back to US915 failed\r\n");
    all_success = false;
  }

  APP_LOG(TS_ON, VLEVEL_H, "\r\n========================================\r\n");
  if (all_success) {
    APP_LOG(TS_ON, VLEVEL_H, "=== ALL PRE-JOINS SUCCESSFUL ===\r\n");
  } else {
    /* SP-05 (#246): seven regions now - report the real denominator */
    APP_LOG(TS_ON, VLEVEL_H, "=== SOME PRE-JOINS FAILED (%d/%d joined) ===\r\n", join_success_count, num_regions);
  }
  APP_LOG(TS_ON, VLEVEL_H, "========================================\r\n\r\n");

  // Clear pre-join flag to allow TX timer to start
  g_multiregion_in_prejoin = 0;

  /* MISSION-01b (#142, DDR-0002 amendment): commissioning no longer enters
   * FLIGHT. A freshly commissioned unit holds COMMISSIONING (quiet watch:
   * no GPS, no telemetry TX) until deliberate arming (PB13 button,
   * arming_input.c - PRETEST-DEC-01 2026-08-16) or autonomous launch detection
   * (BR-LIFE-007 pressure departure, in MissionState_Update). The old
   * join-triggered EnterFlight put a bench unit into the 10 s ASCENT
   * cadence with GPS powered for the whole commissioning-to-launch gap.
   * R30/D6 note preserved: with zero joined banks the unit stays in
   * COMMISSIONING either way (joins only possible there). */
  if (join_success_count == 0) {
    SONDE_LOG_STR("PRE-JOIN: 0/4 regions joined - STAYING IN COMMISSIONING (power cycle to retry)\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "PRE-JOIN: no banks provisioned (R30/D6)\r\n");
  } else {
    SONDE_LOG_STR("PRE-JOIN complete - COMMISSIONING quiet watch until arming/launch (#142)\r\n");
  }

  /* C-01 (#270, DDR-0018): the durable PROVISIONED latch is written only
   * after every required region verifies against the persisted Tier-1
   * bank. MissionState_Update's flight door reads this latch - a unit
   * that could not verify every region can never latch ASCENT and be
   * stranded unable to join. */
  if (!VerifyAndSetProvisioningLatch()) {
    all_success = false;
  }

  return all_success;
}

/**
 * @brief Initialize a region context from Chirpstack session keys
 */
bool MultiRegion_InitializeRegionFromNetworkServer(
    LoRaMacRegion_t region,
    uint32_t dev_addr,
    const uint8_t *app_s_key,
    const uint8_t *nwk_s_key) {
  if (!g_initialized) {
    MultiRegion_Init();
  }

  if (!app_s_key || !nwk_s_key) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Invalid key pointers\r\n");
    return false;
  }

  SONDE_LOG("\r\n=== Initializing %s from Chirpstack keys ===\r\n",
            RegionToString(region));

  // Find or allocate slot for this region
  int8_t slot = FindContextSlot(region);

  if (slot < 0) {
    // Find empty slot
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
      if (g_storage.contexts[i].dev_addr == 0 ||
          g_storage.contexts[i].dev_addr == 0xFFFFFFFF) {
        slot = (int8_t)i;
        g_storage.num_valid++;
        break;
      }
    }
  }

  if (slot < 0) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: No available slots\r\n");
    return false;
  }

  // Initialize context structure
  MinimalRegionContext_t *ctx = &g_storage.contexts[slot];
  memset(ctx, 0, sizeof(MinimalRegionContext_t));

  // Set basic parameters
  ctx->region = region;
  ctx->activation = ACTIVATION_TYPE_ABP;
  ctx->dev_addr = dev_addr;

  // Copy session keys
  memcpy(ctx->app_s_key, app_s_key, 16);
  memcpy(ctx->nwk_s_key, nwk_s_key, 16);

  // F-R3 (#73): identity + radio defaults from the shared table
  const RegionIdentity_t *ident = FindRegionIdentity(region);
  if (!ident) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Unsupported region\r\n");
    return false;
  }
  memcpy(ctx->dev_eui, ident->dev_eui, 8);
  ctx->datarate = ident->datarate;
  ctx->rx2_frequency = ident->rx2_frequency;
  ctx->rx2_datarate = ident->rx2_datarate;

  // Initialize frame counters (will be loaded from flash if they exist)
  ctx->uplink_counter = 0;
  ctx->downlink_counter = 0;
  ctx->last_rx_mic = 0;

  // Radio parameters
  ctx->tx_power = 0;
  ctx->adr_enabled = 0; // ADR off for balloon

  // Timestamp
  ctx->last_used = HAL_GetTick();

  // Calculate CRC
  UpdateContextCRC(ctx);

  // FW-1: credentials written - Tier-1 bank rewrite required (commissioning path)
  g_tier1_dirty = true;

  // Save to flash
  bool result = FlashWriteStorage();

  if (result) {
    SONDE_LOG("%s: DevAddr=0x%08lX initialized\r\n",
              RegionToString(region), dev_addr);
    /* F-R2 (#75): key hex dumps removed outright (see SwitchToRegion). */
    APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: %s context initialized from Chirpstack\r\n",
            RegionToString(region));
  }

  return result;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Calculate CRC16 for context validation
 */
static uint16_t CalculateCRC16(const uint8_t *data, uint32_t length) {
  uint16_t crc = 0xFFFF;

  for (uint32_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }

  return crc;
}

/**
 * @brief Calculate CRC32 for storage validation
 */
static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFF;

  for (uint32_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x00000001) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc = crc >> 1;
      }
    }
  }

  return ~crc;
}

/**
 * @brief Validate context CRC
 */
static bool ValidateContextCRC(MinimalRegionContext_t *ctx) {
  uint16_t stored_crc = ctx->crc16;
  ctx->crc16 = 0;
  /* FR-18 (#99): offsetof span, not sizeof-2 — the old span silently
   * depended on crc16's position and the trailing padding. NOTE: changes
   * CRC values vs pre-FR-18 firmware (no deployed fleet, so no migration). */
  uint16_t calculated_crc = CalculateCRC16((uint8_t *)ctx, offsetof(MinimalRegionContext_t, crc16));
  ctx->crc16 = stored_crc;

  return (stored_crc == calculated_crc);
}

/**
 * @brief R2 (#189): semantic validation - a CRC proves the bytes SURVIVED,
 *        not that they were ever a real session. Reject CRC-valid but
 *        semantically dead contexts: uniformly 0x00 / 0xFF session keys
 *        (erased-flash or incomplete-commissioning patterns that still got
 *        CRC-stamped), a null/broadcast DevAddr, or a region ordinal outside
 *        the enum. Belt-and-braces with the CRC check at every gate.
 */
static bool ValidateContextSemantics(const MinimalRegionContext_t *ctx) {
  if (ctx->region > LORAMAC_REGION_RU864) { /* last enum member */
    return false;
  }
  if (ctx->dev_addr == 0 || ctx->dev_addr == 0xFFFFFFFFUL) {
    return false;
  }
  bool app_uniform = true, nwk_uniform = true;
  for (int i = 1; i < 16; i++) {
    if (ctx->app_s_key[i] != ctx->app_s_key[0])
      app_uniform = false;
    if (ctx->nwk_s_key[i] != ctx->nwk_s_key[0])
      nwk_uniform = false;
  }
  if (app_uniform && (ctx->app_s_key[0] == 0x00 || ctx->app_s_key[0] == 0xFF)) {
    return false;
  }
  if (nwk_uniform && (ctx->nwk_s_key[0] == 0x00 || ctx->nwk_s_key[0] == 0xFF)) {
    return false;
  }
  return true;
}

/**
 * @brief Update context CRC
 */
static void UpdateContextCRC(MinimalRegionContext_t *ctx) {
  ctx->crc16 = 0;
  ctx->crc16 = CalculateCRC16((uint8_t *)ctx, offsetof(MinimalRegionContext_t, crc16)); /* FR-18 */
}

/**
 * @brief Read Tier-1 credential bank from flash (three redundant copies).
 *        First CRC-valid copy wins; any invalid copy is repaired from the
 *        good one (DDR-0018 restore-repair: a bad copy is erased/rewritten
 *        only while at least one good copy remains intact).
 */
static bool FlashReadTier1(Tier1Bank_t *out) {
  bool copy_ok[TIER1_NUM_COPIES] = {false, false, false};
  uint32_t copy_gen[TIER1_NUM_COPIES] = {0, 0, 0};
  int8_t good = -1;
  uint32_t best_gen = 0;

  for (uint8_t i = 0; i < TIER1_NUM_COPIES; i++) {
    Tier1Bank_t tmp;
    if (FLASH_IF_Read(&tmp, (void *)(uintptr_t)TIER1_ADDRS[i], sizeof(Tier1Bank_t)) != FLASH_IF_OK) {
      continue;
    }
    if (tmp.magic != TIER1_MAGIC || tmp.version != MULTIREGION_VERSION) {
      continue;
    }
    uint32_t stored_crc = tmp.crc32;
    tmp.crc32 = 0;
    if (CalculateCRC32((uint8_t *)&tmp, sizeof(Tier1Bank_t) - 4) != stored_crc) {
      APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Tier-1 copy %d CRC mismatch\r\n", i);
      continue;
    }
    tmp.crc32 = stored_crc;
    copy_ok[i] = true;
    copy_gen[i] = tmp.generation;
    /* R8 (#190): NEWEST CRC-valid copy wins, not first-valid. Without a
     * generation a torn re-commission resurrects a stale credential bank
     * (and the old repair path then propagated it over the good copies). */
    /* S-10 (#234): wrap-safe compare, consistent with flash_log.c and
     * the LoRaWAN NVM slot picker. good < 0 stays first - it is what
     * makes the first valid copy win regardless of counter value. */
    if (good < 0 || (int32_t)(tmp.generation - best_gen) > 0) {
      memcpy(out, &tmp, sizeof(Tier1Bank_t));
      best_gen = tmp.generation;
      good = (int8_t)i;
    }
  }

  if (good < 0) {
    return false;
  }
  g_t1_generation = best_gen; /* seed the write-side counter */

  // Restore-repair: rewrite any bad copy OR stale-generation copy from the
  // winner (R8: a lingering older copy must not outlive the newest bank's
  // copies and resurrect later).
  for (uint8_t i = 0; i < TIER1_NUM_COPIES; i++) {
    if (copy_ok[i] && copy_gen[i] == best_gen) {
      continue;
    }
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Repairing Tier-1 copy %d from copy %d\r\n", i, good);
    if (FLASH_IF_Erase((void *)(uintptr_t)TIER1_ADDRS[i], MULTIREGION_FLASH_PAGE_SIZE) != FLASH_IF_OK) {
      continue;
    }
    FLASH_IF_Write((void *)(uintptr_t)TIER1_ADDRS[i], out, sizeof(Tier1Bank_t));
  }

  return true;
}

/**
 * @brief Write Tier-1 credential bank to all three pages and read-back
 *        verify each copy. Commissioning-only: Tier-1 pages are never
 *        erased in flight.
 */
static bool FlashWriteTier1(void) {
  Tier1Bank_t t1;
  memset(&t1, 0, sizeof(t1));
  t1.magic = TIER1_MAGIC;
  t1.version = MULTIREGION_VERSION;
  t1.num_valid = g_storage.num_valid;
  t1.active_slot = g_storage.active_slot;
  t1.generation = ++g_t1_generation; /* R8 (#190): newest valid copy wins on read */
  t1.provisioned = g_provisioned;    /* C-01 (#270): carry the latch */
  memcpy(t1.contexts, g_storage.contexts, sizeof(t1.contexts));
  t1.crc32 = 0;
  t1.crc32 = CalculateCRC32((uint8_t *)&t1, sizeof(Tier1Bank_t) - 4);

  bool all_ok = true;
  for (uint8_t i = 0; i < TIER1_NUM_COPIES; i++) {
    if (FLASH_IF_Erase((void *)(uintptr_t)TIER1_ADDRS[i], MULTIREGION_FLASH_PAGE_SIZE) != FLASH_IF_OK) {
      APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Tier-1 copy %d erase failed\r\n", i);
      all_ok = false;
      continue;
    }
    if (FLASH_IF_Write((void *)(uintptr_t)TIER1_ADDRS[i], &t1, sizeof(Tier1Bank_t)) != FLASH_IF_OK) {
      APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Tier-1 copy %d write failed\r\n", i);
      all_ok = false;
      continue;
    }
    // Commissioning read-back: verify the copy just written
    Tier1Bank_t verify;
    if (FLASH_IF_Read(&verify, (void *)(uintptr_t)TIER1_ADDRS[i], sizeof(Tier1Bank_t)) != FLASH_IF_OK ||
        memcmp(&verify, &t1, sizeof(Tier1Bank_t)) != 0) {
      APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Tier-1 copy %d verify failed\r\n", i);
      all_ok = false;
    }
  }
  return all_ok;
}

/**
 * @brief Read Tier-2 counter bank from flash; newest CRC-valid slot wins.
 */
static bool FlashReadTier2(Tier2Bank_t *out) {
  bool found = false;
  uint32_t best_seq = 0;

  for (uint8_t i = 0; i < TIER2_NUM_SLOTS; i++) {
    Tier2Bank_t tmp;
    if (FLASH_IF_Read(&tmp, (void *)(uintptr_t)TIER2_ADDRS[i], sizeof(Tier2Bank_t)) != FLASH_IF_OK) {
      continue;
    }
    if (tmp.magic != TIER2_MAGIC || tmp.version != MULTIREGION_VERSION) {
      continue;
    }
    uint32_t stored_crc = tmp.crc32;
    tmp.crc32 = 0;
    if (CalculateCRC32((uint8_t *)&tmp, sizeof(Tier2Bank_t) - 4) != stored_crc) {
      APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Tier-2 slot %d CRC mismatch\r\n", i);
      continue;
    }
    tmp.crc32 = stored_crc;
    /* S-10 (#234): wrap-safe compare (see the Tier-1 site above). */
    if (!found || (int32_t)(tmp.sequence - best_seq) > 0) {
      memcpy(out, &tmp, sizeof(Tier2Bank_t));
      best_seq = tmp.sequence;
      g_t2_last_slot = (int8_t)i;
      found = true;
    }
  }
  return found;
}

/**
 * @brief Write Tier-2 counter bank to the alternate ping-pong slot.
 *        Erase-before-write on the *idle* slot: a brownout mid-save leaves
 *        the previous slot fully intact (FlashStorageNotes.md pattern). The C6 counter
 *        margin (applied by the caller) covers the resulting regression.
 */
static bool FlashWriteTier2(void) {
  Tier2Bank_t t2;
  memset(&t2, 0, sizeof(t2));
  t2.magic = TIER2_MAGIC;
  t2.version = MULTIREGION_VERSION;
  t2.active_slot = g_storage.active_slot;
  t2.sequence = g_t2_sequence + 1;

  for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
    MinimalRegionContext_t *ctx = &g_storage.contexts[i];
    if (ctx->dev_addr == 0 || ctx->dev_addr == 0xFFFFFFFF) {
      continue; // Entry stays zeroed/invalid
    }
    t2.entries[i].uplink_counter = ctx->uplink_counter;
    t2.entries[i].downlink_counter = ctx->downlink_counter;
    t2.entries[i].last_rx_mic = ctx->last_rx_mic;
    t2.entries[i].last_used = ctx->last_used;
    t2.entries[i].datarate = ctx->datarate;
    t2.entries[i].tx_power = ctx->tx_power;
    t2.entries[i].adr_enabled = ctx->adr_enabled;
    t2.entries[i].valid = 1;
  }
  t2.crc32 = 0;
  t2.crc32 = CalculateCRC32((uint8_t *)&t2, sizeof(Tier2Bank_t) - 4);

  uint8_t slot = (uint8_t)((g_t2_last_slot + 1) % TIER2_NUM_SLOTS);

  if (FLASH_IF_Erase((void *)(uintptr_t)TIER2_ADDRS[slot], MULTIREGION_FLASH_PAGE_SIZE) != FLASH_IF_OK) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Tier-2 slot %d erase failed\r\n", slot);
    return false;
  }
  if (FLASH_IF_Write((void *)(uintptr_t)TIER2_ADDRS[slot], &t2, sizeof(Tier2Bank_t)) != FLASH_IF_OK) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Tier-2 slot %d write failed\r\n", slot);
    return false;
  }

  g_t2_sequence = t2.sequence;
  g_t2_last_slot = (int8_t)slot;
  return true;
}

/**
 * @brief Restore storage from flash: Tier-1 credentials overlaid with the
 *        newest valid Tier-2 counter bank.
 * @retval true if a valid Tier-1 bank was found (contexts restored)
 */
static bool FlashReadStorage(void) {
  Tier1Bank_t t1;
  if (!FlashReadTier1(&t1)) {
    return false; // Virgin bank (or unrecoverable) -> COMMISSIONING door anchor
  }

  g_storage.magic = MULTIREGION_MAGIC;
  g_storage.version = MULTIREGION_VERSION;
  g_storage.num_valid = t1.num_valid;
  g_storage.active_slot = t1.active_slot;
  memcpy(g_storage.contexts, t1.contexts, sizeof(g_storage.contexts));
  /* C-01 (#270): the PROVISIONED latch restores only from a CRC-valid
   * Tier-1 bank - never invented, never cached across a wipe. */
  g_provisioned = t1.provisioned;

  bool counters_bumped = false; /* R3 (#188): any margin applied */
  Tier2Bank_t t2 = {0};         /* F-17 (#67): silence maybe-uninitialized (false positive, but explicit) */
  if (FlashReadTier2(&t2)) {
    g_t2_sequence = t2.sequence;
    if (t2.active_slot < MAX_REGION_CONTEXTS) {
      g_storage.active_slot = t2.active_slot;
    }
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
      if (!t2.entries[i].valid) {
        continue;
      }
      counters_bumped = true;
      MinimalRegionContext_t *ctx = &g_storage.contexts[i];
      /* R3 (#188): resume AHEAD of the persisted uplink counter. Saves
       * batch every CfgFrameCounterSaveInterval() TXs and persist the
       * true value, so after a mid-batch reset the NS has already seen
       * up to INTERVAL-1 counters beyond the persisted one - resuming
       * at the persisted value replays them (dropped), and a reset loop
       * short of the next save boundary replays the same band forever.
       * The margin burns at most one reserved block per reset; the only
       * property the NS requires is strict monotonicity. (The Tier-2-
       * lost degrade path below already bumps by the same margin.) */
      ctx->uplink_counter = t2.entries[i].uplink_counter + CfgFrameCounterSaveInterval();
      ctx->downlink_counter = t2.entries[i].downlink_counter;
      ctx->last_rx_mic = t2.entries[i].last_rx_mic;
      ctx->last_used = t2.entries[i].last_used;
      ctx->datarate = t2.entries[i].datarate;
      ctx->tx_power = t2.entries[i].tx_power;
      ctx->adr_enabled = t2.entries[i].adr_enabled;
    }
  } else {
    /* Degrade ladder (DDR-0018): keys good, counters lost ->
     * counter = last persisted (Tier-1 commissioning value) + margin.
     * The C6 margin keeps the restored FCntUp ahead of the server. */
    APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Tier-2 counters lost; using Tier-1 values + margin\r\n");
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
      MinimalRegionContext_t *ctx = &g_storage.contexts[i];
      if (ctx->dev_addr != 0 && ctx->dev_addr != 0xFFFFFFFF) {
        ctx->uplink_counter += CfgFrameCounterSaveInterval();
        counters_bumped = true;
      }
    }
  }

  /* Re-stamp per-entry CRCs on the merged working copy so the existing
   * ValidateContextCRC() checks (IsRegionJoined / SwitchToRegion) hold. */
  for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
    UpdateContextCRC(&g_storage.contexts[i]);
  }

  /* R3 (#188): commit the margined counters back to flash NOW. The margin
   * alone is not reset-loop-safe: restore adds +INTERVAL, but if the unit
   * resets again before the next batched save, the same reserved band is
   * replayed (NS drops reused counters; a sustained loop stalls the uplink
   * counter permanently). Committing the new base makes every boot start a
   * FRESH reserved block - monotonic under arbitrary reset patterns. Cost:
   * one ping-ponged Tier-2 write per boot (~20k boots of page endurance),
   * and boot-time resets are exactly the case FR-23/F-6 escalate on. */
  /* Reset history is diagnostic only; it must not suppress counter
   * commit-back or change LoRaWAN session/counter recovery. */
  if (counters_bumped && !FlashWriteTier2()) {
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: R3 counter commit-back failed (RAM margins still monotonic this boot)\r\n");
  }

  return true;
}

/**
 * @brief Persist storage to flash.
 *        Tier-2 (counters) is always written (ping-pong, erase-before-write).
 *        Tier-1 (credentials) is written only when static fields changed
 *        (g_tier1_dirty) and only in COMMISSIONING - Tier-1 pages are never
 *        erased in flight (FW-1 / DDR-0018).
 */
static bool FlashWriteStorage(void) {
  bool ok = true;

  if (g_tier1_dirty) {
    if (MissionState_IsCommissioning()) {
      if (FlashWriteTier1()) {
        g_tier1_dirty = false;
      } else {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Tier-1 write failed\r\n");
        ok = false;
      }
    } else {
      /* Should be unreachable (joins are commissioning-only); never
       * erase credential pages in flight regardless. */
      SONDE_LOG_STR("MultiRegion: Tier-1 write SUPPRESSED in FLIGHT\r\n");
      APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Tier-1 write suppressed in FLIGHT\r\n");
      g_tier1_dirty = false;
    }
  }

  if (!FlashWriteTier2()) {
    ok = false;
  }

  return ok;
}

/**
 * @brief Find context slot for a region
 */
static int8_t FindContextSlot(LoRaMacRegion_t region) {
  /* FR-16 (#97): this used to run six snprintf into a 256-byte stack buffer
   * per call even in flight builds (SONDE_LOG_STR gated, snprintf not). */
  SONDE_LOG("FindContextSlot: Searching for region %s (enum=%d)\r\n",
            RegionToString(region), region);

  SONDE_LOG_STR("Storage contents:\r\n");
  for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
    SONDE_LOG("  Slot %d: region=%s (enum=%d), DevAddr=0x%08lX\r\n",
              i, RegionToString(g_storage.contexts[i].region),
              g_storage.contexts[i].region,
              g_storage.contexts[i].dev_addr);

    /* FW-9: skip empty slots. LORAMAC_REGION_AS923 == 0, so an erased/
     * zeroed slot otherwise "matches" an AS923 lookup. A real slot always
     * has a DevAddr (never 0 or 0xFFFFFFFF). */
    if (g_storage.contexts[i].dev_addr == 0 ||
        g_storage.contexts[i].dev_addr == 0xFFFFFFFFUL) {
      continue;
    }

    if (g_storage.contexts[i].region == region) {
      SONDE_LOG("  -> Found at slot %d!\r\n", i);
      return (int8_t)i;
    }
  }

  SONDE_LOG_STR("  -> NOT FOUND (returning -1)\r\n");
  return -1;
}

/**
 * @brief Capture current MAC context
 * @note This function updates ONLY dynamic values (frame counters, datarate, etc.)
 * @note Static values (DevAddr, DevEUI, session keys) are NOT overwritten
 */
static bool CaptureCurrentContext(MinimalRegionContext_t *ctx) {
  if (!ctx) {
    return false;
  }

  MibRequestConfirm_t mib;

  // CRITICAL: DevAddr, DevEUI, and session keys are region-specific constants
  // They are set during initialization and should NEVER be overwritten
  // Only capture dynamic state that changes with transmissions

  SONDE_LOG("Capturing dynamic context for region %s (DevAddr=0x%08lX preserved)\r\n",
            RegionToString(ctx->region), ctx->dev_addr);

  // Get activation type
  mib.Type = MIB_NETWORK_ACTIVATION;
  LoRaMacMibGetRequestConfirm(&mib);
  ctx->activation = mib.Param.NetworkActivation;

  // Get NVM contexts to extract frame counters ONLY
  mib.Type = MIB_NVM_CTXS;
  LoRaMacMibGetRequestConfirm(&mib);
  LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t *)mib.Param.Contexts;

  if (nvm) {
    // ONLY copy frame counters (keys and DevAddr already set correctly)
    ctx->uplink_counter = nvm->Crypto.FCntList.FCntUp;
    ctx->downlink_counter = nvm->Crypto.FCntList.NFCntDown;
    ctx->last_rx_mic = nvm->MacGroup1.LastRxMic;

    SONDE_LOG("  Captured FCntUp=%lu, FCntDown=%lu\r\n",
              ctx->uplink_counter, ctx->downlink_counter);
  }

  // Get datarate
  mib.Type = MIB_CHANNELS_DATARATE;
  LoRaMacMibGetRequestConfirm(&mib);
  ctx->datarate = (uint8_t)mib.Param.ChannelsDatarate;

  // Get TX power
  mib.Type = MIB_CHANNELS_TX_POWER;
  LoRaMacMibGetRequestConfirm(&mib);
  ctx->tx_power = mib.Param.ChannelsTxPower;

  // Get ADR
  mib.Type = MIB_ADR;
  LoRaMacMibGetRequestConfirm(&mib);
  ctx->adr_enabled = mib.Param.AdrEnable ? 1 : 0;

  // Get RX2 params
  mib.Type = MIB_RX2_CHANNEL;
  LoRaMacMibGetRequestConfirm(&mib);
  ctx->rx2_frequency = mib.Param.Rx2Channel.Frequency;
  ctx->rx2_datarate = mib.Param.Rx2Channel.Datarate;

  // Update timestamp
  ctx->last_used = HAL_GetTick();

  // Calculate CRC
  UpdateContextCRC(ctx);

  SONDE_LOG("Context captured: DevAddr=0x%08lX (preserved), FCntUp=%lu\r\n",
            ctx->dev_addr, ctx->uplink_counter);

  return true;
}

/**
 * @brief Convert region enum to string
 */
const char *RegionToString(LoRaMacRegion_t region) {
  switch (region) {
  case LORAMAC_REGION_AS923:
    return "AS923";
  case LORAMAC_REGION_AU915:
    return "AU915";
  case LORAMAC_REGION_CN470:
    return "CN470";
  case LORAMAC_REGION_CN779:
    return "CN779";
  case LORAMAC_REGION_EU433:
    return "EU433";
  case LORAMAC_REGION_EU868:
    return "EU868";
  case LORAMAC_REGION_KR920:
    return "KR920";
  case LORAMAC_REGION_IN865:
    return "IN865";
  case LORAMAC_REGION_US915:
    return "US915";
  case LORAMAC_REGION_RU864:
    return "RU864";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief Display current session keys for Chirpstack ABP configuration
 */
void MultiRegion_DisplaySessionKeys(void) {
  /* F-017: session keys are secrets — never print them in FLIGHT.
   * COMMISSIONING-only (the bench is where Chirpstack setup happens). */
  if (!MissionState_IsCommissioning()) {
    return;
  }
  if (!g_initialized || g_storage.active_slot >= MAX_REGION_CONTEXTS) {
    SONDE_LOG_STR("ERROR: No active region to display\r\n");
    return;
  }

  MinimalRegionContext_t *ctx = &g_storage.contexts[g_storage.active_slot];
  (void)ctx; /* FR-19: every use below is a gated log line */

  SONDE_LOG_STR("\r\n");
  SONDE_LOG_STR("========================================\r\n");
  SONDE_LOG_STR("=== SESSION KEYS FOR CHIRPSTACK ABP ===\r\n");
  SONDE_LOG_STR("========================================\r\n\r\n");

  // Region
  SONDE_LOG("Region:       %s\r\n", RegionToString(ctx->region));

  // DevEUI
  SONDE_LOG_STR("DevEUI:       ");
  for (int i = 0; i < 8; i++) {
    SONDE_LOG("%02x%s", ctx->dev_eui[i], (i < 7) ? ":" : "");
  }
  SONDE_LOG_STR("\r\n");

  // DevAddr
  SONDE_LOG("DevAddr:      0x%08lx\r\n", ctx->dev_addr);

  // AppSKey (formatted for Chirpstack)
  SONDE_LOG_STR("AppSKey:      ");
  for (int i = 0; i < 16; i++) {
    SONDE_LOG("%02x", ctx->app_s_key[i]);
  }
  SONDE_LOG_STR("\r\n");

  // NwkSKey (formatted for Chirpstack)
  SONDE_LOG_STR("NwkSKey:      ");
  for (int i = 0; i < 16; i++) {
    SONDE_LOG("%02x", ctx->nwk_s_key[i]);
  }
  SONDE_LOG_STR("\r\n");

  // Frame counters
  SONDE_LOG("FCntUp:       %lu\r\n", ctx->uplink_counter);
  SONDE_LOG("FCntDown:     %lu\r\n", ctx->downlink_counter);

  SONDE_LOG_STR("\r\n");
  SONDE_LOG_STR("========================================\r\n");
  SONDE_LOG_STR("Copy AppSKey and NwkSKey to Chirpstack\r\n");
  SONDE_LOG_STR("========================================\r\n\r\n");
}
