/**
  ******************************************************************************
  * @file    test_multiregion.c
  * @brief   R15 (ChatGPT review 2026-08-11): compile the REAL
  *          Core/Src/multiregion_context.c against a fake LoRaMac/LmHandler
  *          surface with fault injection, and drive the recovery matrix the
  *          source scans cannot prove. Because the unit is #included, its
  *          statics (g_initialized, g_storage, g_unsaved_tx_count, ...) are
  *          directly resettable to emulate reboots.
  *
  *          R1 (P0): session restore must fail closed - never report success
  *          after a failed MIB set / verification, never mark the region
  *          active on a dead session.
  *          R3 (P0): FCnt restore must resume AHEAD of the true network
  *          counter (persisted value + margin), or every mid-batch reset
  *          reuses uplink counters the NS already saw.
  *
  *          Every check is CHECK_REGRESSION (expected-fail-before-fix):
  *            make -C tests/host multiregion      (red until fixes land)
  *            EXPECT_UNFIXED=1 ./test_multiregion (pre-fix gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Stub surfaces (tests/host/stubs precedes the real include dirs) */
#include "stm32wlxx_hal.h"
#include "LoRaMac.h"
#include "LmHandler.h"
#include "flash_if.h"
#include "sys_sensors.h"
#include "lora_app.h"
#include "multiregion_h3.h"
#include "mission_state.h"
#include "multiregion_context.h"

/* ------------------------------------------------------------------ */
/* Fake LoRaMac state + fault-injection knobs                          */
/* ------------------------------------------------------------------ */

static struct {
    uint32_t dev_addr;
    ActivationType_t activation;
    uint8_t keys[KEY_LIST_SIZE][16];
    uint8_t dev_eui[8];
    bool busy;
    LoRaMacNvmData_t nvm;
    /* live radio params (R11 round-trip) */
    int8_t channels_datarate;
    int8_t channels_tx_power;
    bool adr_enable;
    uint32_t rx2_freq;
    uint32_t rx2_dr;
    /* fault injection */
    bool fail_mib_set_devaddr;      /* every MIB_DEV_ADDR set fails */
    bool fail_mib_set_activation;   /* every MIB_NETWORK_ACTIVATION set fails */
    bool corrupt_keys_on_mib_get;   /* NVM readback shows wrong session keys */
    bool null_nvm;                  /* MIB_NVM_CTXS get returns NULL contexts */
    bool fail_getkey;               /* F-008 (#205): LmHandlerGetKey fails */
    /* F-01 (#245): lifecycle-return injections for restore steps 1-3 */
    bool fail_reinit;               /* LoRaApp_ReInitStack fails */
    bool fail_configure;            /* LmHandlerConfigure fails */
    bool fail_setdeveui;            /* LmHandlerSetDevEUI fails */
    bool fail_setkey;               /* LmHandlerSetKey fails */
} g_mac;

LoRaMacStatus_t LoRaMacMibSetRequestConfirm(MibRequestConfirm_t *mib)
{
    switch (mib->Type) {
    case MIB_DEV_ADDR:
        if (g_mac.fail_mib_set_devaddr) return LORAMAC_STATUS_ERROR;
        g_mac.dev_addr = mib->Param.DevAddr;
        return LORAMAC_STATUS_OK;
    case MIB_NETWORK_ACTIVATION:
        if (g_mac.fail_mib_set_activation) return LORAMAC_STATUS_ERROR;
        g_mac.activation = mib->Param.NetworkActivation;
        return LORAMAC_STATUS_OK;
    case MIB_CHANNELS_DATARATE:
        g_mac.channels_datarate = mib->Param.ChannelsDatarate;
        return LORAMAC_STATUS_OK;
    case MIB_CHANNELS_TX_POWER:
        g_mac.channels_tx_power = mib->Param.ChannelsTxPower;
        return LORAMAC_STATUS_OK;
    case MIB_ADR:
        g_mac.adr_enable = mib->Param.AdrEnable;
        return LORAMAC_STATUS_OK;
    case MIB_RX2_CHANNEL:
        g_mac.rx2_freq = mib->Param.Rx2Channel.Frequency;
        g_mac.rx2_dr = mib->Param.Rx2Channel.Datarate;
        return LORAMAC_STATUS_OK;
    default:
        return LORAMAC_STATUS_OK;
    }
}

LoRaMacStatus_t LoRaMacMibGetRequestConfirm(MibRequestConfirm_t *mib)
{
    switch (mib->Type) {
    case MIB_DEV_ADDR:
        mib->Param.DevAddr = g_mac.dev_addr;
        return LORAMAC_STATUS_OK;
    case MIB_NETWORK_ACTIVATION:
        mib->Param.NetworkActivation = g_mac.activation;
        return LORAMAC_STATUS_OK;
    case MIB_NVM_CTXS:
        if (g_mac.corrupt_keys_on_mib_get) {
            /* The real hazard: the secure element silently lost the keys the
             * restore just wrote. */
            g_mac.nvm.SecureElement.KeyList[APP_S_KEY].KeyValue[0] ^= 0xFF;
            g_mac.nvm.SecureElement.KeyList[NWK_S_KEY].KeyValue[0] ^= 0xFF;
        }
        mib->Param.Contexts = g_mac.null_nvm ? NULL : &g_mac.nvm;
        return LORAMAC_STATUS_OK;
    case MIB_CHANNELS_DATARATE:
        mib->Param.ChannelsDatarate = g_mac.channels_datarate;
        return LORAMAC_STATUS_OK;
    case MIB_CHANNELS_TX_POWER:
        mib->Param.ChannelsTxPower = g_mac.channels_tx_power;
        return LORAMAC_STATUS_OK;
    case MIB_ADR:
        mib->Param.AdrEnable = g_mac.adr_enable;
        return LORAMAC_STATUS_OK;
    case MIB_RX2_CHANNEL:
        mib->Param.Rx2Channel.Frequency = g_mac.rx2_freq;
        mib->Param.Rx2Channel.Datarate = g_mac.rx2_dr;
        return LORAMAC_STATUS_OK;
    default:
        return LORAMAC_STATUS_OK;
    }
}

LoRaMacStatus_t LoRaMacStart(void) { return LORAMAC_STATUS_OK; }
bool LoRaMacIsBusy(void) { return g_mac.busy; }

/* ------------------------------------------------------------------ */
/* Fake LmHandler                                                       */
/* ------------------------------------------------------------------ */

LmHandlerParams_t LmHandlerParams;

/* F-01 (#245): the real LmHandlerConfigure returns a status; the fake can
 * now be told to fail so restore step 2 is provably fail-closed. */
LmHandlerErrorStatus_t LmHandlerConfigure(LmHandlerParams_t *params)
{
    (void)params;
    return g_mac.fail_configure ? LORAMAC_HANDLER_ERROR : LORAMAC_HANDLER_SUCCESS;
}

LmHandlerErrorStatus_t LmHandlerSetKey(KeyIdentifier_t keyId, uint8_t *key)
{
    if (g_mac.fail_setkey) return LORAMAC_HANDLER_ERROR;   /* F-01 (#245) */
    if (keyId >= KEY_LIST_SIZE || key == NULL) return LORAMAC_HANDLER_ERROR;
    memcpy(g_mac.keys[keyId], key, 16);
    /* SetKey also lands in the NVM mirror, as on target */
    memcpy(g_mac.nvm.SecureElement.KeyList[keyId].KeyValue, key, 16);
    return LORAMAC_HANDLER_SUCCESS;
}

LmHandlerErrorStatus_t LmHandlerGetKey(KeyIdentifier_t keyId, uint8_t *key)
{
    if (g_mac.fail_getkey) return LORAMAC_HANDLER_ERROR;   /* F-008 injection */
    if (keyId >= KEY_LIST_SIZE || key == NULL) return LORAMAC_HANDLER_ERROR;
    memcpy(key, g_mac.keys[keyId], 16);
    return LORAMAC_HANDLER_SUCCESS;
}

/* F-01 (#245): real LmHandlerSetDevEUI returns a status; injectable. */
LmHandlerErrorStatus_t LmHandlerSetDevEUI(uint8_t *devEui)
{
    if (g_mac.fail_setdeveui) return LORAMAC_HANDLER_ERROR;
    memcpy(g_mac.dev_eui, devEui, 8);
    return LORAMAC_HANDLER_SUCCESS;
}
void LmHandlerSetAppEUI(uint8_t *appEui) { (void)appEui; }
void LmHandlerProcess(void) { }
void LmHandlerJoin(ActivationType_t mode, bool forceRejoin) { (void)mode; (void)forceRejoin; }

LmHandlerFlagStatus_t LmHandlerJoinStatus(void)
{
    return (g_mac.activation == ACTIVATION_TYPE_ABP) ? LORAMAC_HANDLER_SET : LORAMAC_HANDLER_RESET;
}

LmHandlerErrorStatus_t LmHandlerSend(LmHandlerAppData_t *appData, LmHandlerMsgTypes_t isTxConfirmed, uint32_t nextTxIn)
{
    (void)appData; (void)isTxConfirmed; (void)nextTxIn;
    return LORAMAC_HANDLER_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* Fake app/infra surfaces                                              */
/* ------------------------------------------------------------------ */

/* F-01 (#245): real ReInitStack returns LmHandlerErrorStatus_t - injectable. */
LmHandlerErrorStatus_t LoRaApp_ReInitStack(LoRaMacRegion_t region)
{
    if (g_mac.fail_reinit) return LORAMAC_HANDLER_ERROR;
    LmHandlerParams.ActiveRegion = region;
    /* stack reinit wipes live MAC state... */
    g_mac.dev_addr = 0;
    g_mac.activation = ACTIVATION_TYPE_NONE;
    memset(g_mac.keys, 0, sizeof(g_mac.keys));
    memset(&g_mac.nvm, 0, sizeof(g_mac.nvm));
    /* ...but NEVER the injection knobs (faults persist across a restore). */
    return LORAMAC_HANDLER_SUCCESS;
}

bool g_erase_nvm_ok = true;
bool LoRaApp_EraseNvmSlots(void) { return g_erase_nvm_ok; }

LoRaMacRegion_t g_h3_region = LORAMAC_REGION_US915;
LoRaMacRegion_t MultiRegion_DetectFromGPS_H3(float lat, float lon) { (void)lat; (void)lon; return g_h3_region; }

int32_t EnvSensors_Read(sensor_t *sensor_data) { sensor_data->temperature = 20.0f; return 0; }

/* R47 precedent: Config_Get stubbed to NULL -> the macro-default fallback,
 * same as the main suite. The unit under test includes config.h itself. */
#include "config.h"
const SystemConfig_t *Config_Get(void) { return NULL; }

uint32_t g_fake_tick = 0;
uint32_t HAL_GetTick(void) { return g_fake_tick; }

IWDG_HandleTypeDef hiwdg = { 0 };
volatile uint8_t g_multiregion_join_success = 0;
volatile uint8_t g_multiregion_in_prejoin = 0;

/* RAM-backed internal flash: pages 120-127 live at 0x0803C000..0x08040000 */
#define FAKE_FLASH_BASE  0x0803C000UL
#define FAKE_FLASH_SIZE  (16u * 2048u)
static uint8_t g_flash[FAKE_FLASH_SIZE];

FLASH_IF_StatusTypedef FLASH_IF_Init(void *pAllocRamBuffer) { (void)pAllocRamBuffer; return FLASH_IF_OK; }
FLASH_IF_StatusTypedef FLASH_IF_DeInit(void) { return FLASH_IF_OK; }

static bool flash_addr_ok(const void *p, uint32_t len)
{
    uint32_t a = (uint32_t)(uintptr_t)p;
    return a >= FAKE_FLASH_BASE && (a + len) <= (FAKE_FLASH_BASE + FAKE_FLASH_SIZE);
}

FLASH_IF_StatusTypedef FLASH_IF_Write(void *dst, const void *src, uint32_t len)
{
    if (!flash_addr_ok(dst, len)) return FLASH_IF_ERROR;
    memcpy(&g_flash[(uint32_t)(uintptr_t)dst - FAKE_FLASH_BASE], src, len);
    return FLASH_IF_OK;
}

FLASH_IF_StatusTypedef FLASH_IF_Read(void *dst, const void *src, uint32_t len)
{
    if (!flash_addr_ok(src, len)) return FLASH_IF_ERROR;
    memcpy(dst, &g_flash[(uint32_t)(uintptr_t)src - FAKE_FLASH_BASE], len);
    return FLASH_IF_OK;
}

/* S-03 (#227): spies for the commit-back bound. */
uint32_t g_host_boot_attempts = 1;
static uint32_t g_flash_erase_count = 0;

FLASH_IF_StatusTypedef FLASH_IF_Erase(void *start, uint32_t len)
{
    if (!flash_addr_ok(start, len)) return FLASH_IF_ERROR;
    g_flash_erase_count++;
    memset(&g_flash[(uint32_t)(uintptr_t)start - FAKE_FLASH_BASE], 0xFF, len);
    return FLASH_IF_OK;
}

/* ------------------------------------------------------------------ */
/* Unit under test: the REAL multiregion_context.c                      */
/* ------------------------------------------------------------------ */

#include "../../Core/Src/multiregion_context.c"

/* ------------------------------------------------------------------ */
/* Test scaffolding                                                     */
/* ------------------------------------------------------------------ */

static int g_failures = 0;
static int g_checks = 0;
static int g_expected_failures = 0;

#define CHECK_REGRESSION(cond, id) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        g_expected_failures++; \
        printf("FAIL [%s] %s:%d: %s\n", id, __FILE__, __LINE__, #cond); \
    } \
} while (0)

static void mac_reset(void)
{
    /* Clean MAC state AND clear all injection knobs; the banked contexts in
     * (fake) flash are untouched, as on target. */
    memset(&g_mac, 0, sizeof(g_mac));
    g_mac.rx2_freq = 923300000;  /* sane defaults, as the real MAC would have */
    g_mac.rx2_dr = DR_8;
    g_h3_region = LORAMAC_REGION_US915;
}

/* Commission two region contexts through the real commissioning entry point
 * (all fakes succeeding). */
static bool commission_two_regions(void)
{
    static const uint8_t app_key_us[16]  = { 0xA0 };
    static const uint8_t nwk_key_us[16]  = { 0xB0 };
    static const uint8_t app_key_eu[16]  = { 0xC0 };
    static const uint8_t nwk_key_eu[16]  = { 0xD0 };
    if (!MultiRegion_InitializeRegionFromNetworkServer(LORAMAC_REGION_US915, 0x26011111, app_key_us, nwk_key_us)) return false;
    if (!MultiRegion_InitializeRegionFromNetworkServer(LORAMAC_REGION_EU868, 0x26012222, app_key_eu, nwk_key_eu)) return false;
    return true;
}

/* ========================================================================== */
/* F-008 (P2) — required key acquisition failure must fail closed              */
/* ========================================================================== */
/* Emulates a fresh OTAA join on a new region: MAC activated, keys live in
 * the active crypto context, no slot yet. With LmHandlerGetKey failing, the
 * save must abort, the slot must roll back to empty, and the region must
 * not become joined - then recover cleanly once the fault clears. */
static void test_f008_key_acquisition_fail_closed(void)
{
    printf("-- F-008 (P2): LmHandlerGetKey failure must abort the save fail-closed\n");

    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED\n"); exit(2); }

    g_mac.activation = ACTIVATION_TYPE_OTAA;
    g_mac.dev_addr = 0x26013333;
    LmHandlerParams.ActiveRegion = LORAMAC_REGION_AS923;
    memset(g_mac.keys[APP_S_KEY], 0xE0, 16);
    memset(g_mac.keys[NWK_S_KEY], 0xF0, 16);

    /* Guard: clean acquisition commits a valid bank. */
    CHECK_REGRESSION(MultiRegion_ForceSaveCurrentContext() == true, "F-008-guard");
    CHECK_REGRESSION(MultiRegion_IsRegionJoined(LORAMAC_REGION_AS923), "F-008-guard-joined");
    printf("   guard: clean OTAA-capture save committed AS923\n");

    /* Roll the slot back to fresh (same-TU statics) and inject the fault. */
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        if (g_storage.contexts[i].region == LORAMAC_REGION_AS923) {
            g_storage.contexts[i].dev_addr = 0;
            g_storage.num_valid--;
        }
    }
    g_mac.fail_getkey = true;
    CHECK_REGRESSION(MultiRegion_ForceSaveCurrentContext() == false, "F-008-save-aborts");
    g_mac.fail_getkey = false;
    CHECK_REGRESSION(!MultiRegion_IsRegionJoined(LORAMAC_REGION_AS923), "F-008-not-joined");

    /* The slot must be empty again (DevAddr==0 is the empty marker), so the
     * next boot cannot select a bank with missing key material. */
    bool slot_empty = false;
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        if (g_storage.contexts[i].region == LORAMAC_REGION_AS923 &&
            g_storage.contexts[i].dev_addr == 0) {
            slot_empty = true;
        }
    }
    CHECK_REGRESSION(slot_empty, "F-008-slot-rolled-back");

    /* Recovery: fault cleared, the same capture now succeeds. */
    CHECK_REGRESSION(MultiRegion_ForceSaveCurrentContext() == true, "F-008-recovery");
    CHECK_REGRESSION(MultiRegion_IsRegionJoined(LORAMAC_REGION_AS923), "F-008-recovery-joined");
    printf("   fail-closed abort verified; recovery after fault clear ok\n");
}

/* ========================================================================== */
/* R1 (P0) — restore must never report success after a failed step            */
/* ========================================================================== */
static void test_r1_restore_fail_closed(void)
{
    printf("-- R1 (P0): failed restore must fail closed (no success, no active mark)\n");

    /* Guard: a clean switch works and marks the target active. */
    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED: commissioning path\n"); exit(2); }
    mac_reset();
    CHECK_REGRESSION(MultiRegion_SwitchToRegion(LORAMAC_REGION_US915) == LORAMAC_HANDLER_SUCCESS, "R1-guard");
    printf("   guard: clean switch to US915 ok, active=%s\n", RegionToString(MultiRegion_GetActiveRegion()));

    /* Case A: the DevAddr MIB set fails (STEP 4/STEP 8). Pre-fix this was a
     * log line; the restore then returned SUCCESS and the caller marked the
     * region active on a dead DevAddr. */
    mac_reset();
    g_mac.fail_mib_set_devaddr = true;
    LmHandlerErrorStatus_t st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    printf("   DevAddr-set failure: switch rc=%d, active=%s (want rc!=0, active=US915)\n",
           st, RegionToString(MultiRegion_GetActiveRegion()));
    CHECK_REGRESSION(st != LORAMAC_HANDLER_SUCCESS, "R1a-rc");
    CHECK_REGRESSION(MultiRegion_GetActiveRegion() == LORAMAC_REGION_US915, "R1a-active");

    /* Case B: the secure element loses the session keys between write and
     * verify (STEP 10). Pre-fix: two ERROR logs, then SUCCESS. */
    mac_reset();
    g_mac.corrupt_keys_on_mib_get = true;
    st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    printf("   key-verify mismatch: switch rc=%d, active=%s (want rc!=0, active=US915)\n",
           st, RegionToString(MultiRegion_GetActiveRegion()));
    CHECK_REGRESSION(st != LORAMAC_HANDLER_SUCCESS, "R1b-rc");
    CHECK_REGRESSION(MultiRegion_GetActiveRegion() == LORAMAC_REGION_US915, "R1b-active");

    /* Case C: NVM context pointer unavailable at restore (STEP 5). */
    mac_reset();
    g_mac.null_nvm = true;
    st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    printf("   NULL NVM: switch rc=%d, active=%s (want rc!=0, active=US915)\n",
           st, RegionToString(MultiRegion_GetActiveRegion()));
    CHECK_REGRESSION(st != LORAMAC_HANDLER_SUCCESS, "R1c-rc");
    CHECK_REGRESSION(MultiRegion_GetActiveRegion() == LORAMAC_REGION_US915, "R1c-active");

    /* Case D (guard): MAC busy still reports BUSY, not a fake success. */
    mac_reset();
    g_mac.busy = true;
    st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    CHECK_REGRESSION(st == LORAMAC_HANDLER_BUSY_ERROR, "R1d-busy");

    /* Post-failure recovery: faults cleared, the same switch must succeed
     * (the banked contexts were not invalidated by the failed restores). */
    mac_reset();
    st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    printf("   recovery after cleared faults: rc=%d (want 0)\n", st);
    CHECK_REGRESSION(st == LORAMAC_HANDLER_SUCCESS, "R1e-recovery");
    CHECK_REGRESSION(MultiRegion_GetActiveRegion() == LORAMAC_REGION_EU868, "R1e-active");
}


/* ========================================================================== */
/* F-01 (#245) — restore steps 1-3 must fail closed too                        */
/* ========================================================================== */
/* R1 (#187) made restore steps 4-10 fail-closed, but the lifecycle calls at
 * the TOP of RestoreSessionToMac - LoRaApp_ReInitStack / LmHandlerConfigure /
 * LmHandlerSetDevEUI / LmHandlerSetKey - were still bare: a failed stack
 * teardown or configure fell through to SUCCESS and the caller marked the
 * region active on a dead session. Same matrix shape as R1. */
static void test_f01_restore_steps123_fail_closed(void)
{
    printf("-- F-01 (P1, #245): restore lifecycle steps 1-3 must fail closed\n");

    /* Guard: clean switch works (fresh commissioning, no faults). */
    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED: commissioning path\n"); exit(2); }
    mac_reset();
    CHECK_REGRESSION(MultiRegion_SwitchToRegion(LORAMAC_REGION_US915) == LORAMAC_HANDLER_SUCCESS, "F-01-guard");

    /* Step 1: stack reinit fails. */
    mac_reset();
    g_mac.fail_reinit = true;
    LmHandlerErrorStatus_t st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    printf("   reinit failure: rc=%d, active=%s (want rc!=0, active=US915)\n",
           st, RegionToString(MultiRegion_GetActiveRegion()));
    CHECK_REGRESSION(st != LORAMAC_HANDLER_SUCCESS, "F-01a-rc");
    CHECK_REGRESSION(MultiRegion_GetActiveRegion() == LORAMAC_REGION_US915, "F-01a-active");

    /* Step 2: handler configure fails. */
    mac_reset();
    g_mac.fail_configure = true;
    st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    printf("   configure failure: rc=%d, active=%s\n", st, RegionToString(MultiRegion_GetActiveRegion()));
    CHECK_REGRESSION(st != LORAMAC_HANDLER_SUCCESS, "F-01b-rc");
    CHECK_REGRESSION(MultiRegion_GetActiveRegion() == LORAMAC_REGION_US915, "F-01b-active");

    /* Step 3a: DevEUI set fails. */
    mac_reset();
    g_mac.fail_setdeveui = true;
    st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    CHECK_REGRESSION(st != LORAMAC_HANDLER_SUCCESS, "F-01c-rc");
    CHECK_REGRESSION(MultiRegion_GetActiveRegion() == LORAMAC_REGION_US915, "F-01c-active");

    /* Step 3b: session-key set fails. */
    mac_reset();
    g_mac.fail_setkey = true;
    st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    CHECK_REGRESSION(st != LORAMAC_HANDLER_SUCCESS, "F-01d-rc");
    CHECK_REGRESSION(MultiRegion_GetActiveRegion() == LORAMAC_REGION_US915, "F-01d-active");

    /* Recovery: faults cleared, the banked switch succeeds. */
    mac_reset();
    st = MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868);
    CHECK_REGRESSION(st == LORAMAC_HANDLER_SUCCESS, "F-01e-recovery");
    CHECK_REGRESSION(MultiRegion_GetActiveRegion() == LORAMAC_REGION_EU868, "F-01e-active");
}

/* ========================================================================== */
/* R3 (P0) — FCnt restore must resume AHEAD of the true network counter       */
/* ========================================================================== */

/* Saves batch every FRAME_COUNTER_SAVE_INTERVAL (10) TXs and persist the TRUE
 * counter; a reset mid-batch resumes from the persisted value with no margin,
 * so the next uplinks reuse counters the NS already saw (dropped), and a reset
 * loop that never reaches the next save boundary replays the same band
 * FOREVER - permanent silent network loss. The +INTERVAL bump exists only on
 * the Tier-2-LOST degrade path, not the normal restore.
 */
static void test_r3_fcnt_reset_margin(void)
{
    printf("-- R3 (P0): FCnt restore must resume ahead of the true counter\n");

    /* Fresh commissioning + clean switch to US915. */
    memset(g_flash, 0xFF, sizeof(g_flash));
    g_initialized = false;
    mac_reset();
    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED: commissioning path\n"); exit(2); }
    mac_reset();
    if (MultiRegion_SwitchToRegion(LORAMAC_REGION_US915) != LORAMAC_HANDLER_SUCCESS) {
        printf("   SETUP FAILED: switch\n"); exit(2);
    }
    int8_t slot = -1;
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        if (g_storage.contexts[i].region == LORAMAC_REGION_US915 &&
            g_storage.contexts[i].dev_addr == 0x26011111UL) { slot = (int8_t)i; break; }
    }
    if (slot < 0) { printf("   SETUP FAILED: slot\n"); exit(2); }

    /* 10 TXs -> one real Tier-2 save; persisted counter == true counter. */
    for (int i = 0; i < 10; i++) {
        g_mac.nvm.Crypto.FCntList.FCntUp++;
        g_mac.nvm.Crypto.FCntList.NFCntDown++;
        MultiRegion_SaveCurrentContext();
    }
    uint32_t persisted = g_storage.contexts[slot].uplink_counter;
    printf("   after 10 TXs: persisted FCntUp=%lu\n", (unsigned long)persisted);

    /* 7 more TXs (unsaved batch), then a reset. NS last saw persisted+7. */
    for (int i = 0; i < 7; i++) {
        g_mac.nvm.Crypto.FCntList.FCntUp++;
        MultiRegion_SaveCurrentContext();
    }
    uint32_t true_counter = persisted + 7;

    /* Simulated reset: RAM statics wiped, re-init from flash only. */
    g_initialized = false;
    g_unsaved_tx_count = 0;
    memset(&g_storage, 0, sizeof(g_storage));
    MultiRegion_Init();

    uint32_t resumed = g_storage.contexts[slot].uplink_counter;
    printf("   reset mid-batch: NS last saw %lu, device resumed at %lu (want > %lu)\n",
           (unsigned long)true_counter, (unsigned long)resumed, (unsigned long)true_counter);
    CHECK_REGRESSION(resumed > true_counter, "R3-margin");

    /* Monotonicity across a SECOND mid-batch reset (the loop case): the
     * counter must advance again, never replay the same band. */
    g_mac.nvm.Crypto.FCntList.FCntUp = resumed;   /* MAC adopts the restored value */
    for (int i = 0; i < 5; i++) {
        g_mac.nvm.Crypto.FCntList.FCntUp++;
        MultiRegion_SaveCurrentContext();
    }
    uint32_t true2 = resumed + 5;
    g_initialized = false;

    g_unsaved_tx_count = 0;
    memset(&g_storage, 0, sizeof(g_storage));
    MultiRegion_Init();
    uint32_t resumed2 = g_storage.contexts[slot].uplink_counter;
    printf("   second reset: NS last saw %lu, resumed at %lu (want > %lu)\n",
           (unsigned long)true2, (unsigned long)resumed2, (unsigned long)true2);
    CHECK_REGRESSION(resumed2 > true2, "R3-margin-chain");
}

/* ========================================================================== */
/* DR-07 (#239) — the C6/R3 margin must be applied EXACTLY ONCE               */
/* ========================================================================== */
/* MultiRegion_SaveCurrentContext advanced ctx->uplink_counter by INTERVAL and
 * persisted the MARGINED value; the restore path then added INTERVAL again on
 * top of it (the R3 comment claiming "saves persist the TRUE value" was
 * false). Every reset therefore burned INTERVAL of FCntUp gap whether or not
 * an uplink occurred; DDR-0006 forbids in-flight rejoin, so ~16384/INTERVAL
 * silent resets is a permanently dead session. The margin belongs on the
 * restore side only: the reset creates the exposure, not the save. */
static void test_dr07_single_margin(void)
{
    printf("-- DR-07 (#239): margin applied exactly once (restore side only)\n");

    /* Fresh commissioning + clean switch to US915 (same setup as R3). */
    memset(g_flash, 0xFF, sizeof(g_flash));
    g_initialized = false;
    mac_reset();
    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED: commissioning path\n"); exit(2); }
    mac_reset();
    if (MultiRegion_SwitchToRegion(LORAMAC_REGION_US915) != LORAMAC_HANDLER_SUCCESS) {
        printf("   SETUP FAILED: switch\n"); exit(2);
    }
    int8_t slot = -1;
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        if (g_storage.contexts[i].region == LORAMAC_REGION_US915 &&
            g_storage.contexts[i].dev_addr == 0x26011111UL) { slot = (int8_t)i; break; }
    }
    if (slot < 0) { printf("   SETUP FAILED: slot\n"); exit(2); }

    /* 10 TXs -> one real Tier-2 save. The persisted value must be the TRUE
     * counter - no save-side margin. */
    for (int i = 0; i < 10; i++) {
        g_mac.nvm.Crypto.FCntList.FCntUp++;
        g_mac.nvm.Crypto.FCntList.NFCntDown++;
        MultiRegion_SaveCurrentContext();
    }
    uint32_t true_at_save = g_mac.nvm.Crypto.FCntList.FCntUp;
    uint32_t persisted = g_storage.contexts[slot].uplink_counter;
    printf("   after 10 TXs: true=%lu persisted=%lu (want equal)\n",
           (unsigned long)true_at_save, (unsigned long)persisted);
    CHECK_REGRESSION(persisted == true_at_save, "DR-07-save-persists-true");

    /* Reset: the restore must add EXACTLY one INTERVAL to the persisted
     * (true) value - not two. */
    g_initialized = false;
    g_unsaved_tx_count = 0;
    memset(&g_storage, 0, sizeof(g_storage));
    MultiRegion_Init();
    uint32_t resumed = g_storage.contexts[slot].uplink_counter;
    uint32_t want = persisted + CfgFrameCounterSaveInterval();
    printf("   resumed=%lu want=%lu (persisted + one INTERVAL)\n",
           (unsigned long)resumed, (unsigned long)want);
    CHECK_REGRESSION(resumed == want, "DR-07-single-margin");
}

/* ========================================================================== */
/* S-03 (P2, #227) — Tier-2 commit-back must be bounded in a reset loop       */
/* ========================================================================== */
static void test_s03_tier2_commit_back_bounded(void)
{
    printf("-- S-03 (P2, #227): Tier-2 commit-back bounded in a reset loop\n");

    /* Fresh commissioning + clean switch + 10 TXs (one real Tier-2 save). */
    memset(g_flash, 0xFF, sizeof(g_flash));
    g_initialized = false;
    mac_reset();
    g_host_boot_attempts = 1;
    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED: commissioning path\n"); exit(2); }
    mac_reset();
    if (MultiRegion_SwitchToRegion(LORAMAC_REGION_US915) != LORAMAC_HANDLER_SUCCESS) {
        printf("   SETUP FAILED: switch\n"); exit(2);
    }
    for (int i = 0; i < 10; i++) {
        g_mac.nvm.Crypto.FCntList.FCntUp++;
        MultiRegion_SaveCurrentContext();
    }

    /* A. Normal boot (single reset): the R3 commit-back must STILL fire. */
    uint32_t e0 = g_flash_erase_count;
    g_host_boot_attempts = 1;
    g_initialized = false;
    g_unsaved_tx_count = 0;
    memset(&g_storage, 0, sizeof(g_storage));
    MultiRegion_Init();
    printf("   normal boot: erase count %lu -> %lu (want advanced)\n",
           (unsigned long)e0, (unsigned long)g_flash_erase_count);
    CHECK_REGRESSION(g_flash_erase_count > e0, "S-03-normal");

    /* B. Sustained reset loop (boot attempts > 4): commit-back skipped -
     *    the flash page must not be touched. */
    g_host_boot_attempts = 5;
    e0 = g_flash_erase_count;
    g_initialized = false;
    g_unsaved_tx_count = 0;
    memset(&g_storage, 0, sizeof(g_storage));
    MultiRegion_Init();
    printf("   reset loop (attempts=5): erase count %lu -> %lu (want unchanged)\n",
           (unsigned long)e0, (unsigned long)g_flash_erase_count);
    CHECK_REGRESSION(g_flash_erase_count == e0, "S-03-loop");
    g_host_boot_attempts = 1;
}

/* ========================================================================== */
/* R2 (P1-high) — CRC-valid but semantically dead contexts must be rejected   */
/* ========================================================================== */
/* ValidateContextCRC proves only that the bytes survived. Zero the session
 * keys of a commissioned context and re-stamp a VALID CRC with the real
 * UpdateContextCRC: the context is CRC-valid and semantically dead.
 * IsRegionJoined / SwitchToRegion must reject it before any restore runs.
 */
static void test_r2_semantic_validation(void)
{
    printf("-- R2 (P1-high): CRC-valid but semantically dead context must be rejected\n");

    memset(g_flash, 0xFF, sizeof(g_flash));
    g_initialized = false;
    mac_reset();
    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED\n"); exit(2); }

    int8_t slot = -1;
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        if (g_storage.contexts[i].region == LORAMAC_REGION_US915 &&
            g_storage.contexts[i].dev_addr == 0x26011111UL) { slot = (int8_t)i; break; }
    }
    if (slot < 0) { printf("   SETUP FAILED: slot\n"); exit(2); }

    /* Guard: the healthy context is joined. */
    CHECK_REGRESSION(MultiRegion_IsRegionJoined(LORAMAC_REGION_US915), "R2-guard");

    /* Poison: keys all-zero, CRC re-stamped VALID (incomplete-commissioning
     * pattern). */
    memset(g_storage.contexts[slot].app_s_key, 0x00, 16);
    memset(g_storage.contexts[slot].nwk_s_key, 0x00, 16);
    UpdateContextCRC(&g_storage.contexts[slot]);
    printf("   zeroed keys + valid CRC: IsRegionJoined=%s (want no)\n",
           MultiRegion_IsRegionJoined(LORAMAC_REGION_US915) ? "yes" : "no");
    CHECK_REGRESSION(!MultiRegion_IsRegionJoined(LORAMAC_REGION_US915), "R2a-keys");

    /* Restore the keys, poison DevAddr instead (broadcast/erased pattern). */
    commission_two_regions();
    memset(g_flash, 0xFF, sizeof(g_flash));
    g_initialized = false;
    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED 2\n"); exit(2); }
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        if (g_storage.contexts[i].region == LORAMAC_REGION_US915 &&
            g_storage.contexts[i].dev_addr == 0x26011111UL) { slot = (int8_t)i; break; }
    }
    g_storage.contexts[slot].dev_addr = 0xFFFFFFFFUL;
    UpdateContextCRC(&g_storage.contexts[slot]);
    printf("   DevAddr=0xFFFFFFFF + valid CRC: IsRegionJoined=%s (want no)\n",
           MultiRegion_IsRegionJoined(LORAMAC_REGION_US915) ? "yes" : "no");
    CHECK_REGRESSION(!MultiRegion_IsRegionJoined(LORAMAC_REGION_US915), "R2b-devaddr");

    /* Poisoned context must not be switchable either (belt: the CRC gate in
     * SwitchToRegion is a second call site). */
    LmHandlerErrorStatus_t st = MultiRegion_SwitchToRegion(LORAMAC_REGION_US915);
    CHECK_REGRESSION(st != LORAMAC_HANDLER_SUCCESS, "R2c-switch");
}

static void test_r8_tier1_generation_order(void);
static void test_r11_capture_restore_symmetry(void);

int main(void)
{
    printf("=== R15 harness: real multiregion_context.c vs fake LoRaMac ===\n\n");
    memset(g_flash, 0xFF, sizeof(g_flash));

    test_f008_key_acquisition_fail_closed();
printf("\n");
test_r1_restore_fail_closed();
test_f01_restore_steps123_fail_closed();
    printf("\n");
    test_r3_fcnt_reset_margin();
    test_dr07_single_margin();
    test_s03_tier2_commit_back_bounded();
    printf("\n");
    test_r2_semantic_validation();
    printf("\n");
    test_r8_tier1_generation_order();
    printf("\n");
    test_r11_capture_restore_symmetry();

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}

/* ========================================================================== */
/* R8 (P1) — newest CRC-valid Tier-1 copy must win, not first-valid           */
/* ========================================================================== */
/* Tier1Bank_t has CRCs but no generation: FlashReadTier1 takes the FIRST
 * CRC-valid copy. After a torn re-commission (copy A still holds the OLD
 * bank, B/C the NEW one), the stale bank resurrects and the repair path
 * propagates it over the good copies.
 */
static void test_r8_tier1_generation_order(void)
{
    printf("-- R8 (P1): newest CRC-valid Tier-1 copy must win, not first-valid\n");

    memset(g_flash, 0xFF, sizeof(g_flash));
    g_initialized = false;
    mac_reset();
    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED\n"); exit(2); }

    /* Re-commissioning on target always follows a boot, and boot READS the
     * banks (seeding the generation counter) - emulate that reboot so the
     * rewrite below stamps a genuinely newer generation. */
    g_initialized = false;
    memset(&g_storage, 0, sizeof(g_storage));
    MultiRegion_Init();

    int8_t slot = -1;
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        if (g_storage.contexts[i].region == LORAMAC_REGION_US915 &&
            g_storage.contexts[i].dev_addr == 0x26011111UL) { slot = (int8_t)i; break; }
    }
    if (slot < 0) { printf("   SETUP FAILED: slot\n"); exit(2); }

    /* Snapshot the current (old-generation) bank bytes. */
    Tier1Bank_t old_bank;
    if (FLASH_IF_Read(&old_bank, (void*)TIER1_ADDRS[1], sizeof(Tier1Bank_t)) != FLASH_IF_OK) {
        printf("   SETUP FAILED: read\n"); exit(2);
    }
    uint32_t old_addr = old_bank.contexts[slot].dev_addr;

    /* Re-commission: new DevAddr for US915, rewrite Tier-1 (new generation). */
    g_storage.contexts[slot].dev_addr = 0x26019999UL;
    UpdateContextCRC(&g_storage.contexts[slot]);
    g_tier1_dirty = true;
    if (!FlashWriteTier1()) { printf("   SETUP FAILED: rewrite\n"); exit(2); }

    /* Torn re-commission: copy A still holds the OLD bank, B/C the NEW. */
    FLASH_IF_Erase((void*)TIER1_ADDRS[0], MULTIREGION_FLASH_PAGE_SIZE);
    FLASH_IF_Write((void*)TIER1_ADDRS[0], &old_bank, sizeof(Tier1Bank_t));

    /* Reboot. */
    g_initialized = false;
    memset(&g_storage, 0, sizeof(g_storage));
    MultiRegion_Init();

    uint32_t got = g_storage.contexts[slot].dev_addr;
    printf("   copy A=old(0x%08lX), B/C=new(0x%08lX): restored 0x%08lX (want new)\n",
           (unsigned long)old_addr, 0x26019999UL, (unsigned long)got);
    CHECK_REGRESSION(got == 0x26019999UL, "R8-newest-wins");

    /* The repair path must have propagated the NEW bank to copy A (not the
     * old one over B/C). */
    Tier1Bank_t repaired;
    FLASH_IF_Read(&repaired, (void*)TIER1_ADDRS[0], sizeof(Tier1Bank_t));
    printf("   copy A after repair: DevAddr=0x%08lX (want new)\n",
           (unsigned long)repaired.contexts[slot].dev_addr);
    CHECK_REGRESSION(repaired.contexts[slot].dev_addr == 0x26019999UL, "R8-repair");
}

/* ========================================================================== */
/* R11 (P1/P2) — capture and restore must be symmetric (round-trip)            */
/* ========================================================================== */
/* CaptureCurrentContext records datarate/tx_power/ADR/RX2 alongside counters
 * and LastRxMic, but RestoreSessionToMac did NOT restore those radio params
 * into the MAC - after a switch/reset the session ran on default radio
 * params. Round-trip: capture -> destroy runtime MAC state -> restore ->
 * recapture; every persisted field must survive.
 */
static void test_r11_capture_restore_symmetry(void)
{
    printf("-- R11 (P1/P2): capture -> destroy -> restore -> recapture round-trip\n");

    memset(g_flash, 0xFF, sizeof(g_flash));
    g_initialized = false;
    mac_reset();
    MultiRegion_Init();
    if (!commission_two_regions()) { printf("   SETUP FAILED\n"); exit(2); }
    mac_reset();
    if (MultiRegion_SwitchToRegion(LORAMAC_REGION_US915) != LORAMAC_HANDLER_SUCCESS) {
        printf("   SETUP FAILED: switch\n"); exit(2);
    }
    int8_t slot = -1;
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        if (g_storage.contexts[i].region == LORAMAC_REGION_US915 &&
            g_storage.contexts[i].dev_addr == 0x26011111UL) { slot = (int8_t)i; break; }
    }
    if (slot < 0) { printf("   SETUP FAILED: slot\n"); exit(2); }

    /* Distinctive live session state in the runtime MAC. */
    g_mac.channels_datarate = DR_3;
    g_mac.channels_tx_power = 5;
    g_mac.adr_enable = true;
    g_mac.rx2_freq = 866550000;
    g_mac.rx2_dr = DR_2;
    g_mac.nvm.Crypto.FCntList.FCntUp = 42;
    g_mac.nvm.Crypto.FCntList.NFCntDown = 7;
    g_mac.nvm.MacGroup1.LastRxMic = 0x00C0FFEE;

    /* Capture + persist. */
    if (!MultiRegion_ForceSaveCurrentContext()) { printf("   SETUP FAILED: capture\n"); exit(2); }
    MinimalRegionContext_t before = g_storage.contexts[slot];

    /* Destroy runtime MAC state (what a reset/reinit does). */
    g_mac.channels_datarate = 0;
    g_mac.channels_tx_power = 0;
    g_mac.adr_enable = false;
    g_mac.rx2_freq = 0;
    g_mac.rx2_dr = 0;
    memset(&g_mac.nvm, 0, sizeof(g_mac.nvm));

    /* Restore via switch away + back. */
    if (MultiRegion_SwitchToRegion(LORAMAC_REGION_EU868) != LORAMAC_HANDLER_SUCCESS ||
        MultiRegion_SwitchToRegion(LORAMAC_REGION_US915) != LORAMAC_HANDLER_SUCCESS) {
        printf("   SETUP FAILED: restore switch\n"); exit(2);
    }

    /* Recapture and compare every persisted field. */
    if (!MultiRegion_ForceSaveCurrentContext()) { printf("   SETUP FAILED: recapture\n"); exit(2); }
    const MinimalRegionContext_t *after = &g_storage.contexts[slot];

    printf("   FCntUp %lu->%lu, FCntDown %lu->%lu, MIC %lX->%lX\n",
           (unsigned long)before.uplink_counter, (unsigned long)after->uplink_counter,
           (unsigned long)before.downlink_counter, (unsigned long)after->downlink_counter,
           (unsigned long)before.last_rx_mic, (unsigned long)after->last_rx_mic);
    printf("   DR %d->%d, TXP %d->%d, ADR %d->%d, RX2 %lu/DR%lu->%lu/DR%lu\n",
           before.datarate, after->datarate, before.tx_power, after->tx_power,
           before.adr_enabled, after->adr_enabled,
           (unsigned long)before.rx2_frequency, (unsigned long)before.rx2_datarate,
           (unsigned long)after->rx2_frequency, (unsigned long)after->rx2_datarate);

    /* Counters: the save path deliberately advances FCntUp by the save
     * interval (R3 reserved block), so the invariant is monotonicity, never
     * regression - exact values belong to the R3 test. */
    CHECK_REGRESSION(after->uplink_counter >= before.uplink_counter &&
                     before.uplink_counter != 0, "R11-fcntup");
    CHECK_REGRESSION(after->downlink_counter == before.downlink_counter, "R11-fcntdown");
    CHECK_REGRESSION(after->last_rx_mic == 0x00C0FFEE, "R11-mic");
    CHECK_REGRESSION(after->datarate == DR_3, "R11-datarate");
    CHECK_REGRESSION(after->tx_power == 5, "R11-txpower");
    CHECK_REGRESSION(after->adr_enabled == 1, "R11-adr");
    CHECK_REGRESSION(after->rx2_frequency == 866550000UL, "R11-rx2freq");
    CHECK_REGRESSION(after->rx2_datarate == DR_2, "R11-rx2dr");
    (void)before;
}

