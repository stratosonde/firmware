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
    /* fault injection */
    bool fail_mib_set_devaddr;      /* every MIB_DEV_ADDR set fails */
    bool fail_mib_set_activation;   /* every MIB_NETWORK_ACTIVATION set fails */
    bool corrupt_keys_on_mib_get;   /* NVM readback shows wrong session keys */
    bool null_nvm;                  /* MIB_NVM_CTXS get returns NULL contexts */
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
    case MIB_RX2_CHANNEL:
        mib->Param.Rx2Channel.Frequency = 923300000;
        mib->Param.Rx2Channel.Datarate = DR_8;
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

void LmHandlerConfigure(LmHandlerParams_t *params) { (void)params; }

LmHandlerErrorStatus_t LmHandlerSetKey(KeyIdentifier_t keyId, uint8_t *key)
{
    if (keyId >= KEY_LIST_SIZE || key == NULL) return LORAMAC_HANDLER_ERROR;
    memcpy(g_mac.keys[keyId], key, 16);
    /* SetKey also lands in the NVM mirror, as on target */
    memcpy(g_mac.nvm.SecureElement.KeyList[keyId].KeyValue, key, 16);
    return LORAMAC_HANDLER_SUCCESS;
}

LmHandlerErrorStatus_t LmHandlerGetKey(KeyIdentifier_t keyId, uint8_t *key)
{
    if (keyId >= KEY_LIST_SIZE || key == NULL) return LORAMAC_HANDLER_ERROR;
    memcpy(key, g_mac.keys[keyId], 16);
    return LORAMAC_HANDLER_SUCCESS;
}

void LmHandlerSetDevEUI(uint8_t *devEui) { memcpy(g_mac.dev_eui, devEui, 8); }
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

void LoRaApp_ReInitStack(LoRaMacRegion_t region)
{
    LmHandlerParams.ActiveRegion = region;
    /* stack reinit wipes live MAC state... */
    g_mac.dev_addr = 0;
    g_mac.activation = ACTIVATION_TYPE_NONE;
    memset(g_mac.keys, 0, sizeof(g_mac.keys));
    memset(&g_mac.nvm, 0, sizeof(g_mac.nvm));
    /* ...but NEVER the injection knobs (faults persist across a restore). */
}

bool g_erase_nvm_ok = true;
bool LoRaApp_EraseNvmSlots(void) { return g_erase_nvm_ok; }

LoRaMacRegion_t g_h3_region = LORAMAC_REGION_US915;
LoRaMacRegion_t MultiRegion_DetectFromGPS_H3(float lat, float lon) { (void)lat; (void)lon; return g_h3_region; }

int32_t EnvSensors_Read(sensor_t *sensor_data) { sensor_data->temperature = 20.0f; return 0; }

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

FLASH_IF_StatusTypedef FLASH_IF_Erase(void *start, uint32_t len)
{
    if (!flash_addr_ok(start, len)) return FLASH_IF_ERROR;
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

int main(void)
{
    printf("=== R15 harness: real multiregion_context.c vs fake LoRaMac ===\n\n");
    memset(g_flash, 0xFF, sizeof(g_flash));

    test_r1_restore_fail_closed();
    printf("\n");
    test_r3_fcnt_reset_margin();
    printf("\n");
    test_r2_semantic_validation();

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}

