/**
  ******************************************************************************
  * @file    test_multiregion.c
  * @brief   R15 (ChatGPT review 2026-08-11): compile the REAL
  *          Core/Src/multiregion_context.c against a fake LoRaMac/LmHandler
  *          surface with fault injection, and drive the recovery matrix the
  *          source scans cannot prove.
  *
  *          First finding covered: R1 (P0) — session restore reported success
  *          after restoration or verification failure. RestoreSessionToMac
  *          logged MIB-set / DevAddr-verify / key-verify failures and fell
  *          through to LORAMAC_HANDLER_SUCCESS; MultiRegion_SwitchToRegion
  *          then marked the region ACTIVE on a dead session -> silent RF
  *          loss with no recovery path.
  *
  *          Every check is CHECK_REGRESSION (expected-fail-before-fix):
  *            make -C tests/host multiregion      (red until the fix lands)
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

static void mac_reset(void);
static bool commission_two_regions(void);

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

int main(void)
{
    printf("=== R15 harness: real multiregion_context.c vs fake LoRaMac ===\n\n");
    memset(g_flash, 0xFF, sizeof(g_flash));

    test_r1_restore_fail_closed();

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}

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
