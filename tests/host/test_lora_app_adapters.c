/* MAINT-01/02/03 (pretest-hardening handoff 2026-08-15, Phase 5): linked
 * behavioral tests for the production adapter module (lora_app_adapters.c)
 * and the semantic FSM queries (tx_fsm.c). Every polarity-bearing mapping
 * is exercised in BOTH directions, so reversing any one of them fails here
 * (the Phase-5 gate runs exactly those reversals on the box).
 *
 * Structural pins (regression guards only, F-010): lora_app.c delegates each
 * of the five mappings to the builders; LoRaMacInterfaces.h keeps
 * LORAMAC_EVENT_INFO_STATUS_OK == 0 (the APP_TX_STATUS_OK mirror); Dispatch
 * reuses the exported stale predicates (MAINT-02). */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "lora_app_adapters.h"
#include "tx_fsm.h"

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

/* ---- source-scan helpers (same shape as test_geo_20260814.c) ---- */
#define SCAN_POOL_MAX 8
static void *g_scan_pool[SCAN_POOL_MAX];
static int g_scan_pool_n;
static void scan_pool_free(void)
{
    while (g_scan_pool_n > 0) {
        free(g_scan_pool[--g_scan_pool_n]);
    }
}
static void scan_pool_track(void *p)
{
    if (g_scan_pool_n < SCAN_POOL_MAX) {
        g_scan_pool[g_scan_pool_n++] = p;
    }
}
static char *slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { printf("FATAL: cannot open %s\n", path); exit(2); }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char *)malloc((size_t)n + 1);
    if (!buf) exit(2);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) exit(2);
    buf[n] = '\0';
    fclose(f);
    scan_pool_track(buf);
    {   /* Sources may be CRLF; scan anchors are written LF. */
        size_t r = 0, w = 0;
        while (r < (size_t)n) {
            if (buf[r] != '\r') buf[w++] = buf[r];
            r++;
        }
        buf[w] = '\0';
    }
    return buf;
}


/* ---- 1. confirm mapping (TX-ADAPTER-01's polarity surface) ---- */
static void test_build_tx_confirm(void)
{
    printf("-- BuildTxConfirm: status/ack/battery/mission polarity\n");

    AppTxConfirmSnapshot_t base = {
        .now_ms = 123456u,
        .tx_status = APP_TX_STATUS_OK,
        .ack_received_raw = 1,
        .battery_mv = 3000,
        .bulk_batt_min_mv = 2800,
        .mission_state = MISSION_FLOAT,
        .has_cache = true,
        .max_bulk_packets = 8
    };

    TxFsmConfirmInput_t in = AppAdapters_BuildTxConfirm(&base);
    CHECK(in.now_ms == 123456u);
    CHECK(in.status_ok == true);
    CHECK(in.ack_received == true);
    CHECK(in.battery_good == true);
    CHECK(in.has_unsent == true);
    CHECK(in.mission_ascent == false); /* FLOAT must not read as ascent */
    CHECK(in.max_bulk_packets == 8);

    /* R3-03: the raw ASCENT enum maps to mission_ascent == true, and only
     * ASCENT does (COMMISSIONING must not either). */
    AppTxConfirmSnapshot_t s = base;
    s.mission_state = MISSION_ASCENT;
    CHECK(AppAdapters_BuildTxConfirm(&s).mission_ascent == true);
    s.mission_state = MISSION_COMMISSIONING;
    CHECK(AppAdapters_BuildTxConfirm(&s).mission_ascent == false);

    /* status: only the OK value is ok */
    s = base;
    s.tx_status = APP_TX_STATUS_OK + 1;
    CHECK(AppAdapters_BuildTxConfirm(&s).status_ok == false);
    s.tx_status = -1;
    CHECK(AppAdapters_BuildTxConfirm(&s).status_ok == false);

    /* ack: raw nonzero is ack (the Mac uses 0/1, but != 0 is the contract) */
    s = base;
    s.ack_received_raw = 0;
    CHECK(AppAdapters_BuildTxConfirm(&s).ack_received == false);
    s.ack_received_raw = 2;
    CHECK(AppAdapters_BuildTxConfirm(&s).ack_received == true);

    /* battery: boundary is inclusive (mv >= min is good) */
    s = base;
    s.battery_mv = s.bulk_batt_min_mv;
    CHECK(AppAdapters_BuildTxConfirm(&s).battery_good == true);
    s.battery_mv = (uint16_t)(s.bulk_batt_min_mv - 1u);
    CHECK(AppAdapters_BuildTxConfirm(&s).battery_good == false);

    /* has_cache polarity: true must not arrive negated */
    s = base;
    s.has_cache = false;
    CHECK(AppAdapters_BuildTxConfirm(&s).has_unsent == false);
}


/* ---- 2. cycle mapping ---- */
static void test_build_tx_cycle(void)
{
    printf("-- BuildTxCycle: marshal + interval_ms always 0\n");

    AppTxCycleSnapshot_t snap = {
        .now_ms = 777u,
        .rf_silence = true,
        .has_unsent = true,
        .recovery_empty = true,
        .unconvertible = true,
        .no_budget = true,
        .max_bulk_packets = 5,
        .burst_max_open_ms = 60000u
    };
    TxFsmCycleInput_t in = AppAdapters_BuildTxCycle(&snap);
    CHECK(in.now_ms == 777u);
    CHECK(in.interval_ms == 0); /* dispatch phase never reschedules */
    CHECK(in.rf_silence == true);
    CHECK(in.has_unsent == true);
    CHECK(in.recovery_empty == true);
    CHECK(in.unconvertible == true);
    CHECK(in.no_budget == true);
    CHECK(in.max_bulk_packets == 5);
    CHECK(in.burst_max_open_ms == 60000u);

    memset(&snap, 0, sizeof(snap));
    in = AppAdapters_BuildTxCycle(&snap);
    CHECK(in.rf_silence == false);
    CHECK(in.has_unsent == false);
    CHECK(in.recovery_empty == false);
    CHECK(in.unconvertible == false);
    CHECK(in.no_budget == false);
    CHECK(in.interval_ms == 0);
}

/* ---- 3. RX mapping ---- */
static void test_build_rx(void)
{
    printf("-- BuildRx: field identity (margin/gateways must not swap)\n");

    AppRxSnapshot_t snap = {
        .linkcheck_received = true,
        .margin = 11,
        .gateways = 2,
        .margin_min = 10,
        .gateways_min = 1,
        .has_unsent = true,
        .max_bulk_packets = 4
    };
    TxFsmRxInput_t in = AppAdapters_BuildRx(&snap);
    CHECK(in.linkcheck_received == true);
    CHECK(in.margin == 11);
    CHECK(in.gateways == 2);
    CHECK(in.margin_min == 10);
    CHECK(in.gateways_min == 1);
    CHECK(in.has_unsent == true);
    CHECK(in.max_bulk_packets == 4);

    snap.linkcheck_received = false;
    snap.has_unsent = false;
    in = AppAdapters_BuildRx(&snap);
    CHECK(in.linkcheck_received == false);
    CHECK(in.has_unsent == false);
}

/* ---- 4. region comparisons / final authorization ---- */
static void test_region_comparators(void)
{
    printf("-- Region comparators: differs / required / matches\n");

    CHECK(AppAdapters_RegionDiffers(1, 2) == true);
    CHECK(AppAdapters_RegionDiffers(2, 2) == false);
    CHECK(AppAdapters_SwitchWasRequired(1, 2) == true);
    CHECK(AppAdapters_SwitchWasRequired(3, 3) == false);
    CHECK(AppAdapters_ActiveMatchesDetected(2, 2) == true);
    CHECK(AppAdapters_ActiveMatchesDetected(1, 2) == false);
}

/* ---- 5. first-flight admission mapping ---- */
static void test_build_first_flight(void)
{
    printf("-- BuildFirstFlightAdmission: stale==0 means fresh\n");

    AppFirstFlightSnapshot_t snap = {
        .temperature_c = -42.5f,
        .temperature_stale = 0,
        .battery_mv_raw = 3100,
        .battery_stale = 0
    };
    FirstFlightAdmissionInput_t in =
        AppAdapters_BuildFirstFlightAdmission(&snap);
    CHECK(in.temperature_c == -42.5f);
    CHECK(in.temperature_fresh == true);
    CHECK(in.battery_mv_raw == 3100);
    CHECK(in.battery_fresh == true);

    snap.temperature_stale = 1;
    snap.battery_stale = 7; /* any nonzero counter is stale */
    in = AppAdapters_BuildFirstFlightAdmission(&snap);
    CHECK(in.temperature_fresh == false);
    CHECK(in.battery_fresh == false);
}

/* ---- MAINT-03: semantic queries against the real tx_fsm.c ---- */
static void test_fsm_queries(void)
{
    printf("-- MAINT-03: TxFsm_InBulk / WaitingForProbeAck / BulkPacketsSent\n");

    TxFsm_t fsm;
    TxFsm_Init(&fsm);
    CHECK(TxFsm_InBulk(&fsm) == false);
    CHECK(TxFsm_WaitingForProbeAck(&fsm) == false);
    CHECK(TxFsm_BulkPacketsSent(&fsm) == 0);

    fsm.state = TX_FSM_WAIT_PROBE_ACK;
    CHECK(TxFsm_WaitingForProbeAck(&fsm) == true);
    CHECK(TxFsm_InBulk(&fsm) == false);

    fsm.state = TX_FSM_BULK_TRANSFER;
    fsm.bulk_packets_sent = 3;
    CHECK(TxFsm_InBulk(&fsm) == true);
    CHECK(TxFsm_WaitingForProbeAck(&fsm) == false);
    CHECK(TxFsm_BulkPacketsSent(&fsm) == 3);

    fsm.state = TX_FSM_COMPLETE;
    CHECK(TxFsm_InBulk(&fsm) == false);
    CHECK(TxFsm_WaitingForProbeAck(&fsm) == false);
}

/* ---- MAINT-02: Dispatch reuses the exported predicates (behavioral) ----
 * A probe whose ACK wait exceeded burst_max_open_ms is forced to COMPLETE
 * (which then resets to PROBE with a flush) - exactly the semantics of
 * TxFsm_ProbeStale. Same for a stale burst. */
static void test_dispatch_stale_reuse(void)
{
    printf("-- MAINT-02: Dispatch stale forcing matches the predicates\n");

    TxFsm_t fsm;
    TxFsm_Init(&fsm);
    fsm.state = TX_FSM_WAIT_PROBE_ACK;
    fsm.probe_sent_ms = 1000u;

    AppTxCycleSnapshot_t snap = {
        .now_ms = 1000u + 60001u, /* past the 60 s bound below */
        .rf_silence = false,
        .has_unsent = false,
        .recovery_empty = true,
        .unconvertible = false,
        .no_budget = false,
        .max_bulk_packets = 4,
        .burst_max_open_ms = 60000u
    };
    TxFsmCycleInput_t in = AppAdapters_BuildTxCycle(&snap);
    TxFsmCycleOutput_t out;
    TxFsm_Dispatch(&fsm, &in, &out);
    /* stale probe -> COMPLETE -> reset to PROBE with a header-sync flush */
    CHECK(out.flush_header_sync == true);
    CHECK(fsm.state == TX_FSM_PROBE_SF10);
    CHECK(fsm.probe_sent_ms == 0);

    TxFsm_Init(&fsm);
    fsm.state = TX_FSM_WAIT_PROBE_ACK;
    fsm.probe_sent_ms = 1000u;
    snap.now_ms = 1000u + 60000u; /* exactly at the bound: NOT stale (>) */
    in = AppAdapters_BuildTxCycle(&snap);
    TxFsm_Dispatch(&fsm, &in, &out);
    /* Not stale: no LT-07 forcing, so the stamp SURVIVES - proving the
     * forcing is predicate-gated. The ordinary completed-cycle reset
     * (C2/SP-15) still parks WAIT -> PROBE with a flush. */
    CHECK(out.flush_header_sync == true);
    CHECK(fsm.state == TX_FSM_PROBE_SF10);
    CHECK(fsm.probe_sent_ms == 1000u);

    TxFsm_Init(&fsm);
    fsm.state = TX_FSM_BULK_TRANSFER;
    fsm.burst_opened_ms = 1000u;
    fsm.bulk_packets_sent = 2;
    snap.now_ms = 1000u + 60001u;
    snap.has_unsent = true;
    in = AppAdapters_BuildTxCycle(&snap);
    TxFsm_Dispatch(&fsm, &in, &out);
    /* stale burst -> COMPLETE -> reset to PROBE, counter cleared */
    CHECK(out.flush_header_sync == true);
    CHECK(fsm.state == TX_FSM_PROBE_SF10);
    CHECK(fsm.burst_opened_ms == 0);
    CHECK(fsm.bulk_packets_sent == 0);
}

/* ---- structural pins (regression guards only) ---- */
static void test_delegation_scans(void)
{
    printf("-- Structural: lora_app.c delegates all five mappings\n");

    char *app = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK(strstr(app, "AppAdapters_BuildTxConfirm(&snap)") != NULL);
    CHECK(strstr(app, "AppAdapters_BuildTxCycle(&snap)") != NULL);
    CHECK(strstr(app, "AppAdapters_BuildRx(&snap)") != NULL);
    CHECK(strstr(app, "AppAdapters_BuildFirstFlightAdmission(&snap)") != NULL);
    CHECK(strstr(app, "AppAdapters_RegionDiffers(") != NULL);
    CHECK(strstr(app, "AppAdapters_SwitchWasRequired(") != NULL);
    CHECK(strstr(app, "AppAdapters_ActiveMatchesDetected(") != NULL);

    /* The inverted confirm marshal must never return inline. */
    CHECK(strstr(app, "MissionState_Get() != MISSION_ASCENT") == NULL);

    /* MAINT-02: Dispatch calls the exported predicates. */
    char *fsm_src = slurp("../../Core/Src/tx_fsm.c");
    const char *dispatch = strstr(fsm_src, "void TxFsm_Dispatch");
    CHECK(dispatch != NULL);
    if (dispatch != NULL) {
        CHECK(strstr(dispatch, "TxFsm_ProbeStale(fsm, in->now_ms") != NULL);
        CHECK(strstr(dispatch, "TxFsm_BurstStale(fsm, in->now_ms") != NULL);
    }

    /* The APP_TX_STATUS_OK mirror tracks the real LoRaMac enum value. */
    char *mac = slurp("../../Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacInterfaces.h");
    CHECK(strstr(mac, "LORAMAC_EVENT_INFO_STATUS_OK = 0") != NULL);
}

int main(void)
{
    printf("=== lora_app adapter builders (MAINT-01/02/03) ===\n\n");

    test_build_tx_confirm();
    test_build_tx_cycle();
    test_build_rx();
    test_region_comparators();
    test_build_first_flight();
    test_fsm_queries();
    test_dispatch_stale_reuse();
    test_delegation_scans();

    scan_pool_free();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
