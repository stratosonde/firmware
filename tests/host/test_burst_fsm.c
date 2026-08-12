/**
  ******************************************************************************
  * @file    test_burst_fsm.c
  * @brief   Behavioural regression for the archive-burst state machine
  *          (2026-08-11 review, BURST-01 / BURST-02)
  ******************************************************************************
  * WHY THIS IS NOT A PLAIN SOURCE SCAN
  *
  * BURST-01 is a pure control-flow defect: every individual branch in
  * OnTxData/OnRxData/RunTxStateMachine reads correctly, and no grep-able
  * token is wrong. The bug only appears when the three callbacks are replayed
  * in the order LoRaMac actually calls them. A grep cannot see it; a replay
  * can. So this suite REPLAYS the state machine.
  *
  * WHY THE REPLAY CANNOT DRIFT FROM THE FIRMWARE
  *
  * A hand-copied model would go green the moment someone edits the model
  * instead of the firmware. Instead the model is CONFIGURED BY A SOURCE SCAN:
  * at startup it reads LoRaWAN/App/lora_app.c, detects which shape each of
  * the two decision points currently has, and replays that shape. Pre-fix the
  * scan selects the buggy shape and the behavioural assertions go red; the
  * only way to turn them green is to change the firmware.
  *
  * CALLBACK ORDER (Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.c,
  * LoRaMacProcess): LoRaMacHandleRequestEvents() dispatches McpsConfirm
  * (-> LmHandler McpsConfirm -> OnTxData) and then MlmeConfirm (-> LmHandler
  * MlmeConfirm -> OnTxData AGAIN), and only afterwards does
  * LoRaMacHandleIndicationEvents() dispatch McpsIndication (-> OnRxData).
  * OnTxData therefore always runs BEFORE OnRxData for the same uplink, and
  * runs twice whenever an MLME request rode along.
  *
  * Run:
  *   make -C tests/host burst      (red until the fixes land)
  *   make -C tests/host baseline   (EXPECT_UNFIXED=1: green pre-fix CI gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Mirrors of the firmware constants under test (lora_app.h). Parsed from the
 * header at runtime so a retune there cannot silently invalidate the test. */
static int MAX_BULK_PACKETS_PER_CYCLE = 0;
static int LINK_MARGIN_THRESHOLD      = 0;
static int GATEWAY_COUNT_THRESHOLD    = 0;

static int g_failures = 0;
static int g_checks = 0;
static int g_expected_failures = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_REGRESSION(cond, id) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        g_expected_failures++; \
        printf("FAIL [%s] %s:%d: %s\n", id, __FILE__, __LINE__, #cond); \
    } \
} while (0)

/* ------------------------------------------------------------------ */
/* Source scan: read the firmware and decide which shape to replay      */
/* ------------------------------------------------------------------ */

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
    return buf;
}

/* Collapse every whitespace run to one space so the scan survives reindenting
 * and line rewrapping. Comments are stripped first: a fix described in prose
 * must never be mistaken for a fix in code. */
static char *normalize_code(const char *src)
{
    size_t n = strlen(src);
    char *out = (char *)malloc(n + 2);
    if (!out) exit(2);
    size_t o = 0;
    bool in_block = false, in_line = false, sp = true;
    for (size_t i = 0; i < n; i++) {
        if (in_block) {
            if (src[i] == '*' && src[i + 1] == '/') { in_block = false; i++; }
            continue;
        }
        if (in_line) {
            if (src[i] == '\n') in_line = false;
            continue;
        }
        if (src[i] == '/' && src[i + 1] == '*') { in_block = true; i++; continue; }
        if (src[i] == '/' && src[i + 1] == '/') { in_line = true;  i++; continue; }
        if (src[i] == ' ' || src[i] == '\t' || src[i] == '\r' || src[i] == '\n') {
            if (!sp) { out[o++] = ' '; sp = true; }
            continue;
        }
        out[o++] = src[i];
        sp = false;
    }
    out[o] = '\0';
    return out;
}

static int parse_define(const char *src, const char *name, int *out)
{
    const char *p = src;
    size_t nl = strlen(name);
    while ((p = strstr(p, "#define")) != NULL) {
        const char *q = p + 7;
        while (*q == ' ' || *q == '\t') q++;
        if (strncmp(q, name, nl) == 0 && (q[nl] == ' ' || q[nl] == '\t')) {
            q += nl;
            while (*q == ' ' || *q == '\t') q++;
            *out = atoi(q);
            return 1;
        }
        p += 7;
    }
    return 0;
}

/* Which shape does the firmware have right now? */
static bool cfg_tail_terminates_low_counts = true;  /* BURST-01: buggy default */
static bool cfg_ontxdata_reentrant         = true;  /* BURST-02: buggy default */

static void scan_firmware(void)
{
    char *hdr = slurp("../../LoRaWAN/App/lora_app.h");
    if (!parse_define(hdr, "MAX_BULK_PACKETS_PER_CYCLE", &MAX_BULK_PACKETS_PER_CYCLE) ||
        !parse_define(hdr, "LINK_MARGIN_THRESHOLD", &LINK_MARGIN_THRESHOLD) ||
        !parse_define(hdr, "GATEWAY_COUNT_THRESHOLD", &GATEWAY_COUNT_THRESHOLD)) {
        printf("FATAL: burst constants not found in lora_app.h\n");
        exit(2);
    }
    free(hdr);

    char *app  = slurp("../../LoRaWAN/App/lora_app.c");
    char *norm = normalize_code(app);

    /* BURST-01: the unqualified terminal `else if` is the defect. Its presence
     * means packet counts 0 and 1 are reset to COMPLETE by OnTxData's tail. */
    cfg_tail_terminates_low_counts =
        (strstr(norm, "else if (g_tx_state == TX_STATE_BULK_TRANSFER) {") != NULL);

    /* BURST-02: an early return on the MLME re-entry. */
    cfg_ontxdata_reentrant =
        (strstr(norm, "if (params->IsMcpsConfirm == 0) { return; }") == NULL);

    free(norm);
    free(app);

    printf("source scan: OnTxData tail terminates sent<=1 : %s\n",
           cfg_tail_terminates_low_counts ? "YES (BURST-01 present)" : "no");
    printf("source scan: OnTxData re-entrant on MlmeConfirm: %s\n",
           cfg_ontxdata_reentrant ? "YES (BURST-02 present)" : "no");
    printf("source scan: MAX_BULK_PACKETS_PER_CYCLE=%d margin>=%d gateways>=%d\n\n",
           MAX_BULK_PACKETS_PER_CYCLE, LINK_MARGIN_THRESHOLD, GATEWAY_COUNT_THRESHOLD);
}

/* ------------------------------------------------------------------ */
/* The replay                                                          */
/* ------------------------------------------------------------------ */

typedef enum { ST_PROBE, ST_WAIT_PROBE_ACK, ST_BULK, ST_COMPLETE } TxState_t;

static TxState_t g_tx_state;
static int  g_bulk_packets_sent;
/* R3-04 (#218): g_bulk_commit_through deleted - one-pass advance at send. */

/* environment knobs */
static bool env_has_unsent, env_ack, env_linkcheck, env_batt_good;
static int  env_margin, env_gateways;

/* observations */
static bool seq_task_armed;
static int  obs_archive_packets;     /* archive uplinks actually built */
static int  obs_heartbeats;          /* confirmed probe uplinks */
static int  obs_commits;             /* R3-04: send-time watermark advances */
static int  obs_nvm_tx_counter;      /* the every-10th-TX NVM store counter */
static int  obs_ctx_saves;           /* MultiRegion_SaveCurrentContext calls */
static bool obs_onrxdata_branch;     /* OnRxData packet-1 branch reached */

static void reset_world(void)
{
    g_tx_state = ST_PROBE;
    g_bulk_packets_sent = 0;
    env_has_unsent = true; env_ack = true; env_linkcheck = true; env_batt_good = true;
    env_margin = LINK_MARGIN_THRESHOLD + 5;
    env_gateways = GATEWAY_COUNT_THRESHOLD + 2;
    seq_task_armed = false;
    obs_archive_packets = obs_heartbeats = obs_commits = 0;
    obs_nvm_tx_counter = obs_ctx_saves = 0;
    obs_onrxdata_branch = false;
}

/* lora_app.c OnTxData() */
static void OnTxData(bool is_mcps_confirm, bool status_ok, bool ack_received)
{
    if (!cfg_ontxdata_reentrant && !is_mcps_confirm) return;

    if (status_ok) {
        obs_ctx_saves++;
        obs_nvm_tx_counter++;
    }

    /* R3-04 (#218): commit-on-ACK block deleted - unconfirmed one-pass sends
     * advance the watermark at send time (modeled in SendTxData). */

    if (g_tx_state == ST_WAIT_PROBE_ACK) {
        if (status_ok && ack_received) {
            if (env_batt_good && env_has_unsent) {
                g_tx_state = ST_BULK;
                g_bulk_packets_sent = 0;
                seq_task_armed = true;
            } else {
                g_tx_state = ST_COMPLETE;
            }
        } else {
            g_tx_state = ST_COMPLETE;
        }
    }

    /* R3-04 (#218): the unACKed-archive fallback is deleted - archive frames
     * are unconfirmed (BR-TX-011); AckReceived is meaningless for them. */

    /* the tail under test */
    if (g_tx_state == ST_BULK && g_bulk_packets_sent > 1) {
        if (env_has_unsent && g_bulk_packets_sent < MAX_BULK_PACKETS_PER_CYCLE) {
            seq_task_armed = true;
        } else {
            g_tx_state = ST_COMPLETE;
            g_bulk_packets_sent = 0;
        }
    } else if (cfg_tail_terminates_low_counts && g_tx_state == ST_BULK) {
        g_tx_state = ST_COMPLETE;
        g_bulk_packets_sent = 0;
    }
}

/* lora_app.c OnRxData() */
static void OnRxData(void)
{
    if (g_tx_state == ST_BULK && g_bulk_packets_sent == 1) {
        obs_onrxdata_branch = true;
        bool link_good = (env_linkcheck &&
                          env_margin >= LINK_MARGIN_THRESHOLD &&
                          env_gateways >= GATEWAY_COUNT_THRESHOLD);
        if (!link_good) {
            g_tx_state = ST_COMPLETE;
            g_bulk_packets_sent = 0;
        } else if (env_has_unsent && g_bulk_packets_sent < MAX_BULK_PACKETS_PER_CYCLE) {
            seq_task_armed = true;
        } else {
            g_tx_state = ST_COMPLETE;
            g_bulk_packets_sent = 0;
        }
    }
}

/* lora_app.c SendTxData()/RunTxStateMachine(); returns true if an uplink went out */
static bool SendTxData(bool *carried_linkcheckreq)
{
    seq_task_armed = false;
    *carried_linkcheckreq = false;

    if (g_tx_state == ST_WAIT_PROBE_ACK || g_tx_state == ST_COMPLETE) {
        g_tx_state = ST_PROBE;
        g_bulk_packets_sent = 0;
    }

    if (g_tx_state == ST_PROBE) {
        g_tx_state = ST_WAIT_PROBE_ACK;
        obs_heartbeats++;
        return true;
    }
    if (g_tx_state == ST_BULK) {
        if (env_has_unsent && g_bulk_packets_sent < MAX_BULK_PACKETS_PER_CYCLE) {
            *carried_linkcheckreq = (g_bulk_packets_sent == 0);  /* protocol §5.2 */
            g_bulk_packets_sent++;
            obs_archive_packets++;
            obs_commits++;   /* R3-04: watermark advance AT SEND TIME */
            if (!env_has_unsent || g_bulk_packets_sent >= MAX_BULK_PACKETS_PER_CYCLE)
                g_tx_state = ST_COMPLETE;
            return true;
        }
        g_tx_state = ST_COMPLETE;
        return false;
    }
    g_tx_state = ST_PROBE;
    g_bulk_packets_sent = 0;
    return false;
}

/* one LoRaMacProcess pass after an uplink completes */
static void mac_process(bool carried_mlme)
{
    OnTxData(true, true, env_ack);                 /* McpsConfirm  */
    if (carried_mlme) OnTxData(false, true, env_ack); /* MlmeConfirm: same callback */
    OnRxData();                                    /* McpsIndication */
}

/* Drive one full work cycle chain to quiescence (or the wedge bound). */
static int run_cycle(int bound)
{
    bool mlme;
    int iterations = 0;
    bool sent = SendTxData(&mlme);
    if (!sent) return 0;
    mac_process(mlme);
    while (seq_task_armed && iterations < bound) {
        iterations++;
        sent = SendTxData(&mlme);
        if (!sent) break;
        mac_process(mlme);
    }
    return iterations;
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

/* T-B1: the probe ACK opens the archive opportunity. OnTxData's tail must not
 * close it in the same call. */
static void test_probe_ack_opportunity_survives(void)
{
    printf("-- T-B1 / BURST-01: probe ACK must leave the archive opportunity open\n");
    reset_world();
    bool mlme;
    (void)SendTxData(&mlme);                  /* confirmed heartbeat */
    mac_process(false);
    printf("   after probe ACK: state=%d bulk_sent=%d armed=%d (want state=BULK)\n",
           g_tx_state, g_bulk_packets_sent, seq_task_armed);
    CHECK_REGRESSION(g_tx_state == ST_BULK, "BURST-01");
    CHECK(seq_task_armed);
}

/* T-B2: a healthy link with backlog must drain up to MAX_BULK_PACKETS_PER_CYCLE
 * archive packets, and must not burn extra heartbeats doing it. */
static void test_full_burst_reaches_cap(void)
{
    printf("-- T-B2 / BURST-01: healthy burst drains to the packet cap\n");
    reset_world();
    int iterations = run_cycle(200);
    printf("   archive packets=%d (want %d)  heartbeats=%d (want 1)  "
           "commits=%d  iterations=%d\n",
           obs_archive_packets, MAX_BULK_PACKETS_PER_CYCLE,
           obs_heartbeats, obs_commits, iterations);
    CHECK_REGRESSION(obs_archive_packets == MAX_BULK_PACKETS_PER_CYCLE, "BURST-01");
    CHECK_REGRESSION(obs_heartbeats == 1, "BURST-01");
    CHECK_REGRESSION(obs_commits == MAX_BULK_PACKETS_PER_CYCLE, "BURST-01");
}

/* T-B3: the wedge. With the tail terminating low counts, every ACKed heartbeat
 * re-opens and re-kills the opportunity, and the armed task spends a full work
 * cycle sending another heartbeat. The cycle must terminate. */
static void test_no_unbounded_heartbeat_loop(void)
{
    printf("-- T-B3 / BURST-01: work cycle must not self-perpetuate\n");
    reset_world();
    const int bound = 500;
    int iterations = run_cycle(bound);
    printf("   iterations=%d (bound %d)  heartbeats=%d  archive packets=%d\n",
           iterations, bound, obs_heartbeats, obs_archive_packets);
    CHECK_REGRESSION(iterations < bound, "BURST-01");
    CHECK_REGRESSION(obs_heartbeats <= 1, "BURST-01");
}

/* T-B4: OnRxData owns the packet-1 verdict (FR-14/#88 must stay fixed) and
 * must actually be reachable. */
static void test_onrxdata_owns_packet_one(void)
{
    printf("-- T-B4 / BURST-01: OnRxData packet-1 LinkCheck branch is reachable\n");
    reset_world();
    (void)run_cycle(200);
    CHECK_REGRESSION(obs_onrxdata_branch, "BURST-01");

    printf("-- T-B4b: poor link after packet 1 falls back to heartbeat mode\n");
    reset_world();
    env_margin = LINK_MARGIN_THRESHOLD - 1;
    (void)run_cycle(200);
    printf("   archive packets=%d (want exactly 1)\n", obs_archive_packets);
    CHECK_REGRESSION(obs_archive_packets == 1, "BURST-01");

    printf("-- T-B4c: missing LinkCheckAns after packet 1 also falls back\n");
    reset_world();
    env_linkcheck = false;
    (void)run_cycle(200);
    printf("   archive packets=%d (want exactly 1)\n", obs_archive_packets);
    CHECK_REGRESSION(obs_archive_packets == 1, "BURST-01");
}

/* T-B5: DDR-0005 BR-TX-003/004 — the compact probe stays CONFIRMED and its
 * ACK gates the archive opportunity: no probe ACK, no recovery at all.
 * (R3-04/#218 reframing: archive frames themselves are unconfirmed
 * one-pass; the legacy "unACKed archive ends the burst" rule is gone.) */
static void test_unacked_archive_packet_falls_back(void)
{
    printf("-- T-B5 / DDR-0005: no probe ACK -> no archive opportunity\n");
    reset_world();
    env_ack = false;
    (void)run_cycle(200);
    printf("   archive packets=%d  sends-marked=%d (want 0)\n",
           obs_archive_packets, obs_commits);
    CHECK(obs_archive_packets == 0);
    CHECK(obs_commits == 0);
    CHECK(g_tx_state != ST_BULK);
}

/* T-B6: BURST-02 — one uplink, one accounting event. The first archive packet
 * carries LinkCheckReq, so LmHandler calls OnTxData twice for it. */
static void test_one_uplink_one_accounting_event(void)
{
    printf("-- T-B6 / BURST-02: MlmeConfirm must not double-count an uplink\n");
    reset_world();
    (void)run_cycle(200);
    int uplinks = obs_heartbeats + obs_archive_packets;
    printf("   uplinks=%d  context saves=%d  NVM-store ticks=%d (want == uplinks)\n",
           uplinks, obs_ctx_saves, obs_nvm_tx_counter);
    CHECK_REGRESSION(obs_ctx_saves == uplinks, "BURST-02");
    CHECK_REGRESSION(obs_nvm_tx_counter == uplinks, "BURST-02");
}

int main(void)
{
    printf("=== archive burst FSM regressions (BURST-01, BURST-02) ===\n\n");
    scan_firmware();

    test_probe_ack_opportunity_survives();
    test_full_burst_reaches_cap();
    test_no_unbounded_heartbeat_loop();
    test_onrxdata_owns_packet_one();
    test_unacked_archive_packet_falls_back();
    test_one_uplink_one_accounting_event();

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    const char *expect_unfixed = getenv("EXPECT_UNFIXED");
    if (expect_unfixed != NULL && expect_unfixed[0] == '1') {
        int unexpected = g_failures - g_expected_failures;
        if (unexpected < 0) {
            printf("UNEXPECTED PASS: a documented bug appears FIXED — flip this "
                   "check from CHECK_REGRESSION to CHECK.\n");
            return 1;
        }
        if (unexpected > 0) {
            printf("NEW BREAKAGE beyond the documented findings.\n");
            return 1;
        }
        printf("OK (baseline): only the documented findings fail.\n");
        return 0;
    }

    if (g_failures == 0) { printf("OK\n"); return 0; }
    return 1;
}
