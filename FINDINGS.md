# FINDINGS.md — Refactor Baseline and Open Findings

**Baseline commit:** `0a34aeb` (first-flight policy A `61647a2` + B `0a34aeb`)
**Captured:** 2026-08-15, build box 10.0.0.110 (gcc 10.3.1 host, arm-none-eabi-gcc 10.3.1 target)
**Purpose:** Stage-0 baseline for the structural refactor programme in
`docs/temp/stratosonde-refactor-handoff.md`. After every refactor stage, the
existing suite summaries and golden vectors below must match this baseline;
new module-contract suites add output only as listed in that stage's commit
message. Bugs found during extraction are recorded here and fixed in separate
red-first commits — never inside a refactor commit.

## Host gate baseline (`make -C tests/host all`, exit 0; `sanitize`, exit 0)

| suite | checks | failures | notes |
|---|---:|---:|---|
| test (core runner) | 292 | 0 | golden heartbeat-v2 + bulk-v6 vectors intact |
| flight (flight-readiness) | 66,428 | 0 | |
| findings | 113 | 0 | |
| burst | 113 | 0 | scan-configured FSM replay model, extended (stage 5 step 1) to a full TX-FSM characterisation: LT-07 stale-wait forcing, R3-01 yield/deadline arithmetic, R3-03 ascent gate, T1/F-5 silence park, abort, bulk failure paths |
| txshadow | 734,060 | 0 | stage 5 steps 2-3: pure `tx_fsm.c` step module shadow-run against the burst model - scripted scenarios + 200k randomized events, per-event state/action agreement |
| stability | 49 | 0 | |
| multiregion | 98 | 0 | |
| timerif | 31 | 0 | |
| review0812 | 6 | 0 | |
| deep | 39 | 0 | |
| config | 17 | 0 | |
| dr | 51 | 0 | |
| sp0813 | 73 | 0 | |
| firstflight | 35 | 0 | first-flight admission/package contract + stage-6 rail-conversion & early GNSS-package gates |
| lt0813-gate | 43 | 2 | **red-by-design** (`EXPECT_UNFIXED=1`): LT-06 battery normalization non-monotonic −40…−55 °C |
| geo-gate | 7 | 3 | **red-by-design**: GEO-01/GEO-04 production `REGION_RESTRICTED` data empty |
| pwr-gate | 6 | 3 | **red-by-design**: PWR-02 legacy power model fixed 4300 mV raw floor |
| ddrmanifest | — | 0 | `tools/check_ddr_manifest.py` |

`EXPECT_UNFIXED=1` gates exist so CI stays meaningful while owner-gated
findings are open. The 8 expected failures are pre-existing at the baseline
and are NOT regressions from the first-flight change. Do not touch the gate
semantics or `*-gate` targets.

## ARM target baseline (`make -C Debug -j16 all`)

| configuration | text | data | bss | result |
|---|---:|---:|---:|---|
| debug (CI `firmware` job) | 209,264 | 936 | 26,160 | link OK |
| flight (`-DSONDE_FLIGHT_BUILD`, CI `firmware-flight` job) | 178,760 | 940 | 26,160 | link OK; `SONDE_BUILD:flight` present, `SONDE_BUILD:debug` absent |

Note: box gcc 10.3.1 rejects `-fcyclomatic-complexity`; the flag is sed'd out
on box copies only (never committed). CI ubuntu toolchains accept it.

## Source-scan test inventory (the §3 hazard list)

The host suite reads firmware sources as text: **115 `slurp()` call sites**
across 10 test files; ~150 positive scans (`strstr(...) != NULL`, fail LOUD)
and **22 negative scans** (`strstr(...) == NULL`, fail SILENT when code moves).

`slurp()` targets by frequency: `lora_app.c` ×23, `main.c` ×10,
`atgm336h.c` ×6, `multiregion_context.c` ×6, `usart_if.c` ×5,
`stm32_lpm_if.c` ×5, `mission_state.c` ×4, `lora_app.h` ×4, `adc_if.c` ×4,
`sys_sensors.c` ×4, then ≤3 each across 23 more files (incl. `h3lite.c`,
`ci.yml`, docs).

### Negative scans — repoint in the same commit that moves the target code

| test file:line | forbidden pattern | scan target | finding |
|---|---|---|---|
| test_deepreview_20260812.c:310 | `g_bulk_commit_through` | lora_app.c | bulk FSM |
| test_deepreview_20260812.c:326-327 | `FlashLog_CommitThrough`, `GetUnsentRecordsFIFO` | flash_log.c | DDR |
| test_deepreview_20260812.c:364 | `GPS reconfigured and saved to flash` | lora_app.c | honesty |
| test_dr_20260812.c:215 | `static bool GeofenceRestricted` | (receiver `src` — confirm at repoint) | DR-02c |
| test_dr_20260812.c:359 | `highres_records[6]` | lora_app.c | DR-18 |
| test_review_findings.c:322 | `>= GPS_LOSS_SILENCE_S` | lora_app.c | S-A |
| test_review_findings.c:343 | `MISSION_FLOAT_MAX_PRESSURE_HPA` | mission_state.h | MISSION-01 |
| test_review_findings.c:347 | `MissionState_EnterFlight` | multiregion (receiver `mr`) | MISSION-01 |
| test_review_findings.c:367 | `HAL_UART_Receive_IT` | usart_if.c | R2-13 |
| test_review_findings.c:382 | `FatalIsDegradable` | main.c | first-flight fatal |
| test_review_findings.c:391 | `return;` (inside Error_Handler_Fatal body) | main.c | first-flight fatal |
| test_review_findings.c:503 | `PRE_FLIGHT` | payload_format (receiver `pf`) | STAB-10 |
| test_sp_20260813.c:289 | `is not overridden` | main.c | SP comment honesty |
| test_sp_20260813.c:303 | `void LoRaApp_ReInitStack` | lora_app.c | F-01 |
| test_sp_20260813.c:379 | `FatalIsDegradable` | main.c | SP-07 |
| test_sp_20260813.c:408 | `GPIO_UART.Pin = GPIO_PIN_6 \| GPIO_PIN_7` | stm32_lpm_if.c | SP |
| test_sp_20260813.c:438 | `case TX_STATE_WAIT_PROBE_ACK:` | lora_app.c | SP-15 |
| test_sp_20260813.c:451 | `Auto-switch completed successfully` | lora_app.c | honesty |
| test_stability_review.c:788 | `BKP_REG_SYSTIME_VALID,` | (receiver `m` — confirm at repoint) | STAB |
| test_stability_review.c:844-845 | `every 25s`, `25s = 51200 counts` | stm32_lpm_if.c | STAB |
| test_stability_review.c:848 | `Configure GPIO pin : PB10` | main.c | STAB |

Rules (handoff §3): positive scans fail loud on moves (repoint at the new
file); negative scans pass for the wrong reason (repoint in the SAME commit,
list under `Scan repoints:` in the message); a scan is retired only by a later
commit demonstrating the same invariant against the linked module.

## Open findings log

(populated as refactor stages discover bugs; each gets a separate red-first
fix commit, never folded into a refactor commit)

### 2026-08-15 — TX-ADAPTER-01: reversed `mission_ascent` polarity (stage 5.4b regression)

**Status: OPEN — fix in progress (Phase A1/A2 of the 2026-08-15 next-step
handoff). `b958a95` is the reviewed Stage-7 baseline, NOT a flight candidate
until this lands.**

External review of `b958a95` found a functional regression introduced by
stage 5.4b (`7a6ab7e`): `OnTxData` marshals the confirm input with
`MissionState_Get() != MISSION_ASCENT` into the field named `mission_ascent`,
and `TxFsm_OnTxConfirm()` opens archive recovery on `!in->mission_ascent`.
The old inline condition (`battery_good && has_cache &&
MissionState_Get() != MISSION_ASCENT`) opened recovery when NOT ascending;
the adapter double-negates it, so **ASCENT permits archive recovery and
FLOAT blocks it** — the primary data-recovery path never opens at float,
and ascent burns power/duty-cycle on recovery at 10 s cadence.

The 734,060-check shadow suite cannot see this: the pure module and the
characterisation model agree with each other; the defect is in the
adapter's field mapping, which is outside the shadow's coverage. Lesson
recorded in the handoff §7.1: adapter mappings need their own wiring tests.

### 2026-08-15 — Stage 8–11 proposal dispositions

The `stratosonde-refactor-handoff-stage8-11.md` proposal is **not** an
execution plan before first flight (see
`docs/temp/stratosonde-next-step-coding-llm-handoff.md` §4): its line-count
and complexity targets are advisory only (and its reference metric output
does not reproduce), `GnssAcquire_Step()` and the single `AppContext_t`
are rejected as designed, and the multiregion split is deferred to
post-flight. Tracked post-flight directions live in §7 of that handoff.
