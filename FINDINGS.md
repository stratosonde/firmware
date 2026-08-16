# FINDINGS.md — Refactor Baseline and Open Findings

**Baseline commit:** `9fdcccc` (h3lite submodule `5480859` — RESTRICTED dataset landed: 53 cells, Yemen + North Korea)
**Captured:** 2026-08-15, build box 10.0.0.110 (gcc 10.3.1 host, arm-none-eabi-gcc 10.3.1 target); rebaselined same day from `0a34aeb`/`3da2100` after the h3lite dataset bump
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
| pq | 45 | 0 | stage 1: packet_queue contract |
| nvm | 34 | 0 | stage 2: nvm_slot contract |
| region | 2,071 | 0 | stage 3: region_policy contract (incl. exhaustive EU polygon rasterisation) |
| flight (flight-readiness) | 66,428 | 0 | |
| findings | 113 | 0 | |
| burst | 113 | 0 | scan-configured FSM replay model, extended (stage 5 step 1) to a full TX-FSM characterisation: LT-07 stale-wait forcing, R3-01 yield/deadline arithmetic, R3-03 ascent gate, T1/F-5 silence park, abort, bulk failure paths |
| txshadow | 734,060 | 0 | stage 5 steps 2-3: pure `tx_fsm.c` step module shadow-run against the burst model - scripted scenarios + 200k randomized events, per-event state/action agreement |
| txadapter | 11 | 0 | TX-ADAPTER-01: confirm-input polarity + designated-field wiring (Phase A1/A2) |
| stability | 49 | 0 | |
| multiregion | 98 | 0 | |
| timerif | 31 | 0 | |
| review0812 | 6 | 0 | |
| deep | 39 | 0 | |
| config | 17 | 0 | |
| dr | 51 | 0 | |
| sp0813 | 73 | 0 | |
| firstflight | 35 | 0 | first-flight admission/package contract + stage-6 rail-conversion & early GNSS-package gates |
| gnssacq | 39 | 0 | stage 4 contract suite (exhaustive civil-date proof, epoch, budget, package truth table) + A5 FixAccepted configured-boundary contract |
| gnssfix | 10 | 0 | A4/A5 (#284): hardcoded-threshold characterization + one-authoritative-predicate wiring |
| gpsloss | 9 | 0 | A6/A7 (#285): same-wake accepted-fix clears only the GPS-loss veto, before region selection and TX |
| lt0813-gate | 43 | 2 | **red-by-design** (`EXPECT_UNFIXED=1`): LT-06 battery normalization non-monotonic −40…−55 °C |
| geo-gate | 9 | 2 | **red-by-design** (`EXPECT_UNFIXED=1`): GEO-04 init/generator guards pending the next h3lite bump; GEO-01 dataset landed `9fdcccc` (53 RESTRICTED cells) — promoted to hard gate + Pyongyang/Sanaa resolve probes |
| pwr-gate | 6 | 3 | **red-by-design**: PWR-02 legacy power model fixed 4300 mV raw floor |
| ddrmanifest | — | 0 | `tools/check_ddr_manifest.py` — 0 failures **and 0 warnings enforced** since Phase 2 / DOC-01 (ambiguous 0014–0021 aliases retired after full 247-occurrence audit; 7 unambiguous aliases remain declared) |

**Warning baseline (Phase 2):** host suite compiles with **zero warnings**
under `-Wall -Wextra -Wsign-conversion` and is locked with **`-Werror`**
(40 warning lines → 0 on box gcc 10.3.1; local gcc 9.2 also clean). ARM
first-party fragments (`Core/Src`, `LoRaWAN/App`) compile `-Wall -Werror`
with zero diagnostics in BOTH debug and flight configurations (the flight
config exposed exactly one latent unused-variable, `link_good`, fixed in
a6308e4). Vendor fragments intentionally unchanged. Raising ARM first-party
to `-Wextra`/`-Wsign-conversion` remains open (#268): `lora_app.c` is never
host-compiled, so its first strict-flag pass is a separate effort.

**Sanitizers (Phase 3 / #263):** `make sanitize` runs **19 instrumented
exes** — every suite that executes production code — under ASan+UBSan+LSan
(was: 1, `test_main` only). First full run: zero UB/overflows; LSan caught
the shared `strip_comments(slurp())` harness leak in 7 scan suites (fixed
by pooling the scan buffers, freed at exit). Text-scan-only suites (burst,
deepreview, findings, txadapter, gpsloss) are excluded by design.

**Static analysis (Phase 3):** `tools/run_cppcheck.sh` gates first-party
sources (Core/Src, LoRaWAN/App; vendor include-only) with
`--error-exitcode=1` — baseline was 3 diagnostics: `sht31.c` uninitvar
(fixed) and 2 vendor CMSIS linker-table comparePointers (audited
suppressions in `tools/cppcheck-suppressions.txt`). `actionlint` v1.7.7
gates the workflows (first catch: an SC2015 in ci.yml itself).
`tools/check_format.sh` runs git-clang-format on changed lines in **report
mode**: the CubeIDE 2-space files vs 4-space new-module split needs an
owner style decision before a hard gate is honest. All wired in the CI
`hygiene` job + root `make lint`.

**Style unification (owner decision 2026-08-16):** the project style is
**2-space LLVM** (`.clang-format`), matching the CubeIDE majority. The 23
four-space first-party files were reformatted once, mechanically
(clang-format 22.1.8, `ColumnLimit: 0`, `DerivePointerAlignment: false`;
endings preserved; 9 scan anchors updated to the formatter-defined shape).
The changed-lines check is now a **hard gate**
(`tools/check_changed_format.py`, `-lines` ranges from `git diff -U0`),
self-tested on the unification diff and negative-tested with a planted
bad line. CubeMX-regen files are never reformat-checked (changed-lines
semantics only).
| arminventory | 4 proofs | 0 | PIPE-05 (Phase 1): ARM source inventory — 18 active fragments, 115 unique C sources, every production `.c` exactly once, no stale `Debug/` fragments |

**Target structure (Phase 1 / PIPE-02):** `check` = every mandatory suite with
NO `EXPECT_UNFIXED` masking; `characterization` = the owner-gated suites below
(prints the exact open-failure IDs); `all` = check + characterization and
remains the CI gate (STAB-09/#156). Root wrapper Makefile provides `make check`,
`make characterization`, `make contracts`, `make integration`, `make structural`,
`make sanitize`, `make release-gate`. Host executables build out-of-tree under
`tests/host/build/<config>/` (PIPE-01); every target leaves `git status` clean.

`EXPECT_UNFIXED=1` gates exist so CI stays meaningful while owner-gated
findings are open. The 7 expected failures (LT-06 ×2, GEO-04 ×2, PWR-02 ×3)
are pre-existing at the baseline and are NOT regressions from the
first-flight change; GEO-01's eighth failure closed when the RESTRICTED
dataset landed (`9fdcccc`). Do not touch the gate semantics or `*-gate`
targets.

## ARM target baseline (`make -C Debug -j16 all`)

| configuration | text | data | bss | result |
|---|---:|---:|---:|---|
| debug (CI `firmware` job) | 210,152 | 936 | 26,160 | link OK (fresh clone @ `df0dcad` post-PIPE-05 prune, build box; was 210,736 @ `9fdcccc` — no source change, link-order wobble) |
| flight (`-DSONDE_FLIGHT_BUILD`, CI `firmware-flight` job) | 180,128 | 940 | 26,160 | link OK; `SONDE_BUILD:flight` present, `SONDE_BUILD:debug` absent (fresh clone @ `9fdcccc`, build box) |

Measured 2026-08-15 on a fresh `9fdcccc` clone (h3lite `5480859`, +54 table
entries vs `3da2100`): +1,472 B debug / +1,368 B flight text against the
previously recorded 209,264 / 178,760.

Note: box gcc 10.3.1 rejects `-fcyclomatic-complexity`; the flag is sed'd out
on box copies only (never committed). CI ubuntu toolchains accept it.

## Source-scan test inventory (the §3 hazard list)

The host suite reads firmware sources as text: **129 `slurp()` call sites**
across 13 test files; ~206 positive scans (`strstr(...) != NULL`, fail LOUD)
and ~30 negative scans (`strstr(...) == NULL`, fail SILENT when code moves);
342 `strstr(` total in those files; 7 dated/review-session test filenames.
Recounted by script 2026-08-15 @ `9fdcccc` (supersedes the stale
115-sites/10-files figure).

`slurp()` literal targets by frequency: `lora_app.c` ×26, `main.c` ×10,
`atgm336h.c` ×7, `multiregion_context.c` ×6, `stm32_lpm_if.c` ×5,
`usart_if.c` ×5, `lora_app.h` ×4, `tx_fsm.c` ×4, `adc_if.c` ×4,
`sys_sensors.c` ×4, `mission_state.c` ×4, `sys_app.c` ×3, then ≤3 each
across the remainder (incl. `h3lite.c`, `ci.yml`, docs).

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

**Status: FIXED 2026-08-15 (A1 red gate `7f5bde7`, A2 fix `ec1428a`). The
A-series rebaseline (A8 commit, `docs(test): rebaseline and record ARM/HIL
evidence`) supersedes `b958a95` as the candidate baseline; its head SHA is
the release-candidate reference recorded in the A8 commit message.**

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
