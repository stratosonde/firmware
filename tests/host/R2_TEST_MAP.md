# R2 Test Map — provability of every 2026-08-09 review finding

Rule (owner, 2026-08-09): **no code change without a test that first proves the
bug and fails; the root-cause fix must turn that exact test green.** Items that
cannot be host-tested are marked with their verification path instead —
undocumented "trust me" fixes are not allowed either.

Run: `make -C tests/host check` — every mandatory suite, NO expected-failure
masking (hardening-pass Phase 1, landed). `make -C tests/host characterization`
holds the owner-gated known failures below (`EXPECT_UNFIXED=1`, prints the exact
open-failure IDs). `make -C tests/host all` = check + characterization and
remains the CI gate (STAB-09/#156). Module contracts only:
`make -C tests/host contracts`. All four also exist at the repo root
(`make check` etc.) via the Phase-1 wrapper Makefile.

---

# Stage-7 two-axis map — "what is proven about X?"

The suites below the line are organised by review session (an archaeological
record of where the campaign dug). Each is labelled ARCHIVE in its header.
The CONTRACT suites are organised by module and are the readable statement of
what each pure module promises.

## Axis 1 — module → contract suite

| module | contract suite | checks | notes |
|---|---|---:|---|
| `Core/Src/packet_queue.c` | `test_packet_queue.c` | 45 | stage 1 |
| `Core/Src/nvm_slot.c` | `test_nvm_slot.c` | 34 | stage 2 |
| `Core/Src/region_policy.c` | `test_region_policy.c` | 2,078 | stage 3 (incl. exhaustive EU row polygon rasterisation) + BEH-03/#301 post-switch authorization (red-first) |
| `Core/Src/gnss_acquire.c` | `test_gnss_acquire.c` | 62 | stage 4 (one check = exhaustive 1970–2100 civil-date proof, ~47k days) + A5 boundary contract + BEH-02/#284 weak-fix disposition (red-first) |
| `Core/Src/tx_fsm.c` | `test_tx_fsm_shadow.c` | 734,060 | stage 5: step module shadow-run against the scan-locked characterisation model in `test_burst_fsm.c` (scripted + 200k randomized events) |
| `Core/Src/first_flight_policy.c` | `test_first_flight_policy.c` | 50 | commit A + stage 6 (admission, package, rail conversion, GNSS-package gate) + BEH-01/#300 admitted-wake outcome (red-first) |
| `Core/Src/gnss_acquire.c` acceptance | `test_gnss_acquire.c` (39) + `test_gnss_fix_acceptance.c` (10) | 49 | A4/A5 (#284): configured boundary contract + hardcoded-threshold characterization + adapter wiring |
| `lora_app.c` TX adapter wiring | `test_tx_adapter.c` | 11 | A1/A2 (TX-ADAPTER-01): confirm-input polarity + designated fields |
| `lora_app.c` GPS-loss recovery wiring | `test_gps_loss_recovery.c` | 9 | A6/A7 (#285): same-wake veto-guarded clear before region selection |
| `Core/Src/power_model.c` | host-compiled in `test_main.c` / `test_flightreadiness.c` | — | pre-refactor pure module; behavioural coverage lives in the core suites |
| `Core/Src/atgm336h.c` | host-compiled in `test_main.c` / `test_dr_20260812.c` / others | — | pre-refactor; NMEA parser behavioural coverage in core + DR suites |

## Axis 2 — finding/review session → archive suite

| session | archive suite(s) |
|---|---|
| 2026-08-09 R2 review | `test_main.c`, `test_flightreadiness.c` (see per-finding table below) |
| 2026-08-11 review (BURST-01/02) | `test_burst_fsm.c` (also the stage-5 FSM characterisation model — load-bearing for the shadow suite) |
| 2026-08-12 reviews | `test_review_20260812.c`, `test_deepreview_20260812.c`, `test_dr_20260812.c` |
| 2026-08-13 reviews | `test_sp_20260813.c`, `test_lt_20260813.c` |
| 2026-08-14 reviews | `test_geo_20260814.c`, `test_pwr_20260814.c` |
| cross-review findings | `test_review_findings.c`, `test_stability_review.c` |
| feature suites (not session archives) | `test_multiregion.c`, `test_config.c`, `test_timerif.c` |

Red-by-design gates (owner-gated, EXPECT_UNFIXED=1): LT-06 ×2
(`test_lt_20260813.c`), GEO-04 ×2 (`test_geo_20260814.c` — GEO-01's dataset
landed 2026-08-15 @ `9fdcccc` and is now a hard gate with Pyongyang/Sanaa
resolve probes), PWR-02 ×3 (`test_pwr_20260814.c`).

---


## Host-provable - ALL FIXED 2026-08-09 (tests now green under make all)

| Issue | Test | Proves |
|---|---|---|
| #106 R2-02 watermark over-advance | test_flightreadiness.c `test_r2_02_watermark_overadvance_on_wrap` | Wrapped ring + stale watermark + the exact lora_app.c caller composition -> watermark lands past untransmitted records |
| #107 R2-03 sequence identity | test_flightreadiness.c `test_r2_03_sequence_identity_crosscheck` | CRC-valid wrong-sequence record is packed instead of skipped |
| #118 R2-14 TS_WRAP false latch | test_flightreadiness.c `test_r2_14_ts_wrap_false_latch` | Undisciplined->disciplined clock jump latches wrap bit |
| #114 R2-10 slope on normalized V | test_main.c `test_r2_10_slope_temperature_contamination` | Constant raw V, -55->-65C: slope reads +1740 mV/h, mode flips NORMAL |
| #115 R2-11 RAM-only slope state | test_main.c `test_r2_11_no_history_default_uses_raw_voltage` | No-history + 4400 mV raw -> CONSERVATIVE with GPS on |
| #121 R2-17 "Stable" at critical | test_main.c `test_r2_17_already_critical_never_reports_stable` | Discharging at 4400 mV -> time_to_target_h == 0 (renders Stable) |
| #130 R2-30 RMC valid latch | test_main.c `test_r2_30_rmc_valid_clears_on_void` | A then V leaves valid set |
| #123 R2-19 DMA overrun blind spot | test_main.c `test_r2_19_dma_overrun_blind_spot` | produced==512, consumed==0 -> overrun not counted |

## Needs production-side extraction first (test becomes host-able after)

| Issue | Extract | Then test |
|---|---|---|
| #105 R2-01 burst teardown + #111 R2-07 IsMcpsConfirm + #110 R2-06 LinkCheck gate | The TX state machine transitions out of OnTxData/OnRxData into a pure `burst_controller.c` (R47/R49 precedent) | Drive McpsConfirm-then-McpsIndication ordering on host; assert packet-1 state survives to OnRxData, stale AckReceived rejected, LinkCheck read as per-cycle event |
| ~~#108 R2-04 all-corrupt retire~~ FIXED | Landed with the #106 CommitThrough rework (00eb6a6) | T-6 now emulates the absolute retire path; green |
| #120 R2-16 Null-Island LastPos | Export parser have_lat/have_lon presence flags (atgm336h.h) | Host: partial GGA (fix_quality+sats+hdop, empty lat/lon) -> presence false; real (0,0) -> true |
| #132 R2-32 t==0 sentinel | Extract the civil->epoch conversion from SysTimeSyncFromGnss (pure) | Host: 00:00:00 UTC syncs; field-presence not zero-as-sentinel |

## Bench / hardware-only (documented, no host test possible)

| Issue | Why not host | Verification |
|---|---|---|
| #112 R2-08 tick 2.4% bias | RTC hardware semantics | Bench: stopwatch vs GPS window; ttf within 1% |
| #113 R2-09 IWDG margin | LSI/LSE hardware clocks | Bench: survival soak, forced failover |
| #116 R2-12 vcom_Resume DMAR | STOP2 register loss | Bench: STOP2 with GPS live, then fix |
| #117 R2-13 vcom_ReceiveInit | UART state machine | Grep-proof + bench DMA acquisition |
| #119 R2-15 snprintf in flight build | Linker artifact | Build box: flight build map/symbols show buffers gone (A-006 #80) |
| #128 PCAS11 >18 km | Hardware/GPS sim | Bench (HackRF sim) per issue |
| #129 confirmed-burst budget | Live network | Bench against single constrained gateway |

## No test applicable

- #122 R2-18 wrong DMA channel: trivial delete; compile + grep proof.
- #109 R2-05 NVM dead code: decision issue (wire or delete); grep-proof documented in #109/#78.
- #124 solar, #125 downlink commands, #127 LR-FHSS, #131 probe backoff: features — tests arrive with the design, TDD applies then.
- #126 descent: decision record (closed wontfix).
- #133 pressure resolution: decision/doc; ties to #61 characterization.
