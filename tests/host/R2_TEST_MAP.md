# R2 Test Map — provability of every 2026-08-09 review finding

Rule (owner, 2026-08-09): **no code change without a test that first proves the
bug and fails; the root-cause fix must turn that exact test green.** Items that
cannot be host-tested are marked with their verification path instead —
undocumented "trust me" fixes are not allowed either.

Run: `make -C tests/host baseline` (EXPECT_UNFIXED=1 — green while fixes are
pending, fails only on NEW breakage). Post-fix gate: `make -C tests/host all`.

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
