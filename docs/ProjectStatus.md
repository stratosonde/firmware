# Stratosonde Firmware — Project Status

**The one page that answers "what is done and what is not."**
Last updated: 2026-08-04 — 2026-08-04 review integrated (R45–R51 → issues #42–#48) · Gate 4 batch split (#39 → #49–#59) · **all work tracking lives in GitHub issues**; historical ledgers/reviews/audits deleted from the repo (git history retains them; per-finding detail is reproduced in the issue bodies).

## Status model

| State | Meaning |
|---|---|
| ✅ DONE | Code complete and build-verified; where hardware matters, also bench-verified with a linked artifact |
| 🟡 CODE-DONE / BENCH-PENDING | Fix is in the code and build-verified; the hardware verification artifact is still missing. **Not** flight sign-off |
| ⬜ OPEN | Not implemented |

Rule: nothing is marked ✅ DONE without a linked verification artifact (bench log, scope capture, test output).

## Where the truth lives

| Question | Authoritative source |
|---|---|
| What should I work on / what is open | **GitHub issues** — the only work tracker (labels `gate:1`…`gate:6`, `bench-gate`, `decision`, `project-health`) |
| Done / bench-pending / open summary | This page |
| Why things are the way they are (design rationale) | `docs/decisions/` — DDR-0001…DDR-0012 |
| Per-finding verification detail (F-001…F-030, R01…R51, N-01…N-04) | **GitHub issues** — each open finding's evidence + fix is reproduced in its issue body (#22–#59) |
| Historical ledgers, reviews, audits (F1–F28, FW-1…FW-22, T1–T4, H3-1…H3-9, bench B1–B8, combined verification ledger) | **Git history only** — deleted from the tree 2026-08-04; last commit containing them is `eaaa1db` |
| Wire formats | `docs/PayloadFormats.md` — ⚠ known-stale until Gate 3 ⑬ (R40–R42, N-01); `docs/LoRaWANApplicationProtocol.md` is the target spec |
| Expansion / Qwiic architecture | `docs/QwiicApplicationArchitecture.md` (package index) |
| Docs index | `docs/README.md` |

## 1. Done ✅

- **Gate 1 — all six flight blockers** @ `db4330d` (build-verified; bench gates 🟡, see §2):
  R01+R02 backup-register ownership map (`Core/Inc/backup_regs.h`) · R24+R23 `$PCAS11,2` pedestrian-mode deletion + NMEA checksum corrections · F-011 GNSS DMA head-wrap fix · R30/D6 per-region join timeout + flight-entry gating · R08 RTC clock-source failover honored by `HAL_RTC_MspInit` · F-001 fatal/recoverable `Error_Handler` split.
- **Earlier workorder (2026-08-01/02):** T1 two-tier session storage, T2 data honesty, T3 mission state machine, T4 flash ring rewrite, F1–F28 and FW-1…FW-22 fixes — row-by-row detail in git history (`ProductionReadinessAssessment.md`, last present @ `eaaa1db`); code-fixed, legacy bench gates B1–B8 🟡.
- **h3lite region engine:** H3-1…H3-7, H3-9 host-verified (H3-8 open → #56).

## 2. Code done, bench pending 🟡

- **Gate 1 bench gates (#26):** GPS-fix backup-register dump (R01/R02) · LSE kill (R08) · NDTR=0 fault injection (F-011) · join-timeout soak (R30) · airborne-mode persistence on hardware (R24) · tri-constellation/1 Hz apply (R23).
- **Legacy bench gates:** B1–B8, IWDG option bits, end-to-end energy budget (detail in git history @ `eaaa1db`; legacy bench issues #12, #20, #21).

## 3. Open work ⬜ — one line per gate, detail in the issues

| Gate | Scope | Issues |
|---|---|---|
| **1 — flight-blocker residuals** | R23 runtime NMEA checksum · F-001 false-success prints · R30/D6 finer points · F-011 absolute DMA counters · **R45 flash timestamps boot-relative not UTC (P0, ~1 line)** | #22, #23, #24, #25, **#42** (+ bench #26) |
| **2 — power budget** | R25/D7 GNSS standby TTF measurement (**first — highest-value bench test**) · R26 GSV off · R07 wake-source latches · R09 LED gating · R17/R39/R16 VREFBUF/W25Q/ADC | #27, #28, #29, #30, #31 |
| **3 — data integrity & coverage** | R03 AU915 DR mapping · payload rework ⑬ (D1–D4, D9 ✅) · confirmed delivery (DDR-0011) · R06+D5 altitude/history fields · R28+R29 silent failures · R10 brownout floor · F-017 key prints · **R46 AS923 channel-plan runtime-settability** | #32, #33, #34, #35, #36, #37, #38, **#43** |
| **4 — persistence & robustness** | F-008 frontier scan · F-007/R12 header generation · F-006/R13 + F-005/R21 scan/ack pair · F-009/R14/D11 EraseAll + F-010 · config group · F-016 NVM context · F-021 header semantics · R11+H3-8 restricted region · R20/R31–R34 GNSS parser · F-029 STOP2 reinit · F-023/D12, F-028/D8, F-027, F-030 | #49–#59 |
| **5 — docs & matrix** | R44 sweep + `PayloadFormats.md` regeneration (after Gate 3) · readiness-matrix artifact rule · A-003/A-005/A-006 · **R47 SendTxData decide/execute split** | #40, **#44** |
| **6 — expansion (Qwiic)** | Settle DDR-0009…0012 open decisions · rail/session state machine · transport parser + tests · first-class persistence · best-effort spool · FPort 12/13 · archive-opportunity state machine · reference apps | #41 |
| **Project health** | **R48 LICENSE (quick win)** · R49 host tests + CI + reproducible build · R50 SONDE_LOG flight gate · R51 minor batch (strcmp→lookup table, vendor API name) | **#45, #46, #47, #48** |

## Ordering hazards (don't unmask bugs while fixing neighbors)

1. **#50 (header generation) before relying on #51's scan/ack changes** — the watermark side effect is only safe once header freshness is a real generation counter; and removing the F-006 CRC-skip before F-005's exact-identity ack exists would re-wedge bulk transfer.
2. **Gate 3 ⑬ payload rework is one wire format** — compact bytes, pressure encoding, bulk length, endianness, and the published decoder change together (#33): version both packets, golden vectors both sides.
3. **R45 (#42) is surgical** — fix the flash write site only; `Deadman_Check()` deliberately keeps MCU time (GPS-disciplined time would false-trip it).
4. **Bench-gate rule** — readiness rows flip to FIXED/VERIFIED only when the fix lands *with* its verification artifact.

## 4. Decisions

- **Decided 2026-08-03:** D9 endianness = **LE-as-truth** with golden-packet release gate · D10 delivery = **confirmed archive, at-least-once, stable record identity, backend dedup** (DDR-0011 — supersedes "document TX-completed").
- **Open maintainer calls (each has an issue):** D1–D4 payload rework (#33) · D5 altitude/history fields (#35) · D6 finer points (#24) · D7 after TTF measurement (#27) · D8 ascent-timer persistence + D12 dead GNSS API (#59) · D11 EraseAll (#52) · R46 AS923 runtime-settability (#43).
- **Open expansion decisions:** addresses `0x42`/`0x50`, rail limits, session durations, producer/schema IDs, FPort 12/13, heartbeat v2 encoding, first-class flash format, delivery evidence, spool policy — #41.

## 5. GitHub tracking notes

- Stale 2025-era issues triaged 2026-08-03: #1–#11 and #13–#19 closed as implemented (with evidence comments); #12, #20, #21 left open (bench/measurement tasks, cross-linked to #26/#27).
- 2026-08-04 consolidation: Gate 4 batch #39 split into #49–#59 and closed; 2026-08-04 review findings filed as R45–R51 = #42–#48; new `project-health` label created.
