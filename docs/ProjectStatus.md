# Stratosonde Firmware — Project Status

**The one page that answers "what is done and what is not."**
Last updated: 2026-08-07 — **2026-08-06/07 session: CI green (tracked build + Linux fixes); 8 issues landed (#33, #34, #35, #57, #36, #29, #31, #47) + #56 firmware half — wire formats heartbeat v2 / archive v4, confirmed delivery per DDR-0011, flash record v4, GNSS parser batch, sensor/flash silent-failure fixes, STOP2 wake latches, ADC/power batch, SONDE_LOG flight gate.** All CODE-DONE / BENCH-PENDING per the status model (host-test-verified where noted; no hardware artifacts yet).

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
| Wire formats | `docs/PayloadFormats.md` — **REGENERATED 2026-08-06** (heartbeat v2 LE, archive v1-v4 with v4 current, golden vectors from CI); `docs/LoRaWANApplicationProtocol.md` §6/§7 normative |
| Expansion / Qwiic architecture | `docs/QwiicApplicationArchitecture.md` (package index) |
| Docs index | `docs/README.md` |

## 1. Done ✅

- **Gate 1 — all six flight blockers** @ `db4330d` (build-verified; bench gates 🟡, see §2):
  R01+R02 backup-register ownership map (`Core/Inc/backup_regs.h`) · R24+R23 `$PCAS11,2` pedestrian-mode deletion + NMEA checksum corrections · F-011 GNSS DMA head-wrap fix · R30/D6 per-region join timeout + flight-entry gating · R08 RTC clock-source failover honored by `HAL_RTC_MspInit` · F-001 fatal/recoverable `Error_Handler` split.
- **Earlier workorder (2026-08-01/02):** T1 two-tier session storage, T2 data honesty, T3 mission state machine, T4 flash ring rewrite, F1–F28 and FW-1…FW-22 fixes — row-by-row detail in git history (`ProductionReadinessAssessment.md`, last present @ `eaaa1db`); code-fixed, legacy bench gates B1–B8 🟡.
- **h3lite region engine:** H3-1…H3-7, H3-9 host-verified (H3-8 open → #56).
- **2026-08-06/07 session (all 🟡 bench-pending; evidence comment on each issue; CI green):** #22–#25, #28, #30, #32–#38, #42, #44–#55, #57–#59, #29, #31, #47 closed; #56 firmware + generator halves. Highlights: #33 heartbeat v2 / archive v4 wire formats + golden vectors · #34 confirmed delivery (DDR-0011) · #35 flash record v4 · #57 GNSS parser batch · #36 sensor/flash silent failures · #29 STOP2 wake latches · #31 ADC/power batch · #47 SONDE_LOG flight gate. CI: tracked Debug build fixed for Linux; host tests 157 checks; `se-identity-select.h` zero-key CI fallback.

## 2. Code done, bench pending 🟡

- **Gate 1 bench gates (#26):** GPS-fix backup-register dump (R01/R02) · LSE kill (R08) · NDTR=0 fault injection (F-011) · join-timeout soak (R30) · airborne-mode persistence on hardware (R24) · tri-constellation/1 Hz apply (R23).
- **Legacy bench gates:** B1–B8, IWDG option bits, end-to-end energy budget (detail in git history @ `eaaa1db`; legacy bench issues #12, #20, #21).

## 3. Open work ⬜ — one line per item, detail in the issues

| Item | Scope |
|---|---|
| **#27** (bench, first) | R25/D7 GNSS standby TTF measurement — highest-value bench test; gates D7 |
| **#26** (bench) | Gate 1 artifacts + 2026-08-06/07 session bench checks (confirmed-delivery behavior, restricted-region archive, STOP2 chunk re-entry, W25Q SR1 log, ADC timeout path) |
| **#20, #12, #21** (bench) | Current by state · flash durability (now exercises generation counter + scan-from-zero) · 24 h soak |
| **#56** (partial) | NK GeoJSON → `RESTRICTED.geojson` → regenerate table + host-verify Pyongyang/ocean points (firmware + generator halves done) |
| **#40** | R44 doc sweep (many module docs predate v2/v4 wire formats, record v4, SONDE_LOG, CI) + readiness-matrix artifact rule. PayloadFormats.md / LoRaWANApplicationProtocol.md / FlashLogging.md already regenerated 2026-08-06 |
| **#41** | Gate 6 Qwiic expansion decisions + implementation |
| **#43, #60, #61** | Post-flight: AS923 sub-group runtime-settability · MS5607 + SHT31 characterization |

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
