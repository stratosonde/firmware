# Stratosonde Firmware — Project Status

**The one page that answers "what is done and what is not."**
Last updated: 2026-08-03 (combined review verification + Gate 1 fixes @ `db4330d` + DDR-0009…0012 import @ `d5dde27`).

## Status model

| State | Meaning |
|---|---|
| ✅ DONE | Code complete and build-verified; where hardware matters, also bench-verified with a linked artifact |
| 🟡 CODE-DONE / BENCH-PENDING | Fix is in the code and build-verified; the hardware verification artifact is still missing. **Not** flight sign-off |
| ⬜ OPEN | Not implemented |

Rule (review process note): nothing is marked ✅ DONE without a linked verification artifact (bench log, scope capture, test output).

## Where the truth lives

| Question | Authoritative doc |
|---|---|
| Per-finding status (F-001…F-030, R01…R44, N-01…N-04) and gate work order | `docs/CombinedReviewVerification-2026-08-03.md` |
| Older workorder rows (F1–F28, FW-1…FW-22, T1–T4, H3-1…H3-9) | `docs/ProductionReadinessAssessment.md` |
| Why things are the way they are (design rationale) | `docs/decisions/` — DDR-0001…DDR-0012 |
| Wire formats | `docs/PayloadFormats.md` — ⚠ known-stale until Gate 3 ⑬ (R40–R42, N-01); `docs/LoRaWANApplicationProtocol.md` is the target spec |
| Expansion / Qwiic architecture | `docs/QwiicApplicationArchitecture.md` (package index) |
| Task tracking | GitHub issues — see §5 |

## 1. Done ✅

- **Gate 1 — all six flight blockers** @ `db4330d` (build-verified; bench gates 🟡, see §2):
  R01+R02 backup-register ownership map (`Core/Inc/backup_regs.h`) · R24+R23 `$PCAS11,2` pedestrian-mode deletion + NMEA checksum corrections · F-011 GNSS DMA head-wrap fix · R30/D6 per-region join timeout + flight-entry gating · R08 RTC clock-source failover honored by `HAL_RTC_MspInit` · F-001 fatal/recoverable `Error_Handler` split.
- **Earlier workorder (2026-08-01/02):** T1 two-tier session storage, T2 data honesty, T3 mission state machine, T4 flash ring rewrite, F1–F28 and FW-1…FW-22 fixes — row-by-row in `ProductionReadinessAssessment.md` (code-fixed; legacy bench gates B1–B8 🟡).
- **h3lite region engine:** H3-1…H3-7, H3-9 host-verified (H3-8 open — restricted-region data).

## 2. Code done, bench pending 🟡

- **Gate 1 bench gates:** GPS-fix backup-register dump (R01/R02) · LSE kill (R08) · NDTR=0 fault injection (F-011) · join-timeout soak (R30) · airborne-mode persistence on hardware (R24) · tri-constellation/1 Hz apply (R23).
- **Legacy bench gates:** B1–B8, IWDG option bits, end-to-end energy budget (`ProductionReadinessAssessment.md` §Bench-Verify).

## 3. Open work ⬜ (by gate — detail in CombinedReviewVerification §10)

- **Gate 1 residuals:** R23 runtime `snprintf` checksum construction + pre-commit guard · F-001 false-success prints (`main.c:198-202`) · R30/D6 finer points (3-attempt/~120 s bound, `MultiRegion_InitializeRegionFromChirpstack()` wiring, `LoRaWAN_Init()` acting on return) · F-011 absolute producer/consumer counters (with R27/F-012).
- **Gate 2 — power budget:** R25/D7 GNSS standby TTF measurement (**first — highest-value bench test**) · R26 GSV off · R07 wake-source latches · R09 LED gating · R17 VREFBUF · R39 W25Q spin-down · R16/F-014+A-001 ADC consolidation.
- **Gate 3 — data integrity & coverage:** R03 AU915 DR mapping · ⑬ payload rework (D1–D4, D9 ✅ decided; R04/R05/R15/R40–R43/N-01–N-03) · confirmed-delivery rework (D10/DDR-0011: F-004/N-04/F-005/R21/F-006 + stable archive record IDs) · R06+D5 altitude/history fields · R28 MS5607 + R29 W25Q silent failures · R10 brownout floor · F-017 key-print gating.
- **Gate 4 — persistence & robustness:** F-008 frontier scan from zero · F-007/R12 header generation · F-006/R13 bounded scan + F-005/R21 exact-identity ack · F-009/R14/D11 EraseAll · F-010 · config group (F-002/F-003/F-018/F-019/F-020) · F-016 NVM context · F-021 header semantics · R11 restricted-region archive · R20, R31–R34 GNSS parser · F-029 STOP2 reinit · F-023/D12, F-028/D8, F-030.
- **Gate 5 — docs & matrix:** R44 sweep + `PayloadFormats.md` regeneration (after Gate 3) · readiness-matrix artifact rule · A-003/A-005/A-006 consolidation.
- **Gate 6 — expansion (Qwiic):** settle DDR-0009…0012 open decisions (§12.4 of the verification doc) · rail/session state machine + controller claim · transport parser + fuzz/deadline tests · first-class record persistence (stable IDs) · best-effort spool · FPort 12/13 serializers + backend decoders · archive-opportunity state machine (with Gate 3) · ESP32 camera + passive sensor reference apps.

## 4. Decisions

- **Decided 2026-08-03:** D9 endianness = **LE-as-truth** with golden-packet release gate · D10 delivery = **confirmed archive, at-least-once, stable record identity, backend dedup** (DDR-0011 — supersedes the "document TX-completed" recommendation).
- **Open maintainer calls:** D1–D5 (payload rework details), D6 finer points, D7 (after TTF measurement), D8, D11, D12 — CombinedReviewVerification §8.
- **Open expansion decisions:** addresses `0x42`/`0x50`, rail limits, session durations, producer/schema IDs, FPort 12/13, heartbeat v2 encoding, first-class flash format, delivery evidence, spool policy — CombinedReviewVerification §12.4.

## 5. GitHub tracking

Gate and finding work is tracked as GitHub issues (labels `gate:1`…`gate:6`, `bench-gate`, `decision`) — issue numbers to be cross-linked here. Stale 2025-era issues #1–#21 were triaged on 2026-08-03.
