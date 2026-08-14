# Stratosonde V2 Design Corpus — Index and Migration Coverage Matrix

**Date:** 2026-08-09 (last merged: 2026-08-13 intent interview)  
**Method:** Intent Interview V2 / Intent-Anchored Development (`intent-interview-v2.1.md`)

This directory holds the product design corpus (DDR-0001 onward). These records are **intent-authoritative**: the implementation is evidence, not the source of truth. The legacy records formerly in this directory (legacy numbering 0001–0013) were **retired 2026-08-09**; the matrix below remains as the migration audit trail.

**Orientation:** for the whole product contract on one page see
[`../SYSTEM-INVARIANTS.md`](../SYSTEM-INVARIANTS.md) (SI-001…SI-020); for how it
composes into a running system see
[`../ARCHITECTURE-OVERVIEW.md`](../ARCHITECTURE-OVERVIEW.md). Both are
**explanatory** — the DDRs remain normative. Merge audit trails:
[`merge-ledger-2026-08-13.md`](merge-ledger-2026-08-13.md).

Migration rule: a legacy record may be retired only when no unique design knowledge would disappear with it. The matrix below is the audit trail.

## V2 Records

**Canonical identity: `manifest.yaml` (R3-11/#223)** — validated in CI by
`tools/check_ddr_manifest.py`. This table was renumbered 2026-08-12 to match
the actual files (it previously showed the pre-2026-08-09 numbering, +13).
Bare references to 0014-0021 are ambiguous across generations; the manifest
`aliases` section records the old mapping.

| DDR | Title | Status |
|---|---|---|
| 0001 | Energy-Gated Resilient Wake Cycle | Draft — open questions resolved by DDR-0016/0019; amended 2026-08-13 (§17 FULL/SLEEP, start-to-start cadence) |
| 0002 | Mission Lifecycle, Privacy, and Energy-Sustainable Cadence | Draft — amended 2026-08-13 (§19 one-way ascent→float latch; `INV-LIFE-005`/`BR-LIFE-013/014` superseded) |
| 0003 | GNSS Position, Time, Provenance, and Stale-Data Semantics | Draft — amended 2026-08-13 (§17 common fix predicate, never-fixed placeholder) |
| 0004 | Immutable Circular Science Archive and Record Identity | Draft — amended 2026-08-13 (§19 coherent package, arbitrary lookup, substitution) |
| 0005 | Live Telemetry and Archive Delivery Protocol | Draft — radio mechanics bound by DDR-0019; amended 2026-08-13 (§18 request preemption, ephemeral requests) |
| 0006 | LoRaWAN Region Selection and Open-Ocean Probing | Draft — stale-position follow-up resolved by DDR-0015 |
| 0007 | RF Transmission Authorization and Restricted-Region Policy | Draft — stale-position follow-up resolved by DDR-0015 |
| 0008 | Predictive Satellite Opportunity Scheduling | Proposed — future scope, not first flight |
| 0009 | Fail-Soft Operation and Deterministic Fault Recovery | Draft — amended 2026-08-13 (§18 bounded cross-subsystem recovery; `OD-FAIL-006` resolved) |
| 0010 | Persistent Mission State and Clean Reset Recovery | Draft — amended 2026-08-13 (§18 consequence-based classification, omit-not-fabricate; `OD-PERSIST-001` narrowed) |
| 0011 | Persistent Storage Integrity and Recovery Mechanics | Draft — amended 2026-08-13 (§25 skip-and-continue, lazy/bounded reconstruction, JIT erase) |
| 0012 | Boot Sequence and Startup Recovery | Draft — amended 2026-08-13 (§22 no bulk archive scan at boot; `OD-BOOT-006` resolved) |
| 0013 | Time Authority, RTC Continuity, and Monotonic Mission Time | Draft — amended 2026-08-13 (§18 RTC age never silences RF) |
| 0014 | Configuration Management and Future Remote Commands | Proposed — future-oriented; power follow-up resolved by DDR-0016; amended 2026-08-13 (§20 safe downlinks, persisted-config repair, non-OTA scope) |
| 0015 | Stale-Position RF Legality and the Staleness Budget | Draft — resolved 2026-08-09; **amended 2026-08-13 (§11 budget bound to 24 h; Band 0 home-region fallback; owns the single RF-silence budget)** |
| 0016 | Power-Management and Energy-Adaptation Policy | Draft — thresholds pending battery profiling (OD-PWR-001); amended 2026-08-13 (§11 FULL/SLEEP admission; `INV-PWR-009`/`BR-PWR-014` superseded by DDR-0015) |
| 0017 | Qwiic Expansion and Application Services | Draft — resolved 2026-08-09 |
| 0018 | Commissioning and LoRaWAN Session Bootstrap | Draft — resolved 2026-08-09; amended 2026-08-13 (§10 provisioning gate vs. readiness check; `home_region`) |
| 0019 | Radio and Payload Policy Bindings | Draft — resolved 2026-08-09 |
| 0020 | Watchdog and Progress Supervision | Draft — resolved 2026-08-09 |
| 0021 | GNSS Receiver Configuration Policy | Draft — resolved 2026-08-09 |
| 0022 | Mission Purpose, Value Hierarchy, and Autonomous Continuity | Draft — added 2026-08-12 (intent interview round 1) |
| 0023 | Scientific Data Truth, Derived Products, and Onboard Interpretation | Draft — added 2026-08-12 (round 1); amended rounds 2–3 |
| 0024 | Device Identity, Provisioning, Ownership, and Backend Registry | Draft — added 2026-08-12 (round 2); claim-PIN revision round 3 |
| 0025 | Firmware Servicing, Bootloader, and Non-OTA Update Policy | Draft — added 2026-08-12 (round 2) |
| 0026 | Verification, Traceability, Hardware-in-the-Loop CI, and Flight-Readiness Evidence | Draft — added 2026-08-12 (round 2); amended round 3 |
| 0027 | Long-Lived Wire Protocol Compatibility | Draft — added 2026-08-13 (intent interview, final pass); encoding open |

## Legacy Migration Coverage Matrix

| Legacy | Absorbed by | Disposition |
|---|---|---|
| 0001 Forward Progress Always | DDR-0009 (fail-soft, degrade-and-continue, reset-not-hang) | Retired 2026-08-09 |
| 0002 Safe-to-Fly Defaults | DDR-0019 (INV-RADIO-001/002/003) | Retired 2026-08-09 |
| 0003 RTC as Flywheel | DDR-0013 (GNSS authoritative, RTC working copy; commissioning seeding nuance change noted) | Retired 2026-08-09 |
| 0004 Erase-Before-Write NOR Invariant | DDR-0011 (intent level); NOR mechanics are implementation bindings | Retired 2026-08-09 — mechanics moved to `../FlashStorageNotes.md` |
| 0005 Worst-Case Payload Sizing | DDR-0019 (INV-RADIO-004; FOpts/LinkCheck note revised by INV-RADIO-005) | Retired 2026-08-09 |
| 0006 Session Integrity / One-Way Door | DDR-0018 (full absorption + commissioning telemetry contents and dual exit trigger added) | Retired 2026-08-09 |
| 0007 Data Honesty | DDR-0003 + DDR-0009 (stale marking, no fabricated defaults); rule 2 (stale temp = COLD) **revised** by DDR-0016 INV-PWR-007 | Retirable (revision documented) |
| 0008 Mission State Machine | DDR-0002 (deliberate redesign: states → cadence dynamics, manual arm → auto launch detection) | Superseded — retired 2026-08-09 |
| 0009 Stratosonde Orchestrator | DDR-0017 (INV-QWIIC-001) | Retired 2026-08-09 |
| 0010 Qwiic Bus Ownership / Discovery | DDR-0017 (claim window kept; descriptor discovery **rejected** — see DDR-0017 §5) | Retired 2026-08-09 |
| 0011 Heartbeat vs Archive Products | DDR-0005 (deliberate redesign: at-least-once → send-once + backend gap repair) | Superseded — retired 2026-08-09 |
| 0012 Mission Data Classes | DDR-0017 (INV-QWIIC-005) | Retired 2026-08-09 |
| 0013 Stale-Position Region Hold | DDR-0015 (deliberate redesign: indefinite hold → configured staleness budget) | Superseded — retired 2026-08-09 |

## Deliberate Intent Revisions vs. Legacy

These are places where V2 records intentionally change prior decisions (not documentation errors):

1. **Auto launch detection** — legacy 0008 required a deliberate ground arm action; DDR-0002 requires autonomous launch detection with a manual trigger as backup (DDR-0018 dual trigger).
2. ~~**Cadence dynamics replace ASCENT/FLOAT states** — DDR-0002.~~ **Reversed 2026-08-13:** DDR-0002 §19 restores an explicit one-way `ASCENT → FLOAT` cadence latch and retires the reversible dynamics controller (`INV-LIFE-005`, `BR-LIFE-013/014` superseded by `INV-LIFE-011`). Pressure history is still the launch/float detector, but cadence no longer follows atmospheric change reversibly.
3. **Send-once + backend gap repair replaces at-least-once archive delivery** — DDR-0005.
4. **Staleness budget replaces indefinite region hold** — DDR-0015.
5. **Last-known-good temperature replaces cold-assumption** for stale temperature — DDR-0016 (revises legacy 0007 rule 2).
6. **Static profiles replace descriptor discovery** for passive Qwiic peripherals — DDR-0017 (revises legacy 0010).
7. **Confirmed-packet probe replaces LinkCheckReq-on-compact** — DDR-0019 (revises legacy 0005).
8. **GNSS cold-lockout removed** — energy droop admission subsumes it; no temperature gate on GNSS attempts — DDR-0021 (supersedes the legacy 0007 lockout consequence).
9. **Out-of-range configuration scalars are clamped, not rejected** — DDR-0014 INV-CONFIG-011 (2026-08-12 round 3) supersedes the round-2 lean toward outright rejection; rejection now applies only to structurally unrecoverable configuration (INV-CONFIG-012).
10. **PCB QR random value is a candidate physical-possession claim PIN** — DDR-0024 §7 (2026-08-12 round 3) supersedes the round-2 placeholder "QR is identity material, not authorization material"; the claim security protocol remains open.
11. **One-way ascent→float cadence latch replaces reversible cadence dynamics** — DDR-0002 §19 `INV-LIFE-011` (2026-08-13) supersedes `INV-LIFE-005` and promotes the 2026-08-11 `BR-LIFE-013/014` descope to normative intent. Float is terminal for the mission.
12. **Stale-position RF silence is regulatory (24 h, DDR-0015), not energy (6 h, DDR-0016)** — DDR-0015 §11 `BR-STALE-017` (2026-08-13) supersedes DDR-0016 `INV-PWR-009`/`BR-PWR-014`. There is now exactly one staleness budget; `BR-STALE-020` forbids any second independent time-based RF cutoff. Code retimed: `GPS_LOSS_SILENCE_S` 6 h → 24 h.
13. **First-flight per-wake energy admission is FULL or SLEEP** — DDR-0016 §11 / DDR-0001 §17 (2026-08-13). Energy policy never deliberately plans a partial observation (for example "no GNSS this wake"); partial data is a failure outcome under DDR-0009, not an energy mode. This also supersedes the 2026-08-13 pass-1 proposal of a three-tier GNSS-shedding ladder.
14. **Exact next-wake timer state is not mandatory durable state** — DDR-0010 §18 `INV-PERSIST-010` (2026-08-13, final pass) supersedes the pass-1 wording. Configured cadence must survive reset; the transient timer may be lost if at most one observation is missed or shifted.

## 2026-08-12 Intent Interview Merge (rounds 1–3)

Three interview packages were merged into the corpus on 2026-08-12:

- **New records:** DDR-0022 through DDR-0026 (see table).
- **Amendments folded into existing records:** DDR-0001 (INV-WAKE-007–011, BR-WAKE-015/016, §5.7), DDR-0002 (§18: pre-commissioning lifecycle, OD-LIFE-006), DDR-0003 (§16 confirmation), DDR-0004 (BR-ARCH-016), DDR-0005 (INV-TX-007/008/009, BR-TX-021/022, OD-TX-009), DDR-0009 (INV-FAIL-009–015, P-FAIL-010), DDR-0010 (INV-PERSIST-008/009), DDR-0011 (INV-STORE-009/010), DDR-0012 (INV-BOOT-008/009), DDR-0014 (INV-CONFIG-008–015, BR-CONFIG-013, OD-CONFIG-004 resolved), DDR-0015 (confirmation + rationale), DDR-0016 (INV-PWR-010–021, BR-PWR-015, OD-PWR-004), DDR-0017 (INV-QWIIC-009–011, BR-QWIIC-016, OD-QWIIC-005), DDR-0018 (BR-COMM-016), DDR-0019 (INV-RADIO-009–011), DDR-0020 (confirmation).
- **Manifest:** old-generation aliases 0022–0026 retired (they collided with the new canonical records; no live references to the old meaning existed).
- **Supporting artifacts:** `system-operational-assumptions.md` and `open-intent-questions.md` live in this directory; the engineering requirements specification, requirements traceability matrix, Flight-1 readiness checklist, and firmware conformance worklist live in `../requirements/`.
- **Interview source packages:** preserved under `../temp/` (git-ignored audit trail).

## 2026-08-13 Intent Interview Merge (three passes)

A three-pass interview package was merged into the corpus on 2026-08-13. The full
audit trail — source sheets, intra-package supersessions, requirement-ID
reassignments, and the resolved 6 h/24 h conflict — is in
**`merge-ledger-2026-08-13.md`**. Read that before re-merging anything from
`../temp/`.

- **New record:** DDR-0027 (Long-Lived Wire Protocol Compatibility). Drafted as
  "DDR-0022"; renumbered because 0022 was taken. The retired old-generation alias
  `DDR-0027 -> DDR-0014` was removed from the manifest at the same time.
- **Amendments folded into existing records:** DDR-0001 (§17: `INV-WAKE-012`,
  `BR-WAKE-017..020`, `P-WAKE-011..013`), DDR-0002 (§19: `INV-LIFE-011`,
  `BR-LIFE-023..027`, `P-LIFE-012..014`), DDR-0003 (§17: `BR-GNSS-021`,
  `P-GNSS-011/012`), DDR-0004 (§19: `INV-ARCH-008`, `BR-ARCH-017..020`,
  `P-ARCH-011..013`), DDR-0005 (§18: `INV-TX-010`, `BR-TX-023..027`,
  `P-TX-011..013`), DDR-0009 (§18: `INV-FAIL-016`, `BR-FAIL-016..018`,
  `P-FAIL-011/012`), DDR-0010 (§18: `INV-PERSIST-010..012`, `P-PERSIST-010..012`),
  DDR-0011 (§25: `INV-STORE-011`, `BR-STORE-001..003`, `P-STORE-011..014`),
  DDR-0012 (§22: `INV-BOOT-010`, `P-BOOT-012..015`), DDR-0013 (§18:
  `INV-TIME-009`, `BR-TIME-015..017`, `P-TIME-009/010`), DDR-0014 (§20:
  `BR-CONFIG-014..017`, `P-CONFIG-009..011`), DDR-0015 (§11: `INV-STALE-008/009`,
  `BR-STALE-013..020`, `P-STALE-011..016`), DDR-0016 (§11: `INV-PWR-022`,
  `BR-PWR-016..020`, `P-PWR-012..015`), DDR-0018 (§10: `INV-COMM-009`,
  `BR-COMM-017..022`, `P-COMM-011..014`).
- **Open decisions closed:** `OD-FAIL-006` and `OD-BOOT-006` (reset loops never
  escalate), `OD-STALE-001` (budget = 24 h). **Narrowed:** `OD-PERSIST-001`,
  `OD-ARCH-002`, `OD-BOOT-005`.
- **Confirmations only:** DDR-0006 (ring search), DDR-0019 (confirmed probe /
  no-ACK), DDR-0020 (no reset escalation), DDR-0021, DDR-0025 (non-OTA).
- **Cross-cutting docs added:** `../SYSTEM-INVARIANTS.md` (SI-001..SI-020,
  explanatory, each SI naming its owning DDR), `../ARCHITECTURE-OVERVIEW.md`, and
  `../requirements/flight1-mission-definition.md`.
- **Interview source package:** preserved under `../temp/` (git-ignored audit
  trail).

## Known Intent-vs-Code Divergences (implementation work queue)

Captured from the 2026-08-09 corpus-vs-code review; these are conformance tasks for the implementation, not doc gaps:

1. ~~`mission_state.c` implements legacy 0008 (ASCENT/FLOAT, manual enter-flight) — DDR-0002/0018 require cadence dynamics + automatic launch detection.~~ **Reframed 2026-08-13:** the explicit ASCENT/FLOAT one-way latch is now the *required* model (DDR-0002 `INV-LIFE-011`), so `mission_state.c` is conformant on that axis. What remains is confirming automatic launch detection works without the operator action (DDR-0018 `P-COMM-013`).
2. `transmit_plan.c` GPS veto treats stale temperature as COLD (cites legacy 0007) — DDR-0016 requires last-known-good; the cold-lockout itself is removed entirely by DDR-0021.
3. `power_model.c` disables GPS by operating mode — DDR-0016 requires cadence-first + per-operation droop admission. **Sharpened 2026-08-13:** DDR-0016 §11 / DDR-0001 §17 forbid energy policy *deliberately* planning a GNSS-less wake; per-wake admission must be FULL or SLEEP.
4. Region hold is indefinite in code — DDR-0015 requires the staleness budget and Band-3 silence. **Partially closed 2026-08-13:** the budget is now bound (24 h) and `GPS_LOSS_SILENCE_S` was retimed to match, so a silence path exists. Still open: Band-0 `home_region` fallback, and confirming held-region expiry follows the same single budget.
5. No open-ocean ring search (`multiregion_h3.c` is direct lookup only) — DDR-0006. **Reconfirmed 2026-08-13** as a conformance defect, not a doc gap: retaining the previous region in no-region geography instead of running the bounded ring search violates DDR-0006/0015.
6. Archive delivery follows legacy 0011 semantics — DDR-0005 requires send-once watermark + backend gap-repair hooks. **Extended 2026-08-13:** also needs explicit-record-request preemption and ephemeral (non-persisted) request handling (DDR-0005 `INV-TX-010`).
7. **New 2026-08-13:** no reset-count/failure-history escalation may exist anywhere (DDR-0009 `OD-FAIL-006` resolved, DDR-0020). Any repeated-reset adaptation should be removed or disabled for first flight unless separately re-decided.
8. **New 2026-08-13:** archive metadata recovery must not be an unconditional boot prerequisite, and an isolated torn record must not truncate the archive (DDR-0011 `INV-STORE-011`, `BR-STORE-001/002`; DDR-0012 `INV-BOOT-010`).

## Remaining Work

1. ~~Watchdog / progress-supervision policy~~ — **done: DDR-0020** (layered timeouts → checkpoint deadman → hardware watchdog; commissioning exempt; no adaptation).
2. ~~GNSS receiver configuration policy~~ — **done: DDR-0021** (commissioning-only config persisted to receiver flash; cold-lockout dropped in favor of droop admission; fixed acquisition timeout).
3. ~~Legacy 0004 disposition~~ — **done**: mechanics moved to `../FlashStorageNotes.md`.
4. **Battery/load profiling campaign** — hardware task unblocking DDR-0016 OD-PWR-001.
5. ~~Retire legacy records~~ — **done 2026-08-09**: the V2 corpus was renumbered to DDR-0001–0021 and moved into this directory; the 13 legacy records were removed.
6. **Proof work from the 2026-08-13 merge** — the newly confirmed requirements need executable evidence before the code can be called conformant. Queued in `../requirements/firmware-conformance-worklist.md` (FW-CONF-023 onward) and `../requirements/requirements-traceability-matrix.md`.
7. **DDR-0027 bindings** — protocol-version identification encoding (`OD-PROTO-001`) and the backend codec retirement horizon (`OD-PROTO-002`) must be decided with DDR-0019 payload bindings.

