# Stratosonde V2 Design Corpus — Index and Migration Coverage Matrix

**Date:** 2026-08-09  
**Method:** Intent Interview V2 / Intent-Anchored Development (`intent-interview-v2.1.md`)

This directory holds the product design corpus (DDR-0001 onward). These records are **intent-authoritative**: the implementation is evidence, not the source of truth. The legacy records formerly in this directory (legacy numbering 0001–0013) were **retired 2026-08-09**; the matrix below remains as the migration audit trail.

Migration rule: a legacy record may be retired only when no unique design knowledge would disappear with it. The matrix below is the audit trail.

## V2 Records

| DDR | Title | Status |
|---|---|---|
| 0014 | Energy-Gated Resilient Wake Cycle | Draft — open questions resolved by 0029/0032 |
| 0015 | Mission Lifecycle, Privacy, and Energy-Sustainable Cadence | Draft |
| 0016 | GNSS Position, Time, Provenance, and Stale-Data Semantics | Draft |
| 0017 | Immutable Circular Science Archive and Record Identity | Draft |
| 0018 | Live Telemetry and Archive Delivery Protocol | Draft — radio mechanics bound by 0032 |
| 0019 | LoRaWAN Region Selection and Open-Ocean Probing | Draft — stale-position follow-up resolved by 0028 |
| 0020 | RF Transmission Authorization and Restricted-Region Policy | Draft — stale-position follow-up resolved by 0028 |
| 0021 | Predictive Satellite Opportunity Scheduling | Proposed — future scope, not first flight |
| 0022 | Fail-Soft Operation and Deterministic Fault Recovery | Draft |
| 0023 | Persistent Mission State and Clean Reset Recovery | Draft |
| 0024 | Persistent Storage Integrity and Recovery Mechanics | Draft |
| 0025 | Boot Sequence and Startup Recovery | Draft |
| 0026 | Time Authority, RTC Continuity, and Monotonic Mission Time | Draft |
| 0027 | Configuration Management and Future Remote Commands | Proposed — future-oriented; power follow-up resolved by 0029 |
| 0028 | Stale-Position RF Legality and the Staleness Budget | Draft — resolved 2026-08-09 |
| 0029 | Power-Management and Energy-Adaptation Policy | Draft — thresholds pending battery profiling (OD-PWR-001) |
| 0030 | Qwiic Expansion and Application Services | Draft — resolved 2026-08-09 |
| 0031 | Commissioning and LoRaWAN Session Bootstrap | Draft — resolved 2026-08-09 |
| 0032 | Radio and Payload Policy Bindings | Draft — resolved 2026-08-09 |
| 0033 | Watchdog and Progress Supervision | Draft — resolved 2026-08-09 |
| 0034 | GNSS Receiver Configuration Policy | Draft — resolved 2026-08-09 |

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
2. **Cadence dynamics replace ASCENT/FLOAT states** — DDR-0002.
3. **Send-once + backend gap repair replaces at-least-once archive delivery** — DDR-0005.
4. **Staleness budget replaces indefinite region hold** — DDR-0015.
5. **Last-known-good temperature replaces cold-assumption** for stale temperature — DDR-0016 (revises legacy 0007 rule 2).
6. **Static profiles replace descriptor discovery** for passive Qwiic peripherals — DDR-0017 (revises legacy 0010).
7. **Confirmed-packet probe replaces LinkCheckReq-on-compact** — DDR-0019 (revises legacy 0005).
8. **GNSS cold-lockout removed** — energy droop admission subsumes it; no temperature gate on GNSS attempts — DDR-0021 (supersedes the legacy 0007 lockout consequence).

## Known Intent-vs-Code Divergences (implementation work queue)

Captured from the 2026-08-09 corpus-vs-code review; these are conformance tasks for the implementation, not doc gaps:

1. `mission_state.c` implements legacy 0008 (ASCENT/FLOAT, manual enter-flight) — DDR-0002/0018 require cadence dynamics + automatic launch detection.
2. `transmit_plan.c` GPS veto treats stale temperature as COLD (cites legacy 0007) — DDR-0016 requires last-known-good; the cold-lockout itself is removed entirely by DDR-0021.
3. `power_model.c` disables GPS by operating mode — DDR-0016 requires cadence-first + per-operation droop admission.
4. Region hold is indefinite in code — DDR-0015 requires the staleness budget and Band-3 silence.
5. No open-ocean ring search (`multiregion_h3.c` is direct lookup only) — DDR-0006.
6. Archive delivery follows legacy 0011 semantics — DDR-0005 requires send-once watermark + backend gap-repair hooks.

## Remaining Work

1. ~~Watchdog / progress-supervision policy~~ — **done: DDR-0020** (layered timeouts → checkpoint deadman → hardware watchdog; commissioning exempt; no adaptation).
2. ~~GNSS receiver configuration policy~~ — **done: DDR-0021** (commissioning-only config persisted to receiver flash; cold-lockout dropped in favor of droop admission; fixed acquisition timeout).
3. ~~Legacy 0004 disposition~~ — **done**: mechanics moved to `../FlashStorageNotes.md`.
4. **Battery/load profiling campaign** — hardware task unblocking DDR-0016 OD-PWR-001.
5. ~~Retire legacy records~~ — **done 2026-08-09**: the V2 corpus was renumbered to DDR-0001–0021 and moved into this directory; the 13 legacy records were removed.

