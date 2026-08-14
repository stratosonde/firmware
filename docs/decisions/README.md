# Stratosonde V2 Design Corpus — Index and Migration Coverage Matrix

**Date:** 2026-08-09  
**Method:** Intent Interview V2 / Intent-Anchored Development (`intent-interview-v2.1.md`)

This directory holds the product design corpus (DDR-0001 onward). These records are **intent-authoritative**: the implementation is evidence, not the source of truth. The legacy records formerly in this directory (legacy numbering 0001–0013) were **retired 2026-08-09**; the matrix below remains as the migration audit trail.

Migration rule: a legacy record may be retired only when no unique design knowledge would disappear with it. The matrix below is the audit trail.

## V2 Records

**Canonical identity: `manifest.yaml` (R3-11/#223)** — validated in CI by
`tools/check_ddr_manifest.py`. This table was renumbered 2026-08-12 to match
the actual files (it previously showed the pre-2026-08-09 numbering, +13).
Bare references to 0014-0021 are ambiguous across generations; the manifest
`aliases` section records the old mapping.

| DDR | Title | Status |
|---|---|---|
| 0001 | Energy-Gated Resilient Wake Cycle | Draft — open questions resolved by DDR-0016/0019 |
| 0002 | Mission Lifecycle, Privacy, and Energy-Sustainable Cadence | Draft |
| 0003 | GNSS Position, Time, Provenance, and Stale-Data Semantics | Draft |
| 0004 | Immutable Circular Science Archive and Record Identity | Draft |
| 0005 | Live Telemetry and Archive Delivery Protocol | Draft — radio mechanics bound by DDR-0019 |
| 0006 | LoRaWAN Region Selection and Open-Ocean Probing | Draft — stale-position follow-up resolved by DDR-0015 |
| 0007 | RF Transmission Authorization and Restricted-Region Policy | Draft — stale-position follow-up resolved by DDR-0015 |
| 0008 | Predictive Satellite Opportunity Scheduling | Proposed — future scope, not first flight |
| 0009 | Fail-Soft Operation and Deterministic Fault Recovery | Draft |
| 0010 | Persistent Mission State and Clean Reset Recovery | Draft |
| 0011 | Persistent Storage Integrity and Recovery Mechanics | Draft |
| 0012 | Boot Sequence and Startup Recovery | Draft |
| 0013 | Time Authority, RTC Continuity, and Monotonic Mission Time | Draft |
| 0014 | Configuration Management and Future Remote Commands | Proposed — future-oriented; power follow-up resolved by DDR-0016 |
| 0015 | Stale-Position RF Legality and the Staleness Budget | Draft — resolved 2026-08-09 |
| 0016 | Power-Management and Energy-Adaptation Policy | Draft — thresholds pending battery profiling (OD-PWR-001) |
| 0017 | Qwiic Expansion and Application Services | Draft — resolved 2026-08-09 |
| 0018 | Commissioning and LoRaWAN Session Bootstrap | Draft — resolved 2026-08-09 |
| 0019 | Radio and Payload Policy Bindings | Draft — resolved 2026-08-09 |
| 0020 | Watchdog and Progress Supervision | Draft — resolved 2026-08-09 |
| 0021 | GNSS Receiver Configuration Policy | Draft — resolved 2026-08-09 |
| 0022 | Mission Purpose, Value Hierarchy, and Autonomous Continuity | Draft — added 2026-08-12 (intent interview round 1) |
| 0023 | Scientific Data Truth, Derived Products, and Onboard Interpretation | Draft — added 2026-08-12 (round 1); amended rounds 2–3 |
| 0024 | Device Identity, Provisioning, Ownership, and Backend Registry | Draft — added 2026-08-12 (round 2); claim-PIN revision round 3 |
| 0025 | Firmware Servicing, Bootloader, and Non-OTA Update Policy | Draft — added 2026-08-12 (round 2) |
| 0026 | Verification, Traceability, Hardware-in-the-Loop CI, and Flight-Readiness Evidence | Draft — added 2026-08-12 (round 2); amended round 3 |

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
9. **Out-of-range configuration scalars are clamped, not rejected** — DDR-0014 INV-CONFIG-011 (2026-08-12 round 3) supersedes the round-2 lean toward outright rejection; rejection now applies only to structurally unrecoverable configuration (INV-CONFIG-012).
10. **PCB QR random value is a candidate physical-possession claim PIN** — DDR-0024 §7 (2026-08-12 round 3) supersedes the round-2 placeholder "QR is identity material, not authorization material"; the claim security protocol remains open.

## 2026-08-12 Intent Interview Merge (rounds 1–3)

Three interview packages were merged into the corpus on 2026-08-12:

- **New records:** DDR-0022 through DDR-0026 (see table).
- **Amendments folded into existing records:** DDR-0001 (INV-WAKE-007–011, BR-WAKE-015/016, §5.7), DDR-0002 (§18: pre-commissioning lifecycle, OD-LIFE-006), DDR-0003 (§16 confirmation), DDR-0004 (BR-ARCH-016), DDR-0005 (INV-TX-007/008/009, BR-TX-021/022, OD-TX-009), DDR-0009 (INV-FAIL-009–015, P-FAIL-010), DDR-0010 (INV-PERSIST-008/009), DDR-0011 (INV-STORE-009/010), DDR-0012 (INV-BOOT-008/009), DDR-0014 (INV-CONFIG-008–015, BR-CONFIG-013, OD-CONFIG-004 resolved), DDR-0015 (confirmation + rationale), DDR-0016 (INV-PWR-010–021, BR-PWR-015, OD-PWR-004), DDR-0017 (INV-QWIIC-009–011, BR-QWIIC-016, OD-QWIIC-005), DDR-0018 (BR-COMM-016), DDR-0019 (INV-RADIO-009–011), DDR-0020 (confirmation).
- **Manifest:** old-generation aliases 0022–0026 retired (they collided with the new canonical records; no live references to the old meaning existed).
- **Supporting artifacts:** `system-operational-assumptions.md` and `open-intent-questions.md` live in this directory; the engineering requirements specification, requirements traceability matrix, Flight-1 readiness checklist, and firmware conformance worklist live in `../requirements/`.
- **Interview source packages:** preserved under `../temp/` (git-ignored audit trail).

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

