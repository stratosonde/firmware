# DDR-0018: Commissioning and LoRaWAN Session Bootstrap

**Status:** Draft — product intent elicited and resolved; implementation validation pending  
**Intent Interview Date:** 2026-08-09  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** Ground-only join policy, two-tier session storage, commissioning telemetry privacy, the one-way commissioning-to-flight door and its anchor, in-flight session degrade ladder, and commissioning-time verification  
**Authority:** Product intent is normative. Flash layouts, copy counts, CRC choices, counter-margin values, and join procedures are implementation bindings.

**Absorbs:** the retired legacy record 'Session Integrity — Two-Tier Storage and the One-Way Commissioning Door' in full, and adds commissioning telemetry contents and dual exit triggers that DDR-0006 never specified. Once this record exists, DDR-0006 contains no remaining unique design knowledge and may be retired.

---

## 1. Intent

Stratosonde cannot be recalled, re-joined, or reconfigured by a human after release. Its LoRaWAN communication therefore has to be **provisioned entirely on the ground** and then survive an unattended mission — including brownouts, watchdog resets, and partial flash corruption — without ever needing or attempting a new join in the air.

The session material has two fundamentally different natures:

- **Credentials** (DevAddr, keys, region parameters) — written once, never legitimately change;
- **Frame counters** — change with every transmission and must never roll back into reuse.

Treating them identically (one blob, one CRC, rewritten together) couples the most fragile write pattern to the most irreplaceable data.

The core design intent is:

> **Join every configured region on the bench, store credentials as verified redundant immutable copies, store counters separately with margin-based recovery, decide commissioning-versus-flight from durable anchors with ambiguity resolving to flight, and degrade per region — never globally — when session state is damaged.**

---

## 2. Product-Level Invariants

### INV-COMM-001 — Joins happen only on the ground

LoRaWAN joins SHALL execute only while the device is in commissioning, against per-region bench gateways, for every configured region.

In flight the device operates on stored session state only. Join code SHALL be unreachable in flight: no error fallback, corruption-recovery path, or region transition may re-enter it.

### INV-COMM-002 — Credentials are immutable, redundant, and verified at commissioning

Per-region credential material SHALL be written once at commissioning as multiple redundant, independently integrity-checked copies, and SHALL NOT be erased or rewritten in flight.

Commissioning SHALL read back and verify every copy after writing. Commissioning SHALL NOT complete with unverified credential storage — redundancy only defeats isolated corruption; a systemic writer fault would clone the same error into every copy.

### INV-COMM-003 — Counters are stored separately with margin recovery

Frame counters SHALL be persisted separately from credentials, using a persist-every-N scheme. On restore, the counter resumes from the last persisted value plus the configured margin, so a rollback can never reuse a counter value.

### INV-COMM-004 — Degradation is per region, never global

Session-state damage SHALL degrade only the affected region's communication capability. Other regions, science acquisition, logging, and mission behavior SHALL continue per their own policies.

### INV-COMM-005 — In-flight degrade ladder

In flight, for a region whose session state fails to restore, firmware SHALL apply this ladder in order:

1. Retry restore from the redundant credential copies.
2. If credentials are good but counters are invalid: reload credentials from a verified copy; counter = last persisted value + margin.
3. If the region's credentials are wholly unrecoverable: **RF silence in that region only** — continue science and local logging, and resume transmission on entering a region with a valid session.

A flight re-join SHALL NOT occur at any rung.

### INV-COMM-006 — Commissioning telemetry is privacy-filtered

During commissioning the device MAY transmit, but its telemetry SHALL include altitude (z) and pressure only — never precise horizontal position (X/Y).

This operationalizes DDR-0002 INV-LIFE-002 (commissioning location privacy) with concrete payload content.

### INV-COMM-007 — The door: one-way, dual-trigger, fail-to-FLIGHT

The commissioning-to-flight transition SHALL be:

- **Preconditioned:** the credential bank is complete and verified (INV-COMM-002);
- **Dual-triggered:** either automatic pressure-drop launch detection (DDR-0002 INV-LIFE-001) or a deliberate operator control (button/arm action) may cause the transition;
- **One-way:** after the transition, commissioning is unreachable for the remainder of the mission;
- **Fail-to-FLIGHT:** the has-flown fact is persistent state (DDR-0010); if that state is corrupt or ambiguous at boot while credentials exist, the device SHALL resolve to flight behavior. Doubt always falls on the flight side — a mid-air reboot must never land in commissioning.

### INV-COMM-008 — Credential state never gains authority over geography

A valid stored session grants the *capability* to communicate in a region, never the *permission*. RF authorization remains governed by DDR-0006/0020/0028 (geography, restricted regions, staleness budget) on every wake.


---

## 3. Behavioral Requirements

Confidence legend:

- **CONFIRMED** — explicitly stated or affirmed during the interview.
- **INFERRED** — strongly implied but wording may still deserve review.
- **OPEN** — product behavior has not yet been fully decided.

| ID | Requirement | Confidence |
|---|---|---|
| BR-COMM-001 | Every configured region SHALL be joined on the bench during commissioning. | CONFIRMED |
| BR-COMM-002 | Join SHALL be unreachable once the device has entered flight. | CONFIRMED |
| BR-COMM-003 | Credential copies SHALL be written once, independently integrity-checked, and never rewritten in flight. | CONFIRMED |
| BR-COMM-004 | Commissioning SHALL read back and verify every credential copy; commissioning SHALL NOT complete with unverified credential storage. | CONFIRMED |
| BR-COMM-005 | A corrupted credential copy SHALL be restorable from a surviving copy. | CONFIRMED |
| BR-COMM-006 | Counters SHALL persist every N frames; restore SHALL resume at last persisted + margin. | CONFIRMED |
| BR-COMM-007 | Counter-area corruption SHALL trigger credential reload + margin resume (ladder rung 2), not credential loss. | CONFIRMED |
| BR-COMM-008 | Wholly unrecoverable regional credentials SHALL produce RF silence in that region only, with science/logging continuing. | CONFIRMED |
| BR-COMM-009 | Commissioning telemetry SHALL carry z + pressure and SHALL NOT carry X/Y. | CONFIRMED |
| BR-COMM-010 | The flight transition SHALL be triggerable by pressure-drop detection or deliberate operator action. | CONFIRMED |
| BR-COMM-011 | Post-transition, commissioning SHALL be unreachable by any software path. | CONFIRMED |
| BR-COMM-012 | Ambiguous/corrupt lifecycle state with credentials present SHALL resolve to flight. | CONFIRMED |
| BR-COMM-013 | Bench recovery from a confused state is a manual re-commissioning action, not a firmware path. | CONFIRMED |
| BR-COMM-014 | Copy count, margin value, N (persist interval), and integrity-check algorithm SHALL be configuration/implementation bindings. | CONFIRMED |
| BR-COMM-015 | Network-server-side frame-counter expectations for the margin-jump case SHALL be documented for operations; firmware does not rely on NS "reset counters" features. | INFERRED |
| BR-COMM-016 | Commissioned identity survives ordinary application servicing: supported firmware service under DDR-0025 SHALL preserve provisioned credential identity unless an explicit factory reprovision operation is invoked. | CONFIRMED — 2026-08-12 interview |

---

## 4. Commissioning Sequence (Intended Flow)

1. Operator powers the device on the bench with bench gateways for each configured region.
2. Firmware detects a virgin credential bank → commissioning behavior.
3. Per region: join, derive session material, write redundant credential copies, **read back and verify every copy** (INV-COMM-002). Commissioning cannot complete otherwise.
4. Commissioning telemetry may transmit — z + pressure only (INV-COMM-006).
5. GNSS may be acquired internally for validation/time (per DDR-0002/0016) without exposing X/Y.
6. Exit: balloon release (pressure drop) or operator button → flight latch persisted → commissioning permanently unreachable (INV-COMM-007).

---

## 5. Relationship to Other Records

- **DDR-0002 (lifecycle):** provides the launch-detection trigger and privacy invariant this record operationalizes; the one-way lifecycle model is shared.
- **DDR-0010 (persistence):** credential bank, counters, and the flight latch are persistence classes with per-item durability policies defined there.
- **DDR-0011 (storage mechanics):** defines the atomicity/integrity mechanics the credential copies and counter areas rely on.
- **DDR-0012 (boot):** boot restores session state per the degrade ladder; the flight latch participates in boot's commissioning-vs-flight determination.
- **DDR-0006/0020/0028 (RF authorization):** a stored session is capability, not permission (INV-COMM-008); region silence for unrecoverable credentials composes with region/restricted/staleness silence.
- **DDR-0009 (fail-soft):** missing credentials never stop sensing/logging; invalid security material is never replaced with fabricated credentials.
- **DDR-0024 (device identity):** regional LoRaWAN provisioning is performed before distribution/flight; end users should not manage keys; credentials belong to the physical hardware across flights (added 2026-08-12).
- **DDR-0025 (firmware servicing):** ordinary application reflash preserves the commissioned identity (BR-COMM-016); only an explicit factory reprovision replaces it.
- **Retired legacy session-integrity record:** absorbed in full (see header).


---

## 6. Open Decisions

### OD-COMM-001 — Operator control details

The deliberate-exit control was specified as "a button" in the interview. Its physical form, debounce/safety against accidental triggering, and whether it is available only before launch detection arms were not resolved.

### OD-COMM-002 — Commissioning completion criteria

Whether commissioning formally requires a successful test join per region in addition to verified credential storage was not fully resolved (verification is required; the join itself is how credentials are produced, so a failed join yields no credentials — but an explicit post-write validation exchange was not decided).

### OD-COMM-003 — Re-commissioning procedure

The manual bench re-commissioning flow (how a virgin bank is re-established deliberately) is an operational/implementation procedure, not flight firmware behavior.

---

## 7. Proof Plan

### P-COMM-001 — Ground-only join

Attempt to trigger every join path from flight state (error fallbacks, region transitions, corruption recovery).

Prove no join is reachable.

### P-COMM-002 — Verified redundant credentials

Complete commissioning, then corrupt one credential copy per region in turn.

Prove restore succeeds from surviving copies and the corrupted copy is repaired or bypassed per the storage policy.

### P-COMM-003 — Unverified storage blocks completion

Force a write fault affecting all credential copies during commissioning.

Prove commissioning does not complete.

### P-COMM-004 — Counter margin recovery

Persist counters, advance several frames without persisting, corrupt the counter area, restore.

Prove credentials reload from a verified copy and the counter resumes at last persisted + margin with no reuse.

### P-COMM-005 — Regional silence on credential loss

Destroy all credential copies for one region; keep another region healthy.

Prove silence only in the dead region, continued science/logging, and normal TX in the healthy region.

### P-COMM-006 — Commissioning privacy

Capture commissioning telemetry.

Prove it contains z + pressure and no X/Y, even with a valid GNSS fix present.

### P-COMM-007 — Dual door triggers

From completed commissioning, trigger (a) simulated pressure-drop launch detection and (b) the operator control, in separate runs.

Prove both produce the one-way transition.

### P-COMM-008 — Door never reopens

After flight entry, exercise resets, corruption injection, and error paths.

Prove commissioning remains unreachable.

### P-COMM-009 — Ambiguity fails to flight

With credentials present, corrupt the lifecycle/latch state; reboot.

Prove flight behavior, never commissioning.

### P-COMM-010 — Session is capability, not permission

Restore a valid session for region A while geography authorizes region B (or restricted).

Prove no region-A transmission occurs (DDR-0006/0020/0028 compose correctly with restored state).


---

## 8. Regeneration Test

A clean-room implementer receiving this DDR should understand that:

- all LoRaWAN joins happen on the ground during commissioning; join is unreachable in flight by any path;
- credentials are written once as redundant, independently verified copies; commissioning cannot complete without read-back verification of every copy;
- counters live separately with persist-every-N plus margin-restore semantics;
- session damage degrades per region: redundant-copy restore → counter-margin recovery → region-only RF silence with science continuing;
- commissioning telemetry carries altitude + pressure but never X/Y;
- the commissioning→flight door is preconditioned on a verified credential bank, triggered by launch detection or operator action, strictly one-way, and fails to FLIGHT under ambiguity;
- a stored session is communication capability only — geography and RF policy still authorize every transmission;
- bench recovery from a confused device is a manual re-commissioning procedure, not firmware behavior.

The implementer should not need today's flash layout, LoRaWAN stack, join code, or commissioning tooling to recreate the intended behavior.

---

## 9. Next Intent Interview

The remaining migration topics are:

1. **Radio and payload policy bindings** — safe-to-fly defaults, debug opt-in, ADR-off, worst-case payload sizing, FOpts usage (absorbs the retired legacy radio/payload records).
2. **Retired legacy erase-before-write record** — disposition decided: mechanics moved to `../FlashStorageNotes.md`.

After these, the legacy DDR set is fully absorbed and retirable, and the V2 corpus is the complete self-contained product design.

