# DDR-0015: Stale-Position RF Legality and the Staleness Budget

**Status:** Draft — product intent elicited and resolved; implementation validation pending  
**Intent Interview Date:** 2026-08-09  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** RF authorization behavior when GNSS position is stale; the configured maximum position-staleness age; behavior during staleness-induced RF silence; recovery on fresh fix; interaction with held-region, no-region ring search, and restricted-region policy  
**Authority:** Product intent is normative. This record is intended to stand alone as part of the regenerable Stratosonde product design.

**Supersedes:** The retired legacy record 'Stale-Position Region Hold Is Deliberate'. It held the last known region *indefinitely* on stale position. This DDR replaces that with a bounded staleness budget. Once this record exists, that retired record contains no remaining unique design knowledge.

---

## 1. Intent

Stratosonde selects its LoRaWAN region from geography (DDR-0006) and obeys absolute RF silence in restricted regions (DDR-0007). Both policies assume a trustworthy current position. GNSS, however, is an intermittent resource: cold soak, iced antennas, and reacquisition gaps can leave the sonde without a fresh fix for hours.

The product must therefore decide what RF behavior is legal and prudent when the only available position is old.

Two failure modes compete:

- **Transmitting on old geography** risks violating regional RF rules if the sonde has drifted across a boundary — including into a restricted region — without knowing it.
- **Going silent on any missed fix** risks the mission itself: presence, archive recovery, and the path home all depend on RF opportunity, and a transient GNSS gap must not cascade into mission loss (DDR-0009).

The core design intent is:

> **A stale position remains fully authoritative for RF authorization within a configured maximum staleness age. Beyond that age, the sonde falls RF-silent — while continuing all science — until the first fresh fix immediately restores normal RF policy.**

The configured maximum staleness age is deliberately the **only** safeguard. It is the mission's regulatory-risk budget: shorter bounds reduce the distance the sonde can unknowingly drift while transmitting; longer bounds reduce silent gaps.

---

## 2. Product-Level Invariants

### INV-STALE-001 — Stale position within budget is fully authoritative

While the age of the last valid position is at or below the configured maximum staleness age, firmware SHALL treat that position as valid input to all RF authorization decisions.

Staleness within budget confers **no downgrade**: held-region transmission, link probes, archive-recovery bursts, and the no-region ring search are all permitted exactly as they would be with a fresh fix.

### INV-STALE-002 — Beyond budget, RF silence

When the age of the last valid position exceeds the configured maximum staleness age, the sonde SHALL NOT transmit LoRaWAN RF.

This silence persists until a fresh valid GNSS fix is obtained.

### INV-STALE-003 — Staleness silence never stops the mission

During staleness-induced RF silence, the sonde SHALL continue to:

- acquire environmental sensor data;
- attempt GNSS acquisition on every wake where energy policy permits it — GNSS is the only path out of silence;
- maintain time and mission state;
- write full-resolution science records to the local archive;
- return to low power and continue the mission.

### INV-STALE-004 — First fresh fix fully restores RF policy immediately

A single new quality-valid GNSS fix SHALL restore the complete normal RF policy on the same wake cycle.

No multi-fix re-qualification, hysteresis, or probationary period is required. This includes the case where the fresh fix reveals a different supported region (immediate switch per DDR-0006) or a restricted region (immediate silence per DDR-0007 INV-RF-001).

### INV-STALE-005 — The staleness bound is pure time

The maximum staleness age SHALL be a single configured time duration.

Firmware SHALL NOT layer on drift-speed estimation, distance-to-boundary calculation, or other geographic safeguards. Simplicity and predictability of the authorization boundary are deliberate.

### INV-STALE-006 — One staleness budget governs all stale-position RF behavior

The same configured maximum staleness age SHALL bound:

- held-region transmission on a stale in-region position; and
- nearest-region ring search on a stale no-region position (per DDR-0006, run against the stale position).

There are no separate budgets per behavior.

### INV-STALE-007 — Position staleness provenance already suffices

The stale/fresh position marking carried in every science record (per DDR-0003 and DDR-0009) SHALL be the only record of RF authorization basis.

No additional event-log entries, telemetry bits, or authorization-history machinery SHALL be created for stale-position transmission or staleness-silence transitions.

---

## 3. The Three Staleness Bands

The RF authorization outcome for a wake cycle follows the position provenance:

| Band | Position state | RF behavior |
|---|---|---|
| 1 | Fresh fix this cycle | Full DDR-0006/0020 policy: direct region authoritative; ring search if no-region; silence if restricted |
| 2 | Stale, age ≤ configured max | Full normal RF policy on the stale geography: held region with full privileges, or ring search from the stale no-region position |
| 3 | Stale, age > configured max | RF silence. Science, archive, and GNSS attempts continue. Band 1 resumes on the wake with the first fresh fix |

---

## 4. Behavioral Requirements

Confidence legend:

- **CONFIRMED** — explicitly stated or affirmed during the interview.
- **INFERRED** — strongly implied but wording may still deserve review.
- **OPEN** — product behavior has not yet been fully decided.

| ID | Requirement | Confidence |
|---|---|---|
| BR-STALE-001 | A stale in-region position within budget SHALL authorize normal transmission in the held region. | CONFIRMED |
| BR-STALE-002 | A stale no-region position within budget SHALL authorize the nearest-region ring search from that stale position (DDR-0006 rules apply: bounded, nearest-first, first ACK wins for that wake, not sticky). | CONFIRMED |
| BR-STALE-003 | Stale-within-budget operation SHALL permit the full RF toolkit: compact heartbeat, link probes, and archive-recovery bursts. | CONFIRMED |
| BR-STALE-004 | Position age beyond the configured maximum SHALL produce RF silence for that wake and all subsequent wakes until a fresh fix. | CONFIRMED |
| BR-STALE-005 | Staleness silence SHALL NOT suppress sensing, GNSS attempts, archive writes, mission state, or low-power cycling. | CONFIRMED |
| BR-STALE-006 | The first quality-valid fresh fix SHALL restore full RF policy on that same wake. | CONFIRMED |
| BR-STALE-007 | A fresh fix mapping to a different supported region SHALL switch region immediately (DDR-0006), regardless of prior staleness band. | CONFIRMED |
| BR-STALE-008 | A fresh fix mapping to a restricted region SHALL produce immediate RF silence (DDR-0007), regardless of prior staleness band. | CONFIRMED |
| BR-STALE-009 | The staleness bound SHALL be one configured time duration with no geographic augmentation. | CONFIRMED |
| BR-STALE-010 | No new provenance machinery SHALL be added for stale-authorization events; per-record stale flags suffice. | CONFIRMED |
| BR-STALE-011 | During staleness silence, GNSS acquisition SHALL remain eligible every wake that energy policy permits; no long-lived GNSS backoff SHALL be created by the silence state (aligns with DDR-0009 INV-FAIL-004). | CONFIRMED |
| BR-STALE-012 | The specific maximum-staleness value, and whether it is per-mission configurable, is a configuration binding. | OPEN |


---

## 5. Accepted Limitation — Residual Drift Risk

During Band 2, the sonde transmits on geography it *may* have drifted out of — including the unknowable case of drifting into a different or restricted region. Without a fresh fix, this is undetectable on-device.

This risk is **explicitly accepted**:

- The configured maximum staleness age is the entire control for this exposure. It is, in effect, the mission's regulatory-risk budget.
- The alternative safeguards (drift estimation, border-distance computation) were considered during the interview and deliberately rejected in favor of a simple, testable, purely time-based bound.
- Consistent with DDR-0007 INV-RF-005, the moment fresh information exists it wins: a fresh fix revealing restricted geography silences the radio on that same wake.

---

## 6. Relationship to Other Records

- **DDR-0003 (GNSS provenance):** provides the fresh/stale position semantics and the per-record stale marking this policy consumes. Age-based discard of last-known-good remains prohibited for *science* (INV-GNSS-005); this DDR adds an age bound only for *RF authorization*.
- **DDR-0006 (region selection):** unchanged for fresh positions; its ring search is explicitly permitted on stale positions within budget.
- **DDR-0007 (RF authorization):** restricted-region silence is unaffected and always immediate on fresh knowledge. This DDR adds the third silence trigger: excessive position age.
- **DDR-0009 (fail-soft):** staleness silence is a degradation of the communications capability only; all other mission capabilities continue. GNSS is retried every eligible wake.
- **DDR-0012 (boot):** after reset, a restored last-known-good position participates in this policy with its true age; boot does not reset the staleness clock.
- **Retired legacy stale-position-hold record:** superseded and absorbed by this record (see header).

---

## 7. Open Decisions

### OD-STALE-001 — Maximum staleness value

The concrete maximum staleness age is undecided. It should be chosen as a mission configuration value (per DDR-0014 configuration classes), balancing regulatory-risk exposure against silent-gap tolerance. Candidate order of magnitude discussed: hours.

### OD-STALE-002 — Interaction with commissioning

This policy was elicited for flight. Whether staleness budgeting has any meaning during commissioning (where GNSS is normally available and X/Y is privacy-protected per DDR-0002) was not explored and is presumed moot.


---

## 8. Proof Plan

### P-STALE-001 — Held region within budget

Last valid fix inside a supported region; age below the configured maximum.

Prove the sonde transmits in the held region with full privileges, including an archive-recovery burst when link policy admits one.

### P-STALE-002 — Ring search on stale no-region position

Last valid fix in no-region geography; age below the configured maximum.

Prove the nearest-region ring search runs from the stale position under DDR-0006 rules (bounded, nearest-first, first ACK wins for the wake, not sticky).

### P-STALE-003 — Crossing the staleness bound

Run with a held in-region stale position; advance time past the configured maximum without a fresh fix.

Prove:

- no LoRaWAN transmission occurs on subsequent wakes;
- sensors, archive writes, and GNSS attempts continue;
- the mission cycle and low-power return are unaffected.

### P-STALE-004 — First-fix recovery

From Band-3 silence, provide one quality-valid fix.

Prove full RF policy resumes on that same wake with no probationary period.

### P-STALE-005 — Fresh fix to different region

From stale held-region operation, provide a fresh fix mapping to a different supported region.

Prove immediate region switch per DDR-0006.

### P-STALE-006 — Fresh fix to restricted region

From stale held-region operation, provide a fresh fix mapping to a restricted region.

Prove immediate RF silence on that wake while science continues.

### P-STALE-007 — No geographic augmentation

Configure two scenarios identical in stale age but differing in distance-to-boundary of the stale position.

Prove identical RF behavior in both — the bound is pure time.

### P-STALE-008 — Reset does not refresh staleness

Establish a stale position of known age, reset the MCU, reboot.

Prove the restored position retains its true age and the staleness policy applies as if no reset occurred (per DDR-0012 restoration semantics).

### P-STALE-009 — No GNSS backoff during silence

Hold the sonde in Band-3 silence for many wakes with GNSS hardware present but failing.

Prove GNSS acquisition is still attempted on every energy-eligible wake.

### P-STALE-010 — No extra provenance machinery

Exercise stale-authorization transitions.

Prove science records carry the ordinary stale flags and no additional event-log or telemetry structures are produced.


---

## 9. Regeneration Test

A clean-room implementer receiving this DDR should understand that:

- RF authorization depends on position provenance, and stale position has a configured maximum age for RF purposes;
- within that age, stale geography is fully authoritative with no privilege downgrade;
- beyond that age, the sonde is RF-silent but fully mission-active otherwise;
- one fresh fix immediately restores full RF policy on the same wake, including immediate region switch or restricted-region silence;
- the bound is one configured time duration with no drift or border computation;
- the same budget governs held-region transmission and no-region ring search;
- residual drift risk during the within-budget window is an explicitly accepted limitation, with the max age serving as the regulatory-risk budget;
- per-record stale flags are the only provenance record required;
- staleness silence never creates GNSS retry backoff or suppresses science.

The implementer should not need today's region tables, GNSS driver, LoRaWAN stack, packet formats, or source-code layout to recreate the intended behavior.

---

## 10. Next Intent Interview

The queued high-value topics are:

1. **Power-management / energy-adaptation policy** — when to honor requested cadence, slow cadence, skip GNSS, skip archive recovery, and recover toward target cadence (queued by DDR-0014; also resolves DDR-0001 §16 open questions).
2. **Qwiic expansion and application services** — orchestrator role, bus ownership, peripheral discovery, and mission data classes (absorbs the retired legacy Qwiic records).
3. **Commissioning and session bootstrap** — join-on-ground policy, credential tiering, and the commissioning/flight door anchor (absorbs the retired legacy session-integrity record).
4. **Radio and payload policy bindings** — safe-to-fly defaults, ADR-off, worst-case payload sizing (absorbs the retired legacy radio/payload records).

These are independent topics and may be interviewed in any order.


---

## Amendment 2026-08-12 (intent interview reconciliation)

The 2026-08-12 interviews (rounds 1 and 2) independently reaffirmed the existing design — a strong match between intent and this DDR:

- stale position remains scientifically useful;
- RF may continue only inside the configured regulatory staleness allowance;
- beyond the allowance RF stops because regulatory certainty is insufficient;
- science/logging/GNSS attempts continue;
- a fresh valid fix automatically restores RF eligibility.

Added rationale:

> The staleness budget is a regulatory-confidence bound, not a declaration that the old position has stopped being scientifically useful.
