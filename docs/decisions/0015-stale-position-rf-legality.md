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

## 3. The Staleness Bands

The RF authorization outcome for a wake cycle follows the position provenance.

Band 0 was added 2026-08-13 (§11) to cover the previously silent "no valid position
ever obtained" case. The configured maximum is **24 h** for first flight
(`BR-STALE-017`).

| Band | Position state | RF behavior |
|---|---|---|
| 0 | **Never** had an accepted mission fix, within budget of the flight transition | Position is invalid/degraded (never fresh). RF MAY be authorized using the commissioned `home_region` (`INV-STALE-008`) |
| 1 | Fresh fix this cycle | Full DDR-0006/0007 policy: direct region authoritative; ring search if no-region; silence if restricted |
| 2 | Stale, age ≤ configured max (24 h) | Full normal RF policy on the stale geography: held region with full privileges, or ring search from the stale no-region position |
| 3 | Stale, age > configured max (24 h), **or** Band 0 expired | RF silence. Science, archive, and GNSS attempts continue. Band 1 resumes on the wake with the first fresh fix |

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

### OD-STALE-001 — Maximum staleness value — **RESOLVED 2026-08-13**

The concrete maximum staleness age was undecided. It should be chosen as a mission
configuration value (per DDR-0014 configuration classes), balancing regulatory-risk
exposure against silent-gap tolerance. Candidate order of magnitude discussed: hours.

**Resolved 2026-08-13 (see §11): the first-flight maximum stale-position RF budget
is 24 hours** (`BR-STALE-017`).

Whether the value remains per-mission configurable is still a configuration binding
(`BR-STALE-012`). This decision also retires the separate 6 h GNSS-outage silence
window formerly held in DDR-0016 `INV-PWR-009`/`BR-PWR-014` — there is now exactly
one budget, and it lives here.

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

---

## 11. Amendment 2026-08-13 (intent interview, passes 1 and 2)

**Disposition:** amend; no new record required. Resolves `OD-STALE-001`.

### Decision delta 1 — the budget is bound to 24 hours

> **The first-flight maximum RF-authoritative position staleness is 24 hours.**

Age is measured from the last accepted quality-valid horizontal GNSS position. For
the abnormal "no valid mission position ever" case (Band 0 below), the flight
transition / start of home-region fallback is the equivalent age anchor.

The boundary convention is `<= 24 h` permitted, `> 24 h` silent.

### Decision delta 2 — one budget, and it lives here

The same 24 h budget SHALL govern:

- held-region RF authorization using the last valid in-region position (Band 2);
- nearest-region ring search based on stale no-region geography (Band 2, DDR-0006);
- commissioned home-region fallback when flight begins before any valid mission X/Y
  exists (Band 0).

**No separate time-based RF cutoff may exist.** This explicitly retires the former
DDR-0016 6-hour GNSS-outage silence window (`INV-PWR-009` / `BR-PWR-014`), which is
now a pointer to this record. See `merge-ledger-2026-08-13.md` §3 for the full
conflict resolution and the corresponding code change.

The silence exists because of **geographic/regulatory uncertainty**, not because
stale-position telemetry is a poor use of energy. Energy policy may still
independently suppress RF work (DDR-0016), but it no longer owns the staleness
cutoff.

### Decision delta 3 — commissioned home region

Provisioning SHALL supply a `home_region` / launch-region fallback suitable for the
deployment (DDR-0018 `BR-COMM-021`).

For the current North American first-flight deployment this may bind to US915. The
architecture SHALL NOT hard-code US915 as a universal worldwide constant; it is a
commissioned/deployment value.

The home region exists specifically to close the abnormal case where flight begins
without any usable horizontal fix.

### INV-STALE-008 — Band 0: no valid mission position ever obtained

If the sonde has entered flight but no valid horizontal GNSS fix has ever been
accepted:

1. position data SHALL be represented as invalid/degraded; `0,0` MAY be encoded only
   with authoritative invalid/stale status (DDR-0003 §17);
2. firmware MAY use the commissioned `home_region` as the temporary RF region;
3. that fallback SHALL be treated as stale/uncertain geography, never as a fabricated
   GNSS fix, and SHALL NOT be promoted to last-known-good;
4. the same 24 h budget applies, anchored at the flight transition;
5. once that budget expires without a fresh fix, RF SHALL become silent (Band 3);
6. science collection, archive logging, RTC timekeeping, and GNSS attempts SHALL
   continue throughout;
7. the first accepted fresh fix immediately replaces the fallback, and normal
   DDR-0006/0007 geography rules apply.

### INV-STALE-009 — RTC age creates no independent RF cutoff

RF silence caused by prolonged GNSS outage is governed by **position/region
uncertainty**, not by any "the RTC has been unsynchronized too long" timer.

While RF is permitted by the position/region policy, RTC-derived absolute UTC remains
acceptable for packet and science timestamps, with GNSS freshness/provenance marked
accordingly (DDR-0013 `INV-TIME-009`).

### New behavioral requirements

| ID | Requirement | Confidence |
|---|---|---|
| BR-STALE-013 | Flight with no accepted mission X/Y SHALL NOT fabricate a valid geographic position. | **CONFIRMED** |
| BR-STALE-014 | While within the budget, firmware MAY authorize RF using the commissioned home region. | **CONFIRMED** |
| BR-STALE-015 | Initial no-fix home-region fallback SHALL expire into RF silence if no fresh valid position is obtained before the budget expires. | **CONFIRMED** |
| BR-STALE-016 | The first fresh valid position SHALL immediately supersede home-region fallback on the same wake. | **CONFIRMED** |
| BR-STALE-017 | The first-flight maximum stale-position RF budget SHALL be 24 hours. | **CONFIRMED** |
| BR-STALE-018 | Position age greater than 24 hours SHALL cause RF silence while science, archive logging, and timekeeping continue. | **CONFIRMED** |
| BR-STALE-019 | RF silence caused solely by position staleness SHALL end immediately after one accepted quality-valid GNSS fix, subject to normal geographic RF authorization. | **CONFIRMED** |
| BR-STALE-020 | RTC synchronization age SHALL NOT create an independent RF-silence timer. | **CONFIRMED** |

### Beyond-budget behavior

When position age exceeds 24 h:

- stop LoRaWAN RF transmission;
- continue science collection;
- continue full-resolution archive logging;
- continue STM32 RTC timekeeping;
- continue GNSS attempts whenever energy policy permits.

### Recovery behavior — one fix is enough

A **single** new quality-valid GNSS fix immediately ends staleness-induced RF silence.

No operator acknowledgement, repeated-fix requirement, dwell period, or special
recovery handshake is required. On that same wake: fresh geography becomes
authoritative, normal region/restricted-area policy is evaluated, and if authorized,
ordinary RF may resume immediately. (This restates `INV-STALE-004` for the Band 0 and
24 h cases.)

### Rationale

The preferred launch path obtains a valid fix before release, so Band 0 is a fault
path, not normal operation. However, a human may omit the readiness step and
automatic pressure launch must still start the mission. Permanent RF silence from the
first second would needlessly discard communication capability, while unbounded
transmission in an assumed region would create regulatory risk. A bounded commissioned
home-region fallback is the predictable middle ground.

After roughly a day without trustworthy position, a drifting sonde may plausibly have
crossed a regulatory boundary, so indefinite transmission on old geography is not
acceptable. The RTC remains useful for science chronology, local science remains
valuable, and GNSS can recover later. Once a fresh valid position returns there is no
product value in delaying RF further — that fix supplies the missing geographic
evidence.

### Proof additions

#### P-STALE-011 — Never-fixed home-region fallback

Provision a home region, enter flight without any accepted X/Y, and remain inside the
budget. Prove position is invalid/degraded, home-region RF policy is used, and no
position is treated as fresh.

#### P-STALE-012 — Never-fixed fallback expires

Continue without a valid position past the budget. Prove RF becomes silent while
science and GNSS attempts continue.

#### P-STALE-013 — First fix supersedes fallback

During home-region fallback, provide a valid fix mapping to another supported region.
Prove the fresh geography becomes authoritative immediately.

#### P-STALE-014 — Twenty-four-hour boundary

Prevent further GNSS fixes after one valid position. Prove RF remains eligible under
normal policy while age is `<= 24 h`, that no LoRaWAN RF is transmitted once age is
`> 24 h`, and that science/archive operation continues.

#### P-STALE-015 — Immediate same-wake recovery

Remain RF-silent due only to expired stale position, then provide one valid GNSS fix.
Prove the fix immediately replaces stale geography and, if normal regional policy
authorizes it, RF is eligible on that same wake.

#### P-STALE-016 — RTC does not independently veto recovery

Let the RTC run without GNSS time synchronization throughout the outage, then obtain a
valid position. Prove RF eligibility is governed by recovered geography and is not
blocked by any separate RTC-age rule.

### Implementation binding

`GPS_LOSS_SILENCE_S` in `LoRaWAN/App/lora_app.h` is the first-flight binding of
`BR-STALE-017` and is set to `24U * 3600U`. It was `6U * 3600U` before 2026-08-13.

### Cross-references

- DDR-0003 §17 — invalid/placeholder position semantics.
- DDR-0006 — nearest-region ring search (required for no-region geography).
- DDR-0007 — restricted-region immediate silence.
- DDR-0013 §18 — RTC remains the working clock; no independent time cutoff.
- DDR-0016 §11 — `INV-PWR-009`/`BR-PWR-014` superseded and redirected here.
- DDR-0018 §10 — `home_region` provisioning.
- `../SYSTEM-INVARIANTS.md` SI-014.

