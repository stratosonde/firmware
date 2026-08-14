# DDR-0016: Power-Management and Energy-Adaptation Policy

**Status:** Draft — product intent elicited and resolved at policy level; all concrete thresholds pending battery/load profiling over temperature (configuration bindings)  
**Intent Interview Date:** 2026-08-09  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** Energy-state model, tier semantics, cadence scaling, per-operation droop admission tests, recovery behavior, temperature handling, and GNSS-missing provenance  
**Authority:** Product intent is normative. Exact voltage thresholds, temperature-normalization curves, droop-model parameters, and tier counts are configuration bindings established by battery/load profiling.

**Revises:** the retired legacy data-honesty record's cold-assumption rule ("stale/unknown temperature is treated as COLD") — see §6.

---

## 1. Intent

Stratosonde's energy comes from a small battery whose behavior shifts with temperature, and whose loads differ by orders of magnitude — a battery-ADC read is nearly free; a GNSS acquisition is one of the most expensive things the sonde does.

The product must extract the most science the energy situation allows **without** treating the battery as an abstraction: an 8-bit voltage reading does not by itself say whether a 30-second, ~75 mA GNSS acquisition will hold the rail above the receiver's minimum operating voltage.

Two structural choices follow:

1. **Slowing down is the primary energy lever.** Sleeping longer lets the cells recover and recharge. Shedding GNSS early is counterproductive — without position, the science largely loses its value, so dropping GNSS is close to dropping the mission's purpose.
2. **Expensive operations are admitted by predicted droop, not by raw voltage.** The question is not "is the battery above X?" but "will this specific load pull the rail below this specific peripheral's minimum-valid voltage?" — assessed with temperature taken into account.

The core design intent is:

> **A small tiered energy-state model (voltage + trend + temperature) scales cadence first; expensive operations individually pass a temperature-normalized predicted-droop admission test; recovery toward requested cadence is driven by rising voltage trend; and CRITICAL tier is pure survival — sleep and re-check, refusing all optional work.**

---

## 2. Product-Level Invariants

### INV-PWR-001 — Cadence is the first energy lever

When energy becomes constrained, firmware SHALL first lengthen the sleep interval (slow the effective cadence below the configured target) before shedding mission-valuable work.

GNSS in particular SHALL NOT be shed merely because the energy tier dropped, when slowing the cadence would let the battery recover enough to run it.

### INV-PWR-002 — Expensive operations require a droop admission test

Each expensive operation — at minimum GNSS acquisition and multi-packet radio transmission bursts — SHALL be admitted only if a prediction step concludes the operation's load will not pull the supply rail below that peripheral's minimum-valid operating voltage.

Example binding from the interview: a 30 s GNSS acquisition at ~75 mA must not droop the rail below the GNSS receiver's 3.3 V minimum.

### INV-PWR-003 — Admission decisions use temperature-normalized battery voltage

The battery voltage used for droop prediction SHALL be normalized for temperature (e.g., expressed as its equivalent at a reference temperature such as −40 °C) so that one threshold is meaningful across the flight temperature range.

The exact normalization curve and thresholds are configuration bindings derived from battery/load profiling over temperature.

### INV-PWR-004 — Energy state is a small tiered model

Firmware SHALL maintain a small energy-state model — conceptually GOOD / MARGINAL / LOW / CRITICAL — fed by:

- battery voltage;
- voltage trend;
- temperature.

Tiers primarily scale the sleep interval (effective cadence). Work-class shedding follows the DDR-0001 cost hierarchy when cadence slowing alone is insufficient.

### INV-PWR-005 — Recovery is trend-driven

Promotion to a better energy tier SHALL require the voltage trend to be actually **rising**, regardless of absolute level.

Demotion SHALL occur on a falling trend or a low level.

Level-crossing hysteresis SHALL NOT be the promotion mechanism; trend is the anti-chattering control. This fulfills DDR-0002's requirement that cadence automatically recovers toward the configured target when charging improves, without oscillation.

### INV-PWR-006 — CRITICAL is survival-only

In the CRITICAL tier the sonde SHALL:

- sleep at the maximum interval and re-check;
- refuse all optional work;
- perform logging only if it can be done safely.

The safety reserve in CRITICAL is guaranteed by **refusal**, not by per-operation budgeting. Brownout/reset remains an exceptional fault condition, never a power-management mechanism (per DDR-0001 §15).

### INV-PWR-007 — Stale temperature uses last-known-good everywhere

When temperature cannot be measured (sensor failed or stale), the energy model and voltage normalization SHALL use the last-known-good temperature, retaining its stale marking.

This deliberately revises the retired legacy data-honesty record's cold-assumption rule, which treated unknown temperature as cold for the GPS lockout. Rationale from the interview: availability wins; the residual risk — admitting an operation whose true droop is worse because the sonde has cooled since the last reading — is accepted.

### INV-PWR-008 — Energy policy never authorizes RF illegality

Energy admission is a *gate on top of* RF authorization, never a bypass. Work that passes energy admission is still subject to DDR-0006/0020/0028 region, restricted, and staleness rules. Conversely, energy policy may suppress otherwise-legal RF work.

### INV-PWR-009 — Sustained GNSS outage silences the radio

If no fresh GNSS fix has been obtained for a configured grace window (implementation: `GPS_LOSS_SILENCE_S`, default **6 hours**), the sonde SHALL stop transmitting while continuing to log science to the archive, and SHALL keep attempting GNSS acquisition every cycle until a fresh fix clears the silence.

Rationale (maintainer policy, 2026-08-12): a stale position is not worth radio energy — the archive is the point, and it is recovered when a fresh fix (or recovery) returns. The radio silence conserves the energy that positionless telemetry would burn.

Operational details (as implemented, issue #141 and hardening trail):

- the grace epoch persists across resets (backup register) and is UTC-disciplined, so a reset cannot restart the window;
- a backward RTC step re-seeds the window rather than evaluating a wrapped delta;
- the forced GNSS retry carries its own acquisition budget and is never forced over the hard electrical floor (MODE_SURVIVAL stays dark);
- commissioning is exempt (DDR-0018).

### INV-PWR-010 — Energy margin protects mission continuity, not hardware asset value

Energy margin SHALL be chosen to protect valid peripheral operation, transient/load headroom, predictable recurring science, controlled return to low power, and subsequent mission cycles. The product does not preserve charge merely to maximize battery reserve or physical hardware longevity. (Added 2026-08-12; see DDR-0022 INV-MISSION-003.)

### INV-PWR-011 — Healthy energy may enable additional information return

After required science and configured operating margin are protected, surplus energy MAY admit additional communication or scientific work per DDR-0022 — archive recovery bursts, additional high-resolution science, derived/event products, and enabled diagnostic/status traffic. The energy manager gates whether such work is physically affordable; it does not redefine the mission-value hierarchy. (Added 2026-08-12.)

### INV-PWR-012 — Brownout is an accident, never an intentional operating state

Policy SHALL attempt to avoid brownout before starting a load expected to collapse the rail. (Added 2026-08-12; restates DDR-0001 §15 as a power-policy obligation.)

### INV-PWR-013 — Slow the mission before destroying the mission

When projected energy cannot sustain the configured target through the expected low-energy interval, firmware SHALL lengthen the wake interval while preserving the same core science mission (strengthens INV-PWR-001). (Added 2026-08-12.)

### INV-PWR-014 — GNSS may be shed when its load cannot be admitted

If GNSS cannot be safely supported, firmware SHALL skip it and preserve honest stale/unavailable GNSS provenance (see DDR-0003 / DDR-0023). (Added 2026-08-12; consistent with the droop admission of INV-PWR-002.)

### INV-PWR-015 — Energy adaptation is reversible

Temporary slowing/work shedding SHALL automatically move back toward the configured target as usable energy recovers; it SHALL NOT rewrite configuration (DDR-0014 INV-CONFIG-010). (Added 2026-08-12; complements the trend-driven promotion of INV-PWR-005.)

### INV-PWR-016 — Energy state is continuously re-evaluated

Energy policy SHALL be evaluated repeatedly during normal mission operation. Firmware SHALL NOT require a day/night boundary or sunrise event to leave a conservative state. (Added 2026-08-12.)

### INV-PWR-017 — Adaptation uses hysteresis/stability, not sticky modes

The controller SHOULD avoid rapid oscillation between policies, but SHALL remain capable of moving both toward more conservative and toward more capable operation whenever measured/predicted energy conditions justify it. Hysteresis, filtered trends, or dwell times MAY be used to prevent chatter (the trend-driven promotion of INV-PWR-005 is one such mechanism). (Added 2026-08-12.)

### INV-PWR-018 — Policy is driven by operation capability, not temperature or voltage alone

Illustrative thresholds such as "below −60 °C do not run GNSS" or "below 3.6 V skip GNSS" are examples only. Final admission SHALL be based on characterized evidence that the requested operation can be supported at the current temperature, voltage/state of charge, load history, predicted droop, and charging/solar condition (per INV-PWR-002/003; parameters await OD-PWR-001). (Added 2026-08-12.)

### INV-PWR-019 — Conceptual energy bands are allowed without freezing implementation to four states

The current mental model (added 2026-08-12):

1. **Critical** — insufficient even for useful sensing; return to sleep.
2. **Conserve** — core low-energy science/logging; GNSS/radio may be withheld.
3. **Nominal** — execute configured mission target.
4. **Surplus** — after core mission, permit additional mission-value work.

These bands are conceptual policy bands; implementation MAY use more states or a continuous controller (the existing GOOD/MARGINAL/LOW/CRITICAL tier model of INV-PWR-004 is one valid binding).

### INV-PWR-020 — Critical energy may skip the entire science wake

If available energy is below the characterized minimum required even for low-cost science, firmware MAY perform only minimum state/power checks and return directly to low power. (Added 2026-08-12; consistent with INV-PWR-006 refusal.)

### INV-PWR-021 — Surplus energy funds mission-value work only

Surplus MAY be spent on increased archive/data return, longer application-payload availability, explicitly configured high-energy science, or other approved mission-value work. Battery heating is **not** currently an approved surplus-energy behavior. (Added 2026-08-12.)

### Non-normative engineering note (2026-08-12)

The interviews expect GNSS to be one of the dominant energy consumers and ordinary short radio packets often to be a smaller energy concern, except during large recovery bursts. This is a **hardware hypothesis, not a normative requirement** — battery/load profiling and Otii Ace measurements (OD-PWR-001, DDR-0026) should determine actual relative costs before thresholds or scheduling policy rely on it.
---

## 3. Behavioral Requirements

Confidence legend:

- **CONFIRMED** — explicitly stated or affirmed during the interview.
- **INFERRED** — strongly implied but wording may still deserve review.
- **OPEN** — product behavior has not yet been fully decided.

| ID | Requirement | Confidence |
|---|---|---|
| BR-PWR-001 | The energy-state model SHALL consume voltage, voltage trend, and temperature. | CONFIRMED |
| BR-PWR-002 | In MARGINAL, firmware SHALL first lengthen sleep intervals before shedding work classes. | CONFIRMED |
| BR-PWR-003 | Work shedding, when cadence slowing is insufficient, SHALL follow the DDR-0001 cost hierarchy (most expensive / most sheddable first; cheapest science preserved longest). | CONFIRMED |
| BR-PWR-004 | GNSS acquisition SHALL be admitted only when the temperature-normalized droop prediction holds the rail at or above the GNSS minimum operating voltage for the acquisition's duration and current. | CONFIRMED |
| BR-PWR-005 | Radio transmission bursts (e.g., archive-recovery bursts) SHALL be subject to an analogous droop admission test against the radio's valid operating range. | CONFIRMED |
| BR-PWR-006 | CRITICAL SHALL refuse all optional work and SHALL log only when safe; its reserve strategy is refusal. | CONFIRMED |
| BR-PWR-007 | Tier promotion SHALL require a rising voltage trend; demotion SHALL follow a falling trend or low level. | CONFIRMED |
| BR-PWR-008 | Cadence SHALL automatically move back toward the configured target as the energy state improves, with no operator intervention (per DDR-0002). | CONFIRMED |
| BR-PWR-009 | Unknown/stale temperature SHALL resolve to last-known-good temperature (stale-marked) for normalization and energy decisions. | CONFIRMED |
| BR-PWR-010 | Energy-based suppression of GNSS SHALL NOT create a long-lived retry backoff; each wake re-evaluates admission under current conditions (aligns with DDR-0009 INV-FAIL-004). | CONFIRMED |
| BR-PWR-011 | Science records SHOULD distinguish GNSS "skipped for energy/policy" from "attempted but no fix" when encoding space permits; this is a nice-to-have, since the backend can usually infer cause from battery voltage, temperature, and scenario context. | CONFIRMED (as SHOULD, not SHALL) |
| BR-PWR-012 | Tier boundaries, tier count, normalization curve, and droop-model parameters SHALL be configuration bindings set by battery/load profiling over temperature. | CONFIRMED |
| BR-PWR-013 | Whether tier-to-cadence scaling is a fixed table or a continuous function is an implementation choice. | INFERRED |
| BR-PWR-014 | After a configured GNSS-outage grace window (default 6 h), firmware SHALL inhibit transmission while continuing science logging and GNSS retry, until a fresh fix clears the silence (INV-PWR-009). | CONFIRMED |
| BR-PWR-015 | Energy scarcity gates feasibility rather than inventing a new mission: moving into a constrained energy state SHALL reduce cadence or suppress physically unaffordable work according to established policy; it SHALL NOT create an unrelated alternate mission objective. | CONFIRMED — 2026-08-12 interview |

---

## 4. Worked Example — GNSS Admission

A wake in MARGINAL tier:

1. Firmware lengthens the previous sleep interval (INV-PWR-001) — this wake happened later than the configured target cadence.
2. Science acquisition proceeds per the DDR-0001 checklist: battery ADC, temperature, other sensors, full-resolution logging.
3. GNSS is considered. The measured battery voltage is temperature-normalized (INV-PWR-003) — using last-known-good temperature if the sensor failed (INV-PWR-007).
4. The droop model predicts whether ~30 s at ~75 mA holds the rail ≥ 3.3 V (INV-PWR-002).
   - Pass → GNSS attempted normally.
   - Fail → GNSS skipped this wake; the record may carry "skipped for energy" provenance (BR-PWR-011); the next wake re-evaluates from scratch (BR-PWR-010).
5. Radio work follows DDR-0005/0019 policy, additionally gated by its own droop admission (BR-PWR-005) and RF authorization (INV-PWR-008).
6. The tier for the next wake is set by trend: promotion only on a rising trend (INV-PWR-005).

---

## 5. Accepted Limitations

- **Profiling dependency:** the droop model is only as good as the battery/load-over-temperature characterization behind it. Until that profiling is done, thresholds are placeholders and this DDR cannot be marked Accepted.
- **Stale-temperature risk:** using last-known-good temperature (INV-PWR-007) can over-admit if the sonde has cooled sharply since the last valid reading. Accepted for availability.
- **Trend noise:** trend-driven promotion assumes voltage trend is measurable with enough fidelity to distinguish "rising" from noise; filtering/averaging is an implementation binding.


---

## 6. Relationship to Other Records

- **DDR-0001 (wake cycle):** this DDR operationalizes the §14 energy-cost hierarchy and resolves most of the §16 open questions (energy thresholds → tiered model + droop tests; reserve policy → refusal in CRITICAL; temperature failure → last-known-good; GNSS skipped vs failed → SHOULD distinguish, space permitting). The archive-recovery burst bound and link-establishment evidence questions remain with DDR-0005's protocol refinement.
- **DDR-0002 (lifecycle/cadence):** provides the configured cadence targets this policy slows and the automatic-recovery requirement INV-PWR-005/BR-PWR-008 fulfill.
- **DDR-0005 (telemetry/archive delivery):** archive-recovery bursts and heartbeats are RF work subject to both RF authorization and the droop admission defined here.
- **DDR-0009 (fail-soft):** energy suppression follows the no-permanent-backoff rule; a skipped GNSS is retried at the next energy-eligible wake.
- **DDR-0013 (time):** longer sleeps are scheduled on the monotonic timebase; cadence changes do not disturb absolute-time provenance.
- **DDR-0015 (stale-position RF):** "energy-eligible wake" in that record gets its concrete meaning from the tier model and admission tests defined here.
- **Retired legacy data-honesty record:** its cold-assumption for the GPS lockout is **revised** — stale temperature now resolves to last-known-good everywhere (INV-PWR-007). The rest of DDR-0007 (stale marking, no fabricated defaults, status-byte honesty) is unaffected.
- **DDR-0014 (configuration):** thresholds, curves, and droop parameters are mission/device configuration per that record's classes.

---

## 7. Open Decisions

### OD-PWR-001 — Profiling-derived parameters

Tier boundaries, the temperature-normalization curve, and droop-model parameters (load profiles per operation, internal-resistance model) await a battery/load profiling campaign over temperature. The interview explicitly deferred these: *"I need to profile the battery and load over temperature again."*

### OD-PWR-002 — Archive-burst admission shape

Whether an archive-recovery burst's droop test is per-packet, per-burst, or per-wake-budget was not resolved; it interacts with DDR-0005's remaining burst-bound questions.

### OD-PWR-003 — Status-byte encoding budget

Whether the compact packet has room for the GNSS skip-cause distinction (BR-PWR-011) depends on the payload format's reserved bits; check the payload format doc before committing the encoding.

### OD-PWR-004 — Anti-chatter mechanism

Choose the concrete stability mechanism(s) for policy transitions: hysteresis band, filtered slope, dwell time, confidence window, or a combination (INV-PWR-017). Trend-driven promotion (INV-PWR-005) is the current default. (Added 2026-08-12.)

---

## 8. Proof Plan

### P-PWR-001 — Marginal slows before shedding

Enter MARGINAL with a configured fast cadence target.

Prove sleep intervals lengthen while GNSS remains attempted (subject to droop admission), before any work-class shedding occurs.

### P-PWR-002 — Droop admission blocks GNSS

Set battery/temperature conditions where the normalized droop prediction fails the 3.3 V GNSS floor.

Prove GNSS is not started, the record carries skip provenance if encoded, and all cheaper science still runs.

### P-PWR-003 — Droop admission passes GNSS

Set conditions just above the admission boundary.

Prove GNSS is attempted and the rail remains within peripheral-valid range throughout acquisition (measurable on hardware or modeled in simulation).

### P-PWR-004 — Temperature normalization matters

Present the same raw battery voltage at two temperatures straddling the admission boundary.

Prove the admission decision differs according to the normalization curve.


### P-PWR-005 — Trend-driven promotion

Hold voltage absolute level constant but manipulate trend:

- falling trend → demotion or no promotion;
- rising trend → promotion when other inputs permit.

Prove absolute level alone never promotes.

### P-PWR-006 — No chattering

Drive conditions that oscillate around a tier boundary.

Prove the tier/cadence does not flap cycle-to-cycle; only genuine trend reversals change it.

### P-PWR-007 — CRITICAL survival

Enter CRITICAL.

Prove:

- maximum sleep interval is used;
- all optional work is refused;
- logging occurs only when safe;
- no brownout is induced by policy-admitted work.

### P-PWR-008 — Automatic cadence recovery

From a slowed cadence in LOW, apply strong charging / rising trend.

Prove cadence moves back toward the configured target across subsequent wakes without operator input (fulfills DDR-0002 P-LIFE-008/009 dependencies).

### P-PWR-009 — Stale temperature handling

Fail the temperature sensor after a warm reading, then present battery conditions that would be admission-marginal at cold.

Prove the model uses the last-known-good (warm, stale-marked) temperature per INV-PWR-007.

### P-PWR-010 — No energy backoff memory

Skip GNSS for energy on several consecutive wakes, then restore healthy conditions.

Prove the very next wake admits GNSS with no residual penalty from the skipped history.

### P-PWR-011 — Energy never overrides RF legality

Place the sonde in a restricted region (or Band-3 staleness silence per DDR-0015) with a healthy battery.

Prove no RF transmission occurs regardless of energy tier.

---

## 9. Regeneration Test

A clean-room implementer receiving this DDR should understand that:

- energy management is a small tiered state model fed by voltage, trend, and temperature;
- slowing cadence (longer sleeps) is the first response to energy pressure — not dropping GNSS;
- expensive operations (GNSS, TX bursts) each face a temperature-normalized predicted-droop admission test against their peripheral's minimum-valid voltage;
- promotion requires a rising voltage trend; demotion follows a falling trend or low level; level hysteresis is not the mechanism;
- CRITICAL is survival-only: sleep, re-check, refuse optional work, log only when safe; reserve comes from refusal;
- brownout is a fault to recover from, never a power-management tool;
- stale temperature resolves to last-known-good everywhere (revising the retired legacy data-honesty record's cold assumption);
- energy suppression never creates retry backoff and never overrides RF legality;
- GNSS skip-cause provenance is a SHOULD subject to encoding space;
- all thresholds, curves, and droop parameters are configuration bindings pending profiling.

The implementer should not need today's battery driver, ADC code, power-rail schematics, scheduler implementation, or packet formats to recreate the intended behavior.

---

## 10. Next Intent Interview

The queued high-value topics are:

1. **Qwiic expansion and application services** — orchestrator role, bus ownership, peripheral discovery, and mission data classes (absorbs the retired legacy Qwiic records).
2. **Commissioning and session bootstrap** — join-on-ground policy, credential tiering, and the commissioning/flight door anchor (absorbs the retired legacy session-integrity record).
3. **Radio and payload policy bindings** — safe-to-fly defaults, ADR-off, worst-case payload sizing (absorbs the retired legacy radio/payload records).
4. **Battery/load profiling campaign** — not an interview; a hardware task that unblocks OD-PWR-001 and moves this DDR toward Accepted.

These are independent topics and may be addressed in any order.

