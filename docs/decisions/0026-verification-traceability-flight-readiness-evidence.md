# DDR-0026: Verification, Traceability, Hardware-in-the-Loop CI, and Flight-Readiness Evidence

**Status:** Draft — verification philosophy substantially elicited 2026-08-12; numeric release thresholds remain open  
**Intent Interview Date:** 2026-08-12  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** V-model traceability, host CI, target execution, hardware-in-the-loop automation, RTT instrumentation, production-image verification, programmable power testing, Otii Ace integration, fault injection, cold-chamber testing, evidence artifacts, and release qualification

---

## 1. Intent

Stratosonde is intended to become a predictable long-duration autonomous scientific instrument.

Compilation and host unit tests are necessary but not sufficient proof for this product class.

Verification should progressively connect:

```text
product intent
    -> DDR requirement
    -> implementation
    -> executable test
    -> captured evidence
    -> flight release decision
```

The project should avoid testing only a special laboratory firmware that materially differs from the image that flies.

> **Use fast software CI for developer feedback, real target hardware for behavioral proof, programmable power and deliberate fault injection for recovery proof, and environmental/endurance testing for flight confidence. Every important requirement should eventually have objective evidence.**

---

## 2. Verification Levels

### Level 0 — Static and build checks
Examples: target build, warnings/static analysis, linker/layout assertions, generated-file checks.

### Level 1 — Host logic tests
Examples: decision logic, packet encoding, ring arithmetic, CRC/integrity, power policy, region logic, time arithmetic, deterministic state transitions.

### Level 2 — Target hardware CI
Run real firmware on an actual Stratosonde PCB.

Minimum first rack:

- production Stratosonde;
- ST-Link/SWD;
- flash/reset control;
- RTT capture for instrumented tests;
- Otii Ace programmable power/current measurement;
- CI/build computer.

### Level 3 — Fault-injection HIL
Programmatically create:

- power removal at arbitrary execution points;
- supply sag;
- reset during persistent writes;
- repeated reset cycles;
- low-voltage entry/recovery;
- communication unavailability when infrastructure permits.

### Level 4 — Environmental/endurance qualification
Use the cold chamber for cold start, cold GNSS, cold radio TX, night-energy profiles, sunrise recovery, long-duration cycling, and combined power/temperature stress.

---

## 3. Product-Level Invariants

### INV-VER-001 — Requirements must be traceable to evidence
Each flight-critical behavioral requirement SHALL have a stable requirement identifier, implementation binding, verification method, and objective pass/fail evidence.

### INV-VER-002 — Production behavior is the verification target
The actual production/flight code path is the object under test. Testability features MAY observe or accelerate tests but SHALL NOT create a second independent mission implementation.

### INV-VER-003 — Instrumentation impact must be checked
RTT, assertions, and extra logging can perturb timing/power. Verification SHALL include instrumented target runs and flight-equivalent/production runs with instrumentation quiet/off as appropriate.

### INV-VER-004 — The release binary receives hardware evidence
At least one release-qualification lane SHALL test the exact or cryptographically identical production application binary intended for flight.

### INV-VER-005 — Power behavior is a first-class regression signal
Power/current behavior SHALL be measurable and retained for important states such as sleep, sensing, GNSS, LoRa TX, flash write, and archive recovery.

### INV-VER-006 — Power loss is deliberately injected
Power interruption SHALL be intentionally injected at difficult execution points, especially persistent-state boundaries.

### INV-VER-007 — Bugs become regression tests
A field, bench, or review-discovered defect SHOULD produce a permanent executable regression test at the cheapest faithful layer.

### INV-VER-008 — Hardware CI is incremental
Do not wait for a fully simulated aircraft. Initial automation can begin with flash, reset, RTT, Otii power control/current capture, voltage sweeps, and power cuts.

### INV-VER-009 — Evidence is reproducible
Hardware results SHOULD identify enough context to reproduce the run.

### INV-VER-010 — Flight readiness is an evidence state
A release is flight ready only when agreed verification gates are satisfied or consciously waived/documented.

### INV-VER-011 — Flight 1 has an explicit validation cycle

Before first flight, the project SHALL maintain a concrete release/readiness checklist tied to the selected firmware build. The checklist SHOULD link requirement, DDR source, implementation binding, verification method, evidence/result, and waiver/open risk if not closed. (Added 2026-08-12, round 3; the living artifact is `docs/requirements/flight1-validation-readiness-checklist.md`.)

### INV-VER-012 — HIL automation is a goal, not a prerequisite to writing requirements

The requirements/test corpus SHALL be written now even if first-flight evidence initially includes manual target testing, bench measurements, inspection, host CI, and cold-chamber runs. Automation can progressively replace manual evidence without redefining requirements. (Added 2026-08-12, round 3.)

---

## 4. Production Code Versus Test Builds

Preferred hierarchy:

1. **Host tests** — pure logic where hardware adds no value.
2. **Instrumented target build** — RTT/assertions/diagnostic counters; narrowly bounded test acceleration permitted.
3. **Flight-equivalent target build** — same production behavior, RTT quiet/non-blocking.
4. **Production release image** — final flight binary with external observations where required.

Test-only time compression MAY be used only where it preserves the invariant being tested.

---

## 5. RTT Policy

RTT is valuable because it provides observability without PCB modifications, but it is not assumed to be behavior-free.

Therefore:

- development/HIL tests MAY use RTT heavily;
- RTT output SHALL be non-blocking in flight-equivalent tests;
- timing/power-sensitive tests SHOULD repeat with RTT quiet/off;
- evidence SHALL identify the instrumentation mode.

A test passes because the system invariant is satisfied, not merely because an RTT string appeared.

---

## 6. Minimal First Hardware-CI Rack

Recommended first stage:

1. **Production Stratosonde PCB**
2. **ST-Link/SWD**
   - flash
   - reset
   - RTT
3. **Otii Ace**
   - programmable power
   - current measurement
   - energy integration
   - scripted power profiles where supported
4. **CI/build computer**
   - build
   - flash
   - control power scenario
   - capture RTT
   - capture power trace
   - evaluate assertions
   - retain artifacts

If useful later, independently drive battery and solar inputs. No PCB modifications are required for the initial layer.

---

## 7. First High-Value HIL Tests

### HIL-001 — Boot smoke
Cold power-on and prove normal mission entry.

### HIL-002 — Repeated reset recovery
Establish flight state, reset at varied phases, prove every reboot converges to a clean ordinary cycle.

### HIL-003 — Power cut during archive writes
Interrupt archive record/metadata writes, restore, prove reconstruction and continued mission.

### HIL-004 — Power cut during LoRaWAN persistence
Interrupt counter/session persistence and prove prohibited counter rollback/reuse does not occur.

### HIL-005 — Identity survives firmware service
Provision identity/credentials, reflash application, prove identity and credentials remain.

### HIL-006 — Voltage sweep
Sweep supply across configured energy thresholds and prove deterministic transitions/hysteresis.

### HIL-007 — Brownout avoidance
Replay declining-energy conditions and prove firmware reduces work/cadence before policy-admitted work induces brownout.

### HIL-008 — GNSS load refusal
Once the electrical admission model exists, create inadequate margin and prove GNSS is skipped before power application and position provenance is stale/unavailable.

### HIL-009 — Wake/sleep endurance
Run accelerated cycles and prove no progressive state corruption or power regression.

### HIL-010 — RTT equivalence
Repeat representative scenarios with RTT enabled and RTT quiet/off; prove mission decisions are equivalent within defined tolerances.

---

## 8. Cold-Chamber Verification

The chamber should provide empirical evidence for DDR-0016, not merely "boots at -X °C".

Campaign dimensions should include:

- temperature;
- battery/SOC proxy;
- solar/input profile;
- GNSS activity;
- radio activity;
- wake cadence.

Collect:

- rail droop;
- load capability;
- recovery after load;
- sleep current;
- operation energy.

Use measured values to set power-admission coefficients/thresholds.

---

## 9. Evidence Artifacts

A hardware-test result SHOULD retain:

- test ID/definition version;
- firmware commit/version;
- binary hash;
- configuration hash/canonical configuration;
- hardware identity/revision;
- power instrument configuration;
- environmental conditions;
- timestamps;
- RTT log where used;
- power/current trace where relevant;
- reset cause;
- persistent-state checks;
- expected invariants;
- pass/fail and failure reason.

---

## 10. V-Model Traceability

```text
Intent interview
      |
      v
DDR invariant / requirement
      |
      v
Implementation binding
      |
      v
Verification requirement
      |
      +--> host test
      +--> target HIL test
      +--> environmental test
      +--> inspection / analysis
      |
      v
objective evidence
      |
      v
release gate
```

A requirement is not verified merely because code appears to implement it. A failing implementation does not redefine the requirement.

---

## 11. Behavioral Requirements

| ID | Requirement | Confidence |
|---|---|---|
| BR-VER-001 | Every flight-critical DDR requirement SHALL have a defined verification method before flight qualification. | CONFIRMED |
| BR-VER-002 | CI SHALL retain fast host/build testing. | CONFIRMED |
| BR-VER-003 | The project SHALL add an automated target-hardware verification lane. | CONFIRMED |
| BR-VER-004 | Initial HIL MAY use only Stratosonde + ST-Link/RTT + programmable power/current instrumentation without PCB modifications. | CONFIRMED |
| BR-VER-005 | Otii Ace SHOULD be used as an early power/current and programmable-power instrument where its automation interface supports the scenario. | CONFIRMED |
| BR-VER-006 | Target tests SHALL exercise power interruption and reset recovery. | CONFIRMED |
| BR-VER-007 | Release qualification SHALL include production/flight-equivalent firmware with RTT disabled/quiet for timing/power-sensitive tests. | CONFIRMED |
| BR-VER-008 | RTT-enabled testing MAY be used for observability but SHALL NOT be the only evidence for RTT-sensitive behavior. | CONFIRMED |
| BR-VER-009 | Power traces SHOULD be regression artifacts for important mission states. | CONFIRMED |
| BR-VER-010 | Discovered bugs SHOULD gain regression tests where practical. | CONFIRMED |
| BR-VER-011 | Cold-chamber testing SHALL validate power/energy assumptions before those assumptions are called flight-proven. | CONFIRMED |
| BR-VER-012 | Hardware verification SHOULD use production code paths rather than a separate long-lived test fork. | CONFIRMED |
| BR-VER-013 | Test-only time compression MAY be used only when it preserves the tested behavior. | INFERRED |
| BR-VER-014 | Each result SHOULD be traceable to firmware, configuration, hardware, and environment. | CONFIRMED |
| BR-VER-015 | Flight readiness SHALL be determined by an explicit release evidence checklist/gate. | CONFIRMED |

---

## 12. Initial Release-Gate Structure

### Gate A — Software integrity
Target build, host tests, static/layout checks, no unresolved flight-blocking known divergence.

### Gate B — Target regression
Boot/reset, persistence fault injection, mission cadence/state-machine, and power-state tests.

### Gate C — Power evidence
Sleep current, GNSS/TX profiles, current regression checks, no intentional brownout.

### Gate D — Environmental evidence
Required cold cases, cold load behavior, low-energy recovery.

### Gate E — Release-image run
Exact release image identified, production instrumentation policy, long-duration soak/mission simulation, artifacts retained.

Numeric criteria should come from measured hardware capability and mission need.

---

## 13. Open Decisions

- **OD-VER-001:** exact release thresholds.
- **OD-VER-002:** exact Otii automation interface.
- **OD-VER-003:** whether initial rack needs independent solar and battery sources.
- **OD-VER-004:** GNSS simulation method, deferred until needed.
- **OD-VER-005:** private LoRaWAN gateway/backend automation, valuable later but not required to start.

---

## 14. Regeneration Test

A clean-room implementer should understand:

- verification is traceability from intent to objective evidence;
- software CI remains fast and cheap;
- real target hardware proves integration;
- the first HIL rack should stay simple;
- ST-Link/RTT plus Otii Ace has high initial leverage;
- power cuts and resets should be injected deliberately;
- current/power is a regression metric;
- RTT perturbation must be checked;
- the production image ultimately needs direct evidence;
- cold testing supplies the physical proof behind the energy model;
- every important defect should become a regression test where practical;
- flight readiness is an explicit evidence gate, not merely "CI is green."
