# Stratosonde Flight 1 Mission Definition

**Mission:** Stratosonde Flight 1  
**Merged:** 2026-08-13 (from the 2026-08-13 intent-interview package)  
**Document purpose:** Define the mission hypothesis, success criteria, primary objectives, secondary objectives, stretch objectives, and post-flight evidence required to evaluate the first Stratosonde platform flight.

**Status:** Mission-level objectives document. It is **not** a DDR — normative
product intent lives in `../decisions/`.

**Related artifacts:**

- `flight1-validation-readiness-checklist.md` — the pre-launch go/no-go evidence
  gate (DDR-0026 `INV-VER-011`). This document says *what the flight must
  demonstrate*; the checklist says *what must be proved before we launch it*.
- `stratosonde-engineering-requirements-specification-first-flight.md` — SYS-*
  requirements for the first-flight build.
- `requirements-traceability-matrix.md` — intent → implementation → proof links.
- `../SYSTEM-INVARIANTS.md` — the cross-cutting product contract the objectives
  assume.

**Objective-to-intent anchors:** PMO-1 lifecycle → DDR-0002/0018; PMO-2 sensing
and logging → DDR-0001/0004/0011; PMO-3 telemetry → DDR-0005/0019; PMO-4/PMO-5
energy → DDR-0016; SMO-4 autonomous recovery → DDR-0009/0010/0012/0020;
SMO-5 backend integrity → DDR-0023/0027; STO-1 region switching → DDR-0006/0015.

---

# 1. Mission Hypothesis

**Stratosonde can operate as an energy-self-sustaining autonomous atmospheric sensing platform over repeated day-night cycles while maintaining reliable sensing, logging, state transitions, and telemetry.**

The most important proof is not a single successful transmission or a high-altitude ascent. The mission should demonstrate a coherent end-to-end autonomous system that:

- detects and responds correctly to flight phase,
- collects scientifically useful measurements,
- manages its energy budget through daytime and nighttime conditions,
- survives overnight operation,
- recharges after sunrise,
- resumes or continues normal operation,
- and returns enough telemetry and logged data to reconstruct what the system did.

A successful multi-day energy surplus would be especially strong evidence that the platform is suitable for long-duration autonomous operation.

---

# 2. Mission Philosophy

Flight 1 is primarily a **platform validation mission**.

The objective is not to prove every advanced firmware feature on the first flight. Core mission success should remain achievable even if optional features such as multi-region operation, archive dumping, or advanced communications behavior do not work perfectly.

The mission should therefore be evaluated in tiers:

1. **Primary Mission Objectives** — required to demonstrate the fundamental platform.
2. **Secondary Mission Objectives** — important validation of subsystem performance and models.
3. **Stretch Objectives** — high-value capabilities that significantly strengthen the demonstration but are not required for basic mission success.
4. **Engineering Observations** — useful data and anomalies that improve later flights even if they do not affect mission success.

---

# 3. Primary Mission Objectives

## PMO-1 — Autonomous Flight-State Operation

Demonstrate that the platform autonomously progresses through its intended mission states without operator intervention.

Expected sequence may include:

```text
commissioning / pre-launch
→ launch detection
→ ascent mode
→ float detection
→ float / cruise operation
```

### Success Criteria

- Launch or ascent transition occurs automatically.
- Float transition occurs automatically when the configured criteria are met.
- State transitions are recorded in telemetry or persistent logs.
- The system does not require a remote command to continue normal flight operation.
- No unintended reset or state regression causes the platform to return to an inappropriate earlier mission state.

### Evidence

- timestamped state-transition log,
- altitude profile,
- GNSS position data,
- pressure data if applicable,
- firmware event log.

---

## PMO-2 — Reliable Autonomous Sensing and Logging

Demonstrate sustained collection and preservation of the core sensor dataset throughout flight.

### Success Criteria

- Core sensors continue to be sampled according to the expected schedule.
- Measurements remain timestamped and attributable to the correct mission state.
- Data logging continues through ascent, float, nighttime, and recovery from normal resets if any occur.
- Temporary communications loss does not cause loss of locally logged science data.

### Evidence

- raw science archive,
- timestamp continuity,
- sample cadence,
- sensor status/error records,
- reconstructed flight profile.

---

## PMO-3 — Continuous or Predictable Telemetry

Demonstrate that the platform can provide reliable remote telemetry while within a supported communications region.

### Success Criteria

- Heartbeat or compact telemetry packets are received during ascent and float.
- Transmission cadence is broadly consistent with commanded or configured behavior.
- Temporary missed packets do not cause runaway retransmission or system instability.
- Telemetry resumes after ordinary transient communication interruptions.

### Evidence

- gateway/backend packet timestamps,
- frame counters,
- RSSI/SNR where available,
- transmission decision logs,
- gaps and recovery events.

---

## PMO-4 — Overnight Energy Survival

Demonstrate that the platform can survive an entire night on stored energy while maintaining the intended reduced-power autonomous behavior.

### Success Criteria

- The system remains operational from sunset through sunrise.
- Battery voltage remains above the defined survival threshold.
- The platform continues the intended nighttime sensing/logging/telemetry strategy.
- No energy-related brownout or unrecoverable reset occurs.
- The unit wakes or continues operating correctly after sunrise.

### Evidence

- battery voltage profile,
- power-mode transitions,
- nighttime telemetry,
- sunrise recovery behavior,
- reset-reason history.

---

## PMO-5 — Daytime Energy Recovery and Positive Energy Margin

Demonstrate that daytime solar input replenishes the energy consumed overnight and supports another day-night cycle.

### Preferred Success Criterion

By the end of the following charging day:

> Stored energy should recover to at least the previous day's comparable state, preferably with measurable surplus.

### Strong Platform Demonstration

If:

```text
energy at sunset day 2 >= energy at sunset day 1
```

while the payload continues sensing and communicating, the flight provides strong evidence of sustainable operation.

Repeated cycles strengthen the conclusion substantially.

---

# 4. Secondary Mission Objectives

## SMO-1 — Validate the Power Model

Compare actual battery and charging behavior against the pre-flight power model.

Evaluate:

- daytime generation,
- nighttime consumption,
- GNSS power consumption,
- LoRa transmission cost,
- sensor load,
- sleep current,
- thermal effects on battery voltage,
- daily energy margin.

## SMO-2 — Validate Ascent-to-Float Mode Transition

Confirm that the firmware correctly recognizes the transition from ascent to float and changes operating behavior accordingly.

Particularly important behaviors may include:

- GNSS duty cycle,
- sleep strategy,
- sensor cadence,
- transmission cadence,
- power-management policy.

## SMO-3 — Validate GNSS Duty Cycling

Demonstrate that GNSS can be used more aggressively when required and then reliably powered down or duty-cycled during lower-power float operation.

## SMO-4 — Validate Autonomous Recovery

If natural resets, watchdog events, communications failures, or sensor faults occur, evaluate whether the platform recovers without losing the mission.

## SMO-5 — Validate Backend Data Integrity

Demonstrate that the complete data path works:

```text
sensor
→ firmware
→ radio packet
→ LoRaWAN network
→ backend
→ storage
→ visualization / analysis
```

Success requires correct timestamps, units, packet decoding, and reconciliation between backend telemetry and onboard logs.

---

# 5. Stretch Mission Objectives

## STO-1 — Automatic Regulatory Region Switching

Demonstrate automatic transition from one LoRaWAN regulatory region to another based on a fresh, valid position.

An especially significant result would be:

```text
North American operation
→ Atlantic crossing
→ automatic European region selection
→ successful European uplink
```

This is not required for basic Flight 1 success, but would be a major demonstration of autonomous global operation.

## STO-2 — Multi-Continent Communications

Receive valid telemetry through physically separated network infrastructure during the same flight.

## STO-3 — Archive / Bulk Data Recovery

Demonstrate retrieval of higher-resolution stored data beyond the compact heartbeat telemetry.

## STO-4 — Multiple Sustainable Day-Night Cycles

A hierarchy of evidence:

```text
1 night survived      = power architecture works once
2 cycles sustained    = repeatability demonstrated
3+ cycles sustained   = convincing autonomous energy balance
many days sustained   = long-duration platform behavior demonstrated
```

## STO-5 — Demonstrate Long-Distance Drift Continuity

Maintain a coherent mission record through major geographic displacement.

---

# 6. Mission Success Levels

## Level 0 — Pre-Flight / Launch Failure

Examples:

- payload does not boot,
- launch state is not entered,
- antenna or power system mechanically fails,
- telemetry never begins,
- unrecoverable hardware failure occurs before meaningful ascent data is obtained.

## Level 1 — Basic Flight Success

The platform:

- launches,
- detects ascent,
- records sensor data,
- transmits useful telemetry,
- produces a coherent flight profile.

## Level 2 — Primary Mission Success

The platform additionally:

- transitions correctly into float behavior,
- survives overnight,
- remains autonomous,
- wakes or continues operating after sunrise,
- returns sufficient data to reconstruct its behavior.

## Level 3 — Sustainable Platform Success

The platform additionally:

- recharges after sunrise,
- reaches a positive or non-degrading daily energy balance,
- remains capable of repeating the cycle.

## Level 4 — Extended Capability Success

The platform additionally demonstrates one or more stretch capabilities such as:

- automatic region switching,
- Europe or another continent receiving telemetry,
- bulk/archive recovery,
- multiple repeated day-night cycles.

## Level 5 — Exceptional Flight

A long-duration flight that demonstrates:

- repeatable energy self-sufficiency,
- reliable autonomous operation,
- scientifically coherent measurements,
- multi-region communications,
- persistent data preservation,
- robust fault recovery.

---

# 7. Explicit Non-Goals for Flight 1

Flight 1 does **not** need to demonstrate perfection.

The following should not automatically invalidate the mission:

- occasional packet loss,
- inability to complete a full archive dump,
- failure to encounter a region boundary,
- inability to demonstrate every optional science payload,
- occasional recoverable sensor faults,
- minor deviations from modeled power consumption,
- incomplete global communications coverage.

The goal is to prove the platform architecture, not every future feature.

---

# 8. Flight 1 Pre-Launch Evidence Package

Before launch, preserve a reproducible mission snapshot.

Recommended contents:

```text
mission/
├── README.md
├── mission_definition.md
├── flight_configuration.json
├── firmware/
│   ├── git_commit.txt
│   ├── build_manifest.txt
│   ├── firmware.bin
│   ├── firmware.elf
│   └── firmware.map
├── hardware/
│   ├── pcb_revision.txt
│   ├── schematic.pdf
│   ├── bom.csv
│   └── payload_photos/
├── calibration/
│   ├── certificates/
│   ├── calibration_data/
│   └── uncertainty_notes.md
├── simulation/
│   ├── trajectory/
│   └── power_model/
├── preflight/
│   ├── test_results.md
│   ├── battery_state.csv
│   └── checklist.md
└── notes/
    └── known_deviations.md
```

---

# 9. During-Flight Data Package

Preserve raw data before interpretation.

Recommended categories:

- raw LoRaWAN packets,
- decoded telemetry,
- gateway metadata,
- onboard science records if recovered,
- GNSS records,
- battery/power records,
- mission-state transitions,
- reset reasons,
- fault/event logs,
- backend processing logs.

Derived products should remain separate from immutable raw data.

---

# 10. Post-Flight Report Template

## 10.1 Mission Summary

- launch time,
- launch location,
- last received position,
- total flight duration,
- estimated distance,
- maximum altitude,
- number of day-night cycles,
- final mission status.

## 10.2 Mission Hypothesis Result

State clearly:

```text
Supported
Partially supported
Not supported
Inconclusive
```

## 10.3 Objective Results

For each objective:

```text
PASS
PARTIAL
FAIL
NOT TESTED
```

Include objective evidence rather than only narrative conclusions.

## 10.4 Expected vs Observed Behavior

| Subsystem | Expected | Observed | Difference |
|---|---|---|---|
| Flight state | | | |
| GNSS | | | |
| Power | | | |
| LoRaWAN | | | |
| Logging | | | |
| Backend | | | |
| Sensors | | | |

## 10.5 Anomalies

Every unexplained behavior should receive an identifier, e.g.:

```text
FLT001-A01
```

Record:

- timestamp,
- observed behavior,
- expected behavior,
- available evidence,
- suspected cause,
- confidence,
- whether reproduced later.

## 10.6 Lessons Learned

Separate lessons into:

- hardware,
- firmware,
- power,
- communications,
- backend,
- mechanical,
- launch operations,
- calibration/science.

---

# 11. Suggested Public Flight Catalog Structure

Each public mission can have a unique identifier such as:

```text
STRATO-FLT-001
STRATO-FLT-002
STRATO-FLT-003
```

A catalog entry could contain:

```text
flight ID
date
launch location
hardware revision
firmware commit
configuration
payload/science packs
calibration references
trajectory simulation
power simulation
flight photos
raw telemetry
processed telemetry
onboard archive
objective results
anomalies
lessons learned
post-flight report
```

This creates a longitudinal engineering record rather than a collection of disconnected flights.

---

# 12. Scientific and Publication Value

A successful Flight 1 could support a platform paper centered on:

> An open, autonomous, energy-aware, long-duration atmospheric sensing platform with reproducible hardware, firmware, configuration, calibration, and flight records.

The most compelling evidence would include:

- autonomous ascent-to-float transition,
- repeated day-night operation,
- measured energy balance,
- power-model comparison,
- communication continuity,
- onboard-vs-backend data integrity,
- fault recovery,
- complete reproducibility package.

Science-pack papers can then build on the validated platform.

---

# 13. Flight 1 Decision Principle

The final launch decision should answer:

> Are all known credible failures that could destroy the primary mission either fixed, mitigated, or consciously accepted?

Optional capability should not indefinitely delay Flight 1.

Once release-blocking firmware defects, basic HIL tests, backend verification, mechanical checks, and power-system validation are complete, the next major source of knowledge is the flight itself.

---

# 14. One-Sentence Flight 1 Goal

> **Demonstrate that Stratosonde can autonomously sense, log, communicate, survive the night, recharge after sunrise, and sustain a positive energy balance suitable for repeated long-duration atmospheric operation.**

---

# 15. Stretch Vision

If trajectory and conditions cooperate, the ideal Flight 1 story becomes:

```text
launch
→ autonomous ascent
→ correct float transition
→ overnight survival
→ sunrise recharge
→ positive energy recovery
→ continued autonomous operation
→ ocean crossing
→ fresh-position regulatory region switch
→ successful European telemetry
```

That entire sequence is not required to call Flight 1 successful.

But if it occurs, it would be an unusually strong first demonstration of the platform.
