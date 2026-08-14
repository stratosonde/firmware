# DDR-0022: Mission Purpose, Value Hierarchy, and Autonomous Continuity

**Status:** Draft — intent substantially elicited 2026-08-12; implementation comparison and executable proof pending  
**Intent Interview Date:** 2026-08-12  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** Mission purpose, mission-value hierarchy, hardware disposability, science-versus-communications priorities, autonomous continuity, energy as a mission resource, degraded operation, and end-of-mission philosophy  
**Authority:** Product intent is normative. This record defines cross-cutting priorities used to resolve conflicts among narrower DDRs.

---

## 1. Intent

Stratosonde is a long-duration, autonomous, solar-powered stratospheric scientific instrument.

Its purpose is not merely to remain alive. Its purpose is to:

1. repeatedly collect trustworthy atmospheric and positional science;
2. preserve that science locally;
3. report useful science to the ground whenever communication is legally and technically possible; and
4. continue doing so autonomously for as long as energy and functioning hardware permit.

Once launched, the physical sonde is effectively expendable. Recovery is unlikely and hardware preservation has no independent mission value.

Protecting the hardware, conserving battery energy, or minimizing component wear are therefore useful only insofar as they preserve future scientific acquisition and information return.

The central mission principle is:

> **Keep producing trustworthy fresh science at a sustainable recurring cadence, and keep trying to get that science home. Preserve enough operating margin to keep the mission predictable, but do not conserve hardware or energy merely for their own sake.**

Stratosonde is intended to operate indefinitely. There is no planned software end-of-mission condition.

---

## 2. Mission-Level Invariants

### INV-MISSION-001 — Fresh science is the primary product

The sonde exists first to create new scientifically useful observations.

Historical recovery, diagnostics, optional payload work, and secondary telemetry SHALL NOT cause a due current science observation to be skipped or unnecessarily delayed.

### INV-MISSION-002 — Science that is never reported has reduced mission value

Communication is a continuing mission obligation, not merely a convenience.

Whenever RF transmission is authorized, technically possible, and energetically admissible, the sonde SHALL continue attempting to deliver useful current science.

Local archival storage is essential resilience but is not considered a substitute for eventual information return.

### INV-MISSION-003 — Hardware preservation is instrumental, not an objective

Firmware SHALL NOT sacrifice useful mission activity merely to preserve hardware longevity, battery cycle life, or recoverability.

Energy and electrical margins SHALL instead exist to:

- prevent uncontrolled brownout;
- preserve peripheral validity;
- maintain predictable recurring science cycles;
- allow later mission work to continue.

### INV-MISSION-004 — A recurring mission outranks a heroic single cycle

The sonde SHALL NOT deliberately consume energy in a manner that predictably prevents subsequent scheduled mission cycles merely to maximize one current observation or transmission opportunity.

The intended goal is sustained useful operation, not maximum expenditure on an individual wake.

### INV-MISSION-005 — Failed capabilities degrade; they do not redefine the mission

Failure of GNSS, radio, archive storage, or an individual sensor SHALL remove only the capabilities that actually depend on the failed subsystem.

The fundamental mission loop remains:

> wake → acquire what science remains possible → preserve it where possible → communicate where possible → return to low power → try again.

### INV-MISSION-006 — Autonomy is deterministic and configuration-bounded

Stratosonde SHALL autonomously execute its configured mission policy.

The baseline product SHALL NOT autonomously:

- learn new mission policy;
- rewrite its own objectives;
- permanently alter behavior based on an opaque adaptive model;
- silently evolve thresholds or priorities from flight experience.

Long-term policy changes shall arise from:

- validated configuration;
- explicit firmware revision; or
- future authenticated ground commands where separately authorized.

### INV-MISSION-007 — Regulatory authority outranks communication value

The desire to report science SHALL NOT authorize RF transmission when the sonde lacks sufficient regulatory confidence to transmit legally.

If the radio must become silent because position or authorization is insufficient, sensing, archive work, GNSS recovery attempts, and mission cycling SHALL continue.

### INV-MISSION-008 — No ordinary mission-end state

There is no normal software `MISSION_COMPLETE` condition.

The sonde SHALL continue operating while sufficient energy and functioning hardware remain.

Mission termination occurs only because continued operation becomes physically impossible or power is deliberately removed after recovery.

### INV-MISSION-009 — Energy surplus may be converted into information return

Once required science, operating margin, and normal mission obligations are satisfied, additional available energy MAY be used to increase information returned from the sonde.

Examples include:

- full-resolution archive recovery;
- additional first-class science products;
- high-rate sensor products;
- system-status telemetry;
- GNSS diagnostic products;
- other configured diagnostic or best-effort data.

Surplus energy does not create permission to violate cadence, RF authorization, or electrical operating limits.

---

## 3. Mission Value Hierarchy

When objectives genuinely conflict, the product value hierarchy is:

1. **Maintain the ability to execute the recurring mission predictably.**
2. **Acquire the due fresh science observation.**
3. **Preserve the current observation with honest provenance.**
4. **Deliver fresh/current science to the ground when RF is allowed.**
5. **Maintain or regain navigation and other capabilities required to keep future science useful.**
6. **Recover retained historical first-class science.**
7. **Return additional enabled scientific, status, and diagnostic information.**
8. **Preserve hardware longevity or stored energy beyond the margin required for continued mission operation.**

This hierarchy expresses mission value, not necessarily source-code execution order.

Dependencies may require operations to execute in another order.

For example, GNSS may execute before telemetry construction even though transmission of fresh science is a higher-level mission objective.

---

## 4. Behavioral Requirements

| ID | Requirement | Confidence |
|---|---|---|
| BR-MISSION-001 | Every scheduled ordinary mission cycle SHALL attempt to produce a fresh science observation using the capabilities currently available. | CONFIRMED |
| BR-MISSION-002 | If only one useful transmission can be afforded during a cycle, current/fresh science SHALL take priority over historical archive recovery and ordinary diagnostic traffic. | CONFIRMED |
| BR-MISSION-003 | Loss of radio connectivity SHALL NOT change the fundamental science-acquisition mission; science and archive work continue and communication is retried later. | CONFIRMED |
| BR-MISSION-004 | Loss of GNSS SHALL NOT stop unrelated science; last-known information may continue only with honest stale/validity semantics and subject to RF-authorization policy. | CONFIRMED |
| BR-MISSION-005 | Loss of an individual environmental sensor SHALL NOT stop unrelated sensors or the wider mission. | CONFIRMED |
| BR-MISSION-006 | Archive exhaustion SHALL follow the circular-retention policy rather than causing mission termination. | CONFIRMED |
| BR-MISSION-007 | Historical recovery or optional traffic SHALL yield whenever a current science cycle becomes due. | CONFIRMED |
| BR-MISSION-008 | Firmware MAY perform bounded recovery actions such as peripheral reset, power cycling, bus recovery, or equivalent attempts when a subsystem appears failed. | CONFIRMED |
| BR-MISSION-009 | Recovery actions SHALL be bounded and SHALL NOT permanently monopolize the mission loop. | CONFIRMED |
| BR-MISSION-010 | Energy management SHALL preserve sufficient operating and transient margin for predictable continued operation rather than maximizing stored energy for its own sake. | CONFIRMED |
| BR-MISSION-011 | When surplus energy and RF opportunity exist, firmware MAY spend additional resources returning enabled secondary information. | CONFIRMED |
| BR-MISSION-012 | Secondary information return SHALL NOT displace current science or violate RF, power, or cadence policy. | CONFIRMED |
| BR-MISSION-013 | Mission policy SHALL remain deterministic and configuration-controlled rather than self-learning or self-modifying. | CONFIRMED |
| BR-MISSION-014 | No ordinary software mission-end condition SHALL be required. | CONFIRMED |

---

## 5. Failure Philosophy

A subsystem may fail permanently.

That does not imply a mission-wide mode change.

### Radio permanently unavailable

The sonde continues sensing, GNSS as permitted, local archival storage, scheduled wakes, and periodic communication recovery attempts.

The circular archive eventually overwrites its oldest records according to normal retention policy.

### GNSS permanently unavailable

The sonde continues environmental science using approved stale/unavailable-position semantics.

RF transmission remains subject to the stale-position and regulatory policies.

### One sensor permanently unavailable

The failed measurement is stale or unavailable. All unrelated measurements continue.

### External archive unavailable

Current science and live communication continue wherever possible, although historical recovery capability is lost.

The product philosophy is:

> **A broken sonde should remain as useful as its surviving capabilities allow.**

---

## 6. Energy Philosophy

Battery energy is a mission resource and temporary buffer between solar availability and load demand.

It is not an asset to preserve indefinitely.

The sonde should use the available energy budget to sustain useful operation while maintaining sufficient margin against peripheral undervoltage, uncontrolled reset, data corruption, and inability to execute subsequent expected cycles.

The product does not intentionally spend the final available energy on a single high-cost action when doing so predictably destroys recurring operation.

Likewise, once the battery is healthy and the configured operating margin is satisfied, unused solar opportunity may be converted into additional useful information return.

---

## 7. Communication Philosophy

The compact current-science product is the minimum useful proof that the sonde remains scientifically productive and reachable.

If greater RF opportunity exists, transmission may expand into:

1. current high-resolution science where applicable;
2. retained first-class science;
3. additional configured scientific products;
4. status and diagnostic information;
5. other best-effort products.

The exact ordering within secondary classes remains configurable and may be refined by later DDRs.

---

## 8. Determinism

Stratosonde should be explainable after the fact.

Given the firmware version, mission configuration, persistent state, sensor inputs, energy state, and RF/network inputs, an engineer should be able to explain why the sonde made a particular mission decision.

Complex algorithms are permitted where the mission requires them. Opaque autonomous policy evolution is not.

---

## 9. Relationship to Existing DDRs

- **DDR-0001** defines the mechanics and priorities of one wake cycle.
- **DDR-0002** defines commissioning, flight lifecycle, cadence, and indefinite mission continuation.
- **DDR-0005** defines current-science versus archive transmission priority.
- **DDR-0009** defines fail-soft subsystem behavior.
- **DDR-0014** defines configuration and future ground-command authority.
- **DDR-0015** defines the regulatory boundary when position becomes too stale.
- **DDR-0016** defines energy state and operation-level power admission.
- **DDR-0017** defines first-class and best-effort application data classes.
- **DDR-0019** defines radio and payload mechanics.
- **DDR-0023** defines scientific truth, derived data, and onboard interpretation.

Where a narrower DDR appears to conflict with this record's mission-value hierarchy, the conflict SHALL be reviewed explicitly rather than resolved silently in implementation.

---

## 10. Open Decisions

### OD-MISSION-001 — Day/night versus atmospheric-dynamics cadence

The 2026-08-12 interview described operation as approximately a consistent daytime cadence and consistent nighttime cadence, potentially with additional daytime work under solar surplus.

DDR-0002 currently also defines cadence adaptation according to atmospheric pressure/altitude dynamics.

The interview did not explicitly reject that behavior.

A focused follow-up interview SHALL determine whether:

1. day/night energy state and atmospheric dynamics both affect cadence;
2. atmospheric dynamics remain the primary cadence selector;
3. day/night uses separate configured targets;
4. first-flight fixed ASCENT/FLOAT behavior remains an intentional temporary implementation.

Until resolved, the current DDR-0002 cadence policy remains authoritative.

### OD-MISSION-002 — Secondary information ordering

Fresh/current science is unequivocally first.

The strict priority among historical full-resolution science, event products, high-rate science, GNSS diagnostics, system status, and ordinary debug information has not yet been fully ordered.

### OD-MISSION-003 — Aggressive subsystem recovery

Bounded power-cycle/reset/bus-recovery attempts are acceptable.

The exact escalation sequence and how often unusually aggressive recovery should occur remain subsystem-specific decisions.

---

## 11. Proof Targets

### P-MISSION-001 — Current science preempts backlog

Create a large retained backlog under excellent RF conditions. Prove new science observations continue at their required cadence regardless of backlog size.

### P-MISSION-002 — Radio failure does not change mission

Remove all usable RF connectivity for an extended simulated mission. Prove sensing, archive storage, GNSS attempts, wake cadence, and later radio retries continue.

### P-MISSION-003 — Failed subsystem isolation

Permanently fail each major subsystem independently. Prove only its dependent capability is lost.

### P-MISSION-004 — Deterministic replay

Replay identical mission inputs from identical persistent state. Prove policy decisions are identical.

### P-MISSION-005 — Surplus information return

Provide healthy battery, strong charging, permitted RF, and sufficient link opportunity. Prove additional enabled information may be returned without delaying current science.

### P-MISSION-006 — No terminal state

Run accelerated long-duration mission simulation. Prove there is no ordinary transition into a permanent software mission-complete state.

---

## 12. Regeneration Test

A clean-room implementer should understand that:

- Stratosonde is an indefinitely operating scientific instrument;
- the hardware itself is expendable;
- hardware protection exists only to preserve future mission value;
- fresh science is the primary product;
- reporting fresh science is a continuing mission obligation;
- archive recovery is important but secondary to new science;
- surplus energy should be usable for additional information return;
- subsystem failures degrade capabilities independently;
- regulatory uncertainty may silence RF but never stops science;
- mission behavior is deterministic and configuration-bounded;
- the device does not autonomously learn or rewrite its mission;
- there is no normal software end-of-mission condition.

That intent must survive a complete firmware rewrite.
