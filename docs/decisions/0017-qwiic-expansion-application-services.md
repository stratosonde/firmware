# DDR-0017: Qwiic Expansion and Application Services

**Status:** Draft — product intent elicited and resolved; implementation validation pending  
**Intent Interview Date:** 2026-08-09  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** Qwiic session bus ownership, the claim window, Application Controller service boundary, session lifecycle and deadlines, published-data classes, passive-peripheral commissioning model, and energy gating of application sessions  
**Authority:** Product intent is normative. Claim-window timing, service-protocol framing, addresses, and profile encodings are implementation bindings.

**Absorbs:** the retired legacy records covering the orchestrator role, mission data classes, and bus ownership — with one deliberate revision: the old bus-ownership record's descriptor-EEPROM runtime discovery is **rejected**; passive peripherals are statically commissioned only. Once this record exists, legacy 0009/0010/0012 contain no remaining unique design knowledge and may be retired.

---

## 1. Intent

Stratosonde's expansion connector must support two very different kinds of attachment without either one endangering an unattended flight:

- **Intelligent Application Controllers** (e.g., an ESP32 camera) that need to control the I2C bus for a bounded session; and
- **Simple passive sensors** that expect Stratosonde to control the bus.

The product intent is that expansion is expressive but never trusted:

> **Stratosonde remains the mission orchestrator in every configuration. An Application Controller may borrow the I2C bus for one bounded, energy-admitted session and use a narrow service protocol; it never owns flash, radio, mission state, or scheduling. Passive peripherals are commissioned statically and treated as ordinary sensors.**

Optional payloads must never be able to compromise core sensing, logging, heartbeat, regional compliance, or return to sleep.

---

## 2. Product-Level Invariants

### INV-QWIIC-001 — Stratosonde is always the mission orchestrator

Regardless of attached hardware, Stratosonde SHALL retain ownership of:

- wake/sleep timing;
- Qwiic rail power and the hard session deadline;
- mission state and safety policy;
- core sensors and the scientific archive;
- persistent-storage allocation;
- LoRaWAN session state, data rate, and transmit scheduling;
- the decision to accept, reject, defer, evict, or transmit application data.

Bus mastership and mission ownership are separate concepts.

### INV-QWIIC-002 — One controller per powered session, decided by a claim window

At rail power-up:

1. Stratosonde releases the bus and enables its application-service target.
2. A claim window opens.
3. An Application Controller may claim the session explicitly.
4. If a claim is accepted, that controller is the sole I2C controller until the rail is removed.
5. If no valid claim arrives before the window closes, Stratosonde owns the bus for the remainder of the session.

Ownership SHALL NOT change again during that powered session. A late claim SHALL be rejected. No multi-controller operation is used after ownership is decided.

### INV-QWIIC-003 — Narrow service boundary

While an Application Controller owns the bus, Stratosonde SHALL expose only these services:

- publish a first-class science record;
- publish an opaque best-effort object or fragment;
- read mission time and current navigation context;
- report session completion.

The Application Controller SHALL NOT receive direct ownership of Stratosonde flash, LoRaWAN state, radio configuration, mission state, or sleep scheduling.

### INV-QWIIC-004 — Sessions are budgeted with a hard deadline

At claim time the application SHALL receive a session-length budget.

The application MAY report completion early. The rail MAY be removed at the negotiated or hard deadline regardless of application state. Application firmware SHALL treat the deadline as authoritative and SHALL tolerate immediate power removal at any point.

### INV-QWIIC-005 — Two data classes with different guarantees

**First-class published science records** SHALL be:

- validated for framing, length, and declared producer identity;
- schema identified and versioned;
- assigned a stable archive record ID (per DDR-0004);
- stored transactionally in durable flash;
- retained and transmitted under the same policy as core science records — full equals, not second-class guests.

**Best-effort objects** MAY be buffered in a bounded spool and MAY be rejected, evicted, partially transmitted, or permanently lost without affecting mission success. Acceptance into the spool means only that Stratosonde copied the accepted bytes under its control; it is not a promise of retention or delivery.

Best-effort traffic SHALL NOT starve heartbeat or first-class archive traffic.

### INV-QWIIC-006 — Passive peripherals are statically commissioned only

Passive sensor peripherals SHALL come from a commissioned static profile. There is **no runtime discovery**: no descriptor EEPROM reading and no blind address scan as a production mechanism.

A profiled peripheral that fails to respond SHALL be handled with ordinary fail-soft semantics (DDR-0009): its data is marked stale/absent, it is retried on subsequent eligible wakes, and it SHALL NOT block or delay the rest of the cycle.

### INV-QWIIC-007 — Smart-controller sessions are energy-gated

Powering the rail for an Application Controller session SHALL be treated as an expensive operation subject to DDR-0016 energy policy (tier gating / droop admission). In insufficient energy tiers, the session simply does not occur that wake.

Passive profiled sensors are cheap enough that they SHALL run whenever their cycle is due, subject only to the ordinary science-acquisition policy.

### INV-QWIIC-008 — Optional payloads cannot bypass mission policy

Application activity SHALL NOT bypass data honesty (stale marking), persistence rules, regional RF compliance, energy policy, or transmit scheduling. Service acceptance means Stratosonde has accepted responsibility under the declared data class; it does not imply network delivery.

### INV-QWIIC-009 — Stratosonde core is the resource orchestrator

Application payloads/controllers SHALL NOT independently decide that they may consume additional power, awake time, or radio airtime. (Added 2026-08-12; strengthens INV-QWIIC-001 for the resource dimension.)

### INV-QWIIC-010 — Applications may request; core decides

A future application interface MAY expose energy state/surplus indication, requests for extended powered time, requests for radio delivery, and requests for higher-cost science work. The core SHALL accept, defer, clamp, or deny those requests according to mission policy. (Added 2026-08-12.)

### INV-QWIIC-011 — Application work cannot extend the absolute wake deadline

If an application has not completed inside its granted time budget, its work SHALL be deferred/terminated according to the application contract. The core mission sleep deadline remains authoritative (DDR-0001 INV-WAKE-010). (Added 2026-08-12; restates the hard-deadline rule of INV-QWIIC-004 from the orchestrator side.)

### Derived science relationship

First-class application-published science MAY contain raw measurements, calibrated observations, and deterministic derived/event products; derived products SHALL satisfy DDR-0023 provenance and versioning rules (BR-QWIIC-016). Debug or opaque best-effort objects remain lower priority unless deliberately promoted to a defined first-class scientific product. (Added 2026-08-12.)


---

## 3. Behavioral Requirements

Confidence legend:

- **CONFIRMED** — explicitly stated or affirmed during the interview.
- **INFERRED** — strongly implied but wording may still deserve review.
- **OPEN** — product behavior has not yet been fully decided.

| ID | Requirement | Confidence |
|---|---|---|
| BR-QWIIC-001 | Each powered Qwiic session SHALL have exactly one I2C controller, resolved by the claim window. | CONFIRMED |
| BR-QWIIC-002 | A claim arriving after the window closes SHALL be rejected. | CONFIRMED |
| BR-QWIIC-003 | An accepted claim grants bus ownership until rail removal; mid-session handback SHALL NOT occur. | CONFIRMED |
| BR-QWIIC-004 | The claiming application SHALL be told its session-length budget at claim time. | CONFIRMED |
| BR-QWIIC-005 | Early completion reporting SHALL end the session gracefully; deadline rail removal SHALL be survivable by the application at any time. | CONFIRMED |
| BR-QWIIC-006 | The service set SHALL be limited to: publish science record, publish best-effort object/fragment, read mission time/nav context, report completion. | CONFIRMED |
| BR-QWIIC-007 | Published science records SHALL be validated, schema-versioned, durably archived with record IDs, and treated as full equals of core records in retention and transmission. | CONFIRMED |
| BR-QWIIC-008 | Best-effort objects SHALL occupy a bounded spool and SHALL NOT affect retention or delivery of first-class data. | CONFIRMED |
| BR-QWIIC-009 | Best-effort data SHALL NOT delay or displace heartbeat or first-class archive traffic. | CONFIRMED |
| BR-QWIIC-010 | Passive peripherals SHALL be bound from a commissioned static profile; firmware SHALL NOT perform runtime discovery. | CONFIRMED |
| BR-QWIIC-011 | A non-responsive profiled peripheral SHALL be marked stale/absent and retried on later eligible wakes without blocking the cycle. | CONFIRMED |
| BR-QWIIC-012 | Application Controller sessions SHALL be admitted only when DDR-0016 energy policy permits the rail load. | CONFIRMED |
| BR-QWIIC-013 | Passive profiled sensors SHALL run per ordinary science policy without application-session energy gating. | CONFIRMED |
| BR-QWIIC-014 | An Application Controller SHALL be able to publish both data classes within one session. | INFERRED |
| BR-QWIIC-015 | The session budget value, claim-window duration, and spool bounds SHALL be configuration bindings. | CONFIRMED |
| BR-QWIIC-016 | Derived science can be first class: a valid schema-versioned derived/event product from an Application Controller MAY be accepted as first-class science when its provenance satisfies DDR-0023. | CONFIRMED — 2026-08-12 interview |

---

## 4. Session Lifecycle Summary

**Smart-controller session:**

1. Wake → energy policy admits an application session (INV-QWIIC-007).
2. Rail powered; Stratosonde releases bus; service target enabled; claim window opens (INV-QWIIC-002).
3. Claim accepted → controller owns the bus; receives its session budget (INV-QWIIC-004).
4. During the session: services per INV-QWIIC-003 only.
5. Session ends on completion report or deadline; rail removed; Stratosonde resumes ordinary cycle work.

**Passive-peripheral session:**

1. Rail powered; claim window passes with no claim.
2. Stratosonde owns the bus and reads the commissioned static profile.
3. Profiled peripherals are sampled per their profile; failures are ordinary stale/absent results (INV-QWIIC-006).
4. Rail removed per the ordinary wake-cycle schedule.

---

## 5. Deliberate Revision of the Legacy Qwiic Discovery Design

The retired legacy bus-ownership record specified descriptor-EEPROM self-description as the preferred passive-peripheral identification mechanism, with static profiles as the legacy/prototype fallback. The interview **reversed** this:

- Runtime discovery (descriptor reading, address scanning) is rejected entirely.
- The commissioned static profile is the **only** mechanism.
- Rationale: simplicity and determinism; the attached peripheral set is known at commissioning time, and manufacturing each passive board with a descriptor EEPROM adds component and process cost for no flight benefit.

That record's bus-ownership and claim-window rules survive unchanged in INV-QWIIC-002.


---

## 6. Relationship to Other Records

- **DDR-0004 (archive):** published science records enter the archive with the same identity, immutability, and retention rules as core records.
- **DDR-0005 (delivery):** first-class application records schedule with core data during archive opportunities; best-effort objects rank below all first-class traffic.
- **DDR-0009 (fail-soft):** peripheral and application failures are ordinary subsystem failures; optional payloads can never prevent core sensing, logging, heartbeat, or return to sleep.
- **DDR-0016 (energy):** application sessions are an expensive, droop-admitted load; passive sensors are not energy-gated.
- **Retired legacy orchestrator and data-class records:** absorbed without revision (orchestrator role; two data classes).
- **Retired legacy bus-ownership record:** ownership/claim-window rules absorbed; runtime discovery rejected (see §5).
- **DDR-0014 (configuration):** session budgets, claim-window timing, spool bounds, and static profiles are mission/device configuration.

---

## 7. Open Decisions

### OD-QWIIC-001 — Service protocol encoding

The claim handshake, service-request framing, and completion-report encoding are implementation bindings for a protocol specification (the legacy design referenced `QwiicTransportProtocol.md` and `ApplicationServicesProtocol.md`; those documents remain the binding location).

### OD-QWIIC-002 — Static profile contents

The exact fields of a commissioned passive-peripheral profile (address, driver/schema identity, sample policy, calibration references) were not enumerated in the interview.

### OD-QWIIC-003 — Best-effort spool location

Whether the best-effort spool lives in serial flash, internal flash, or RAM-only was not decided; it interacts with DDR-0011's storage partitioning.

### OD-QWIIC-004 — Multiple simultaneous peripherals

Whether the static profile may define several passive peripherals on one bus, and any address-collision policy, was not explored.

### OD-QWIIC-005 — Application resource request API

Define the future request interface of INV-QWIIC-010: request types, grant duration, energy-surplus indication, cancellation/deadline behavior, radio semantics, and application persistence permissions. (Added 2026-08-12.)

---

## 8. Proof Plan

### P-QWIIC-001 — Claim window grants ownership

Attach a controller that claims within the window.

Prove it becomes sole I2C controller until rail removal and receives a session budget.

### P-QWIIC-002 — No claim, Stratosonde owns

Power the rail with only passive peripherals attached.

Prove the window expires, Stratosonde takes the bus, and static-profile sampling proceeds.

### P-QWIIC-003 — Late claim rejected

Issue a claim after window expiry.

Prove rejection and uninterrupted Stratosonde ownership.

### P-QWIIC-004 — Hard deadline enforced

Run an application that never reports completion.

Prove the rail is removed at the deadline and the mission cycle continues normally.

### P-QWIIC-005 — Early completion

Report completion before the deadline.

Prove graceful session end and rail removal.

### P-QWIIC-006 — First-class publish equality

Publish a valid science record from an Application Controller.

Prove it receives an archive record ID, durable storage, and transmission scheduling identical to a core record.


### P-QWIIC-007 — First-class validation rejection

Publish a malformed or schema-invalid science record.

Prove rejection without archive write and without affecting the rest of the session.

### P-QWIIC-008 — Best-effort boundedness

Fill the best-effort spool; continue publishing.

Prove rejection/eviction per policy and zero impact on heartbeat and first-class archive traffic.

### P-QWIIC-009 — Profiled peripheral failure

Commission a profiled peripheral that does not respond.

Prove stale/absent marking, no cycle blocking, and retry on subsequent eligible wakes.

### P-QWIIC-010 — No runtime discovery

Attach an uncommissioned passive peripheral.

Prove it is not sampled and cannot affect the mission (no descriptor read, no scan-driven binding).

### P-QWIIC-011 — Application session energy gating

Configure an Application Controller and place the sonde in a LOW energy tier (or fail the rail's droop admission per DDR-0016).

Prove the application session does not occur that wake while passive profiled sensors still run.

### P-QWIIC-012 — Orchestrator invariants under hostile application

Have a claimed application attempt out-of-scope actions (direct flash access, radio requests, ignoring the deadline).

Prove none take effect and the mission cycle is unaffected.

---

## 9. Regeneration Test

A clean-room implementer receiving this DDR should understand that:

- Stratosonde is always the mission orchestrator; bus ownership and mission ownership are distinct;
- each powered Qwiic session has exactly one I2C controller, resolved by a claim window at rail power-up;
- a claiming Application Controller owns the bus for that session only, under a budget with a hard deadline, and must tolerate instant power loss;
- the service boundary is narrow: publish science record, publish best-effort object, read time/nav, report completion — nothing else;
- published science records are full archive equals of core records; best-effort objects are bounded-spool, loss-tolerant, and never starve first-class traffic;
- passive peripherals are statically commissioned only — no runtime discovery;
- peripheral failures are ordinary fail-soft events;
- smart-controller sessions are energy-gated; passive sensors are not;
- optional payloads can never bypass data honesty, persistence, energy, or RF policy.

The implementer should not need today's I2C driver, protocol framing, profile structures, or source-code layout to recreate the intended behavior.

---

## 10. Next Intent Interview

The queued high-value topics are:

1. **Commissioning and session bootstrap** — join-on-ground policy, credential tiering, and the commissioning/flight door anchor (absorbs the retired legacy session-integrity record).
2. **Radio and payload policy bindings** — safe-to-fly defaults, ADR-off, worst-case payload sizing (absorbs the retired legacy radio/payload records).
3. **Protocol binding documents** — refresh `QwiicTransportProtocol.md` / `ApplicationServicesProtocol.md` against this DDR's rejected-discovery revision (not an interview; a documentation task).

These are independent topics and may be addressed in any order.

