# DDR-0009: Stratosonde Remains the Mission Orchestrator

Status: Proposed  
Date: 2026-08-03

Context: The Qwiic expansion connector must support intelligent application controllers, such as an ESP32 camera, without allowing an optional payload to compromise an unattended flight. An application controller may need to become I2C controller for a powered session, but bus-mastership is not mission ownership.

## Decision

Stratosonde remains the mission orchestrator in every configuration.

It owns:

- Wake and sleep timing.
- Qwiic rail power and the hard session deadline.
- Mission state and safety policy.
- Core sensors and the scientific archive.
- Persistent storage allocation.
- LoRaWAN session state, data rate, ports, and transmit scheduling.
- The decision to accept, reject, defer, evict, or transmit application data.

An external Application Controller may claim I2C controller ownership for one powered Qwiic session. While claimed, Stratosonde operates as an I2C target and exposes a narrow application-services protocol.

The Application Controller may request services such as:

- Publish a first-class mission record.
- Publish an opaque best-effort object or fragment.
- Read mission time and current navigation context.
- Report session completion.

The Application Controller never receives direct ownership of Stratosonde flash, LoRaWAN state, radio configuration, mission state, or sleep scheduling.

The Qwiic rail may be removed at the negotiated or hard session deadline. Application firmware must treat the deadline as authoritative and must tolerate immediate power removal.

## Rules

- Bus ownership and mission ownership are separate concepts.
- Optional application failure must not prevent core sensing, logging, heartbeat transmission, or return to sleep.
- Every application request is bounded by time, size, power, and storage policy.
- Service acceptance means Stratosonde has accepted responsibility under the declared data class; it does not necessarily mean network delivery.
- Protocol details live in `../QwiicTransportProtocol.md` and `../ApplicationServicesProtocol.md`.

## Consequences

- Third-party applications can be expressive without being trusted with mission-critical internals.
- Stratosonde can reclaim power deterministically.
- Optional payloads cannot bypass data honesty, persistence, regional compliance, or radio policy.
- Application developers need only implement the service protocol, not LoRaWAN or flash management.
- More code is required at the service boundary, but the core mission remains testable and predictable.
