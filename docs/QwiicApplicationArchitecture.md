# Stratosonde Qwiic Application Architecture — Document Set

**Status:** Draft package  
**Date:** 2026-08-03

This package introduces a bounded Qwiic application architecture while preserving Stratosonde as the mission orchestrator.

## Architecture decisions

- `decisions/0009-stratosonde-remains-mission-orchestrator.md`
- `decisions/0010-qwiic-bus-ownership-and-peripheral-discovery.md`
- `decisions/0011-heartbeat-and-scientific-archive.md`
- `decisions/0012-mission-data-classes.md`

## Protocol specifications

- `QwiicTransportProtocol.md` — power session, I2C ownership, framing, discovery, descriptor, deadlines.
- `ApplicationServicesProtocol.md` — developer-facing services for first-class records and best-effort objects.
- `LoRaWANApplicationProtocol.md` — FPorts, archive opportunity behavior, delivery semantics, and proposed extension payloads.

## Developer guide

- `ApplicationPayloadDeveloperGuide.md` — ESP32 camera and passive AD7745-style sensor examples.

## Proposed implementation order

1. Accept ADRs and settle names, addresses, and FPorts.
2. Resolve existing LoRaWAN byte-order documentation versus actual emitted bytes.
3. Repair durable record acknowledgement and stable archive identity.
4. Implement Qwiic rail/session state machine and target claim.
5. Implement transport parser with fuzz and deadline tests.
6. Implement first-class application record persistence.
7. Implement best-effort spool with strict quota and eviction.
8. Add FPort 12 and 13 serializers and backend decoders.
9. Implement the confirmed-heartbeat to SF7 archive-opportunity state machine.
10. Build the ESP32 camera reference application and passive sensor reference board.

## Open decisions before acceptance

- Confirm service I2C address `0x42`.
- Confirm descriptor EEPROM address `0x50`.
- Set electrical current and rail-capacitance limits.
- Set default claim and hard-session durations.
- Assign producer IDs, sensor profile IDs, and schema IDs.
- Confirm FPorts 12 and 13 with the network/backend deployment.
- Choose heartbeat v2 pressure and time encoding.
- Choose the durable first-class variable-record flash format.
- Define exact confirmed-uplink and LinkCheck delivery evidence.
- Decide best-effort spool quota, eviction, and partial-object policy.
