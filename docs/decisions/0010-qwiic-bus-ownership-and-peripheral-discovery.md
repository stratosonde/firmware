# DDR-0010: Qwiic Bus Ownership and Peripheral Discovery

Status: Proposed  
Date: 2026-08-03

Context: The same powered Qwiic connector must support two materially different cases: an intelligent Application Controller that needs to control the I2C bus, and a simple sensor peripheral that expects Stratosonde to control the bus. Allowing both sides to become controller without an explicit handoff risks bus contention and undefined behavior.

## Decision

Each Qwiic power session has exactly one I2C controller.

At wake:

1. Stratosonde configures SDA and SCL released, enables its application-service I2C target, and powers the Qwiic rail.
2. A configurable claim window opens.
3. An Application Controller may claim the session by issuing `CLAIM_SESSION` to the Stratosonde service address.
4. If the claim is accepted, the Application Controller is the sole I2C controller until the rail is removed.
5. If no valid claim arrives before the window closes, Stratosonde disables target mode, becomes the sole I2C controller, and performs peripheral discovery.
6. Controller ownership never changes again during that powered session.

Simple Stratosonde-compatible sensor boards identify themselves through a Stratosonde Expansion Descriptor. The preferred implementation is a small descriptor EEPROM at the reserved descriptor address. The descriptor contains vendor, product, hardware revision, sensor profile, native I2C address, schema identity, capabilities, and CRC.

A manually commissioned static profile is permitted for legacy or prototype boards that lack a descriptor. A blind full-address scan is not the primary production discovery mechanism.

## Rules

- No I2C multi-controller operation is used after ownership is decided.
- A late Application Controller claim is rejected.
- A malformed descriptor is treated as an unknown peripheral, not guessed from partial data.
- Unknown peripherals must not block core mission operation.
- Sensor drivers are selected by explicit profile and version, not by address alone.
- Timing, addresses, frame format, and descriptor layout live in `../QwiicTransportProtocol.md`.

## Consequences

- Bus contention becomes structurally unreachable.
- ESP32-class controllers can use a simple controller-to-target service protocol.
- Passive sensor boards remain low power and inexpensive.
- New sensor boards can be added without ambiguous address-based identification.
- Descriptor EEPROM adds component and manufacturing work to compliant passive sensor boards.
- Prototype boards may still use a commissioned static profile, but that path is explicit and less portable.
