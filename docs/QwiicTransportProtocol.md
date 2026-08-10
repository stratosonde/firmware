# Stratosonde Qwiic Transport Protocol

**Status:** Draft v0.2  
**Date:** 2026-08-03 (revised 2026-08-09)  
**Related decisions:** DDR-0017  
**Audience:** Stratosonde firmware developers, expansion-board firmware developers, and hardware designers

> **v0.2 revision (DDR-0017):** Runtime discovery is removed. Passive peripherals are bound exclusively from a commissioned static profile — the Stratosonde Expansion Descriptor (§11) is withdrawn and address `0x50` is no longer reserved. The claim window, session lifecycle, framing, deadlines, and error behavior are unchanged.

## 1. Purpose

This document defines the electrical-session, I2C ownership, discovery, framing, timing, and error behavior of the Stratosonde Qwiic expansion interface.

It does not define the meaning of application service calls. Those are specified in `ApplicationServicesProtocol.md`.

## 2. Design goals

- One I2C controller per powered session.
- Simple implementation from STM32 C, ESP32 C/C++, or MicroPython.
- Deterministic power reclamation by Stratosonde.
- Bounded memory, transfer size, and processing time.
- Duplicate-safe requests.
- Explicit versioning and integrity checks.
- Support for intelligent Application Controllers and passive sensor boards.

## 3. Electrical assumptions

The connector follows the Qwiic four-wire convention:

| Signal | Meaning |
|---|---|
| 3V3_SW | Stratosonde-switched 3.3 V application rail |
| GND | Common ground |
| SDA | I2C data |
| SCL | I2C clock |

Normative requirements:

- Expansion hardware must not back-power Stratosonde through SDA, SCL, or any other signal.
- SDA and SCL must be open-drain and tolerate removal of `3V3_SW`.
- The expansion must not add pull-ups that make the combined pull-up resistance unsafe.
- The v1 nominal bus speed is 100 kHz. A faster mode requires an explicit future capability negotiation.
- The Qwiic rail is not an always-on supply.
- The hard session deadline permits Stratosonde to remove power without further handshake.

Exact current limits, connector pin numbering, pull-up population, and rail capacitance belong in the hardware design document and schematic.

## 4. Powered-session lifecycle

### 4.1 Startup

1. Stratosonde releases SDA and SCL.
2. Stratosonde enables I2C target mode at address `0x42`.
3. Stratosonde enables `3V3_SW`.
4. Stratosonde waits through a configurable controller-claim window.
5. The default proposed claim window is 3000 ms to allow an ESP32-class board to boot an interpreted application.

### 4.2 Application Controller path

An intelligent controller probes address `0x42` and sends `CLAIM_SESSION`.

If accepted:

- The external device is the sole I2C controller.
- Stratosonde remains target at `0x42`.
- Ownership is valid only for this powered session.
- The response gives a session token, soft deadline, hard deadline, maximum frame payload, and current service budgets.
- The controller performs service requests and ends with `COMPLETE_SESSION`.
- Stratosonde may remove power after completion or at the hard deadline.

### 4.3 Passive peripheral path

If no valid claim arrives before the window closes:

1. Stratosonde disables I2C target mode.
2. Stratosonde becomes the sole I2C controller.
3. Stratosonde loads the commissioned static peripheral profile (§11/§12). There is no runtime discovery.
4. A recognized driver reads the peripheral.
5. Unknown or failed peripherals are isolated and core mission work continues.
6. Stratosonde removes Qwiic power before sleep.

### 4.4 Ownership rules

- There is no production multi-controller mode.
- A claim after the claim window is rejected or receives no response because Stratosonde has changed role.
- Ownership cannot be transferred within a session.
- Recovery from bus lockup may reset or terminate the Qwiic session.
- Qwiic failure must not prevent Stratosonde from returning to sleep.

## 5. Reserved I2C addresses

| Address | Use |
|---:|---|
| `0x42` | Stratosonde Application Services target while an external controller owns the bus |
| Other | Native peripheral addresses declared by the commissioned static profile |

These are 7-bit addresses. Address assignments are proposed and must be checked against final hardware before acceptance. (Address `0x50` was reserved for the withdrawn descriptor EEPROM in v0.1 and is now unreserved.)

## 6. Transport frame

All service messages use this frame:

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 1 | Magic 0 | `0x53` (`S`) |
| 1 | 1 | Magic 1 | `0x51` (`Q`) |
| 2 | 1 | Protocol version | `0x01` |
| 3 | 1 | Message type | Request command; response is command OR `0x80` |
| 4 | 1 | Flags | Defined below |
| 5 | 1 | Sequence | Request identifier, wraps modulo 256 |
| 6 | 2 | Payload length | Unsigned little-endian |
| 8 | N | Payload | Command-specific |
| 8+N | 2 | Frame CRC16 | CRC16/MODBUS over bytes 0 through `7+N` |

Total frame length is `10 + N`.

### 6.1 Flags

| Bit | Name | Meaning |
|---:|---|---|
| 0 | RETRY | Sender is retrying a request after a missing response |
| 1 | MORE | More related frames follow |
| 2 | FINAL | Final frame of a logical operation |
| 3-7 | Reserved | Must be zero in v1 |

### 6.2 CRC

CRC16/MODBUS parameters:

- Initial value: `0xFFFF`
- Reflected polynomial: `0xA001`
- No final XOR
- CRC field transmitted little-endian

A bad CRC receives `ERR_CRC` when a response is still possible. The receiver must not perform the requested side effect.

## 7. Request-response transaction

The Application Controller performs:

1. One complete I2C write containing one request frame.
2. A bounded processing delay or status poll.
3. One I2C read for the response frame.

The v1 recommended maximum payload is 224 bytes, giving a maximum frame of 234 bytes. The accepted maximum is returned by `CLAIM_SESSION`.

The transport permits only one outstanding request. Pipelining is not supported in v1.

The response payload begins with a one-byte status code. A successful response may append command-specific data.

## 8. Duplicate and retry behavior

Requests must be safe under a lost response.

Within one session, Stratosonde caches the most recent completed side-effecting request by:

- Session token
- Sequence
- Message type
- Request CRC

If the identical request is repeated with `RETRY`, Stratosonde returns the previous response without repeating the side effect.

If a sequence is reused with different request content, Stratosonde returns `ERR_SEQUENCE`.

Application code should retry a missing response with the identical frame and `RETRY` set.

## 9. Transport status codes

| Value | Name | Meaning |
|---:|---|---|
| `0x00` | OK | Request completed |
| `0x01` | ACCEPTED | Accepted for asynchronous or queued handling |
| `0x02` | PARTIAL | Some but not all requested data accepted |
| `0x10` | ERR_VERSION | Unsupported protocol or service version |
| `0x11` | ERR_FRAME | Malformed frame or length |
| `0x12` | ERR_CRC | Frame integrity failure |
| `0x13` | ERR_SEQUENCE | Invalid sequence reuse |
| `0x14` | ERR_NOT_CLAIMED | Service requires an active session |
| `0x15` | ERR_SESSION | Invalid or expired session token |
| `0x16` | ERR_BUSY | Retry may succeed |
| `0x17` | ERR_DEADLINE | Insufficient session time remains |
| `0x18` | ERR_POWER | Power budget does not permit operation |
| `0x19` | ERR_STORAGE | Storage unavailable |
| `0x1A` | ERR_TOO_LARGE | Declared object or frame exceeds limits |
| `0x1B` | ERR_UNSUPPORTED | Command, schema, or capability unsupported |
| `0x1C` | ERR_INVALID | Command parameters invalid |
| `0x1D` | ERR_POLICY | Mission policy rejected the request |
| `0x1E` | ERR_INTEGRITY | Object-level integrity failure |
| `0x1F` | ERR_INTERNAL | Internal failure; core mission must continue |

## 10. Controller claim payload

`CLAIM_SESSION` is command `0x02`.

Request payload:

| Size | Field |
|---:|---|
| 2 | Application protocol major/minor |
| 4 | Controller ID |
| 4 | Controller boot/session nonce |
| 4 | Requested session duration in ms |
| 4 | Capability flags |
| 2 | Requested maximum frame payload |
| 2 | Reserved, zero |

Successful response after status byte:

| Size | Field |
|---:|---|
| 4 | Session token |
| 4 | Granted soft deadline in ms from response |
| 4 | Hard deadline in ms from response |
| 2 | Maximum frame payload |
| 2 | First-class byte budget for this session |
| 4 | Best-effort spool byte budget |
| 4 | Service capability flags |

A grant may be smaller than requested.

## 11. Passive peripheral identification — static profile only

**Withdrawn in v0.2 (DDR-0017):** the Stratosonde Expansion Descriptor (a 64-byte EEPROM at address `0x50` carrying vendor/product/profile/schema/CRC) is no longer part of this protocol. Runtime discovery — descriptor reading or blind address scanning — is not performed.

Passive peripherals are identified exclusively by the commissioned static profile (§12). Rationale: the attached peripheral set is known at commissioning time; descriptor EEPROMs add component and manufacturing cost for no flight benefit.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Magic `SSED` |
| 4 | 1 | Descriptor version, `0x01` |
| 5 | 1 | Descriptor length, `64` |
| 6 | 2 | Vendor ID |
| 8 | 2 | Product ID |
## 12. Static commissioned profile

Every passive peripheral is bound from an explicit commissioned static profile. The static profile:

- Is explicit in configuration.
- Declares the peripheral's native address, driver/sensor profile and version, and schema identity.
- Disables any form of guessing or scanning.
- Is not portable to arbitrary boards.
- Must fail safely if the expected device does not respond: the peripheral's data is marked stale/absent, it is retried on later eligible wakes, and it never blocks the cycle (DDR-0009, DDR-0017).

## 13. Timeouts and power removal

Recommended v1 defaults:

| Item | Default |
|---|---:|
| Controller claim window | 3000 ms |
| Service response target | 20 ms |
| Busy retry delay | 10-100 ms, returned by service where applicable |
| Soft application session | Application/config dependent |
| Hard application session | Strict, application/config dependent |
| Bus recovery attempts | 1 |
| Deadline guard before new operation | 100 ms |

The hard deadline overrides all protocol state. External application firmware must save its own state before the deadline and must tolerate power removal during any operation.

## 14. Bus recovery

If SDA is held low:

1. Stratosonde or the active controller may release SDA and pulse SCL up to nine times.
2. Generate a STOP when possible.
3. Reinitialize the peripheral once.
4. If the bus remains stuck, terminate the Qwiic session.
5. Record a mission event and continue core operation.

The optional expansion must never create an unbounded wait.

## 15. Versioning

- Protocol version is in every frame.
- Unknown major versions are rejected.
- Minor-compatible additions use new commands, capability bits, or appended response fields.
- Existing field meaning is never silently changed.
- Static-profile sensor schemas and application data schemas are independently versioned.

## 16. Security and trust

The v1 interface assumes physical attachment before flight and does not provide cryptographic authentication.

Despite that trust assumption:

- All lengths are bounded.
- All identifiers and schema versions are validated.
- Application data cannot directly alter LoRaWAN credentials or mission state.
- Remote configuration is outside this protocol unless separately designed and authenticated.
- Malformed application traffic must degrade only the optional session.

## 17. Required tests

- Claim just before and just after timeout.
- External controller absent.
- Duplicate claim and duplicate publish request.
- Bad magic, length, CRC, version, and sequence reuse.
- Bus stuck low at startup and during transfer.
- Power removal during every service.
- Application exceeds soft and hard deadlines.
- Profiled peripheral absent, non-responsive, and responding mid-cycle-recovery.
- Uncommissioned peripheral attached (must be ignored — no discovery).
- Known passive sensor read failure.
- Fuzz every frame parser with bounded execution time.
