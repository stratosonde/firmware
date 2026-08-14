# DDR-0025: Firmware Servicing, Bootloader, and Non-OTA Update Policy

**Status:** Draft — first-flight intent substantially resolved 2026-08-12  
**Intent Interview Date:** 2026-08-12  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** Firmware replacement after manufacture, OTA exclusion, physical-access servicing, SWD/ST-Link programming, future local USB/UART convenience, preservation of identity/credentials across reflash, bootloader complexity, and firmware/DDRs version relationship

---

## 1. Intent

A launched Stratosonde is expected to operate without firmware replacement.

The product does not need an over-the-air firmware update mechanism. Remote communications may eventually change bounded configuration, but they SHALL NOT replace executable firmware.

If a device is physically available on the bench, firmware may be updated through a local service interface. The current accepted mechanism is SWD/ST-Link.

A future USB/UART-assisted updater may make local servicing easier, but it is not required for first flight and must not introduce OTA-style complexity merely for convenience.

> **Firmware is fixed in flight. Configuration may eventually change through validated downlinks. Executable firmware changes require physical access, and ordinary servicing must preserve the instrument's permanent identity and provisioned credentials.**

---

## 2. Product-Level Invariants

### INV-FW-001 — No OTA firmware updates
Flight firmware SHALL NOT implement a general mechanism to download, stage, swap, patch, or execute a replacement application image received over LoRaWAN or another remote radio path.

### INV-FW-002 — Remote control is configuration, not code replacement
Future remote commands/configuration under DDR-0014 MAY alter validated mission parameters. They SHALL NOT replace executable firmware.

### INV-FW-003 — Firmware update requires physical access
Changing application firmware SHALL require intentional local physical servicing. The current first-flight path is SWD/ST-Link.

### INV-FW-004 — Ordinary reflash preserves permanent identity
A supported application update SHALL NOT silently erase or regenerate DevEUI, device identity, provisioned LoRaWAN credentials, or other explicitly immutable provisioning data.

### INV-FW-005 — Provisioning storage is logically separate from the application image
The memory/storage layout and service procedure SHALL distinguish executable application firmware from permanent provisioning identity and mutable mission/session state.

### INV-FW-006 — Factory reprovisioning is a different operation
Any deliberate erase/replacement of identity or credentials SHALL be a separately named, intentional provisioning procedure and SHALL NOT occur as a side effect of normal firmware programming.

### INV-FW-007 — A custom field bootloader is not a first-flight requirement
The product SHALL NOT add a custom bootloader merely because firmware updates are theoretically possible.

### INV-FW-008 — Firmware and DDRs are versioned together
The firmware version/source commit SHALL identify the repository revision containing the governing DDR corpus. A separate DDR version field in telemetry is not required solely for traceability.

### INV-FW-009 — Update convenience must not weaken flight isolation
A future USB/UART local updater MAY improve bench servicing but SHALL remain a physical/local feature and SHALL NOT create a remote executable-update path.

---

## 3. First-Flight Service Model

```text
normal flight:
    firmware fixed
    no executable update path
    future bounded configuration downlinks only

recovered / bench device:
    physical access
    connect ST-Link/SWD
    program application
    preserve identity + credentials
    validate device
```

The mandatory preservation requirement established by interview is:

- permanent identity;
- LoRaWAN keys/credentials.

Whether other mutable mission state survives an intentional service update remains an explicit open decision.

---

## 4. Behavioral Requirements

| ID | Requirement | Confidence |
|---|---|---|
| BR-FW-001 | Flight firmware SHALL provide no OTA executable-firmware update feature. | CONFIRMED |
| BR-FW-002 | Future LoRaWAN downlinks MAY change validated configuration but SHALL NOT install firmware. | CONFIRMED |
| BR-FW-003 | Current supported firmware servicing SHALL use physical SWD/ST-Link access. | CONFIRMED |
| BR-FW-004 | A normal application reflash SHALL preserve DevEUI. | CONFIRMED |
| BR-FW-005 | A normal application reflash SHALL preserve provisioned LoRaWAN credentials. | CONFIRMED |
| BR-FW-006 | Reflash tooling SHOULD avoid mass erase where mass erase destroys protected provisioning data. | INFERRED |
| BR-FW-007 | A future USB/UART programming path MAY be added if it remains local/physical. | CONFIRMED |
| BR-FW-008 | A custom field bootloader is not required for first flight. | CONFIRMED |
| BR-FW-009 | Firmware version/source commit SHALL provide traceability to the DDR set in the same repository revision. | CONFIRMED |
| BR-FW-010 | Factory reset/reprovision SHALL be distinct from ordinary firmware update. | INFERRED |
| BR-FW-011 | Update failure SHALL NOT be considered a flight-recovery mechanism because firmware update is unavailable in flight. | CONFIRMED |

---

## 5. Storage Boundary

### Application image
May be replaced during local firmware servicing.

### Permanent provisioning identity
Examples: DevEUI, per-region immutable credential copies, other factory identity. Ordinary application servicing must preserve this class.

### Mutable mission/session state
Examples: frame counters, archive metadata, flight latch, mission configuration, last-known position/time, delivery watermarks.

Whether a specific service operation preserves or intentionally resets each mutable item must be explicit. DDR-0010/0011 remain authoritative for runtime reset/power-loss persistence.

---

## 6. Why OTA Is Rejected

OTA executable update would add:

- image staging;
- image authentication/integrity;
- rollback;
- interrupted-update recovery;
- dual-image or bootloader complexity;
- extra NVM;
- bandwidth demand;
- attack surface;
- new mission-ending failure paths.

The current mission does not justify that complexity. Configuration downlink provides useful remote adaptation without remote executable replacement.

---

## 7. Relationship to Existing DDRs

- **DDR-0010** — runtime mission state persistence.
- **DDR-0011** — storage integrity/atomicity.
- **DDR-0012** — normal boot/reset recovery.
- **DDR-0014** — configuration and future downlink commands.
- **DDR-0018** — commissioning and credential bootstrap.
- **DDR-0024** — stable device identity/provisioning.
- **DDR-0026** — verification of reflash/persistence behavior.

---

## 8. Open Decisions

### OD-FW-001 — Mutable-state preservation across service reflash
Beyond DevEUI and credentials, decide whether update-in-place preserves flight latch, frame counters/session state, mission configuration, archive, watermarks, and reset history.

### OD-FW-002 — Future local updater transport
If easier servicing becomes necessary, choose USB, UART, ROM bootloader, custom local bootloader, or another physical path.

### OD-FW-003 — Local image authenticity/integrity
Secure boot or cryptographic service-image authenticity was not established by the interview and must not be silently assumed.

### OD-FW-004 — Debug/security lock policy
Decide later whether production SWD remains open, uses readout protection, or another service/security balance.

---

## 9. Proof Plan

### P-FW-001 — No remote executable update path
Review the flight command/downlink surface and prove no command can install or execute replacement firmware.

### P-FW-002 — Reflash preserves identity
Record DevEUI, perform supported application reflash, prove identity unchanged.

### P-FW-003 — Reflash preserves credentials
Provision regional credential sets, reflash the application, prove credentials remain valid/restorable.

### P-FW-004 — Factory reset is explicit
Exercise ordinary update tooling and prove it cannot accidentally invoke factory reprovisioning.

### P-FW-005 — Source traceability
For a release artifact record firmware commit/version, binary hash, configuration, and DDR source revision; prove the governing corpus can be reconstructed.

---

## 10. Regeneration Test

A clean-room implementer should understand:

- firmware is fixed once in flight;
- future downlinks may change configuration, never code;
- firmware replacement requires physical access;
- SWD/ST-Link is sufficient for first flight;
- custom field bootloader complexity is intentionally deferred;
- ordinary firmware updates preserve permanent identity and LoRaWAN credentials;
- factory reprovisioning is a different explicit operation;
- firmware source revision naturally ties executable behavior to the DDR corpus.
