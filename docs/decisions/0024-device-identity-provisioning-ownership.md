# DDR-0024: Device Identity, Provisioning, Ownership, and Backend Registry

**Status:** Draft — product intent substantially elicited 2026-08-12; backend claim/security details remain open  
**Intent Interview Date:** 2026-08-12  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** Physical-device identity, DevEUI lifetime, PCB-readable identity, per-region LoRaWAN provisioning, backend registry, user ownership metadata, reuse after recovery, and separation between identity, credentials, missions, and users  
**Authority:** Product intent is normative. Credential storage/recovery mechanics remain governed by DDR-0010, DDR-0011, and DDR-0018.

---

## 1. Intent

A Stratosonde is a persistent physical instrument identity.

Its identity belongs to the hardware itself, not to a particular flight, owner, firmware build, LoRaWAN region, or backend mission record.

The intended user experience is that LoRaWAN provisioning complexity is handled before the device is distributed. An end user should be able to identify or claim a device through the Stratosonde backend without manually managing LoRaWAN keys.

The current physical-identity concept is:

- each PCB has a unique DevEUI;
- each PCB also carries a unique random value;
- the identifying values are machine-readable from a QR code on the PCB;
- Stratosonde backend infrastructure maintains the registry that relates the hardware identity to provisioning, calibration, ownership, and flight metadata.

The exact security semantics of the random value remain intentionally open.

> **The hardware keeps one stable identity for its lifetime. Provisioning belongs to the instrument, ownership and flights are backend metadata, and ordinary reflashing or reuse must not silently create a new instrument.**

---

## 2. Product-Level Invariants

### INV-ID-001 — DevEUI belongs to the physical instrument
The LoRaWAN DevEUI SHALL identify the physical Stratosonde hardware and remain stable across multiple flights, recovery/relaunch, ownership changes, ordinary firmware updates, and configuration changes.

### INV-ID-002 — Flight identity is separate from hardware identity
A flight/mission record SHALL be a backend object associated with a physical Stratosonde. The same Stratosonde MAY have multiple historical flights.

### INV-ID-003 — Ownership is backend metadata
Changing ownership SHALL NOT by itself require changing DevEUI, hardware identity, firmware, or historical mission identity.

### INV-ID-004 — End users should not manage LoRaWAN keys
Normal user operation SHOULD NOT require the owner to create or enter LoRaWAN keys or manually manage regional session state.

### INV-ID-005 — Per-region credentials are provisioned before flight
Each supported LoRaWAN region SHALL be commissioned according to DDR-0018 before the instrument is considered ready for autonomous flight in that region.

### INV-ID-006 — Public identity and secret provisioning data are separate
The PCB-readable QR representation SHALL NOT intentionally expose LoRaWAN root/session secret keys.

### INV-ID-007 — Recovery does not create a new device
A physically recovered and reflown Stratosonde remains the same registered hardware identity unless an explicit factory reprovisioning process deliberately retires/replaces it.

### INV-ID-008 — Backend calibration may bind to physical sensor identity
Calibration metadata MAY be associated with the Stratosonde identity, an individual sensor serial number, or another explicitly versioned physical sensor identity.

### INV-ID-009 — Identity corruption fails conservatively
If valid communication identity/credentials cannot be established, firmware SHALL NOT fabricate replacements. The affected radio capability may become unavailable while science and logging continue.

### INV-ID-010 — Device ownership claim requires physical-possession evidence

The intended backend claim flow (added 2026-08-12, round 3):

1. user scans PCB QR;
2. QR resolves to a Stratosonde backend endpoint containing permanent device identity;
3. authenticated user confirms device association;
4. a unique printed random PIN/claim secret may be required to prove physical possession;
5. backend marks device as owned/claimed.

### INV-ID-011 — Already-claimed devices do not silently transfer

If a device is already claimed, the ordinary first-claim flow SHALL report that it is already claimed. Ownership transfer requires a separate explicit backend process. (Added 2026-08-12, round 3.)

---

## 3. Conceptual Identity Model

```text
Physical Stratosonde
    |
    +-- stable hardware identity
    |      +-- DevEUI
    |      +-- PCB unique random value
    |      +-- hardware revision
    |
    +-- provisioned communication state
    |      +-- per-region credentials/session material
    |      +-- counters
    |
    +-- backend metadata
           +-- owner/account relationship
           +-- calibration records
           +-- mission/flight records
           +-- photographs/notes
           +-- device history
```

---

## 4. Behavioral Requirements

| ID | Requirement | Confidence |
|---|---|---|
| BR-ID-001 | Each production Stratosonde SHALL have a unique DevEUI bound to the physical instrument. | CONFIRMED |
| BR-ID-002 | DevEUI SHALL persist across ordinary firmware reflashing and recovered-device reuse. | CONFIRMED |
| BR-ID-003 | The PCB SHOULD expose the DevEUI in a scannable QR representation. | CONFIRMED |
| BR-ID-004 | The PCB SHOULD also contain a unique random value in the QR/identity data. | CONFIRMED |
| BR-ID-005 | The purpose/security role of the unique random value remains open until explicitly decided. | CONFIRMED OPEN BOUNDARY |
| BR-ID-006 | Normal users SHOULD be able to associate/claim a Stratosonde through the backend without entering LoRaWAN keys. | CONFIRMED |
| BR-ID-007 | LoRaWAN credentials SHALL be provisioned before distribution/flight according to the commissioning policy. | CONFIRMED |
| BR-ID-008 | Credentials and DevEUI SHALL belong to the hardware rather than to an individual flight. | CONFIRMED |
| BR-ID-009 | Recovered hardware MAY be reused with the same identity and credentials. | CONFIRMED |
| BR-ID-010 | Ownership transfer SHALL be representable without changing device identity. | CONFIRMED |
| BR-ID-011 | Backend mission records SHALL be many-to-one with physical device identity. | CONFIRMED |
| BR-ID-012 | Secret LoRaWAN key material SHALL NOT be encoded in the public PCB QR code. | INFERRED |
| BR-ID-013 | Backend calibration data MAY be associated with the physical device or individual sensor identity. | CONFIRMED |
| BR-ID-014 | Loss of identity/credential validity SHALL disable only the affected communication capability, not unrelated science. | CONFIRMED |

---

## 5. Provisioning Flow

```text
manufacture board
    -> assign permanent hardware identity
    -> register device in Stratosonde backend
    -> create/provision supported LoRaWAN regional credentials
    -> commission and verify regional communication state
    -> persist verified identity/credentials
    -> mark device ready for distribution/flight
```

Owner-facing flow:

```text
receive Stratosonde
    -> scan PCB identity
    -> authenticate to stratosonde.org
    -> associate device with account
    -> create/view mission metadata
```

---

## 6. Backend Authority

The Stratosonde backend is expected to maintain the long-lived registry linking:

- DevEUI;
- physical-device identity;
- commissioning/provisioning state;
- supported regional identities;
- calibration metadata;
- owner/account relationships;
- flight/mission history.

The backend does not replace firmware local persistence. A sonde must remain autonomous while disconnected.

---

## 7. Security Boundary

Still undecided:

- device claim and transfer authorization details;
- credential revocation;
- lost/stolen-device handling;
- deliberate re-key/reprovision flow.

**Revision (2026-08-12, round 3):** the earlier conservative placeholder — "the QR code is identity material, not authorization material" — is **superseded**. The PCB random value is now intended as a **candidate physical-possession claim secret** (INV-ID-010).

Before implementation, define:

- whether the PIN is inside or outside the QR;
- entropy/length;
- rate limiting;
- one-time or reusable behavior;
- transfer/recovery semantics;
- the public-photo threat (an attacker photographing the PCB).

Treat the claim-PIN mechanism as **intent confirmed, security protocol still open** (see `open-intent-questions.md` item 5). INV-ID-006 remains normative regardless: the QR SHALL NOT expose LoRaWAN root/session secret keys.

---

## 8. Relationship to Existing DDRs

- **DDR-0010** — persistent mission/session state.
- **DDR-0011** — integrity, redundancy, atomic storage.
- **DDR-0014** — mission configuration/future downlink configuration.
- **DDR-0018** — bench commissioning and regional LoRaWAN bootstrap.
- **DDR-0023** — scientific raw/calibrated/derived data.
- **DDR-0025** — firmware service/reflash behavior.

---

## 9. Open Decisions

### OD-ID-001 — QR random-value semantics

**Narrowed 2026-08-12 (round 3):** the random value is intended as a candidate physical-possession claim PIN (INV-ID-010). What remains open is the security protocol: placement, entropy, one-time/reusable semantics, rate limiting, and the public-photo threat.

### OD-ID-002 — Ownership claim and transfer

Define first claim (flow in INV-ID-010), transfer, recovered/unclaimed handling, disputes, and administrative override. INV-ID-011 fixes one rule: already-claimed devices never silently transfer.

### OD-ID-003 — Credential revocation
Define compromised-key, retirement, and deliberate re-key procedures.

### OD-ID-004 — Manufacturing source of identity
Define generation, uniqueness checking, registry insertion, and PCB application.

### OD-ID-005 — Hardware revision identity
Deferred until a second real hardware revision exists.

---

## 10. Proof Plan

### P-ID-001 — Stable identity across reflash
Record DevEUI, reflash the application using the supported service process, and prove the same identity returns.

### P-ID-002 — Multi-flight identity
Create multiple missions for one recovered device and prove all remain associated with one physical identity.

### P-ID-003 — Public QR contains no radio secrets
Inspect PCB QR payload and prove no LoRaWAN secret key material is exposed.

### P-ID-004 — Regional commissioning
Provision, verify, persist, power-cycle, and restore each supported regional credential set.

### P-ID-005 — Credential failure containment
Corrupt one regional credential object and prove science/logging continue and no fake keys are generated.

### P-ID-006 — Backend calibration binding
Prove a raw measurement can be mapped to the correct calibration record by physical identity.

---

## 11. Regeneration Test

A clean-room implementer should understand:

- one physical Stratosonde keeps one stable identity;
- DevEUI belongs to hardware, not a mission or owner;
- users should not need to manage LoRaWAN keys;
- public PCB identity is separate from secret credentials;
- ownership and missions live in the backend;
- recovered hardware can be reused with the same identity;
- calibration metadata may bind to device/sensor identity;
- QR claim-token/security semantics remain intentionally open.
