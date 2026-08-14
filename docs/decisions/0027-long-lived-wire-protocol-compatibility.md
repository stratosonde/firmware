# DDR-0027: Long-Lived Wire Protocol Compatibility

**Status:** Draft — added 2026-08-13 (intent interview, final pass); exact encoding remains open
**Intent Interview Date:** 2026-08-13
**Method:** Intent Interview V2 / Intent-Anchored Development (`../intent-interview-v2.1.md`)
**Scope:** Compatibility between long-lived deployed sondes and evolving backend/firmware protocol versions

> **Numbering note.** This record was drafted during the interview as
> "DDR-0022". `DDR-0022` was already assigned to *Mission Purpose, Value
> Hierarchy, and Autonomous Continuity* (2026-08-12), and `manifest.yaml`
> forbids renumbering an assigned ID. It was therefore merged as **DDR-0027**.
> See `2026-08-13-merge-ledger.md` §4.

---

## 1. Intent

A Stratosonde may remain airborne for months or a year while the backend and
newly manufactured firmware evolve.

The backend must not require an old airborne sonde to receive a firmware update
merely because a newer packet format exists.

Core intent:

> A packet emitted by a deployed sonde remains interpretable for the lifetime of
> that deployment, even after newer protocol versions are introduced.

This record states the *compatibility obligation*. It deliberately does not
choose the encoding that satisfies it.

---

## 2. Product-Level Invariants

### INV-PROTO-001 — Wire records are immutable facts

A science record transmitted under protocol version V remains exactly what that
sonde emitted.

The backend MAY reinterpret or normalize it, but the original device record is
not rewritten. (Consistent with DDR-0004 `INV-ARCH-003`.)

### INV-PROTO-002 — Protocol version is determinable

For any supported/deployed packet, the backend SHALL be able to deterministically
identify which decoding rules apply.

This does not mandate a dedicated version byte. FPort allocation, message-type
discrimination, payload shape, or an explicit field are all acceptable
mechanisms.

### INV-PROTO-003 — New backend releases do not strand deployed sondes

When a new protocol version is introduced, the backend SHALL retain a decode path
for older versions that may still be airborne.

### INV-PROTO-004 — No firmware OTA dependency

Protocol evolution SHALL NOT depend on over-the-air firmware upgrade of existing
sondes.

Firmware OTA is not a product capability; see DDR-0025 (firmware servicing and
non-OTA update policy) and DDR-0014 §4 (first-flight scope).

### INV-PROTO-005 — Backend normalization may evolve

The backend MAY map historical packet versions into a common current internal
representation, provided the original packet identity and scientific meaning
remain recoverable.

---

## 3. Behavioral Requirements

Confidence legend:

- **CONFIRMED** — explicitly stated or affirmed during the interview.
- **INFERRED** — strongly implied but wording may still deserve review.
- **OPEN** — product behavior has not yet been fully decided.

| ID | Requirement | Confidence |
|---|---|---|
| BR-PROTO-001 | Every deployed packet format SHALL be unambiguously decodable by a matching backend codec. | **CONFIRMED** |
| BR-PROTO-002 | Adding a new packet format SHALL NOT remove decode capability for still-supported deployed versions. | **CONFIRMED** |
| BR-PROTO-003 | Backend gap repair SHALL continue to use immutable science-record IDs across retransmission (DDR-0004, DDR-0005). | **CONFIRMED** |
| BR-PROTO-004 | The exact version-identification encoding is an implementation binding. | **CONFIRMED** |
| BR-PROTO-005 | Compatibility SHALL cover both live packets and historical archive retransmissions emitted by old firmware. | **CONFIRMED** |

---

## 4. Rationale

Long mission duration reverses a common software assumption.

**The backend is easier to update than the balloon.** The compatibility cost
therefore belongs primarily on the backend, not on the deployed device.

Keeping historical codecs is simpler and safer than creating an in-flight upgrade
mechanism over a low-bandwidth, intermittently available radio link. An OTA path
would also introduce exactly the class of failure this product cannot tolerate:
a partially updated, unreachable, unrecoverable flight unit.

This also means a deployed sonde is permitted to be "old" indefinitely. Age of
firmware is not a defect and is not a reason to stop trusting its data.

---

## 5. Relationship to Other Records

- **DDR-0004 / DDR-0005 (archive and delivery):** record identity is the
  gap-repair primitive; retransmitted archive records carry their original ID and
  their original-format semantics.
- **DDR-0019 (radio and payload bindings):** owns the concrete packet/FPort
  bindings this record constrains.
- **DDR-0025 (firmware servicing):** owns the non-OTA policy that
  `INV-PROTO-004` depends on.
- **DDR-0023 (scientific data truth):** backend reprocessing of historical raw
  observables assumes the decode path still exists.
- **DDR-0002 (no mission-end state):** the reason "still airborne" has no
  natural expiry, which is what makes `OD-PROTO-002` hard.

---

## 6. Open Decisions

### OD-PROTO-001 — Version identification mechanism

Choose an explicit byte/bitfield, FPort mapping, payload-shape/message-type
discrimination, or another unambiguous mechanism. Must be decided jointly with the
DDR-0019 payload bindings.

### OD-PROTO-002 — Codec retirement horizon

Define when a historical backend codec may be removed.

Minimum obvious rule: not while corresponding sondes can plausibly still be
alive. Given DDR-0002's no-mission-end model, "plausibly alive" needs an explicit
operational definition.

### OD-PROTO-003 — Downlink command versioning

Any future gap-repair/config downlink protocol should itself be versionable.
Exact encoding remains future work (see DDR-0014 `OD-CONFIG-001/002`).

---

## 7. Proof Plan

### P-PROTO-001 — Multi-version decode

Feed representative packets from every deployed version into the latest backend
decoder. Prove each maps to the correct scientific record.

### P-PROTO-002 — Historical retransmission after backend upgrade

Retransmit an old archived record after a backend upgrade and prove the same
logical record ID and science values are recovered.

### P-PROTO-003 — New version does not regress old vectors

Add a new protocol version and prove old-version test vectors remain accepted by
the regression suite.

### P-PROTO-004 — No OTA dependency

Inspection/analysis: prove no protocol-evolution path requires a firmware update
of a deployed unit.

---

## 8. Regeneration Test

A clean-room implementer receiving this DDR should understand that:

- deployed packets must stay decodable for as long as the sonde may live;
- the device is not obliged to modernize itself;
- the backend keeps historical codecs;
- record IDs remain the stable cross-version identity;
- the version-identification mechanism is theirs to choose, but it must be
  deterministic;
- OTA firmware update is not available and must not be assumed.

The implementer should **not** need to know the current payload byte layout to
recreate this intent.

---

## 9. Next Intent Interview

Natural follow-ups:

- the concrete version-identification encoding (with DDR-0019);
- the codec retirement horizon, and what "plausibly still airborne" means
  operationally;
- the downlink command protocol and its own versioning.
