# DDR-0019: Radio and Payload Policy Bindings

**Status:** Draft — product intent elicited and resolved; implementation validation pending  
**Intent Interview Date:** 2026-08-09  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** Default build posture, debug opt-in discipline, ADR policy, data-rate pinning, worst-case payload sizing, the confirmed-probe link test, SF escalation, archive-burst link supervision, and failure-branch behavior  
**Authority:** Product intent is normative. Exact byte layouts, packet counts, margin thresholds, and spreading-factor choices per region are configuration/implementation bindings.

**Absorbs:** the retired legacy records 'Safe-to-Fly Out of the Box' in full, and 'Worst-Case Region Payload Sizing' with one revision: LinkCheckReq does **not** ride the compact packet — the opportunity probe is a confirmed packet and the ACK itself is the link evidence. Once this record exists, DDR-0002 and DDR-0005 contain no remaining unique design knowledge and may be retired.

---

## 1. Intent

Stratosonde transmits over LoRaWAN from an unreachable platform, across multiple regulatory regions, at data rates where every byte of airtime is expensive. The radio policy must therefore be conservative by construction:

- the default build must be safe to fly with no special flags;
- the smallest packet must be legal in every target region;
- the network must never be able to push the device onto an unplanned data rate;
- high-throughput archive work must be earned with fresh link evidence and continuously supervised.

The core design intent is:

> **Default to a minimum-airtime, worst-case-sized compact packet with ADR off and the data rate pinned. Escalate to fast archive bursts only after a confirmed probe proves the link, supervise the burst with periodic margin re-checks, and abandon it immediately when the evidence goes bad.**

---

## 2. Product-Level Invariants

### INV-RADIO-001 — Safe-to-fly default build

The default firmware build SHALL produce a minimum-airtime, single-compact-packet, no-debug-traffic image. Debug/diagnostic packets SHALL be compile-time opt-in, never default-on.

### INV-RADIO-002 — Debug paths never mutate flight radio state

Any debug/diagnostic code path SHALL have zero side effects on production transmission state (e.g., it must save and restore the data rate around any debug send).

### INV-RADIO-003 — ADR is off; the data rate is pinned and verified

Adaptive Data Rate SHALL be disabled: the network SHALL NOT drive data-rate changes on a device that cannot be reached reliably.

The intended data rate SHALL be pinned and verified committed before each transmission.

### INV-RADIO-004 — The compact packet is sized to the worst-case region

The compact packet SHALL fit the most restrictive data rate across all target regions — the interview binding is **11 bytes application payload at SF10 / US915 DR0** — so it is legal everywhere the mission may operate.

### INV-RADIO-005 — The opportunity probe is a confirmed packet

Link establishment SHALL use a confirmed compact packet; the returned ACK is itself the link-quality evidence.

LinkCheckReq SHALL NOT be attached to the compact packet. (This revises the retired legacy payload-sizing record's FOpts note.)

### INV-RADIO-006 — Escalation is earned and supervised

After a successful probe ACK, firmware MAY escalate to a faster data rate (interview binding: SF7) for archive work, attaching LinkCheckReq to the first large packet.

If the returned margin satisfies configured thresholds, the burst SHALL continue unconfirmed, newest-first (per DDR-0004/0018), up to a configured packet bound, with periodic margin re-checks.

### INV-RADIO-007 — Failure branches are immediate and conservative

- **No ACK to the confirmed probe:** end RF work for that wake; retry on the next cycle. There are no same-wake probe retries.
- **Bad margin on any re-check:** stop the burst immediately; the next cycle returns to the compact-packet posture.

### INV-RADIO-008 — Radio policy never overrides higher policy

Escalation and bursts remain subject to energy admission (DDR-0016), RF authorization (DDR-0006/0020/0028), and current-science priority (DDR-0005 INV-TX-001). Radio opportunity never outranks any of them.

---

## 3. Behavioral Requirements

Confidence legend:

- **CONFIRMED** — explicitly stated or affirmed during the interview.
- **INFERRED** — strongly implied but wording may still deserve review.
- **OPEN** — product behavior has not yet been fully decided.

| ID | Requirement | Confidence |
|---|---|---|
| BR-RADIO-001 | The default build SHALL emit only the compact packet; debug packets require explicit compile-time opt-in. | CONFIRMED |
| BR-RADIO-002 | Debug sends SHALL save/restore production radio state (at minimum the data rate). | CONFIRMED |
| BR-RADIO-003 | ADR SHALL be disabled in all builds. | CONFIRMED |
| BR-RADIO-004 | The data rate SHALL be set and verified before each send. | CONFIRMED |
| BR-RADIO-005 | The compact packet SHALL be ≤ 11 bytes application payload, fitting US915 DR0/SF10. | CONFIRMED |
| BR-RADIO-006 | Link establishment SHALL use a confirmed compact packet; LinkCheckReq SHALL NOT ride the compact packet. | CONFIRMED |
| BR-RADIO-007 | On probe ACK, firmware MAY escalate to SF7 and attach LinkCheckReq to the first large (archive) packet. | CONFIRMED |
| BR-RADIO-008 | A good margin SHALL allow an unconfirmed newest-first burst up to a configured packet bound. | CONFIRMED |
| BR-RADIO-009 | The burst SHALL re-check margin periodically (configured interval) via LinkCheckReq. | CONFIRMED |
| BR-RADIO-010 | No probe ACK SHALL end RF work for the wake with no same-wake retries. | CONFIRMED |
| BR-RADIO-011 | A bad margin re-check SHALL stop the burst immediately; the next cycle returns to compact posture. | CONFIRMED |
| BR-RADIO-012 | Bulk/archive packet size MAY use the faster rate's larger allowance (interview binding: ~200 bytes at SF7). | CONFIRMED |
| BR-RADIO-013 | Margin thresholds, burst packet bound, and re-check interval SHALL be configuration bindings. | CONFIRMED |
| BR-RADIO-014 | Which SF/DR the escalation targets per region, and the exact bulk-packet size per region, SHALL be configuration bindings verified against regional limits. | INFERRED |

---

## 4. Intended Float-Mode Radio Sequence (Binding to DDR-0005)

1. Science cycle completes; current record archived (DDR-0005 INV-TX-002).
2. Confirmed compact probe @ SF10 sent.
   - No ACK → RF work ends for the wake (INV-RADIO-007).
3. ACK → escalate to SF7; first ~200-byte archive packet carries LinkCheckReq.
   - Margin insufficient → burst ends; compact posture next cycle.
4. Margin good → unconfirmed newest-first burst, up to the configured packet bound, re-checking margin at the configured interval.
5. Burst yields instantly to any due science cycle (DDR-0005) and respects energy/RF gates (INV-RADIO-008).


---

## 5. Relationship to Other Records

- **DDR-0005 (delivery protocol):** this record supplies the concrete radio mechanics (confirmed probe, SF escalation, supervised unconfirmed burst) for that record's float-mode transmission sequence. Its invariants (current science first, no autonomous retransmit, backend gap repair) are unchanged.
- **DDR-0004 (archive):** newest-first burst order follows the archive's recovery preference.
- **DDR-0016 (energy):** bursts are expensive radio work subject to droop admission and tier gating.
- **DDR-0006/0020/0028 (RF authorization):** every transmission, probe included, requires current RF authorization.
- **DDR-0014 (configuration):** thresholds, bounds, and intervals are mission/device configuration.
- **Retired legacy safe-to-fly record:** absorbed in full (INV-RADIO-001/002/003).
- **Retired legacy payload-sizing record:** absorbed with revision (INV-RADIO-004 keeps worst-case sizing; INV-RADIO-005 replaces the FOpts/LinkCheck note with the confirmed-probe mechanism).

---

## 6. Open Decisions

### OD-RADIO-001 — Per-region escalation targets

The interview bound escalation as SF10 → SF7 with ~200-byte bulk packets (US915 framing). Equivalent fast-rate/bulk-size bindings for other regions (EU868, AS923, AU915) need verification against their regional limits and dwell-time rules.

### OD-RADIO-002 — DeviceTimeReq

The retired legacy payload-sizing record mentioned DeviceTimeReq riding FOpts. Its role, if any, in the current design (given GNSS is the time authority per DDR-0013) was not discussed and is presumed dropped.

### OD-RADIO-003 — Margin thresholds and burst bounds

Concrete margin thresholds, the burst packet bound, and the re-check interval await link-budget analysis and flight data; they are configuration bindings.

---

## 7. Proof Plan

### P-RADIO-001 — Default build posture

Build with no debug flags.

Prove only compact packets are emitted and no debug traffic exists.

### P-RADIO-002 — Debug isolation

Enable a debug packet, send it, then send a production packet.

Prove the production send uses the pinned production data rate (debug path restored state).

### P-RADIO-003 — ADR off / DR pinned

Inject network downlinks attempting ADR changes.

Prove the data rate never changes; every send uses the pinned, verified rate.

### P-RADIO-004 — Worst-case fit

Verify the compact packet against the most restrictive target-region data rate (US915 DR0).

Prove ≤ 11 bytes application payload.

### P-RADIO-005 — Confirmed probe, no ACK

Withhold the probe ACK.

Prove RF work ends for that wake with no same-wake retry, and science/logging are unaffected.

### P-RADIO-006 — Escalation and supervised burst

ACK the probe; return good margin on the first LinkCheckReq.

Prove SF7 escalation, newest-first unconfirmed burst, and periodic margin re-checks at the configured interval.

### P-RADIO-007 — Bad margin aborts burst

Return a failing margin mid-burst.

Prove the burst stops immediately and the next cycle returns to compact posture.

### P-RADIO-008 — Burst respects higher policy

During a valid burst, make a science cycle due (or drop the energy tier, or lose RF authorization).

Prove the burst yields/ends per DDR-0005/0029/0028.

---

## 8. Regeneration Test

A clean-room implementer receiving this DDR should understand that:

- the default build is safe to fly: compact packet only, debug opt-in, ADR off, DR pinned and verified;
- the compact packet is sized to the worst-case target region (11 bytes @ SF10/US915 DR0) so it is legal everywhere;
- link establishment is a confirmed compact packet — never LinkCheckReq on compact;
- escalation to a fast rate is earned by the probe ACK and supervised by LinkCheckReq margin checks;
- bursts are unconfirmed, newest-first, count-bounded, and abort immediately on bad margin;
- no-ACK means a quiet wake; there are no same-wake probe retries;
- radio opportunity never outranks science priority, energy admission, or RF authorization.

The implementer should not need today's LoRaWAN stack, packet structs, region tables, or build system to recreate the intended behavior.

---

## 9. Migration Note

With this record, the legacy DDR set (0001–0013) is fully absorbed or superseded except the legacy erase-before-write record, whose mechanics now live in `../FlashStorageNotes.md`.

