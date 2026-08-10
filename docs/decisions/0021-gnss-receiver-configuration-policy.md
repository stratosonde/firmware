# DDR-0021: GNSS Receiver Configuration Policy

**Status:** Draft — product intent elicited and resolved; implementation validation pending  
**Intent Interview Date:** 2026-08-09  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** When GNSS receiver configuration is allowed, persistence of receiver configuration, the disposition of the temperature cold-lockout, and acquisition timeout policy  
**Authority:** Product intent is normative. Specific receiver configuration values (dynamic model, message set, rates) and the timeout value are configuration bindings.

**Supersedes:** the GPS cold-lockout veto in the retired legacy data-honesty record's consequences (already partially revised by DDR-0016) and the GPS-configure gating consequence of the retired legacy mission-state record.

---

## 1. Intent

The GNSS receiver is configured once, correctly, on the bench — and then trusted. An unattended balloon must not contain code paths that rewrite receiver configuration in flight: a misapplied reconfiguration at altitude is an unrecoverable, invisible failure, and the receiver's own non-volatile storage already survives resets.

Separately, the historical temperature cold-lockout (skip GNSS below a temperature threshold because the receiver/supercap is inoperative) is subsumed by the DDR-0016 energy model: whether to attempt GNSS is a droop-admission question about the supply rail, not a separate temperature gate.

The core design intent is:

> **Configure the receiver during commissioning only, persist that configuration in the receiver's own flash, never reconfigure in flight, and let energy admission — not a temperature lockout — decide whether each acquisition attempt happens, bounded by a fixed configured timeout.**

---

## 2. Product-Level Invariants

### INV-GNSSCFG-001 — Receiver configuration happens in commissioning only

Firmware SHALL configure the GNSS receiver (dynamic model, message set, rates, power-related settings) only during commissioning, and SHALL persist that configuration to the receiver's own non-volatile storage.

In flight, receiver reconfiguration SHALL be unreachable. The persisted configuration is trusted on every boot.

### INV-GNSSCFG-002 — No temperature cold-lockout

There SHALL be no separate temperature-based GNSS lockout.

Whether a GNSS acquisition is attempted is decided by DDR-0016 energy admission (tier + temperature-normalized droop prediction against the receiver's minimum supply voltage). If the rail is predicted to hold, GNSS is attempted regardless of temperature.

### INV-GNSSCFG-003 — Fixed configured acquisition timeout

Each GNSS acquisition attempt SHALL be bounded by a fixed, configured timeout.

The timeout SHALL NOT adapt to outage history, temperature, or energy tier — energy policy already decides *whether* to attempt; the timeout only bounds the attempt.

### INV-GNSSCFG-004 — Stale temperature handling follows DDR-0016

When temperature is stale/unknown, the energy model uses last-known-good temperature (DDR-0016 INV-PWR-007). There is no cold-assumption anywhere in the GNSS admission path.

---

## 3. Behavioral Requirements

Confidence legend:

- **CONFIRMED** — explicitly stated or affirmed during the interview.
- **INFERRED** — strongly implied but wording may still deserve review.
- **OPEN** — product behavior has not yet been fully decided.

| ID | Requirement | Confidence |
|---|---|---|
| BR-GNSSCFG-001 | Receiver configuration SHALL be written only during commissioning and persisted to the receiver's flash. | CONFIRMED |
| BR-GNSSCFG-002 | No in-flight code path SHALL reconfigure the receiver. | CONFIRMED |
| BR-GNSSCFG-003 | GNSS attempt decisions SHALL be made solely by DDR-0016 energy admission (tier + droop prediction); no temperature lockout SHALL exist. | CONFIRMED |
| BR-GNSSCFG-004 | Acquisition SHALL be bounded by a fixed configured timeout; no adaptive timeout logic SHALL exist. | CONFIRMED |
| BR-GNSSCFG-005 | Stale temperature SHALL resolve to last-known-good in the admission path (DDR-0016). | CONFIRMED |
| BR-GNSSCFG-006 | Specific receiver configuration values (airborne dynamic model, message set, update rates) SHALL be configuration bindings set at commissioning. | INFERRED |
| BR-GNSSCFG-007 | The acquisition timeout value SHALL be a configuration binding sized for worst-case legitimate cold start. | INFERRED |

---

## 4. Relationship to Other Records

- **DDR-0016 (energy):** provides the sole GNSS admission mechanism (INV-GNSSCFG-002/003/004). The droop test's minimum-valid voltage for GNSS was bound in that interview (~3.3 V, ~75 mA, ~30 s).
- **DDR-0003 (GNSS provenance):** unchanged — fix acceptance still uses configured receiver-quality criteria; this record governs configuration and admission only.
- **DDR-0018 (commissioning):** receiver configuration is one of the commissioning-time activities; the one-way door makes it unreachable in flight alongside join.
- **DDR-0020 (supervision):** the fixed acquisition timeout is one of the per-operation timeouts the supervision layers rely on.
- **Retired legacy data-honesty record:** the GPS cold-lockout consequence (and its stale-temp=COLD rule, already revised by DDR-0016) is fully superseded — the lockout no longer exists.
- **Retired legacy mission-state record:** the "GPS configure + flash-save only in COMMISSIONING" gate is absorbed into INV-GNSSCFG-001.

---

## 5. Open Decisions

### OD-GNSSCFG-001 — Exact receiver configuration set

The specific dynamic model, message set, rates, and power settings written at commissioning are implementation bindings for the commissioning procedure, not decided here.

### OD-GNSSCFG-002 — Timeout value

The concrete acquisition timeout awaits receiver characterization (cold-start worst case at temperature extremes); configuration binding.

---

## 6. Proof Plan

### P-GNSSCFG-001 — Commissioning configuration persists

Configure the receiver during commissioning; power-cycle repeatedly.

Prove the receiver retains its configuration and firmware never rewrites it.

### P-GNSSCFG-002 — No in-flight reconfiguration

Exercise all in-flight error and recovery paths.

Prove no receiver-configuration command is ever issued.

### P-GNSSCFG-003 — No cold lockout

With a fresh temperature reading far below the legacy lockout threshold, and energy admission passing, prove GNSS is attempted.

With admission failing (droop), prove GNSS is skipped — for energy reasons, not temperature.

### P-GNSSCFG-004 — Fixed timeout

Run acquisitions across temperatures, outage histories, and energy tiers.

Prove the timeout bound is identical in all cases.

### P-GNSSCFG-005 — Stale temperature admission

Fail the temperature sensor after a warm reading in marginal energy conditions.

Prove admission uses last-known-good temperature (no cold-assumption veto).

---

## 7. Regeneration Test

A clean-room implementer receiving this DDR should understand that:

- the GNSS receiver is configured once during commissioning and persisted in the receiver's own flash;
- no in-flight reconfiguration path exists;
- there is no temperature cold-lockout — energy admission (tier + droop prediction) alone decides GNSS attempts;
- acquisition is bounded by a fixed configured timeout with no adaptivity;
- stale temperature resolves to last-known-good in the admission path.

The implementer should not need today's GNSS driver, PCAS command set, or lockout code to recreate the intended behavior.

---

## 8. Corpus Status

This was the last queued interview topic from the 2026-08-09 corpus review. Remaining work is non-interview: legacy-record retirement (after the 0004 disposition), battery/load profiling (DDR-0016 OD-PWR-001), and implementation conformance per the README work queue.

