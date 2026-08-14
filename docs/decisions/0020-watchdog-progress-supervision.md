# DDR-0020: Watchdog and Progress Supervision

**Status:** Draft — product intent elicited and resolved; implementation validation pending  
**Intent Interview Date:** 2026-08-09  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** Supervision of a wedged wake cycle, progress deadman with checkpoints, per-operation timeouts, hardware watchdog backstop, commissioning exemption, reset evidence, and repeated-wedge behavior  
**Authority:** Product intent is normative. Checkpoint placement, deadline values, watchdog timeout configuration, and reset-cause register usage are implementation bindings.

**Absorbs:** the supervision-related remnants of the retired legacy forward-progress record (reset-only-for-unrecoverable rule) and the retired legacy mission-state record (commissioning deadman exemption) that no other V2 record captured.

---

## 1. Intent

Stratosonde cannot be power-cycled by a human after launch. A wake cycle that is technically executing but making no forward progress — a sensor transaction that never completes, a flash operation that never returns — must be detected and escaped deterministically, or the mission is silently over.

Supervision is layered so that cheap, local escapes handle common cases and progressively stronger mechanisms handle rarer, worse ones:

> **Per-operation timeouts catch and degrade individual wedged operations. A progress deadman — checkpoints with deadlines across the wake cycle — catches anything that slips past local timeouts. An independent hardware watchdog is the final backstop beneath both. Reset returns the mission to the clean boot path; nothing fancier is attempted from inside a wedge.**

---

## 2. Product-Level Invariants

### INV-SUP-001 — Every peripheral operation is bounded

Each sensor, flash, GNSS, radio, and expansion-bus operation SHALL have its own bounded timeout. A timed-out operation SHALL be failed locally and degraded per DDR-0009 (stale marking, skip-and-continue) rather than allowed to stall the cycle.

### INV-SUP-002 — A progress deadman supervises the wake cycle

Each wake cycle SHALL advance through defined checkpoints, each with a deadline. Missing a checkpoint deadline SHALL trigger a reset into the clean-boot path (DDR-0012).

### INV-SUP-003 — An independent hardware watchdog is the final backstop

A hardware watchdog (e.g., IWDG class — independent clock, not software-cancellable by wedged code) SHALL run beneath the deadman so that even a failure of the supervision logic itself results in reset.

### INV-SUP-004 — The wedged path is dead simple: reset, record nothing extra

A supervision-triggered reset SHALL NOT attempt diagnostic logging, checkpoint recording, or any other best-effort work before resetting. The reset-cause register is sufficient evidence; it is captured later by the normal startup/event-log path (DDR-0011). Logging must never delay or endanger the reset.

### INV-SUP-005 — Commissioning is exempt from the deadman

The progress deadman SHALL arm only in flight. During commissioning a human is present to power-cycle; supervision timeouts that are tuned for flight would false-fire on legitimate ground operations (joins, GNSS cold start).

### INV-SUP-006 — No adaptation across repeated supervision resets

Firmware SHALL NOT count supervision resets per checkpoint, disable work items, or escalate modes in response to repeated deadman resets. Every boot is a clean wake cycle (DDR-0010): per-operation timeouts and fail-soft degradation should already isolate the failing subsystem, so the deadman firing twice on the same cause indicates a defect to fix, not a condition to route around.

---

## 3. Supervision Layers

| Layer | Catches | Response |
|---|---|---|
| 1. Per-operation timeout | One wedged peripheral operation | Fail the operation locally; stale/skip per DDR-0009; cycle continues |
| 2. Progress deadman | Cycle no longer advancing (missed checkpoint deadline) | Reset into clean boot (DDR-0012) |
| 3. Hardware watchdog | Anything, including supervision-logic failure | Reset into clean boot |

Layer 1 is expected to absorb nearly all real events. Layers 2 and 3 exist for the cases layer 1 cannot see.

---

## 4. Behavioral Requirements

Confidence legend:

- **CONFIRMED** — explicitly stated or affirmed during the interview.
- **INFERRED** — strongly implied but wording may still deserve review.
- **OPEN** — product behavior has not yet been fully decided.

| ID | Requirement | Confidence |
|---|---|---|
| BR-SUP-001 | Every peripheral operation SHALL have a bounded timeout. | CONFIRMED |
| BR-SUP-002 | A timed-out operation SHALL degrade locally (stale/skip-and-continue) rather than stall the cycle. | CONFIRMED |
| BR-SUP-003 | The wake cycle SHALL define checkpoints with deadlines; a missed deadline SHALL reset. | CONFIRMED |
| BR-SUP-004 | A hardware watchdog with an independent clock SHALL backstop the deadman. | CONFIRMED |
| BR-SUP-005 | Supervision resets SHALL NOT perform pre-reset logging or diagnostics; reset-cause evidence comes from the reset-cause register via the normal startup path. | CONFIRMED |
| BR-SUP-006 | The deadman SHALL arm only in flight; commissioning is exempt. | CONFIRMED |
| BR-SUP-007 | Firmware SHALL NOT implement per-checkpoint reset counters, work-item disabling, or mode escalation in response to repeated supervision resets. | CONFIRMED |
| BR-SUP-008 | Checkpoint set, deadline values, and watchdog timeout SHALL be configuration/implementation bindings. | CONFIRMED |
| BR-SUP-009 | Per-operation timeout values SHALL be sized generously enough that healthy slow peripherals (cold GNSS acquisition, flash erase) do not false-fire. | INFERRED |
| BR-SUP-010 | A reset caused by the hardware watchdog or deadman SHALL be distinguishable in the reset-cause capture (DDR-0011 event log) from ordinary power events where the platform allows. | INFERRED |

---

## 5. Relationship to Other Records

- **DDR-0009 (fail-soft):** defines the local degradation semantics layer 1 relies on, and the deterministic-reset philosophy for unrecoverable faults; this record adds the supervision machinery that detects the wedge.
- **DDR-0010 / DDR-0012 (persistence, boot):** a supervision reset is an ordinary reset for recovery purposes — clean boot, fresh cycle, never a resumed one.
- **DDR-0011 (storage):** the reset-cause event-log entry is captured at the next boot via the normal path, not at wedge time.
- **DDR-0016 (energy):** longer sleeps in lower tiers interact with deadman deadline placement; deadlines apply to the active cycle, not to sleep.
- **DDR-0018 (commissioning):** the commissioning exemption aligns supervision arming with the lifecycle door.
- **Retired legacy forward-progress and mission-state records:** supervision remnants absorbed (reset-never-hang; commissioning deadman exemption).

---

## 6. Open Decisions

### OD-SUP-001 — Checkpoint set and deadlines

Which phases of the wake cycle are checkpoints (sensor acquisition, GNSS, archive write, radio work, sleep entry) and their deadline values were not enumerated; they follow from the cycle design and must tolerate worst-case legitimate durations (e.g., GNSS cold start, archive burst bounds).

### OD-SUP-002 — Watchdog service strategy

Where and how often the hardware watchdog is kicked during a healthy cycle (single point per cycle vs. per checkpoint) was not decided.

### OD-SUP-003 — Sleep-period supervision

Whether the hardware watchdog runs across low-power sleep, or only across the active cycle (with sleep wake guaranteed by RTC), was not discussed.


---

## 7. Proof Plan

### P-SUP-001 — Wedged peripheral operation

Force one sensor transaction to never complete.

Prove its per-operation timeout fires, the reading is marked stale/absent, and the cycle completes normally (GNSS, archive, radio per policy).

### P-SUP-002 — Deadman catches a stalled cycle

Wedge the cycle in a way no per-operation timeout covers (e.g., a logic stall between operations).

Prove the checkpoint deadline expires and the device resets into the clean-boot path.

### P-SUP-003 — Hardware watchdog backstop

Disable/wedge the deadman servicing itself.

Prove the hardware watchdog still resets the device.

### P-SUP-004 — No pre-reset work

Instrument the supervision-reset path.

Prove no flash writes, log calls, or other side effects occur between detection and reset.

### P-SUP-005 — Reset-cause evidence

Trigger deadman and hardware-watchdog resets.

Prove the next boot's reset-cause capture (DDR-0011) distinguishes them from power-on events where the platform allows.

### P-SUP-006 — Commissioning exemption

Place the device in commissioning and stall a ground operation beyond a flight checkpoint deadline.

Prove no supervision reset occurs.

### P-SUP-007 — Deadman arms in flight

Transition to flight and repeat P-SUP-006's stall.

Prove the deadman now fires.

### P-SUP-008 — No repeated-reset adaptation

Force several consecutive deadman resets from the same cause.

Prove each boot is an identical clean wake cycle: no counters, no disabled work items, no mode escalation.

### P-SUP-009 — No false-fire on slow healthy operations

Run worst-case legitimate durations (GNSS cold start, full flash erase, bounded archive burst).

Prove no supervision timeout fires.

---

## 8. Regeneration Test

A clean-room implementer receiving this DDR should understand that:

- supervision is layered: per-operation timeouts, then a checkpoint deadman, then an independent hardware watchdog;
- local timeouts fail and degrade operations individually; the deadman and watchdog reset into the clean-boot path;
- supervision resets do no pre-reset logging — the reset-cause register is the evidence;
- the deadman arms only in flight; commissioning is exempt because a human can power-cycle;
- repeated supervision resets cause no adaptation — every boot is a clean cycle;
- all deadlines, checkpoints, and timeout values are configuration bindings sized to tolerate worst-case healthy operations.

The implementer should not need today's watchdog driver, timer configuration, or main-loop structure to recreate the intended behavior.

---

## 9. Next Intent Interview

One queued topic remains from the corpus review:

1. **GNSS receiver configuration policy** — when receiver configuration is allowed (commissioning-only), dynamics/airborne settings, and the cold-lockout disposition after DDR-0016's temperature revision.

Beyond that, the corpus review found no further coverage gaps; remaining work is implementation conformance and the legacy-record retirements.


---

## Amendment 2026-08-12 (intent interview reconciliation)

No substantive amendment required. The 2026-08-12 interview (round 2) independently reconfirmed:

- commissioning may be exempt/relaxed;
- mission flight must have watchdog/progress supervision enabled;
- a hang should return through reset/startup (see also DDR-0009 INV-FAIL-013/014: reset cause is diagnostic, and reset loops are faults to fix, not new mission modes).
