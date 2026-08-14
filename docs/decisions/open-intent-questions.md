# Stratosonde Open Intent Questions

**Consolidated:** 2026-08-12 (merges the round-2 and round-3 interview lists; round-3 items are current, residual round-2 items folded in)
**Updated:** 2026-08-13 (three-pass intent interview merged; items 2/4/6 revised, items 20–30 added)
**Purpose:** Preserve what is still undecided so firmware does not silently invent product policy.

The broad first-flight architecture is now highly constrained. Remaining questions are mostly exact protocols, thresholds, and procedures. The next useful interviews should mostly ask:

> "What exact bound, field, transition, or protocol completes this already-decided behavior?"

---

## 1. Exact cadence controller

Freeze interaction among configured target, day/night model, ascent cadence, power/energy trend, atmospheric dynamics if retained, and surplus work. (Tracks DDR-0022 OD-MISSION-001 / DDR-0002 OD-LIFE-006.)

## 2. Energy threshold derivation

Measured rules for full mission, immediate sleep, and surplus. Do not use illustrative
−60 °C / 3.6 V / 3.4 V values until characterized. (DDR-0016 OD-PWR-001.)

**Narrowed 2026-08-13:** the *shape* of the decision is now fixed — per-wake admission
is **FULL CYCLE or SLEEP** for first flight (DDR-0016 §11 `INV-PWR-022`, DDR-0001 §17
`INV-WAKE-012`). So "skip GNSS" and "sensor/log-only" are no longer thresholds to
derive: energy policy never deliberately selects them. What still needs measurement is
the single admission boundary (plus its hysteresis) and the surplus boundary.

## 3. Energy anti-chatter mechanism

Choose hysteresis, filtered slope, dwell time, confidence window, or combination. (DDR-0016 OD-PWR-004.)

## 4. Complete persistent-state inventory and loss budgets

Audit all actual persistent objects: regional LoRaWAN dynamic state, keys, counters, config, flight latch, archive frontier, delivery watermark, navigation/time anchors, adaptive state if needed. Each object needs a concrete loss bound per the DDR-0010 INV-PERSIST-008 consequence hierarchy (identity/keys: effectively zero tolerance; counters: never prohibited reuse; archive: bounded newest-record loss).

**Narrowed 2026-08-13:** the classification *rule* and the confirmed minimum inventory
are now normative (DDR-0010 §18, `OD-PERSIST-001` narrowed), including battery/energy
trend and calibration coefficients. Two bounds are explicitly still open: the archive
**replay-watermark rollback bound** (DDR-0005 `BR-TX-027`; the interview's "about 5-10"
was illustrative only, not a decision), and the per-object durability policy for each
real flash/backup-register object.

## 5. Claim PIN security protocol

Decide PIN placement (inside/outside QR), entropy, one-time/reusable semantics, rate limiting, transfer, recovery, admin override, and public-photo threat. (DDR-0024 OD-ID-001/002.)

## 6. Reset-loop policy — **CLOSED as product intent 2026-08-13**

**Decided:** no reset-count adaptation, no reduced recovery mode, no permanent
feature disable, no give-up state. Every reset retries the same clean recovery path
indefinitely. (DDR-0009 `OD-FAIL-006` and DDR-0012 `OD-BOOT-006` both resolved;
DDR-0020 already said this.)

**Residual, engineering not product:** whether extreme *immediate* reset loops
warrant additional hardware/boot-level brownout protection. That is a
power-electronics/boot question, and it must not reintroduce software policy
escalation.

## 7. Application resource request API

Define request types, grant duration, energy-surplus indication, cancellation/deadline behavior, radio semantics, and app persistence permissions. (DDR-0017 OD-QWIIC-005.)

## 8. Exact config normalization rules

Per field: min, max, quantization, cross-field normalization, and which malformed combinations reject (versus clamp per DDR-0014 INV-CONFIG-011/012).

## 9. Configuration event schema

Define generation/version, timestamp, source, hash/changed-field summary. (DDR-0014 INV-CONFIG-013.)

## 10. Manufacturing/pre-commissioning behavior

Define first-power-on behavior for a never-commissioned board: radio, sensors, sleep, indication, and the irreversible transition out of commissioning. (DDR-0002 INV-LIFE-009/010.)

## 11. Sensor-specific stale encoding

Each sensor/product defines unavailable semantics, last-value retention, stale bit/encoding, and sensor-native status propagation. (DDR-0003, DDR-0023.)

## 12. First-flight quantitative release limits

Set measured limits for sleep current, wake energy, max wake duration, GNSS timeout, cadence jitter, power margin, chamber temperature, soak duration, and archive power-cut loss bound. (DDR-0026 OD-VER-001.)

## 13. Backend scientific provenance

Define schema for sensor identity, calibration version/effective dates, firmware version, payload schema, mission record, and reprocessing provenance. (DDR-0023 OD-DATA-002; includes how backend calibration keys to whole-device identity, sensor serial, or slot/module, and how replacements/calibration revisions are represented.)

## 14. Secondary transmission ordering

Fresh science is first. Still order archive science, event products, high-rate derived science, GNSS diagnostics, and system status. (DDR-0005 OD-TX-009 / DDR-0022 OD-MISSION-002 / DDR-0023 OD-DATA-004.)

## 15. Credential lifecycle / revocation

Compromised-key handling, deliberate re-keying, recovered-device reprovision, and backend retired-key history. (DDR-0024 OD-ID-003.)

## 16. Mutable state across intentional firmware service

Confirmed preservation: DevEUI and LoRaWAN credentials. Still decide flight latch, counters/session state, mission config, archive, watermarks, reset history. (DDR-0025 OD-FW-001.)

## 17. Local servicing security

SWD accessibility/readout protection, local firmware authenticity, and whether physical possession is sufficient service authority. (DDR-0025 OD-FW-003/004.)

## 18. Future downlink command surface

Which config fields are remotely writable, authentication/replay model, persistent versus one-shot commands, command expiry/rate limiting. (DDR-0014 OD-CONFIG-001/002.)

## 19. Hardware revision policy

One supported board now; revisit compatibility machinery only when a second real revision exists. (DDR-0024 OD-ID-005; no firmware action expected.)

---

# Added 2026-08-13 (three-pass intent interview)

These are the bindings the 2026-08-13 package deliberately declined to invent. They
were **not** guessed during the merge. Most are numeric or encoding choices sitting on
top of already-decided behavior.

## 20. Automatic-launch detection thresholds

Exact pressure delta, rate, and persistence/confirmation required to declare launch,
and the exact float/stability qualification window. Current implementation values
(`MISSION_LAUNCH_DP_HPA`, the 900 s flat window, `MISSION_FLOAT_MIN_ASCENT_DP_HPA`)
are live bindings from 2026-08-11, not interview-confirmed product requirements.
(DDR-0002 §19.)

## 21. Deliberate launch/readiness gesture

Exact operator gesture (press duration, double-press, magnet, etc.) and the exact
LED/indication pattern distinguishing "ready" from "not ready". Only the *semantics*
are fixed: unambiguous, no false positives, no PC required. (DDR-0018 `INV-COMM-009`,
`BR-COMM-022`.)

## 22. Local recovery attempt counts and sequences

Per-subsystem retry counts and the deterministic recovery ladder (bus re-clock,
peripheral reset, power-cycle) for sensors, GNSS, radio hardware, and radio stack.
Must be bounded and must fit the wake/supervision budget. (DDR-0009 `INV-FAIL-016`.)

## 23. Archive record-ID width and wrap handling

ID width, and the behavior at wrap for a mission with no planned end. (DDR-0004 §13,
`BR-ARCH-020` context.)

## 24. Substitute-record selection when a requested ID is gone

Earliest retained, nearest retained, or newest. The *contract* is settled — the
returned record's own ID is authoritative and no separate status field is needed — but
the selection rule is open. Existing `BR-ARCH-011` direction (earliest available)
stands until revisited. (DDR-0004 §19 `BR-ARCH-020`, `OD-ARCH-002` narrowed.)

## 25. Replay-watermark checkpoint interval and duplicate bound

How often the delivery watermark is checkpointed, and the maximum acceptable duplicate
count after rollback. (DDR-0005 `BR-TX-027`.)

## 26. Flash erase geometry and bad-area bookkeeping

Erase unit, just-in-time erase trigger point, and how (or whether) bad areas are
tracked. Elaborate bad-block management is explicitly out of scope for first flight.
(DDR-0011 §25.)

## 27. Protocol-version identification encoding

Explicit byte/bitfield, FPort mapping, payload shape, or message type. Must be decided
jointly with DDR-0019 payload bindings. (DDR-0027 `OD-PROTO-001`.)

## 28. Backend codec retirement horizon

When a historical decoder may be removed. Needs an operational definition of
"plausibly still airborne", which is awkward because DDR-0002 defines no mission end.
(DDR-0027 `OD-PROTO-002`.)

## 29. Calibration-invalid science representation

How a science record represents its values when calibration state is unrecoverable —
raw-only, flagged, or withheld. (DDR-0023 / DDR-0010 calibration durability.)

## 30. Scheduler overrun behavior

What happens when a wake's work overruns one or more scheduled start-to-start epochs:
skip to the next grid point, or run immediately. Constraint already fixed: neither drop
indefinitely many epochs nor produce a rapid catch-up burst. (DDR-0001 `BR-WAKE-020`.)

---

## Completion assessment

The broad product philosophy is largely closed. Remaining interviews can focus on boundary values, protocols, and operational procedures rather than first-principles mission intent.

**2026-08-13 update.** The three-pass interview closed the last significant
*first-principles* gaps: the lifecycle is now an explicit one-way chain, energy
admission is a single FULL/SLEEP decision, reset semantics are "omit, never fabricate",
storage corruption is locally contained, and the RF-legality budget has a number (24 h).

What is left is almost entirely **bindings**: numbers to measure, encodings to pick,
gestures to choose. The one genuinely conceptual item remaining is item 28 (codec
retirement horizon), which is hard only because the product deliberately has no mission
end. The next interviews should ask "what exact bound completes this decided behavior?"
rather than reopening intent.
