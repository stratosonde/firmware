# Stratosonde Open Intent Questions

**Consolidated:** 2026-08-12 (merges the round-2 and round-3 interview lists; round-3 items are current, residual round-2 items folded in)
**Purpose:** Preserve what is still undecided so firmware does not silently invent product policy.

The broad first-flight architecture is now highly constrained. Remaining questions are mostly exact protocols, thresholds, and procedures. The next useful interviews should mostly ask:

> "What exact bound, field, transition, or protocol completes this already-decided behavior?"

---

## 1. Exact cadence controller

Freeze interaction among configured target, day/night model, ascent cadence, power/energy trend, atmospheric dynamics if retained, and surplus work. (Tracks DDR-0022 OD-MISSION-001 / DDR-0002 OD-LIFE-006.)

## 2. Energy threshold derivation

Measured rules for full mission, skip GNSS, skip radio, sensor/log-only, immediate sleep, and surplus. Do not use illustrative −60 °C / 3.6 V / 3.4 V values until characterized. (DDR-0016 OD-PWR-001.)

## 3. Energy anti-chatter mechanism

Choose hysteresis, filtered slope, dwell time, confidence window, or combination. (DDR-0016 OD-PWR-004.)

## 4. Complete persistent-state inventory and loss budgets

Audit all actual persistent objects: regional LoRaWAN dynamic state, keys, counters, config, flight latch, archive frontier, delivery watermark, navigation/time anchors, adaptive state if needed. Each object needs a concrete loss bound per the DDR-0010 INV-PERSIST-008 consequence hierarchy (identity/keys: effectively zero tolerance; counters: never prohibited reuse; archive: bounded newest-record loss).

## 5. Claim PIN security protocol

Decide PIN placement (inside/outside QR), entropy, one-time/reusable semantics, rate limiting, transfer, recovery, admin override, and public-photo threat. (DDR-0024 OD-ID-001/002.)

## 6. Reset-loop policy

First-flight preference is simple retry (DDR-0009 INV-FAIL-013/014). Decide whether extreme immediate reset loops require any additional hardware/boot-level protection. (DDR-0009 OD-FAIL-006 / DDR-0012 OD-BOOT-006.)

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

## Completion assessment

The broad product philosophy is largely closed. Remaining interviews can focus on boundary values, protocols, and operational procedures rather than first-principles mission intent.
