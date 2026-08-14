# DDR-0023: Scientific Data Truth, Derived Products, and Onboard Interpretation

**Status:** Draft — intent substantially elicited 2026-08-12; per-sensor bindings and proof pending  
**Intent Interview Date:** 2026-08-12  
**Method:** Intent Interview V2 / Intent-Anchored Development  
**Scope:** Raw measurements, calibration, derived values, onboard event detection, bandwidth-driven processing, provenance, deterministic algorithms, anomaly detection, future trajectory/satellite calculations, and the boundary between firmware interpretation and backend interpretation

---

## 1. Intent

Stratosonde is a scientific instrument.

Its usefulness depends on users being able to trust what a reported value actually means.

The preferred default is therefore simple:

> **Report what the instrument actually observed. Calibrate when scientifically justified. Derive or summarize data onboard when the mission requires it. Never allow a synthetic, stale, corrected, inferred, or derived value to silently masquerade as an unmodified fresh sensor observation.**

The firmware does not need to remain a passive recorder.

Bandwidth and autonomy constraints may require significant onboard processing.

Examples include:

- infrasound or acoustic event detection where raw continuous audio cannot reasonably be transmitted;
- atmospheric-wave or gravity-wave detection from pressure-series data;
- anomaly detection;
- high-rate signal reduction;
- trajectory or geometric calculations;
- future satellite-opportunity calculations using uploaded ephemeris;
- local decisions required when communication with the ground is unavailable.

The principle is not "never interpret."

The principle is:

> **Interpret only when doing so produces mission value, preserve provenance, and keep the transformation deterministic and explainable.**

---

## 2. Product-Level Invariants

### INV-DATA-001 — No fabricated measurement may masquerade as sensor truth

Firmware SHALL NOT invent a plausible measurement merely because fresh data is unavailable.

If a fresh value does not exist, the record shall express stale, unavailable, invalid, derived, or another explicitly defined state.

### INV-DATA-002 — Sensor-reported values are the default scientific authority

For ordinary low-rate sensors, the value reported by the sensor is the primary observation.

Filtering, smoothing, substitution, or correction SHALL NOT silently replace that observation unless the resulting quantity is an intentionally defined scientific product.

### INV-DATA-003 — Calibration is permitted and must remain identifiable

Known calibration, compensation, unit conversion, or physically justified correction MAY be applied in firmware.

The data model SHALL make the semantics of the stored/transmitted quantity clear.

Where scientifically useful and practical, sufficient information should remain available to reconstruct or audit the correction.

### INV-DATA-004 — Raw, corrected, and derived products are semantically distinct

A derived product SHALL NOT be presented as if it were the raw source measurement.

Schemas, metadata, product identity, or other provenance SHALL distinguish meaningful processing classes.

### INV-DATA-005 — Onboard processing is permitted when required by mission constraints

Firmware MAY perform signal processing, event detection, anomaly detection, prediction, geometric calculations, or data reduction when:

- raw data exceeds practical RF bandwidth;
- a timely local decision is required;
- the backend cannot provide the decision before it is needed;
- transmitting a derived product creates substantially more mission value than transmitting an arbitrary subset of raw samples.

### INV-DATA-006 — Onboard scientific algorithms are deterministic and versioned

Given the same input samples, configuration, algorithm version, and state, an onboard scientific transformation SHALL produce the same result.

The baseline design SHALL NOT use self-learning or autonomously evolving scientific policy in flight.

### INV-DATA-007 — Processing failure shall not contaminate source science

Failure of an optional derived-product algorithm SHALL NOT invalidate unrelated raw/base science.

Where the source measurement remains available, it should continue through the ordinary archive and telemetry pipeline.

### INV-DATA-008 — Derived products need provenance sufficient for interpretation

A derived or event product SHALL identify enough context to allow a backend or scientist to understand what generated it.

Depending on the product, this may include schema/product version, algorithm version, source sensor identity, source time interval, configuration/profile identifier, quality flags, and relevant navigation/time provenance.

Exact encoding is a protocol binding.

### INV-DATA-009 — Bandwidth selection does not change scientific truth

A decision not to transmit a raw data stream because of bandwidth limits does not make a derived summary equivalent to that raw stream.

The product semantics SHALL remain explicit.

### INV-DATA-010 — Regulatory decisions fail conservatively when uncertainty affects legality

Scientific uncertainty normally results in degraded-but-continuing operation.

Regulatory uncertainty is different.

When uncertainty means the sonde cannot establish that transmission is permitted, RF authorization policy SHALL control even if scientifically useful data is waiting.

### INV-DATA-011 — Preserve raw physical observables when practical

When a sensor exposes a stable compact raw observable, firmware SHALL archive/transmit that source value where practical. (Added 2026-08-12, round 2.)

The objective is to permit future backend reprocessing when calibration improves, compensation models improve, scientific interpretation changes, or historical comparisons require consistent re-analysis.

### INV-DATA-012 — Calibration may be backend-owned

Sensor-specific calibration coefficients MAY be stored in the Stratosonde backend and bound to device/sensor identity under DDR-0024. This is preferred when firmware does not require the calibrated value locally and later reprocessing is scientifically useful. (Added 2026-08-12, round 2.)

### INV-DATA-013 — Firmware calibration is justified by local need

Firmware MAY calculate calibrated values when required for autonomous control, power policy, event detection, onboard scientific processing, or an explicit first-class product. (Added 2026-08-12, round 2.)

Proof: preserve raw data and show historical data can be deterministically reprocessed with an original or later-corrected calibration.

### INV-DATA-014 — Minimal validity metadata

Firmware does not need a generalized probabilistic confidence model for every measurement. The default data contract is: (Added 2026-08-12, round 3.)

- report the measured/raw value;
- explicitly identify only known failure provenance that changes meaning, such as stale/unavailable/substituted data;
- place sensor-specific quality/status fields in the product schema only when defined during sensor/product design.

The backend/scientist remains responsible for broader plausibility interpretation.

### INV-DATA-015 — Backend owns long-term reinterpretability

When calibration is not required for autonomous firmware action, backend calibration SHOULD remain separable from immutable historical raw measurements. A later improved calibration MUST be able to reinterpret historical raw data without changing what the instrument originally observed. (Added 2026-08-12, round 3.)

---

## 3. Behavioral Requirements

| ID | Requirement | Confidence |
|---|---|---|
| BR-DATA-001 | Ordinary sensor data SHALL be recorded according to the value actually produced by the sensor, subject to explicit validation/calibration policy. | CONFIRMED |
| BR-DATA-002 | Missing fresh measurements SHALL NOT be replaced by fabricated nominal values. | CONFIRMED |
| BR-DATA-003 | Last-known-good substitution SHALL carry explicit stale provenance. | CONFIRMED |
| BR-DATA-004 | Calibration and temperature compensation MAY be performed when scientifically justified. | CONFIRMED |
| BR-DATA-005 | A calibrated or derived value SHALL be distinguishable from the unmodified source quantity whenever that distinction materially affects interpretation. | CONFIRMED |
| BR-DATA-006 | Firmware MAY perform event/anomaly detection on high-rate data when continuous raw transmission is impractical. | CONFIRMED |
| BR-DATA-007 | Firmware MAY make time-critical local predictions or geometric decisions when waiting for a ground calculation would make the information useless. | CONFIRMED |
| BR-DATA-008 | Onboard scientific algorithms SHALL be deterministic and controlled by firmware/configuration versions rather than autonomous learning. | CONFIRMED |
| BR-DATA-009 | Failure of a derived-product algorithm SHALL NOT suppress otherwise valid source science where the source remains available. | CONFIRMED |
| BR-DATA-010 | Derived/event products intended as first-class science SHALL use explicit schema/version/provenance. | CONFIRMED |
| BR-DATA-011 | The backend SHALL be able to distinguish fresh measured values, stale values, unavailable values, and intentionally derived products. | CONFIRMED |
| BR-DATA-012 | Bandwidth limitations MAY determine what is transmitted, but SHALL NOT justify falsely representing reduced or derived information as raw data. | CONFIRMED |

---

## 4. Data Product Classes

### 4.1 Source observations

Direct sensor or receiver output, after only decoding and basic validity checking.

Examples include pressure, humidity, temperature, GNSS position, and high-rate microphone samples.

### 4.2 Calibrated observations

A source observation transformed using a known scientifically justified correction.

Examples include sensor calibration coefficients, temperature compensation, unit conversion, and calibrated pressure.

Calibration may be performed onboard. The transformation must remain defined and reproducible.

### 4.3 Derived/event products

Information created by combining, filtering, classifying, summarizing, or predicting from observations.

Examples include detected infrasound events, gravity-wave candidates, spectral features, anomaly events, trajectory predictions, and predicted satellite opportunities.

Derived products are legitimate first-class scientific products when intentionally designed as such. They are not raw measurements.

---

## 5. High-Rate Sensors and Bandwidth Reduction

Some future payloads may generate data many orders of magnitude faster than LoRaWAN can transmit.

For such payloads the intended pattern is:

```text
high-rate acquisition
        |
        v
bounded local processing
        |
        +--> retained raw/high-rate data where feasible
        |
        +--> deterministic event/features/summary
                       |
                       v
             archive/transmit selected product
```

The exact retention strategy is sensor-specific.

A product may retain all raw data locally, retain only event windows, retain a rolling buffer, retain features only, or use another explicit policy.

That decision must be documented for each high-rate science instrument.

---

## 6. Time-Critical Autonomous Interpretation

Some calculations are useful only if performed onboard.

For example, a future satellite-opportunity feature may receive ephemeris/configuration from the ground but need to determine locally whether the sonde's current trajectory and timing create a relevant opportunity.

Likewise, a sonde may need to decide locally that an operation cannot safely be attempted because environmental or power conditions have crossed a known configured limit.

Such logic is acceptable autonomy.

The distinction is:

> **The firmware autonomously executes approved deterministic rules; it does not autonomously invent new rules.**

---

## 7. Relationship to Data Transmission

Fresh compact science remains the primary transmission product under DDR-0022 and DDR-0005.

When additional bandwidth becomes available, the system may transmit more information.

A derived scientific event may be more valuable than an arbitrary historical record, but the exact cross-product priority is not defined here.

That prioritization should be configuration- or mission-specific and must not interfere with due current science.

---

## 8. Relationship to Existing DDRs

- **DDR-0003** owns GNSS freshness and position/time provenance.
- **DDR-0004** owns archive identity and retention.
- **DDR-0005** owns live-versus-historical delivery priority.
- **DDR-0009** owns stale/unavailable fail-soft semantics.
- **DDR-0014** owns configuration and future ground authority.
- **DDR-0017** owns first-class versus best-effort application-published data.
- **DDR-0019** owns radio/payload transmission mechanics.
- **DDR-0022** owns mission-level value hierarchy.

This DDR supplies the missing scientific-processing contract shared across those records.

---

## 9. Open Decisions

### OD-DATA-001 — Raw retention per high-rate instrument

Each high-rate instrument requires an explicit decision regarding continuous raw retention, event-window retention, rolling-buffer depth, feature-only retention, and overwrite policy.

### OD-DATA-002 — Transformation metadata

The minimum metadata needed to reproduce or audit calibrated and derived products remains to be defined per protocol/schema.

### OD-DATA-003 — Algorithm update authority

Future downlink updates may alter algorithm parameters.

Whether entirely new scientific algorithms may be delivered without reflashing firmware is not currently decided.

### OD-DATA-004 — Derived-event transmission priority

The ordering among current routine science, urgent event products, recent archive science, and diagnostics/status requires mission-specific policy.

Current routine science remains protected from starvation.

### OD-DATA-005 — Confidence representation

Some future detectors may naturally produce confidence, quality, or significance values.

Whether those become standardized product metadata is undecided.

---

## 10. Proof Targets

### P-DATA-001 — No fabricated source data

Force each sensor into unavailable state with and without historical valid data. Prove output is stale/unavailable as designed and never appears as fabricated fresh science.

### P-DATA-002 — Calibration reproducibility

Feed a known input vector through every onboard calibration. Prove results match the defined calibration model and algorithm version.

### P-DATA-003 — Deterministic event processing

Replay identical high-rate sample data. Prove identical event output and metadata.

### P-DATA-004 — Processing failure isolation

Force a derived algorithm to fail or exceed its execution budget. Prove base science collection, archive, and mission cycling continue.

### P-DATA-005 — Provenance survives archive and radio

Generate raw/calibrated/derived products. Prove the backend can identify their product class and required provenance after storage and transmission.

### P-DATA-006 — Bandwidth reduction honesty

Run a high-rate source whose complete raw stream cannot be transmitted. Prove the transmitted summary/event is explicitly identified as a derived product rather than raw data.

---

## 11. Regeneration Test

A clean-room implementer should understand that:

- Stratosonde is a scientific instrument and data truth is paramount;
- the ordinary default is to report what sensors actually observed;
- stale values are acceptable only when explicitly stale;
- calibration and compensation are legitimate;
- derived data is legitimate and sometimes essential;
- derived data must never silently masquerade as raw data;
- onboard event detection and prediction are expected future capabilities;
- processing must remain deterministic and version-controlled;
- bandwidth may determine which information is sent but cannot change its scientific meaning;
- one failed processing algorithm must not take the rest of the mission down;
- regulatory uncertainty is treated more conservatively than ordinary scientific uncertainty.
