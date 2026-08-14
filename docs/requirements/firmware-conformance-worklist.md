# Stratosonde Firmware Conformance Worklist — Interview Round 2

**Date:** 2026-08-12  
**Purpose:** Convert the intent corpus into an implementation/proof queue. This is not itself a DDR.

## P0/P1 — Mission correctness

### FW-CONF-001 — Science scheduler preempts archive recovery
**Intent:** DDR-0022 / DDR-0005.  
**Proof:** Large backlog + excellent link + accelerated run; fresh science interval stays inside allowed cadence/jitter.

### FW-CONF-002 — GNSS wake failure cannot report prior fix as fresh
**Intent:** DDR-0003 / DDR-0009 / DDR-0023.  
A failed acquisition may yield stale last-known fix or unavailable position, never old coordinates marked fresh.

### FW-CONF-003 — Archive recovery follows newest-first intent
**Intent:** DDR-0004 / DDR-0005.  
Newest eligible retained record first, then backward under bounded opportunity.

### FW-CONF-004 — Fresh science wins a one-packet budget
**Intent:** DDR-0022 / DDR-0005.

## P1 — Power predictability

### FW-CONF-005 — Predictive energy degradation, not intentional brownout
**Intent:** DDR-0016.  
Reduce work/cadence before an admitted load collapses the rail.

### FW-CONF-006 — GNSS load admission
If GNSS cannot be supported, skip it, preserve stale provenance, retry later.

### FW-CONF-007 — Automatic return toward configured target
Temporary power adaptation never rewrites the operator's nominal target.

## P1 — Persistence / recovery

### FW-CONF-008 — Reflash preserves identity and credentials
Provision, record DevEUI/credential checks, reflash, verify unchanged.

### FW-CONF-009 — Counters advance safely and never illegally reuse
Power-cut around persistence checkpoints.

### FW-CONF-010 — Metadata corruption does not unnecessarily abandon recoverable archive science
Reconstruct from valid self-checking records where possible.

## P1/P2 — Fault handling

### FW-CONF-011 — Fatal exception path resets
Inject HardFault/fatal assertion/deadman and prove deterministic reset + clean boot.

### FW-CONF-012 — Sensor-specific bounded retries
Each driver defines retryable faults, count/time budget, and optional bus/peripheral recovery. No infinite local recovery.

### FW-CONF-013 — Surprising sensor values are not silently censored
Only explicit scientific-product rules may reject a protocol-valid measurement.

## P2 — RF / protocol

### FW-CONF-014 — Stale-position RF legality timeout
Prove RF silence at policy boundary, continued science/logging, and automatic recovery on fresh GNSS.

### FW-CONF-015 — FPort is authoritative product discriminator
Maintain an explicit allocation table in the protocol binding.

### FW-CONF-016 — Decoder compatibility is explicit
Incompatible changes require a new FPort, schema/version field, or another documented discriminator.

## P2 — Calibration / data

### FW-CONF-017 — Preserve raw sensor observable where practical
Backend calibration binds to device/sensor identity.

### FW-CONF-018 — Firmware calibration only where local behavior needs it
Avoid duplicating backend-only correction logic without mission need.

## P2 — Verification infrastructure

### FW-CONF-019 — First target HIL lane
Build box controls build, ST-Link flash/reset, RTT capture, Otii power/current capture, pass/fail parser, artifact retention.

### FW-CONF-020 — Power-cut campaign
Randomize interruption around archive write, metadata/header update, LoRaWAN counter persistence, and session/config writes.

### FW-CONF-021 — RTT equivalence
Representative scenarios with RTT enabled and quiet/off; compare mission decisions and characterize power/timing effect.

### FW-CONF-022 — Cold-chamber characterization
Measure temperature/load/droop behavior for sleep, GNSS, radio TX, and recovery bursts; feed results back into DDR-0016.

## Deferred complexity

Do not add merely for completeness:

- OTA firmware update;
- custom field bootloader;
- runtime multi-hardware-revision autodetection;
- GNSS simulator;
- switched-I2C fault hardware;
- self-learning mission adaptation;
- elaborate generic sensor-fault framework.

Add them only when a concrete requirement or verification gap justifies them.
