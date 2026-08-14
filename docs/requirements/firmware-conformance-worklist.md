# Stratosonde Firmware Conformance Worklist

**Date:** 2026-08-12 (round 2); extended 2026-08-13 (three-pass intent interview)  
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

---

# Added 2026-08-13 (three-pass intent interview merge)

Each item names the requirement it proves. See
`../decisions/merge-ledger-2026-08-13.md` for how these arose.

## P0 — the 24 h RF-legality change (code already retimed)

### FW-CONF-023 — Stale-position RF budget is 24 h and single
**Intent:** DDR-0015 `BR-STALE-017/018/020` · **Proof:** `P-STALE-014`.  
`GPS_LOSS_SILENCE_S` is now `24U * 3600U`; a source-scan regression exists in
`tests/host/test_review_findings.c` (`test_stale_position_budget_is_24h`). Still needed:
a behavioral test walking position age across the boundary — `<= 24 h` transmits,
`> 24 h` silent — with science/archive/GNSS continuing while dark.

### FW-CONF-024 — One valid fix clears staleness silence on the same wake
**Intent:** DDR-0015 `BR-STALE-019` · **Proof:** `P-STALE-015`.  
No dwell, no re-qualification, no operator action.

### FW-CONF-025 — No independent RTC-age RF cutoff exists
**Intent:** DDR-0015 `BR-STALE-020`, DDR-0013 `INV-TIME-009` · **Proof:**
`P-STALE-016`, `P-TIME-010`.  
Audit: prove no second time-based silence timer is reachable.

### FW-CONF-026 — Band 0 home-region fallback
**Intent:** DDR-0015 `INV-STALE-008`, `BR-STALE-013..016` · **Proof:**
`P-STALE-011..013`.  
Flight begins with no accepted fix: position invalid/degraded and never promoted to
last-known-good; `home_region` RF permitted within budget; silence after; first real fix
supersedes immediately. **Not yet implemented** — `home_region` provisioning does not
exist in code.

## P0/P1 — scheduling and energy

### FW-CONF-027 — Fixed start-to-start scheduler
**Intent:** DDR-0001 `BR-WAKE-017` · **Proof:** `P-WAKE-011`.  
Variable wake duration must not accumulate cadence drift.

### FW-CONF-028 — FULL/SLEEP per-wake admission
**Intent:** DDR-0001 `BR-WAKE-018`, DDR-0016 `BR-PWR-018` · **Proof:** `P-WAKE-012`,
`P-PWR-013`.  
Includes the negative case: prove no code path *deliberately* plans a GNSS-less wake for
energy reasons. Touches `power_model.c` (divergence 3).

### FW-CONF-029 — Durable battery/energy trend
**Intent:** DDR-0016 `BR-PWR-016` · **Proof:** `P-PWR-015`.  
Reset must not restart cadence adaptation from a neutral assumption.

### FW-CONF-030 — Degradation is failure-driven, not plan-driven
**Intent:** DDR-0001 `BR-WAKE-003` vs `INV-WAKE-012` · **Proof:** `P-WAKE-013`.  
An admitted FULL cycle whose GNSS fails still archives honest stale provenance, and that
path is distinguishable from a SLEEP decision.

## P1 — reset and persistence

### FW-CONF-031 — Reset never fabricates an observation
**Intent:** DDR-0010 `INV-PERSIST-011` · **Proof:** `P-PERSIST-010`.  
Inject reset before/among sensors, after GNSS, during archive commit, and after commit
before TX. Every backend-visible record is complete-and-valid or absent.

### FW-CONF-032 — Transient reset never becomes permanent telemetry loss
**Intent:** DDR-0010 `INV-PERSIST-012` · **Proof:** `P-PERSIST-011/012`.  
Also prove no reset creates a new mission identity or restarts record numbering.

### FW-CONF-033 — Float latch survives reset
**Intent:** DDR-0002 `BR-LIFE-024` · **Proof:** `P-LIFE-013`, `P-BOOT-012`.

### FW-CONF-034 — Cadence survives reset; transient timer may not
**Intent:** DDR-0010 `INV-PERSIST-010` · **Proof:** `P-BOOT-013`.  
No rapid reset/wake loop, no unintended multi-day sleep; at most one shifted observation.

## P1 — storage

### FW-CONF-035 — Torn record is skipped, never truncating
**Intent:** DDR-0011 `BR-STORE-001` · **Proof:** `P-STORE-011`.  
A/B valid, C torn, D/E valid: A/B and D/E all readable, iteration reports one gap.

### FW-CONF-036 — No routine archive scan on a valid-cursor boot
**Intent:** DDR-0011 `BR-STORE-002`, DDR-0012 `INV-BOOT-010` · **Proof:** `P-STORE-014`,
`P-BOOT-014`.

### FW-CONF-037 — Brownout-loop-safe boot and deferred reconstruction
**Intent:** DDR-0011 `BR-STORE-002` · **Proof:** `P-STORE-012/013`.

### FW-CONF-038 — No mandatory read-back after a successful science write
**Intent:** DDR-0011 `BR-STORE-003` · **Proof:** inspection + write-path timing.

## P1/P2 — recovery

### FW-CONF-039 — Bounded cross-subsystem recovery, immediate normal service on success
**Intent:** DDR-0009 `BR-FAIL-016/017` · **Proof:** `P-FAIL-011`.  
Extends FW-CONF-012 beyond sensors to GNSS, radio hardware, radio stack, and buses. No
probation state.

### FW-CONF-040 — Radio recovery never triggers an in-flight join
**Intent:** DDR-0009 `BR-FAIL-018`, DDR-0018 `INV-COMM-001` · **Proof:** fault injection
on radio/stack reset paths.

### FW-CONF-041 — No reset-count or failure-history escalation anywhere
**Intent:** DDR-0009 `OD-FAIL-006` resolved, DDR-0020 · **Proof:** `P-FAIL-012`.  
Audit for any repeated-reset adaptation and remove/disable it for first flight.

## P2 — archive delivery and gap repair

### FW-CONF-042 — Arbitrary retained-ID lookup
**Intent:** DDR-0004 `BR-ARCH-018` · **Proof:** `P-ARCH-011`.  
Linear search acceptable; must not require ordered or near-cursor access.

### FW-CONF-043 — Requested-ID-unavailable substitution
**Intent:** DDR-0004 `BR-ARCH-019` · **Proof:** `P-ARCH-012`.  
Returned record's own ID is authoritative; no separate status field.

### FW-CONF-044 — Explicit request preempts backfill, and is ephemeral
**Intent:** DDR-0005 `BR-TX-023/024` · **Proof:** `P-TX-011/012`.  
Prove **no** persistent request queue survives sleep or reset.

### FW-CONF-045 — Request never overrides legality or energy
**Intent:** DDR-0005 `BR-TX-025` · **Proof:** `P-TX-013`.

### FW-CONF-046 — Record shape is stable under partial failure
**Intent:** DDR-0004 `BR-ARCH-017` · **Proof:** `P-ARCH-013`.

## P2 — lifecycle and commissioning

### FW-CONF-047 — Float latch is one-way in flight
**Intent:** DDR-0002 `BR-LIFE-023/025` · **Proof:** `P-LIFE-012`.  
Post-float pressure excursions must not restore ascent cadence.

### FW-CONF-048 — Automatic launch without the operator action
**Intent:** DDR-0002 `BR-LIFE-027`, DDR-0018 `BR-COMM-020` · **Proof:** `P-LIFE-014`,
`P-COMM-013`.

### FW-CONF-049 — Readiness check uses the normal GNSS predicate
**Intent:** DDR-0003 `BR-GNSS-021`, DDR-0018 `BR-COMM-018` · **Proof:** `P-GNSS-011`,
`P-COMM-011/012`.  
No weaker commissioning-only fix criterion; no false-positive ready indication.

### FW-CONF-050 — Provisioning cannot complete with a missing region session
**Intent:** DDR-0018 `BR-COMM-017` · **Proof:** `P-COMM-014`.

### FW-CONF-051 — Placeholder position cannot masquerade as a fix
**Intent:** DDR-0003 §17 · **Proof:** `P-GNSS-012`.

## P2 — configuration and protocol

### FW-CONF-052 — Unsupported downlink is safe
**Intent:** DDR-0014 `BR-CONFIG-014` · **Proof:** `P-CONFIG-011`.  
Fuzz arbitrary downlink content: no crash, reset, or memory corruption.

### FW-CONF-053 — Persisted operational config is repaired, not fatal
**Intent:** DDR-0014 `BR-CONFIG-015/016` · **Proof:** `P-CONFIG-010`.  
Credentials explicitly excluded from fail-soft repair.

### FW-CONF-054 — Multi-version backend codec regression suite
**Intent:** DDR-0027 `BR-PROTO-001/002/005` · **Proof:** `P-PROTO-001..003`.  
Ground/backend work: every deployed format decodes; adding a version does not regress
old vectors; retransmitted historical records recover the same logical ID.

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
