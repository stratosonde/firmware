# Stratosonde Firmware Conformance Worklist

**Date:** 2026-08-12 (round 2); extended 2026-08-13 (three-pass intent interview); status pass 2026-08-15  
**Purpose:** Convert the intent corpus into an implementation/proof queue. This is not itself a DDR.

> **Status legend (2026-08-15 pass, against master @ `dc8026a`):**
> - 🟡 **IMPLEMENTED** — code does it (evidence named: issue/commit/host test); the item's named proof (target, HIL, accelerated, or behavioural run) is still open.
> - 🔴 **OPEN** — not implemented; tracker issue named.
> - ⚪ **NOT ASSESSED** — this pass did not verify it; do not read anything into the marker.
>
> Status lines name issues/commits/tests only — the finding detail lives in the
> issue, not here. Review-sweep trackers: #269-#289 (see docs/temp/LT_C01_HANDOFF.md).
>
> **2026-08-15 second pass (2026-08-14 flight-readiness reviews):** the FR wave
> closed #282, #283, #290, #292-#296 (commits `fd5262d`/`25a35a4`/`3b800d7`; host
> red-first, ARM +368 B flash / +2048 B bss). The second 2026-08-14 review
> (GEO/PWR/MAC) triaged to: #297 (PWR-02, bench-gated P0 — see FW-CONF-005),
> #298 (MAC-01, post-flight), #299 (GEO-05, post-flight), #257 + h3lite#1
> (GEO-01/04 dataset + guards, owner), #258 (GEO-03 disposition: no rework,
> recorded). New gated host suites: `test_pwr`, `test_geo` (EXPECT_UNFIXED until
> the bench data / dataset land).

## P0/P1 — Mission correctness

### FW-CONF-001 — Science scheduler preempts archive recovery
**Intent:** DDR-0022 / DDR-0005.  
**Proof:** Large backlog + excellent link + accelerated run; fresh science interval stays inside allowed cadence/jitter.
**Status:** 🟡 IMPLEMENTED — absolute phase-preserving science deadline `g_science_due_ms` (R3-01/#215), LT-01 underflow fix (#269, host sweep in `test_lt_20260813.c`), burst deadlines (LT-07/#277). Accelerated-run proof open.

### FW-CONF-002 — GNSS wake failure cannot report prior fix as fresh
**Intent:** DDR-0003 / DDR-0009 / DDR-0023.  
A failed acquisition may yield stale last-known fix or unavailable position, never old coordinates marked fresh.
**Status:** 🟡 IMPLEMENTED — R28/#136 provenance flags; caveat: weak-fix promotion gap → #284 (H-08).

### FW-CONF-003 — Archive recovery follows newest-first intent
**Intent:** DDR-0004 / DDR-0005.  
Newest eligible retained record first, then backward under bounded opportunity.
**Status:** 🔴 OPEN — fresh-outage backlog drains oldest-first → #286 (H-10).

### FW-CONF-004 — Fresh science wins a one-packet budget
**Intent:** DDR-0022 / DDR-0005.
**Status:** 🟡 IMPLEMENTED — `ScienceIsDue()` yield in `SendTxData` (BR-TX-001/002); host burst suite. Behavioural proof open.

## P1 — Power predictability

### FW-CONF-005 — Predictive energy degradation, not intentional brownout
**Intent:** DDR-0016.  
Reduce work/cadence before an admitted load collapses the rail.
**Status:** 🟡 IMPLEMENTED — `DecideTransmitPlan` + `power_model.c` predictions; caveats: −40→−50 compensation-table non-monotonicity #248 (LT-06), FULL/SLEEP admission gap #288 (H-12), cold full-pack false SURVIVAL #297 (PWR-02: fixed 4300 mV raw floor fires below −62.3 °C; red-gated `test_pwr`; bench-derived floor(T) pending #261).

### FW-CONF-006 — GNSS load admission
If GNSS cannot be supported, skip the whole science cycle and retry later.
**Status:** 🟡 IMPLEMENTED — fresh temperature/raw-battery admission is FULL or SLEEP; all admitted planner modes carry a nonzero GNSS budget. Hardware droop proof remains open.

### FW-CONF-007 — Automatic return toward configured target
Temporary power adaptation never rewrites the operator's nominal target.
**Status:** 🟡 IMPLEMENTED — plan recomputed per cycle from config; sawtooth fixed (RV-08/#208); caveat: trend state lost on reset → #288 (H-12).

## P1 — Persistence / recovery

### FW-CONF-008 — Reflash preserves identity and credentials
Provision, record DevEUI/credential checks, reflash, verify unchanged.
**Status:** 🟡 IMPLEMENTED — credentials/identity in RTC backup registers (STAB-11/#158), provisioning latch in Tier-1 flash (#270). Reflash proof open.

### FW-CONF-009 — Counters advance safely and never illegally reuse
Power-cut around persistence checkpoints.
**Status:** 🟡 IMPLEMENTED — FCnt persistence + NVM store every 10 TX (#109); restore now reads config after `Config_Init` (H-02/#273). Target power-cut proof open; NVM slot store atomicity → #282 (H-03).

### FW-CONF-010 — Metadata corruption does not unnecessarily abandon recoverable archive science
Reconstruct from valid self-checking records where possible.
**Status:** 🟡 IMPLEMENTED — header reconstruction from records (R3-07/#221); caveat: double-header loss triggers a full-ring boot scan → #289 (M-01).

## P1/P2 — Fault handling

### FW-CONF-011 — Fatal exception path resets
Inject HardFault/fatal assertion/deadman and prove deterministic reset + clean boot.
**Status:** 🟡 IMPLEMENTED — per-code degrade escape (STAB-02/#149), deadman supervision (DDR-0020); host stability suite. Injection proof open.

### FW-CONF-012 — Sensor-specific bounded retries
Each driver defines retryable faults, count/time budget, and optional bus/peripheral recovery. No infinite local recovery.
**Status:** 🟡 IMPLEMENTED — R28/#136 staleness/bounded-retry treatment per driver; STOP2 wake re-init failures now degrade via capabilities (LT-05/#275). Per-driver audit open.

### FW-CONF-013 — Surprising sensor values are not silently censored
Only explicit scientific-product rules may reject a protocol-valid measurement.
**Status:** 🟡 IMPLEMENTED — stale flags preserve the value; caveat: solar channel has no gate/flag at all → #279 (LT-09).

## P2 — RF / protocol

### FW-CONF-014 — Stale-position RF legality timeout
Prove RF silence at policy boundary, continued science/logging, and automatic recovery on fresh GNSS.
**Status:** 🟡 IMPLEMENTED — 24 h silence enforced; caveats: same-wake recovery gap → #285 (H-09); boundary behavioural test open (FW-CONF-023).

### FW-CONF-015 — FPort is authoritative product discriminator
Maintain an explicit allocation table in the protocol binding.
**Status:** ✅ DONE (document-side) — allocation lives in `../PayloadFormats.md` + DDR-0019 binding.

### FW-CONF-016 — Decoder compatibility is explicit
Incompatible changes require a new FPort, schema/version field, or another documented discriminator.
**Status:** 🟡 IMPLEMENTED — DDR-0027 policy + versioned wire formats; next payload bump tracked in #266 (and #279's solar bit batches with it).

## P2 — Calibration / data

### FW-CONF-017 — Preserve raw sensor observable where practical
Backend calibration binds to device/sensor identity.
**Status:** 🟡 IMPLEMENTED — records carry raw observables (mV, raw counts) with provenance flags; backend binding is ground-side work, open.

### FW-CONF-018 — Firmware calibration only where local behavior needs it
Avoid duplicating backend-only correction logic without mission need.
**Status:** 🟡 IMPLEMENTED — only the battery temperature-compensation table drives local behavior (power decisions); its −40/−50 values await bench data → #248 (LT-06).

## P2 — Verification infrastructure

### FW-CONF-019 — First target HIL lane
Build box controls build, ST-Link flash/reset, RTT capture, Otii power/current capture, pass/fail parser, artifact retention.
**Status:** 🔴 OPEN — no HIL lane in-tree; release-evidence gap tracked in #261/#262 (M-03).

### FW-CONF-020 — Power-cut campaign
Randomize interruption around archive write, metadata/header update, LoRaWAN counter persistence, and session/config writes.
**Status:** 🔴 OPEN — blocked on FW-CONF-019; NVM slot atomicity finding → #282 (H-03).

### FW-CONF-021 — RTT equivalence
Representative scenarios with RTT enabled and quiet/off; compare mission decisions and characterize power/timing effect.
**Status:** 🟡 IMPLEMENTED (partial) — flight builds compile all debug blocks out (R2-15/#119, CI flight-build check); the equivalence *campaign* is open.

### FW-CONF-022 — Cold-chamber characterization
Measure temperature/load/droop behavior for sleep, GNSS, radio TX, and recovery bursts; feed results back into DDR-0016.
**Status:** 🔴 OPEN — also feeds the #248 (LT-06) compensation-table bench data.

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
**Status:** 🟡 IMPLEMENTED — `GPS_LOSS_SILENCE_S = 24U*3600U` verified (lora_app.h:206), source-scan regression exists; the behavioural boundary walk is still the open proof.

### FW-CONF-024 — One valid fix clears staleness silence on the same wake
**Intent:** DDR-0015 `BR-STALE-019` · **Proof:** `P-STALE-015`.  
No dwell, no re-qualification, no operator action.
**Status:** 🔴 OPEN — RF policy is evaluated before GNSS acquisition, so a fresh fix clears silence only on the next wake → #285 (H-09).

### FW-CONF-025 — No independent RTC-age RF cutoff exists
**Intent:** DDR-0015 `BR-STALE-020`, DDR-0013 `INV-TIME-009` · **Proof:**
`P-STALE-016`, `P-TIME-010`.  
Audit: prove no second time-based silence timer is reachable.
**Status:** ⚪ NOT ASSESSED this pass — the 2026-08-13 stability review found only the GPS-loss silence path, but the formal audit is not written up.

### FW-CONF-026 — Band 0 home-region fallback
**Intent:** DDR-0015 `INV-STALE-008`, `BR-STALE-013..016` · **Proof:**
`P-STALE-011..013`.  
Flight begins with no accepted fix: position invalid/degraded and never promoted to
last-known-good; `home_region` RF permitted within budget; silence after; first real fix
supersedes immediately. **Not yet implemented** — `home_region` provisioning does not
exist in code.
**Status:** 🔴 OPEN — tracked in #287 (H-11); note the loss epoch is also reset-unstable pre-UTC (same issue).

## P0/P1 — scheduling and energy

### FW-CONF-027 — Fixed start-to-start scheduler
**Intent:** DDR-0001 `BR-WAKE-017` · **Proof:** `P-WAKE-011`.  
Variable wake duration must not accumulate cadence drift.
**Status:** 🟡 IMPLEMENTED — phase-preserving absolute deadline (`g_science_due_ms`, R3-01/#215); LT-01 sweep (#269) covers deadline/underflow edges; the blind TxTimer re-arm (an early-fire path) is removed (LT-07/#277).

### FW-CONF-028 — FULL/SLEEP per-wake admission
**Intent:** DDR-0001 `BR-WAKE-018`, DDR-0016 `BR-PWR-018` · **Proof:** `P-WAKE-012`,
`P-PWR-013`.  
Includes the negative case: prove no code path *deliberately* plans a GNSS-less wake for
energy reasons. Touches `power_model.c` (divergence 3).
**Status:** 🟡 IMPLEMENTED — first-flight admission rejects stale/invalid or below-threshold temperature/battery inputs at `tx_interval_survival`; every admitted science wake enables GNSS with a nonzero budget. Host policy tests exist; hardware droop proof remains open.

### FW-CONF-029 — Durable battery/energy trend
**Intent:** DDR-0016 `BR-PWR-016` · **Proof:** `P-PWR-015`.  
Reset must not restart cadence adaptation from a neutral assumption.
**Status:** 🔴 OPEN — voltage trend state is function-static RAM → #288 (H-12).

### FW-CONF-030 — Complete package or no new science record
**Intent:** first-flight maintainer decision superseding DDR-0001 `BR-WAKE-003` for flight one · **Proof:** `P-WAKE-013`.
An admitted wake that cannot assemble time + fresh good location + fresh readings
becomes SLEEP and creates no new science record.
**Status:** 🟡 IMPLEMENTED — incomplete packages end without archive/live TX; cached bulk recovery remains separate. End-to-end hardware proof remains open.

## P1 — reset and persistence

### FW-CONF-031 — Reset never fabricates an observation
**Intent:** DDR-0010 `INV-PERSIST-011` · **Proof:** `P-PERSIST-010`.  
Inject reset before/among sensors, after GNSS, during archive commit, and after commit
before TX. Every backend-visible record is complete-and-valid or absent.
**Status:** 🟡 IMPLEMENTED — provenance/stale flags make partial records honest; backend-visible records carry veto bits. The reset-injection matrix itself is open (blocked on FW-CONF-019/020).

### FW-CONF-032 — Transient reset never becomes permanent telemetry loss
**Intent:** DDR-0010 `INV-PERSIST-012` · **Proof:** `P-PERSIST-011/012`.  
Also prove no reset creates a new mission identity or restarts record numbering.
**Status:** 🟡 IMPLEMENTED — identity/credentials survive reset via backup regs (STAB-11/#158) and Tier-1 bank (#270); MAC NVM store survives reset (#109); deadman (DDR-0020) bounds wedges. NVM slot-store atomicity caveat → #282 (H-03).

### FW-CONF-033 — Float latch survives reset
**Intent:** DDR-0002 `BR-LIFE-024` · **Proof:** `P-LIFE-013`, `P-BOOT-012`.
**Status:** 🟡 IMPLEMENTED — mission state persists in the RTC backup domain; caveat: backup-domain loss can invent ASCENT on the ground → #281 (H-01).

### FW-CONF-034 — Cadence survives reset; transient timer may not
**Intent:** DDR-0010 `INV-PERSIST-010` · **Proof:** `P-BOOT-013`.  
No rapid reset/wake loop, no unintended multi-day sleep; at most one shifted observation.
**Status:** 🟡 IMPLEMENTED — cadence is recomputed from config each boot; the science deadline re-establishes phase on first wake (R3-01/#215). Reset-loop proof open.

## P1 — storage

### FW-CONF-035 — Torn record is skipped, never truncating
**Intent:** DDR-0011 `BR-STORE-001` · **Proof:** `P-STORE-011`.  
A/B valid, C torn, D/E valid: A/B and D/E all readable, iteration reports one gap.
**Status:** 🟡 IMPLEMENTED — per-record CRC with explicit corrupt-skip accounting (F-006/#51); host flight suite covers torn/corrupt records. Target proof open.

### FW-CONF-036 — No routine archive scan on a valid-cursor boot
**Intent:** DDR-0011 `BR-STORE-002`, DDR-0012 `INV-BOOT-010` · **Proof:** `P-STORE-014`,
`P-BOOT-014`.
**Status:** 🟡 IMPLEMENTED — valid headers give a fast-path init (R3-07/#221); the invalid-headers path is the M-01 gap → #289.

### FW-CONF-037 — Brownout-loop-safe boot and deferred reconstruction
**Intent:** DDR-0011 `BR-STORE-002` · **Proof:** `P-STORE-012/013`.
**Status:** 🔴 OPEN — double-header loss currently forces a synchronous full-ring scan at boot → #289 (M-01).

### FW-CONF-038 — No mandatory read-back after a successful science write
**Intent:** DDR-0011 `BR-STORE-003` · **Proof:** inspection + write-path timing.
**Status:** ⚪ NOT ASSESSED this pass.

## P1/P2 — recovery

### FW-CONF-039 — Bounded cross-subsystem recovery, immediate normal service on success
**Intent:** DDR-0009 `BR-FAIL-016/017` · **Proof:** `P-FAIL-011`.  
Extends FW-CONF-012 beyond sensors to GNSS, radio hardware, radio stack, and buses. No
probation state.
**Status:** 🟡 IMPLEMENTED — `SysCaps` capability degrade/restore (incl. region-switch rollback #272, STOP2 re-init #275); injection proof open.

### FW-CONF-040 — Radio recovery never triggers an in-flight join
**Intent:** DDR-0009 `BR-FAIL-018`, DDR-0018 `INV-COMM-001` · **Proof:** fault injection
on radio/stack reset paths.
**Status:** 🟡 IMPLEMENTED — the one-way flight door forbids joining post-provisioning (C-01/#270); failed switches roll back rather than rejoin (#272). Fault-injection proof open.

### FW-CONF-041 — No reset-count or failure-history escalation anywhere
**Intent:** DDR-0009 `OD-FAIL-006` resolved, DDR-0020 · **Proof:** `P-FAIL-012`.  
Audit for any repeated-reset adaptation and remove/disable it for first flight.
**Status:** 🟡 IMPLEMENTED — boot-attempt counter resets on a completed work cycle; the FR-23/#104 degrade escape replaces the 6th reset. Formal audit open.

## P2 — archive delivery and gap repair

### FW-CONF-042 — Arbitrary retained-ID lookup
**Intent:** DDR-0004 `BR-ARCH-018` · **Proof:** `P-ARCH-011`.  
Linear search acceptable; must not require ordered or near-cursor access.
**Status:** 🔴 OPEN — no arbitrary-ID lookup API exists in `flash_log.h` (verified 2026-08-15); part of the unimplemented explicit-request feature set (see FW-CONF-044).

### FW-CONF-043 — Requested-ID-unavailable substitution
**Intent:** DDR-0004 `BR-ARCH-019` · **Proof:** `P-ARCH-012`.  
Returned record's own ID is authoritative; no separate status field.
**Status:** 🔴 OPEN — same unimplemented feature set as FW-CONF-042.

### FW-CONF-044 — Explicit request preempts backfill, and is ephemeral
**Intent:** DDR-0005 `BR-TX-023/024` · **Proof:** `P-TX-011/012`.  
Prove **no** persistent request queue survives sleep or reset.
**Status:** 🔴 OPEN — no downlink explicit-request handling exists yet (OnRxData parses nothing), so there is no queue to persist; the feature and its proof are both open.

### FW-CONF-045 — Request never overrides legality or energy
**Intent:** DDR-0005 `BR-TX-025` · **Proof:** `P-TX-013`.
**Status:** 🔴 OPEN — same unimplemented feature set as FW-CONF-042.

### FW-CONF-046 — Record shape is stable under partial failure
**Intent:** DDR-0004 `BR-ARCH-017` · **Proof:** `P-ARCH-013`.
**Status:** 🟡 IMPLEMENTED — fixed-size 64 B records; provenance/veto flags (b0-b7) carry degradation without changing shape.

## P2 — lifecycle and commissioning

### FW-CONF-047 — Float latch is one-way in flight
**Intent:** DDR-0002 `BR-LIFE-023/025` · **Proof:** `P-LIFE-012`.  
Post-float pressure excursions must not restore ascent cadence.
**Status:** 🟡 IMPLEMENTED — one-way FLOAT latch in host-tested `mission_logic.c`; post-float excursions cannot re-enter ASCENT.

### FW-CONF-048 — Automatic launch without the operator action
**Intent:** DDR-0002 `BR-LIFE-027`, DDR-0018 `BR-COMM-020` · **Proof:** `P-LIFE-014`,
`P-COMM-013`.
**Status:** 🟡 IMPLEMENTED — autonomous launch detection (6 hPa cumulative drop) gated on the durable provisioning latch (C-01/#270). Target proof open.

### FW-CONF-049 — Readiness check uses the normal GNSS predicate
**Intent:** DDR-0003 `BR-GNSS-021`, DDR-0018 `BR-COMM-018` · **Proof:** `P-GNSS-011`,
`P-COMM-011/012`.  
No weaker commissioning-only fix criterion; no false-positive ready indication.
**Status:** 🟡 IMPLEMENTED — readiness uses the GNSS quality predicate; caveat: the predicate is not yet the single authoritative one everywhere → #284 (H-08).

### FW-CONF-050 — Provisioning cannot complete with a missing region session
**Intent:** DDR-0018 `BR-COMM-017` · **Proof:** `P-COMM-014`.
**Status:** 🟡 IMPLEMENTED — `VerifyAndSetProvisioningLatch` requires a valid session bank in all 7 regions, verified by flash read-back (C-01/#270); host multiregion suite includes the partial-bank rejection.

### FW-CONF-051 — Placeholder position cannot masquerade as a fix
**Intent:** DDR-0003 §17 · **Proof:** `P-GNSS-012`.
**Status:** 🟡 IMPLEMENTED — validity gates on placeholder/sentinel coordinates; caveat: sub-threshold-quality fixes can still be promoted → #284 (H-08).

## P2 — configuration and protocol

### FW-CONF-052 — Unsupported downlink is safe
**Intent:** DDR-0014 `BR-CONFIG-014` · **Proof:** `P-CONFIG-011`.  
Fuzz arbitrary downlink content: no crash, reset, or memory corruption.
**Status:** 🟡 IMPLEMENTED (partial) — no application downlink is parsed today (`OnRxData` only logs port/size), so there is no parse surface; the fuzz proof stays open for when downlink handling lands (FW-CONF-042..045).

### FW-CONF-053 — Persisted operational config is repaired, not fatal
**Intent:** DDR-0014 `BR-CONFIG-015/016` · **Proof:** `P-CONFIG-010`.  
Credentials explicitly excluded from fail-soft repair.
**Status:** 🟡 IMPLEMENTED — `Config_Load` CRC failure falls back to defaults and re-saves (config.c); credentials live in backup registers, outside the repair path (STAB-11/#158).

### FW-CONF-054 — Multi-version backend codec regression suite
**Intent:** DDR-0027 `BR-PROTO-001/002/005` · **Proof:** `P-PROTO-001..003`.  
Ground/backend work: every deployed format decodes; adding a version does not regress
old vectors; retransmitted historical records recover the same logical ID.
**Status:** 🔴 OPEN — ground-side suite; the next wire change batches in #266 (and #279's solar bit).

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
