# Stratosonde Flight 1 Validation and Readiness Checklist

**Status:** Living go/no-go instrument (annotated 2026-08-15 against master @ `9fdcccc`, h3lite `5480859`)  
**Purpose:** Explicit go/no-go evidence cycle for the first-flight firmware/hardware build.

> **Convention (2026-08-15):** a box is checked only when the evidence artifact exists
> and is recorded against a specific build (§1 filled first). Standing CI/host evidence
> is annotated per item as `— standing:`; items needing target/bench/backend evidence
> are annotated `— open:` with the tracker. Current failures are annotated
> `— FAILS today:` with the issue. Annotations name suites/issues only; detail lives in
> the tracker. Review-sweep trackers: #269-#289.

---

# 1. Release Identity

Filled at release-candidate time; none of these exist until a candidate is cut (#261/#262).

- [ ] Firmware commit/tag recorded
- [ ] Flight binary hash recorded
- [ ] Hardware revision recorded
- [ ] Device DevEUI / hardware identity recorded
- [ ] Effective mission configuration exported and archived
- [ ] DDR/requirements repository revision recorded
- [ ] Payload/FPort decoder version recorded

---

# 2. Build and Static Integrity

- [ ] Flight target builds successfully — standing: CI `firmware-flight` job green on every push (re-record against the candidate)
- [ ] Host/unit regression suite passes — standing: CI `host-tests` job, 13 suites + ASan/UBSan build, green @ `9fdcccc` (run 31923381786)
- [ ] Compiler warnings reviewed — open: #268 (first-party `-Wall -Wextra` cleanliness)
- [ ] Linker layout confirms provisioning/NVM cannot overlap executable application — open: no recorded inspection artifact; the layout to check is the Tier-1 session bank (#270) + backup-register credentials (#158)
- [ ] Flight-build flags reviewed — standing: CI flight-build marker check (ci.yml)
- [ ] No unintended test-only mission behavior enabled — standing: `SONDE_FLIGHT_BUILD` gates debug blocks out (R2-15/#119), checked by CI

---

# 3. Mission-Cycle Behavior

- [ ] Cold boot converges to ordinary wake behavior — host: boot-resilience suite (#107); open: target run
- [ ] Watchdog reset converges to ordinary wake behavior — code: deadman supervision (DDR-0020); open: target run
- [ ] HardFault/fatal path resets rather than hangs — code: STAB-02/#149; open: fault injection (HIL)
- [ ] Wake operations are bounded — code: LT-05/#275, LT-07/#277; open: formal audit + target timing
- [ ] Optional/app work cannot indefinitely delay sleep — open: no application work exists yet (DDR-0017 stage-3); revisit when Qwiic services land
- [ ] Fresh science cannot be starved by archive recovery — host: LT-01 sweep (#269), burst suite (#215); open: accelerated target run
- [ ] Large backlog still preserves current-science cadence — host: burst suite + absolute deadline (#215); open: accelerated target run

---

# 4. GNSS

- [ ] Valid fix produces fresh position — host: GNSS/mission suites; open: target run
- [ ] GNSS timeout produces stale/unavailable semantics — code: R28/#136 provenance; caveat: weak-fix promotion → #284
- [ ] GNSS wake/start failure cannot reuse prior position as fresh — code: LT-04/#276 teardown; caveat → #284
- [ ] GNSS power/load denial sleeps without creating a new science record — code: FULL/SLEEP admission + complete-package gate; open: target load test
- [ ] RF continues only inside stale-position regulatory window — code: 24 h budget verified (`lora_app.h:206`); open: boundary behavioural test (FW-CONF-023)
- [ ] RF stops after stale-position limit — code: 24 h silence enforced; open: target boundary run
- [ ] Fresh valid fix automatically restores RF eligibility — host: `gpsloss` suite (A6/A7, #285 closed 2026-08-15): same-wake accepted fix clears only the GPS-loss veto, before region selection and TX; open: target run

---

# 5. Sensors and Science Truth

Section note (2026-08-15): host suites cover SHT31/MS5607/GNSS/battery provenance and
bounded retries (R28/#136); the solar channel has no gate/flag → #279. The per-sensor
target sweep is open (HIL, #261/#262).

For each first-flight sensor:

- [ ] Normal read succeeds
- [ ] Bus/status failure is detected
- [ ] Retry behavior is bounded
- [ ] Sensor failure does not block unrelated sensors
- [ ] Protocol-valid surprising values are preserved
- [ ] Known stale/substituted data is identifiable
- [ ] Raw/fundamental observable preserved where required
- [ ] Payload units/schema documented

---

# 6. Archive

- [ ] Circular fill/wrap passes — host: flight suite
- [ ] Oldest record overwritten when full — host: flight suite
- [ ] New science continues when full — host: flight suite
- [ ] Recovery is newest-first — **FAILS today** → #286 (H-10): a fresh-outage backlog drains oldest-first
- [ ] Current science preempts recovery — host: burst suite + LT-01/#269
- [ ] Torn newest record detected — host: flight suite (F-006/#51)
- [ ] Corrupt metadata/header recovery preserves valid records where feasible — host: R3-07/#221; caveat: double-header loss triggers a full-ring boot scan → #289
- [ ] Power interruption causes only bounded loss — open: power-cut campaign (store atomicity fixed: #282 closed 2026-08-15, FR-02 `3b800d7`; campaign evidence still open)

---

# 7. LoRaWAN Persistence

For each supported region:

- [ ] Provisioned credentials valid — commissioning-time procedure; open
- [ ] Session/credential persistence verified — host: multiregion suite + NVM store (#109); open: target reset matrix
- [ ] Frame-counter checkpoint/recovery verified — code: #109 + restore-after-`Config_Init` (H-02/#273); open: target power-cut
- [ ] Reset cannot cause prohibited frame-counter rollback/reuse — code: #273; open: power-cut campaign (#282)
- [ ] Region switch does not damage other region state — host: multiregion suite; code: rollback on failure (#272); ~~caveat: middleware `CtxRestoreDone` stickiness → #283~~ closed 2026-08-15 (FR-16, `3b800d7`); new caveat: mask-MIB failure handling verified (FR-17, `fd5262d`)
- [ ] Ordinary application reflash preserves required credential state — code: backup registers (STAB-11/#158); open: automated reflash proof

---

# 8. Configuration

- [ ] Effective default config documented — standing: `../ConfigurationModule.md`; caveat: advertised-but-unused fields → #280 (M-02)
- [ ] Min/max clamping verified — host: config suite (verified in `config.c` 2026-08-15)
- [ ] Corrupt config rejected/recovered safely — host: config suite (CRC → defaults → re-save)
- [ ] Config change logged as an event — ⚪ not assessed this pass
- [ ] Config change takes effect at next wake boundary — code: next-wake apply (INV-CONFIG-014)
- [ ] Runtime energy adaptation does not rewrite target config — host: config suite
- [ ] Same semantic validation path used by every enabled config source — trivially true today: only one config source exists (no downlink config yet); revisit when DDR-0014 remote commands land

---

# 9. Energy / Power

Section note (2026-08-15): every box here is bench/target measurement — all open. The
power model itself is host-tested; its −40/−50 compensation values await bench data
(#248) and the FULL/SLEEP admission redesign is #288.

- [ ] Sleep current measured
- [ ] Sensor-only wake energy measured
- [ ] GNSS acquisition load measured
- [ ] LoRa TX load measured
- [ ] Archive recovery burst load measured
- [ ] Low-energy cadence adaptation observed
- [ ] GNSS denied before predicted rail collapse
- [ ] Critical-energy path returns directly to sleep when required
- [ ] Policy returns toward nominal as energy recovers
- [ ] No unacceptable policy chatter

---

# 10. Cold Environment

Section note (2026-08-15): all open — cold-chamber campaign (#261/#262); results feed the
#248 compensation table and the DDR-0016 energy model.

- [ ] Cold startup tested at required first-flight temperature
- [ ] Sleep current characterized cold
- [ ] GNSS load/droop characterized cold
- [ ] Radio TX load/droop characterized cold
- [ ] Sensor behavior characterized cold
- [ ] Low-energy recovery characterized
- [ ] Power thresholds/model reviewed against measurements
- [ ] Cold full-pack mode selection verified against flight-pack data — **FAILS today:** #297 (PWR-02: fixed 4300 mV raw floor selects SURVIVAL below −62.3 °C for a fully charged pack; red-gated host suite `test_pwr`; fix = bench-derived floor(T) + monotonic comp table from the #261 campaign, also ungates LT-06/#248)
- [ ] RESTRICTED enforcement set present and probed — open: dataset population #257 (owner), init/generator guards h3lite#1 (GEO-04); red-gated host suite `test_geo`

---

# 11. Identity / Commissioning / Ownership

- [ ] Device DevEUI unique and recorded — open: production procedure
- [ ] PCB QR decodes to intended backend identity — open: production procedure
- [ ] Claim PIN/random secret present according to production decision — open: decision pending (OD-ID-001)
- [ ] Public QR material exposes no LoRaWAN secret keys — standing (design): credentials live in backup registers (#158), never in QR material; confirm against the production QR at release
- [ ] Commissioning completes for required regions — code: 7-region prejoin + verified provisioning latch (C-01/#270); open: physical commissioning runs
- [ ] Device can be associated with backend account — open: backend
- [ ] Already-claimed device cannot silently transfer — open: backend
- [ ] Backend mission record remains separate from permanent device identity — open: backend

---

# 12. Firmware Service

- [ ] Flight image contains no OTA executable-update path — standing (inspection): no OTA surface exists (DDR-0025); re-verify on the release binary
- [ ] ST-Link/SWD reflash procedure documented — ⚪ not assessed this pass
- [ ] Ordinary reflash preserves DevEUI — code: backup registers (#158); open: automated proof
- [ ] Ordinary reflash preserves required LoRaWAN credentials — code: #158; open: automated proof
- [ ] Factory reprovision, if available, is distinct from ordinary reflash — code: provisioning latch (#270) separates provisioning from service reflash; open: procedure doc

---

# 13. Endurance / Fault Campaign

Section note (2026-08-15): all open — these are the HIL/soak campaigns (#261/#262). The
review's fault matrix (T-01..T-15 in docs/temp/stratosonde-firmware-stability-review-5930294.md)
is the scenario seed list.

- [ ] Repeated wake/sleep soak completed
- [ ] Repeated random resets completed
- [ ] Power-cut campaign completed at key write boundaries
- [ ] Extended no-coverage scenario completed
- [ ] Large-backlog reconnection scenario completed
- [ ] Permanent single-sensor failure scenario completed
- [ ] GNSS-unavailable scenario completed
- [ ] No unexpected permanent mission-stop state observed

---

# 14. Open Risk Review

For each unresolved first-flight issue:

```text
Risk ID:
Requirement:
Observed gap:
Consequence:
Mitigation:
Why acceptable for this flight:
Owner:
```

- [ ] Every known P0/P1 issue closed or explicitly declared no-go — sweep P0/P1 items closed (#269-#274); the wave-2 priority:high items (#282, #284-#288) must be closed or waived here before GO
- [ ] Every accepted waiver has owner/rationale — open
- [ ] Decoder/backend compatibility confirmed — open: next wire bump (#266, with #279's solar bit)

---

# 15. Flight Decision

**Firmware build:**  
**Hardware:**  
**Configuration:**  
**Date:**  

- [ ] GO
- [ ] NO-GO

Decision rationale:

```text

```
