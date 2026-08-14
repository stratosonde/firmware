# Stratosonde Flight 1 Validation and Readiness Checklist

**Status:** Draft skeleton  
**Purpose:** Explicit go/no-go evidence cycle for the first-flight firmware/hardware build.

---

# 1. Release Identity

- [ ] Firmware commit/tag recorded
- [ ] Flight binary hash recorded
- [ ] Hardware revision recorded
- [ ] Device DevEUI / hardware identity recorded
- [ ] Effective mission configuration exported and archived
- [ ] DDR/requirements repository revision recorded
- [ ] Payload/FPort decoder version recorded

---

# 2. Build and Static Integrity

- [ ] Flight target builds successfully
- [ ] Host/unit regression suite passes
- [ ] Compiler warnings reviewed
- [ ] Linker layout confirms provisioning/NVM cannot overlap executable application
- [ ] Flight-build flags reviewed
- [ ] No unintended test-only mission behavior enabled

---

# 3. Mission-Cycle Behavior

- [ ] Cold boot converges to ordinary wake behavior
- [ ] Watchdog reset converges to ordinary wake behavior
- [ ] HardFault/fatal path resets rather than hangs
- [ ] Wake operations are bounded
- [ ] Optional/app work cannot indefinitely delay sleep
- [ ] Fresh science cannot be starved by archive recovery
- [ ] Large backlog still preserves current-science cadence

---

# 4. GNSS

- [ ] Valid fix produces fresh position
- [ ] GNSS timeout produces stale/unavailable semantics
- [ ] GNSS wake/start failure cannot reuse prior position as fresh
- [ ] GNSS power/load denial leaves honest stale/unavailable provenance
- [ ] RF continues only inside stale-position regulatory window
- [ ] RF stops after stale-position limit
- [ ] Fresh valid fix automatically restores RF eligibility

---

# 5. Sensors and Science Truth

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

- [ ] Circular fill/wrap passes
- [ ] Oldest record overwritten when full
- [ ] New science continues when full
- [ ] Recovery is newest-first
- [ ] Current science preempts recovery
- [ ] Torn newest record detected
- [ ] Corrupt metadata/header recovery preserves valid records where feasible
- [ ] Power interruption causes only bounded loss

---

# 7. LoRaWAN Persistence

For each supported region:

- [ ] Provisioned credentials valid
- [ ] Session/credential persistence verified
- [ ] Frame-counter checkpoint/recovery verified
- [ ] Reset cannot cause prohibited frame-counter rollback/reuse
- [ ] Region switch does not damage other region state
- [ ] Ordinary application reflash preserves required credential state

---

# 8. Configuration

- [ ] Effective default config documented
- [ ] Min/max clamping verified
- [ ] Corrupt config rejected/recovered safely
- [ ] Config change logged as an event
- [ ] Config change takes effect at next wake boundary
- [ ] Runtime energy adaptation does not rewrite target config
- [ ] Same semantic validation path used by every enabled config source

---

# 9. Energy / Power

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

- [ ] Cold startup tested at required first-flight temperature
- [ ] Sleep current characterized cold
- [ ] GNSS load/droop characterized cold
- [ ] Radio TX load/droop characterized cold
- [ ] Sensor behavior characterized cold
- [ ] Low-energy recovery characterized
- [ ] Power thresholds/model reviewed against measurements

---

# 11. Identity / Commissioning / Ownership

- [ ] Device DevEUI unique and recorded
- [ ] PCB QR decodes to intended backend identity
- [ ] Claim PIN/random secret present according to production decision
- [ ] Public QR material exposes no LoRaWAN secret keys
- [ ] Commissioning completes for required regions
- [ ] Device can be associated with backend account
- [ ] Already-claimed device cannot silently transfer
- [ ] Backend mission record remains separate from permanent device identity

---

# 12. Firmware Service

- [ ] Flight image contains no OTA executable-update path
- [ ] ST-Link/SWD reflash procedure documented
- [ ] Ordinary reflash preserves DevEUI
- [ ] Ordinary reflash preserves required LoRaWAN credentials
- [ ] Factory reprovision, if available, is distinct from ordinary reflash

---

# 13. Endurance / Fault Campaign

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

- [ ] Every known P0/P1 issue closed or explicitly declared no-go
- [ ] Every accepted waiver has owner/rationale
- [ ] Decoder/backend compatibility confirmed

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
