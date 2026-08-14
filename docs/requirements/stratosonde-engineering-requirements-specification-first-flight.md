# Stratosonde Engineering Requirements Specification — First-Flight Baseline

**Document type:** Derived engineering requirements specification  
**Status:** Draft baseline from intent interviews and DDR reconciliation  
**Date:** 2026-08-12

This document is derived from the DDR corpus. If it conflicts with an accepted DDR, the DDR is intent-authoritative until explicitly reconciled.

**SHALL** = required. **SHOULD** = strongly desired. **MAY** = permitted.

---

# 1. Mission and Lifecycle

### SYS-MSN-001
The system SHALL operate autonomously without ground connectivity for ordinary mission execution.

### SYS-MSN-002
The system SHALL continue mission cycles until physical power/hardware conditions prevent continued operation.

### SYS-MSN-003
The system SHALL NOT contain a normal software mission-complete state that permanently stops an otherwise viable sonde.

### SYS-MSN-004
Fresh/current science SHALL take priority over historical recovery, diagnostics, and optional payload traffic.

### SYS-MSN-005
Loss of one subsystem SHALL remove only dependent capabilities wherever safe continuation remains possible.

### SYS-MSN-006
The lifecycle SHALL distinguish at least pre-commissioning/manufacturing, commissioning, pre-flight/armed, ascent, and float.

### SYS-MSN-007
A never-commissioned device SHALL NOT silently behave as a commissioned flight unit merely because power is applied.

---

# 2. Boot, Reset, and Wake

### SYS-BOOT-001
Cold boot, watchdog reset, fault reset, and ordinary low-power wake SHALL converge into one ordinary mission-cycle orchestration after minimum required startup processing.

### SYS-BOOT-002
Startup SHALL establish a safe known low-power hardware state before enabling expensive subsystems.

### SYS-BOOT-003
Startup SHALL restore/validate persistent mission state required for ordinary operation.

### SYS-BOOT-004
Startup SHALL evaluate energy/power capability before enabling GNSS, radio, or optional high-cost work.

### SYS-BOOT-005
After reset, firmware SHALL abandon the interrupted wake and begin a fresh ordinary mission cycle.

### SYS-BOOT-006
Reset cause MAY be recorded diagnostically but SHALL NOT normally select persistent alternate mission policy.

### SYS-BOOT-007
The watchdog SHALL be active during flight operation and MAY be relaxed/disabled during commissioning.

### SYS-BOOT-008
Fatal core faults such as HardFault SHALL result in deterministic reset rather than indefinite hang.

---

# 3. Wake Timing and Resource Authority

### SYS-ORCH-001
The Stratosonde core SHALL be authoritative owner of wake timing, power admission, radio admission, and application resource grants.

### SYS-ORCH-002
Every peripheral or application operation SHALL have a deterministic completion bound.

### SYS-ORCH-003
No peripheral or application SHALL indefinitely extend the wake cycle.

### SYS-ORCH-004
Independent sensor/GNSS/application activity SHOULD be overlapped where safe and useful to minimize awake time.

### SYS-ORCH-005
Applications MAY request time, power, or radio service; core SHALL decide whether to grant it.

### SYS-ORCH-006
Optional application work SHALL NOT indefinitely preempt the scheduled low-power transition.

---

# 4. Energy Management

### SYS-PWR-001
Brownout SHALL be treated as a fault condition, not a normal energy-control mechanism.

### SYS-PWR-002
The system SHALL attempt to avoid starting an operation expected to collapse the supply rail.

### SYS-PWR-003
Energy policy SHALL be dynamically re-evaluated during the mission.

### SYS-PWR-004
Energy adaptation SHALL be reversible and move operation back toward configured target as capability recovers.

### SYS-PWR-005
The controller SHOULD use hysteresis, filtered trends, dwell time, or equivalent means to prevent rapid policy chatter.

### SYS-PWR-006
If projected energy cannot sustain configured cadence through expected low-energy conditions, the system MAY lengthen the wake interval.

### SYS-PWR-007
If GNSS cannot be safely powered, the system SHALL skip GNSS rather than intentionally brown out.

### SYS-PWR-008
When GNSS is skipped for energy, retained GNSS information SHALL be stale/unavailable according to the GNSS data contract.

### SYS-PWR-009
If energy is insufficient for useful science acquisition, the system MAY perform only minimum state/power checks and return directly to low power.

### SYS-PWR-010
When energy is nominal, the system SHALL attempt to meet operator-configured targets.

### SYS-PWR-011
When energy is surplus, the system MAY perform additional approved mission-value work after current core science obligations are protected.

### SYS-PWR-012
Surplus-energy work MAY include additional data return, longer application availability, or configured high-energy science.

### SYS-PWR-013
Battery heating is NOT a first-flight approved surplus-energy behavior unless later explicitly added.

### SYS-PWR-014
Final operation-admission thresholds SHALL be based on characterized hardware behavior including low-temperature droop/load capability.

---

# 5. Configuration

### SYS-CFG-001
Configuration SHALL represent operator target mission behavior.

### SYS-CFG-002
Runtime energy/regulatory degradation SHALL NOT silently rewrite stored target configuration.

### SYS-CFG-003
Scalar configuration values with defined feasible bounds SHALL be clamped into those bounds.

### SYS-CFG-004
Structurally invalid/corrupt configuration that cannot be deterministically normalized SHALL be rejected in favor of known valid configuration.

### SYS-CFG-005
All configuration sources SHALL use one common semantic validation/normalization path.

### SYS-CFG-006
A successful configuration change SHOULD generate a configuration-change event.

### SYS-CFG-007
Individual clamping operations need not generate separate events.

### SYS-CFG-008
A new configuration SHALL take effect on the next ordinary wake boundary rather than restructuring the current wake.

### SYS-CFG-009
Future remote configuration MAY be supported, but executable firmware SHALL NOT be remotely replaced.

---

# 6. Sensor and Scientific Data Truth

### SYS-DATA-001
If a sensor transaction completes normally and reports no defined status fault, firmware SHALL preserve/report the returned measurement even if scientifically surprising.

### SYS-DATA-002
Firmware SHALL NOT fabricate a plausible fresh value when a fresh measurement is unavailable.

### SYS-DATA-003
When a prior value is reused, the data contract SHALL identify it as stale/substituted where that distinction is relevant.

### SYS-DATA-004
Firmware need not maintain generalized confidence scoring for every observation.

### SYS-DATA-005
Sensor-specific quality/status fields SHALL be added only when deliberately defined by that sensor/product contract.

### SYS-DATA-006
Drivers MAY perform a small bounded number of sensor-specific retries.

### SYS-DATA-007
A failed sensor SHALL NOT indefinitely block unrelated science acquisition.

### SYS-DATA-008
Where practical, the system SHOULD preserve the lowest-level scientifically meaningful raw observable.

### SYS-DATA-009
Backend calibration MAY transform raw observations into calibrated scientific quantities.

### SYS-DATA-010
Historical raw data SHOULD remain reprocessable with improved later calibration models.

### SYS-DATA-011
Firmware MAY calculate calibrated/derived values where local autonomous behavior or a defined onboard science product requires them.

### SYS-DATA-012
Derived/calibrated products SHALL NOT silently masquerade as unmodified raw observations.

---

# 7. GNSS and Regulatory RF

### SYS-GNSS-001
GNSS failure SHALL NOT stop unrelated environmental science.

### SYS-GNSS-002
A previous valid GNSS fix MAY remain available as last-known position if explicitly stale.

### SYS-GNSS-003
Failure to wake/acquire GNSS SHALL NOT cause a previous fix to be represented as fresh.

### SYS-GNSS-004
RF authorization MAY continue using last-known regulatory context only within configured position-staleness allowance.

### SYS-GNSS-005
When GNSS staleness exceeds the configured regulatory allowance, RF transmission SHALL stop.

### SYS-GNSS-006
Science, archive activity, and GNSS recovery attempts SHALL continue while RF is silenced for stale regulatory position.

### SYS-GNSS-007
A new valid fresh GNSS fix SHALL automatically restore normal RF-authorization evaluation.

### SYS-GNSS-008
The first-flight product need not implement generalized GNSS spoof detection unless separately specified.

---

# 8. Radio and Archive Recovery

### SYS-RF-001
If only one useful packet can be transmitted during a cycle, current science SHALL be selected ahead of archive, diagnostics, and optional data.

### SYS-RF-002
Historical archive recovery SHALL NOT delay a due current science observation.

### SYS-RF-003
After outage, normal archive recovery SHALL begin with newest eligible retained data and work backward under allowed recovery budget.

### SYS-RF-004
A good link after long outage SHALL NOT create a special archive-dump mission mode.

### SYS-RF-005
Distinct incompatible application payload families SHOULD use distinct LoRaWAN FPorts.

### SYS-RF-006
A payload need not carry redundant packet-type data solely to repeat an unambiguous FPort.

### SYS-RF-007
Incompatible payload schema changes SHALL remain unambiguously decodable using FPort, schema/version, or another documented discriminator.

---

# 9. Archive and Persistence

### SYS-NVM-001
The science archive SHALL operate as a circular buffer.

### SYS-NVM-002
When archive capacity is exhausted, oldest retained science SHALL be overwritten.

### SYS-NVM-003
Archive exhaustion SHALL NOT terminate the mission.

### SYS-NVM-004
Committed archive records SHALL be self-validating with integrity metadata such as CRC.

### SYS-NVM-005
Power interruption during archive update MAY lose a small bounded number of newest records but SHALL NOT normally destroy older committed archive content.

### SYS-NVM-006
Where valid records remain recoverable, damaged archive metadata SHOULD be reconstructed rather than treating the archive as empty solely because headers/index metadata are invalid.

### SYS-NVM-007
Permanent identity and LoRaWAN credentials SHALL survive ordinary reset/power loss and supported application reflash.

### SYS-NVM-008
LoRaWAN frame counters/protocol state SHALL NOT roll backward into prohibited reuse.

### SYS-NVM-009
Commissioned configuration SHALL survive ordinary power loss/reset.

### SYS-NVM-010
Mission lifecycle/flight latch SHALL survive reset/power loss as required to prevent unintended return to commissioning.

### SYS-NVM-011
Persistent storage SHALL prefer bounded last-update loss over whole-object loss.

---

# 10. Identity, Commissioning, and Ownership

### SYS-ID-001
Each physical Stratosonde SHALL have one stable DevEUI for its hardware lifetime.

### SYS-ID-002
A new flight SHALL NOT require a new DevEUI.

### SYS-ID-003
Ownership SHALL be backend metadata and SHALL NOT redefine hardware identity.

### SYS-ID-004
The PCB SHOULD include a scannable QR representation of device identity.

### SYS-ID-005
The PCB SHOULD include a unique random claim PIN/secret associated with the device.

### SYS-ID-006
A normal first-claim flow SHOULD require an authenticated backend user and proof of physical possession using the claim mechanism.

### SYS-ID-007
If a device is already claimed, first-claim flow SHALL report that state and SHALL NOT silently transfer ownership.

### SYS-ID-008
Ownership transfer SHALL require a separate explicit backend process.

### SYS-ID-009
LoRaWAN secret keys SHALL NOT be publicly exposed as ordinary PCB identity data.

### SYS-ID-010
The exact QR/PIN security protocol SHALL be completed before production backend implementation.

---

# 11. Firmware Service

### SYS-FW-001
The flight product SHALL NOT support OTA executable firmware replacement.

### SYS-FW-002
Executable firmware changes SHALL require local physical access.

### SYS-FW-003
The current first-flight local programming mechanism MAY be SWD/ST-Link.

### SYS-FW-004
Supported ordinary application reflash SHALL preserve permanent identity and provisioned LoRaWAN credentials.

### SYS-FW-005
Factory reprovisioning SHALL be distinct from ordinary application firmware update.

---

# 12. Verification and Flight Readiness

### SYS-VER-001
Each flight-critical system requirement SHALL have a defined verification method.

### SYS-VER-002
Requirements SHOULD map to governing DDR, implementation location, verification case, and evidence artifact.

### SYS-VER-003
Fast host/build CI SHALL remain part of ordinary development.

### SYS-VER-004
Target-hardware verification is a project goal and SHOULD progressively automate real-board tests.

### SYS-VER-005
Initial HIL MAY begin with production hardware, ST-Link/RTT, Otii Ace, and controlled power without PCB modifications.

### SYS-VER-006
Power interruption SHALL be deliberately tested around persistent-state write boundaries.

### SYS-VER-007
Power/current behavior SHOULD be captured as regression evidence for important mission states.

### SYS-VER-008
RTT-enabled verification SHALL be supplemented by RTT-quiet/off testing for timing/power-sensitive behavior.

### SYS-VER-009
Cold-chamber testing SHALL establish physical evidence behind the low-temperature energy model.

### SYS-VER-010
First flight SHALL have an explicit validation/readiness checklist tied to the selected firmware build.

### SYS-VER-011
A release checklist SHOULD record requirement ID, DDR, code binding, result, evidence, and open risk/waiver.

### SYS-VER-012
A discovered bug SHOULD become a permanent regression test when practical.

---

# 13. Deferred First-Flight Complexity

Not required merely for architectural completeness:

- OTA firmware update;
- custom field bootloader;
- generalized self-learning mission adaptation;
- generalized GNSS spoof detection;
- runtime multi-hardware-revision autodetection;
- generalized confidence scoring for every sensor value;
- elaborate reset-loop safe mode;
- switched-I2C fault-injection hardware;
- GNSS simulator.

These may be added later when a concrete requirement justifies them.

---

# 14. ERS Exit Criteria

This ERS is mature enough for implementation closure when:

1. every `SYS-*` requirement has an accepted DDR owner or explicit temporary rationale;
2. contradictory DDR language is reconciled;
3. implementation bindings are identified;
4. critical requirements have a planned proof method;
5. unresolved security/protocol questions are not silently filled in by code.
