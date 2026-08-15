# Stratosonde Requirements Traceability Matrix

**Date:** 2026-08-12 (merges the round-2 and round-3 interview matrices into one living V-model artifact)
**Updated:** 2026-08-13 (three-pass intent interview merged; see §"2026-08-13 rows" and `../decisions/merge-ledger-2026-08-13.md`)
**Annotated:** 2026-08-15 (row-status layer added against master @ `dc8026a`, after the LT/C-01 fix sweep; annotations name issues/commits/tests only — finding detail lives in the tracker, not here. Markers: 🟡 implemented / proof open · 🔴 open, tracker named · ⚪ not assessed this pass)
**Purpose:** Link product intent to DDR ownership, implementation responsibility, and objective proof.

Every flight-critical row should answer:

1. Why must the product behave this way? → intent/DDR
2. Where is it implemented? → module/function/config binding
3. How is it proved? → test/analysis/inspection ID
4. Where is this release's proof? → CI/HIL/environmental artifact tied to the binary

If a link is missing, the requirement is not fully closed.

---

## 1. Flight-Critical System Requirements (round 3)

| Requirement | Intent | DDR | Implementation binding | Proof |
|---|---|---|---|---|
| SYS-BOOT-001 | Boot/reset/wake converge | DDR-0001 / 0012 | startup + mission scheduler | cold boot/reset equivalence |
| SYS-BOOT-002 | Start from safe low-power state | DDR-0001 / 0012 | board/peripheral init | power trace/state inspection |
| SYS-BOOT-004 | Power truth before expensive work | DDR-0001 / 0016 | power arbiter | low-voltage startup HIL |
| SYS-BOOT-006 | Reset cause diagnostic, not policy | DDR-0009 / 0012 | reset handling | reset-cause matrix |
| SYS-ORCH-001 | Core owns time/power/radio | DDR-0001 / 0017 | central orchestrator | app request denial tests |
| SYS-ORCH-002 | All operations bounded | DDR-0001 / 0009 | drivers/state machines | timeout/retry audit |
| SYS-ORCH-004 | Safe overlap minimizes awake time | DDR-0001 | scheduler | RTT/power timeline |
| SYS-PWR-003 | Continuous energy reevaluation | DDR-0016 | power model | changing-profile HIL |
| SYS-PWR-004 | Reversible adaptation | DDR-0016 | power/cadence state | low→high recovery |
| SYS-PWR-005 | Anti-chatter | DDR-0016 | hysteresis/filtering | threshold dither test |
| SYS-PWR-009 | Critical energy may sleep immediately | DDR-0016 | wake admission | critical-energy test |
| SYS-CFG-003 | Clamp scalar out-of-range config | DDR-0014 | config validator | min/max boundary tests |
| SYS-CFG-004 | Reject unrecoverably malformed config | DDR-0014 | validator | CRC/schema corruption |
| SYS-CFG-006 | Config change logged | DDR-0014 | event logger | config event test |
| SYS-CFG-008 | Config applies next wake | DDR-0014 | config generation/latch | mid-wake update test |
| SYS-DATA-001 | Preserve protocol-valid surprising value | DDR-0009 / 0023 | sensor pipeline | unusual-value injection |
| SYS-DATA-004 | No generalized confidence score | DDR-0023 | schemas | schema inspection |
| SYS-DATA-008 | Preserve raw observable | DDR-0023 | archive/payload schema | golden vectors |
| SYS-DATA-010 | Historical raw reprocessable | DDR-0023 / 0024 | backend calibration | recalibration replay |
| SYS-GNSS-003 | Old fix never appears fresh after failed acquire | DDR-0003 / 0009 | GNSS acquire/merge | forced wake-failure test |
| SYS-GNSS-005 | RF stops after stale regulatory limit | DDR-0015 | RF authorization | boundary test |
| SYS-GNSS-007 | Fresh fix restores RF | DDR-0015 | RF authorization | stale→fresh test |
| SYS-RF-002 | Archive recovery never starves science | DDR-0005 / 0022 | TX scheduler | large-backlog HIL |
| SYS-RF-003 | Archive recovery newest-first | DDR-0004 / 0005 | archive traversal | ordering test |
| SYS-NVM-005 | Torn write causes bounded newest loss | DDR-0011 | flash log | randomized power cuts |
| SYS-NVM-006 | Recover archive despite metadata damage | DDR-0011 | flash recovery | header corruption test |
| SYS-NVM-008 | Frame counter never prohibited rollback | DDR-0010 / 0018 | LoRaWAN persistence | counter power-cut campaign |
| SYS-ID-006 | Claim uses authenticated user + physical possession | DDR-0024 | backend claim flow | claim integration test |
| SYS-ID-007 | Already claimed means no silent transfer | DDR-0024 | ownership backend | duplicate-claim test |
| SYS-FW-004 | Reflash preserves identity/credentials | DDR-0025 | linker/NVM/service | before/after service test |
| SYS-VER-010 | First flight has explicit validation checklist | DDR-0026 | release process | completed checklist artifact |

**2026-08-15 row annotations:** SYS-BOOT-006 🟡 (fatal always resets; no reset-count escape; target fault injection open) · SYS-ORCH-002 🟡 (burst-FSM deadline LT-07/#277; STOP2 re-init degrade LT-05/#275) · SYS-PWR-004/005 🟡 (caveat: trend lost on reset → #288) · SYS-CFG-003/004 🟡 (clamps + CRC→defaults verified in `config.c` this pass) · SYS-CFG-008 🟡 (next-wake apply, INV-CONFIG-014) · SYS-DATA-001 🟡 (caveat: solar channel unflagged → #279) · SYS-GNSS-003 🟡 (caveat → #284) · SYS-GNSS-005 🟡 (24 h constant verified `lora_app.h:206`; boundary walk open, FW-CONF-023) · SYS-GNSS-007 🔴 (silence recovery follow-up → #285) · SYS-RF-002 🟡 (#215, #269, #277) · SYS-RF-003 🔴 (fresh backlog drains oldest-first → #286) · SYS-NVM-005 🟡 (host flight suite; power-cut campaign open) · SYS-NVM-006 🟡 (R3-07/#221; caveat: boot full-ring scan → #289) · SYS-NVM-008 🟡 (#109 + H-02/#273; caveat: NVM slot atomicity → #282) · SYS-FW-004 🟡 (STAB-11/#158; reflash proof open) · SYS-VER-010 🔴 (checklist unchecked → `flight1-validation-readiness-checklist.md`; #261/#262) · SYS-ID-006/007 ⚪ (backend-side, not assessed).


## 2. Mission / Product Intent Coverage (round 2)

Round-2 "action" dispositions are now **merged** into the corpus (2026-08-12); the DDR columns below cite the merged items.

| ID | Requirement | DDR owner | Primary implementation binding | Verification/evidence |
|---|---|---|---|---|
| TR-MISSION-001 | Fresh science is the primary product. | DDR-0022 / 0005 | science scheduler/TX state machine | backlog-starvation test |
| TR-MISSION-002 | Radio outage does not change science mission. | DDR-0022 / 0009 | mission orchestration | multi-day no-link run |
| TR-MISSION-003 | Archive is circular; overwrite oldest. | DDR-0004 | archive layer | fill/wrap tests |
| TR-MISSION-004 | Recovery after outage is newest-first. | DDR-0004 / 0005 (BR-TX-022) | recovery traversal | ordering test |
| TR-MISSION-005 | Config is target; temporary adaptation returns toward it. | DDR-0014 / 0016 (INV-CONFIG-010, INV-PWR-015) | mission/power scheduler | low-energy recovery test |
| TR-POWER-001 | Brownout is a fault, not a strategy. | DDR-0016 (INV-PWR-012) | power arbiter | declining-voltage HIL |
| TR-POWER-002 | Slow cadence when energy projection cannot sustain target. | DDR-0016 (INV-PWR-013) | cadence/power policy | night-profile test |
| TR-POWER-003 | Skip GNSS when load cannot be safely admitted; mark stale. | DDR-0016 / 0003 (INV-PWR-014) | GNSS admission | GNSS refusal HIL |
| TR-DATA-001 | Protocol-valid sensor values are not censored merely for being surprising. | DDR-0009 / 0023 (INV-FAIL-011) | sensor drivers | unusual-value test |
| TR-DATA-002 | Sensor/bus retries are small, bounded, and sensor-specific. | DDR-0009 (INV-FAIL-012) | drivers | injected I2C faults |
| TR-DATA-003 | Preserve raw observable where practical. | DDR-0023 (INV-DATA-011) | payload/archive schema | golden vectors |
| TR-DATA-004 | Backend may own calibration bound to physical sensor identity. | DDR-0023 / 0024 (INV-DATA-012) | backend calibration registry | reprocessing test |
| TR-RF-001 | One-packet budget goes to fresh science before archive/debug. | DDR-0005 / 0022 (BR-TX-021) | TX planner | one-packet test |
| TR-RF-002 | Excessive GNSS staleness forces RF silence for regulatory confidence. | DDR-0015 | RF authorization | staleness boundary test |
| TR-RF-003 | Fresh GNSS automatically restores RF eligibility. | DDR-0015 | RF authorization | stale→fresh test |
| TR-RF-004 | FPort identifies incompatible application product families. | DDR-0019 (INV-RADIO-009) | encoder/router | FPort golden vectors |
| TR-RF-005 | Do not repeat packet type when FPort is unambiguous. | DDR-0019 (INV-RADIO-010) | payload schema | inspection |
| TR-RF-006 | Incompatible schemas remain explicitly decodable. | DDR-0019 (INV-RADIO-011) | schema/FPort assignment | compatibility tests |
| TR-FAULT-001 | Peripheral failure degrades only dependent capability. | DDR-0009 | orchestration/drivers | fault injection |
| TR-FAULT-002 | HardFault/core fatal state resets. | DDR-0009 | exception handlers | injected fatal fault |
| TR-FAULT-003 | Bounded bus/peripheral recovery is allowed. | DDR-0009 (INV-FAIL-010) | drivers | retry/escalation test |
| TR-FAULT-004 | Watchdog required in flight; commissioning may be exempt. | DDR-0020 | watchdog/deadman | mission/commissioning stall |
| TR-PERSIST-001 | Mission-critical state survives reset or is reconstructible. | DDR-0010 | NVM/session/archive | reset-boundary matrix |
| TR-PERSIST-002 | Identity/credentials are highest durability; small recent log loss may be acceptable. | DDR-0010 / 0011 (INV-PERSIST-008) | storage policy | power-cut campaign |
| TR-ID-001..005 | DevEUI belongs to hardware; flights/ownership are backend metadata; per-region provisioning before flight; recovery keeps identity. | DDR-0024 / 0018 | backend/NVM | recovery/reflight, backend acceptance |
| TR-ID-006 | QR random-value security semantics — claim PIN intent confirmed, protocol open. | DDR-0024 (INV-ID-010, OD-ID-001) | claim flow | threat-model interview |
| TR-FW-001 | No OTA executable firmware update. | DDR-0025 (INV-FW-001) | command surface | reachability review |
| TR-FW-002 | Future downlink changes validated config only. | DDR-0014 / 0025 (INV-FW-002) | config downlink | validation/auth tests |
| TR-FW-003 | Current local service uses ST-Link/SWD. | DDR-0025 (INV-FW-003) | service tooling | service procedure |
| TR-FW-004 | Reflash preserves DevEUI and credentials. | DDR-0025 / 0024 (INV-FW-004, BR-COMM-016) | linker/NVM/tooling | automated reflash |
| TR-FW-005 | Custom field bootloader not first-flight requirement. | DDR-0025 (INV-FW-007) | architecture | inspection |
| TR-CONFIG-001 | Configuration is operator target intent. | DDR-0014 (INV-CONFIG-009) | config object | semantic tests |
| TR-CONFIG-002 | Out-of-range scalars clamped; structurally invalid config rejected. | DDR-0014 (INV-CONFIG-011/012) | validator | boundary tests |
| TR-CONFIG-003 | Runtime degradation never silently rewrites target config. | DDR-0014 / 0016 (INV-CONFIG-010) | config/power state | persistence comparison |
| TR-HW-001 | One hardware revision now; avoid speculative complexity. | ../decisions/open-intent-questions.md #19 | HAL/build structure | architecture review |
| TR-VER-001 | Every flight-critical requirement maps to implementation and proof. | DDR-0026 (INV-VER-001) | CI metadata | traceability audit |
| TR-VER-002 | Keep fast host CI. | DDR-0026 | host tests | CI |
| TR-VER-003 | Add actual target HIL. | DDR-0026 | lab runner | hardware CI |
| TR-VER-004 | Initial HIL needs no PCB mods: ST-Link/RTT + programmable power. | DDR-0026 (BR-VER-004) | lab rack | rack smoke |
| TR-VER-005 | Otii Ace provides current/power evidence where automatable. | DDR-0026 (BR-VER-005) | lab automation | power traces |
| TR-VER-006 | Power cuts at dangerous moments become regressions. | DDR-0026 (INV-VER-006) | lab runner | fault campaign |
| TR-VER-007 | RTT and RTT-quiet/off runs establish instrumentation equivalence. | DDR-0026 (INV-VER-003) | build variants | equivalence test |
| TR-VER-008 | Exact production/release image receives hardware evidence. | DDR-0026 (INV-VER-004) | release pipeline | release qualification |
| TR-VER-009 | Cold chamber validates energy model, not just boot survival. | DDR-0026 / 0016 | chamber + Otii | temp/load campaign |

**2026-08-15 row annotations:** TR-MISSION-001 🟡 (#215, #269, #277) · TR-MISSION-002 🟡 (science continues dark; same-wake recovery caveat → #285) · TR-MISSION-003 🟡 (host flight suite: fill/wrap) · TR-MISSION-004 🔴 (→ #286) · TR-MISSION-005 🟡 (caveat: trend lost on reset → #288) · TR-POWER-001/002 🟡 (caveats #248, #288) · TR-POWER-003 🟡 (caveat: intent is FULL/SLEEP admission → #288) · TR-DATA-001 🟡 (caveat → #279) · TR-DATA-002 🟡 (R28/#136; LT-05/#275) · TR-DATA-003 🟡 · TR-DATA-004 ⚪ (backend-side) · TR-RF-001 🟡 (`ScienceIsDue` yield) · TR-RF-002 🟡 (24 h verified; boundary test open) · TR-RF-003 🔴 (→ #285) · TR-RF-004/005/006 🟡 (DDR-0019 + `../PayloadFormats.md` allocation) · TR-FAULT-001/002/003 🟡 (STAB-02/#149; #272; #275) · TR-FAULT-004 🟡 (DDR-0020 deadman; commissioning exempt) · TR-PERSIST-001/002 🟡 (#109, STAB-11/#158, C-01/#270; caveats → #282, #289) · TR-ID-001..005 🟡 (per-region provisioning + read-back, #270; backend halves ⚪) · TR-ID-006 ⚪ (claim protocol open by design, OD-ID-001) · TR-FW-001..005 🟡 (no OTA surface; identity in backup regs #158; reflash proof open) · TR-CONFIG-001/002/003 🟡 (verified `config.c` this pass) · TR-HW-001 ✅ (single revision; tracked in `../decisions/open-intent-questions.md` #19) · TR-VER-001 🟡 (this matrix + the worklist are the audit; statuses now dated) · TR-VER-002 ✅ (13-suite host baseline green in CI, incl. sanitize build) · TR-VER-003..006 🔴 (no HIL lane; #261/#262) · TR-VER-007 🟡 (flight build compiles debug out, R2-15/#119; equivalence campaign open) · TR-VER-008 🔴 (release qualification; #261/#262) · TR-VER-009 🔴 (cold chamber also feeds #248).

---

## 3. Highest-Priority Implementation Checks (round 3)

1. Confirm startup code converges reset and wake paths as intended. — 🟡 (host boot-resilience #107; target open)
2. Confirm no reset-cause branch changes long-term mission policy. — 🟡 (#149 per-code degrade; formal audit open)
3. Audit every driver/blocking call for bounded execution. — 🟡 (LT-05/#275, LT-07/#277; formal audit open)
4. Reconcile current config behavior with the explicit **clamp** policy. — 🟡 (verified `config.c` 2026-08-15)
5. Verify config changes apply at wake boundary. — 🟡 (INV-CONFIG-014)
6. Verify power model supports reversible reevaluation without chatter. — 🟡 (caveats #248, #288)
7. Confirm app/Qwiic code cannot independently hold power/radio/wake time. — ⚪ not assessed this pass
8. Add GNSS failed-wake provenance regression. — 🟡 (teardown anchored by LT-04/#276; freshness caveat → #284)
9. Fix/prove archive newest-first and science-preemption behavior. — preemption 🟡 (#215/#269/#277); newest-first 🔴 → #286
10. Audit persistent objects against the must-never-lose/bounded-loss hierarchy. — 🟡 (#158, #270; caveats → #281, #282)
11. Define the backend QR/PIN claim security protocol. — ⚪ backend-side, open by design (OD-ID-001)
12. Complete the Flight-1 checklist with actual evidence. — 🔴 (see `flight1-validation-readiness-checklist.md`; #261/#262)

## 4. Immediate Conformance Priorities (round 2)

1. Guarantee fresh science preempts archive recovery. — 🟡 done in code (#215/#269/#277); accelerated proof open
2. Fix GNSS wake-failure provenance so old coordinates cannot appear fresh. — 🟡 (LT-04/#276; caveat → #284)
3. Bring archive recovery to newest-first/current-science-preemptive behavior. — 🔴 newest-first → #286
4. Move power policy toward measured operation-level droop admission. — 🔴 FULL/SLEEP admission → #288
5. Verify stale-position RF timeout and automatic fresh-fix recovery. — 🟡 24 h enforced; 🔴 same-wake recovery → #285
6. Verify fatal exception handlers reset. — 🟡 (STAB-02/#149)
7. Verify watchdog/deadman is armed across all flight paths. — 🟡 (DDR-0020 deadman; path audit open)
8. Add bounded driver/bus retries. — 🟡 (R28/#136; LT-05/#275)
9. Verify reflash preserves identity/credential banks. — 🟡 (STAB-11/#158; automated reflash proof open)
10. Formalize FPort/schema bindings and decoder golden vectors. — 🟡 bindings exist (DDR-0019 + `../PayloadFormats.md`); golden vectors open
11. Verify archive reconstruction under metadata corruption/power cuts. — 🟡 reconstruction (R3-07/#221); 🔴 power-cut campaign + boot scan → #289
12. Build the first target-HIL lane and retain evidence. — 🔴 (#261/#262)


---

## 5. 2026-08-13 rows (three-pass intent interview)

New/changed requirements from the 2026-08-13 merge. Implementation bindings marked
**(none yet)** are intent without code — they are the real gap list.

| Requirement | Intent | DDR | Implementation binding | Proof |
|---|---|---|---|---|
| SYS-RF-010 | Stale-position RF budget is 24 h, and it is the only time-based RF cutoff | DDR-0015 `BR-STALE-017/018/020` | `GPS_LOSS_SILENCE_S` = `24U*3600U` (`LoRaWAN/App/lora_app.h`), evaluated in `lora_app.c` | `P-STALE-014`; `test_stale_position_budget_is_24h` (source scan) + FW-CONF-023 behavioral |
| SYS-RF-011 | One valid fix clears staleness silence on the same wake | DDR-0015 `BR-STALE-019` | `s_last_fresh_fix_s` age reference | `P-STALE-015` / FW-CONF-024 |
| SYS-RF-012 | RTC sync age never silences RF | DDR-0013 `INV-TIME-009`, DDR-0015 `BR-STALE-020` | audit — no second timer may exist | `P-STALE-016`, `P-TIME-010` / FW-CONF-025 |
| SYS-RF-013 | Band 0 commissioned `home_region` fallback, expiring into silence | DDR-0015 `INV-STALE-008` | **(none yet)** — no `home_region` provisioning in code | `P-STALE-011..013` / FW-CONF-026 |
| SYS-LIFE-010 | One-way ascent→float cadence latch, terminal for the mission | DDR-0002 `INV-LIFE-011`, `BR-LIFE-023/025` | `mission_state.c` FLOAT latch, `mission_logic.c` detector | `P-LIFE-012` / FW-CONF-047 |
| SYS-LIFE-011 | Float latch survives reset | DDR-0002 `BR-LIFE-024` | DR3-persisted mission state | `P-LIFE-013`, `P-BOOT-012` / FW-CONF-033 |
| SYS-LIFE-012 | Automatic pressure launch works without the operator action | DDR-0002 `BR-LIFE-027` | launch detector + arming hook | `P-LIFE-014`, `P-COMM-013` / FW-CONF-048 |
| SYS-PWR-010 | Per-wake admission is FULL CYCLE or SLEEP | DDR-0001 `BR-WAKE-018`, DDR-0016 `BR-PWR-018` | `power_model.c` / `transmit_plan.c` (divergence 3) | `P-WAKE-012`, `P-PWR-013` / FW-CONF-028 |
| SYS-PWR-011 | Battery/energy trend is durable across reset | DDR-0016 `BR-PWR-016` | backup-register/flash trend state | `P-PWR-015` / FW-CONF-029 |
| SYS-SCHED-001 | Cadence is start-to-start, not finish-plus-sleep | DDR-0001 `BR-WAKE-017` | wake scheduler | `P-WAKE-011` / FW-CONF-027 |
| SYS-PERSIST-010 | Reset may omit an observation but never fabricate one | DDR-0010 `INV-PERSIST-011` | archive commit ordering | `P-PERSIST-010` / FW-CONF-031 |
| SYS-PERSIST-011 | Transient reset never permanently disables telemetry; one mission identity | DDR-0010 `INV-PERSIST-012` | persistent-state restore path | `P-PERSIST-011/012` / FW-CONF-032 |
| SYS-PERSIST-012 | Cadence survives reset; exact transient timer may be lost | DDR-0010 `INV-PERSIST-010` | scheduler anchor | `P-BOOT-013` / FW-CONF-034 |
| SYS-STORE-010 | Isolated torn record is skipped, never truncating the archive | DDR-0011 `BR-STORE-001` | archive traversal/validation | `P-STORE-011` / FW-CONF-035 |
| SYS-STORE-011 | No bulk archive scan as a boot prerequisite; lazy validation | DDR-0011 `BR-STORE-002/003`, DDR-0012 `INV-BOOT-010` | boot + archive index recovery | `P-STORE-012..014`, `P-BOOT-014` / FW-CONF-036/037/038 |
| SYS-FAIL-010 | Bounded cross-subsystem recovery; normal service immediately on success | DDR-0009 `BR-FAIL-016/017` | sensor/GNSS/radio drivers | `P-FAIL-011` / FW-CONF-039 |
| SYS-FAIL-011 | Radio recovery never triggers an in-flight join | DDR-0009 `BR-FAIL-018` | radio/stack reset paths | fault injection / FW-CONF-040 |
| SYS-FAIL-012 | No reset-count or failure-history escalation | DDR-0009 `OD-FAIL-006` resolved, DDR-0020 | audit | `P-FAIL-012` / FW-CONF-041 |
| SYS-ARCH-010 | Arbitrary retained-ID lookup; linear search acceptable | DDR-0004 `BR-ARCH-018` | archive lookup | `P-ARCH-011` / FW-CONF-042 |
| SYS-ARCH-011 | Unavailable requested ID returns a useful substitute; returned ID authoritative | DDR-0004 `BR-ARCH-019` | archive request handler | `P-ARCH-012` / FW-CONF-043 |
| SYS-ARCH-012 | Record structure stable under partial failure | DDR-0004 `BR-ARCH-017` | record encoder | `P-ARCH-013` / FW-CONF-046 |
| SYS-TX-010 | Explicit record request preempts backfill and is ephemeral | DDR-0005 `BR-TX-023/024` | **(none yet)** — no request path implemented | `P-TX-011/012` / FW-CONF-044 |
| SYS-TX-011 | Energy and RF legality outrank any request | DDR-0005 `BR-TX-025` | veto ordering in `transmit_plan.c` | `P-TX-013` / FW-CONF-045 |
| SYS-GNSS-010 | Readiness uses the same fix-acceptance predicate as flight | DDR-0003 `BR-GNSS-021` | shared acceptance helper | `P-GNSS-011` / FW-CONF-049 |
| SYS-GNSS-011 | Never-fixed placeholder cannot masquerade as a position | DDR-0003 §17 | position provenance encoding | `P-GNSS-012` / FW-CONF-051 |
| SYS-COMM-010 | Provisioning is a hard gate; readiness check is separate and unambiguous | DDR-0018 `BR-COMM-017..020`, `INV-COMM-009` | commissioning state machine, LED | `P-COMM-011/012/014` / FW-CONF-049/050 |
| SYS-CFG-010 | Unsupported downlink is safe (no crash/reset/corruption) | DDR-0014 `BR-CONFIG-014` | downlink handler | `P-CONFIG-011` / FW-CONF-052 |
| SYS-CFG-011 | Corrupt persisted operational config is repaired, not fatal; credentials excluded | DDR-0014 `BR-CONFIG-015/016` | `config.c` validator | `P-CONFIG-010` / FW-CONF-053 |
| SYS-PROTO-001 | Deployed packet versions stay decodable; no OTA dependency | DDR-0027 `BR-PROTO-001..005` | backend codecs (ground-side) | `P-PROTO-001..004` / FW-CONF-054 |

**2026-08-15 row annotations:** SYS-WAKE-012 🟡 (veto provenance b5-b7; LT-03/#271) · SYS-PWR-013 🔴 / SYS-PWR-014 🟡 / SYS-PWR-015 🔴 (→ #288) · SYS-SCHED-001 🟡 (#215, #269, #277) · SYS-PERSIST-010/011/012 🟡 (see FW-CONF-031/032/034) · SYS-STORE-010 🟡 (F-006/#51) · SYS-STORE-011 🟡 with 🔴 half (fast path #221; deferred reconstruction → #289) · SYS-FAIL-010 🟡 (#272, #275) · SYS-FAIL-011 🟡 (#270 door; #272 rollback) · SYS-FAIL-012 🟡 (#104) · SYS-ARCH-010/011 🔴 (no lookup API in `flash_log.h` — verified this pass; part of the unimplemented request set) · SYS-ARCH-012 🟡 · SYS-TX-009 🔴 (binding "backfill controller" is optimistic — there is no request path at all; same set as SYS-TX-010) · SYS-TX-010 🔴 (confirmed still **(none yet)** this pass) · SYS-TX-011 🟡 (veto ordering exists in `transmit_plan.c`; no request path to outrank yet) · SYS-GNSS-010/011 🟡 (caveat → #284) · SYS-COMM-010 🟡 (C-01/#270) · SYS-CFG-010 🟡 (no downlink parse surface today; fuzz proof open) · SYS-CFG-011 🟡 (verified `config.c` CRC→defaults; credentials excluded, STAB-11/#158) · SYS-PROTO-001 🔴 (ground-side; next wire bump #266 + #279) · SYS-STALE-010 🟡 (24 h verified; boundary walk open) · SYS-STALE-011 🔴 (→ #285) · SYS-STALE-012 ⚪ (only the GPS-loss path was found in the 2026-08-13 review; formal audit open) · SYS-COMM-008/009 🟡 (C-01/#270 latch + all-7-regions read-back; caveat: durable pre-flight state → #281) · SYS-LIFE-010/011 🟡 (one-way latch host-tested; #270 gate) · SYS-GNSS-009 🟡 (caveat → #284).

**2026-08-15 (second pass, 2026-08-14 review triage):** RF-authorization rows (DDR-0007 BR-RF-*) — restricted enforcement implemented but inert (empty dataset → #257, guards → h3lite#1); ring-search bound not configurable (BR-RF-007/008 → #299, post-flight); BR-RF-009 zero-candidate silence reconciled to sanctioned keep-current by owner disposition 2026-08-15 (#258; strict rework remains available). Power rows — SYS-PWR-* caveat added: fixed 4300 mV raw floor selects SURVIVAL for a full pack below −62.3 °C (PWR-02 → #297, bench-gated, red-first `test_pwr` EXPECT_UNFIXED). MAC hardening — network-commanded `ChannelsNbTrans` unbounded → #298 (post-flight). GEO-02 F-006 comment rewritten to the actual policy in `lora_app.c`. New gated suites: `test_pwr`, `test_geo`.

### Closure note

Rows whose implementation binding reads **(none yet)** — SYS-RF-013 (`home_region`
Band 0 fallback) and SYS-TX-010 (explicit record request) — are the two places where the
2026-08-13 intent has no code at all. **Re-verified 2026-08-15:** still the only two —
no `home_region` provisioning exists (#287), and `flash_log.h` has no arbitrary-ID
lookup / request path (FW-CONF-042..045). Everything else is either already implemented,
partially implemented, or a test gap. Per the header rule, none of these rows are closed
until the proof column points at real evidence tied to a build.
