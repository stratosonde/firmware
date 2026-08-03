# Stratosonde Firmware — Production Readiness Status Matrix

**Living document.** This file tracks *code truth only* — a row is FIXED only when the fix is in the code, not when the design is agreed. Design rationale lives in `docs/adr/`; the implementation plan and verification evidence live in `docs/FixWorkorderPlan.md`; the authoritative finding list is `archive/stratosonde-fix-workorder-2026-07-31.md`.

**Base commit:** `181f997` · **Last reconciled with code:** 2026-08-02 (post `ReviewFixImplementationPlan.md` items 1–11, 14–16; build clean, 191,904 B flash used)

---

## Status Matrix

| ID | Priority | Issue | Status | Verified in code at |
|----|----------|-------|--------|---------------------|
| P0-1 | BLOCKER | Lat/lon saturates ±29.5° | **FIXED** | `payload_encode.c:30-31` full int16 scaling |
| P0-2 / F2 | BLOCKER | IWDG reboot-loop in STOP2 → wake classification dead code | **PARTIAL** — chunked sleep works but flag classification is dead (`stm32_lpm_if.c:253`); UTIL_TIMER tick redesign pending | `stm32_lpm_if.c`, `stm32wlxx_it.c:269,306` |
| P0-3 / F14 | BLOCKER | Flash headers both in sector 0, no erase; sequence reuse; reader wedges on CRC error | **FIXED** — T4/ADR-0004 rewrite: headers in separate sectors 0/1 (v3), erase-before-write, torn-write-tolerant reader (bench gate B1 remains) | `flash_log.c`, `flash_log.h` |
| P0-4 | BLOCKER | Error_Handler bricks sonde | **FIXED** | `main.c:804` degrade-and-continue |
| P0-5/6 / F4+F5 | BLOCKER | Multi-region: only US915 joined; infinite join loop reachable in flight | **FIXED** — T1 ladder (ADR-0006): rejoin is COMMISSIONING-only; FLIGHT with no session = RF silence, profile keeps flying (GPS + flash). Bench gate B2 remains | `lora_app.c`, `multiregion_context.c` |
| P0-NEW | BLOCKER | GNSS PCAS04→PCAS11 airborne mode | **FIXED** (bench gate B8 remains) | `atgm336h.h:167` `$PCAS11,5*18` |
| F1 | BLOCKER | Fault handlers are brick traps; IWDG armed late | **FIXED** — fault handlers breadcrumb (magic+code) to RTC backup reg then `NVIC_SystemReset`; cause surfaces in status byte. FW-2: RTCAPB clock + backup access now enabled before breadcrumb read/write (was silently unreadable pre-RTC-init). FW-5: IWDG now armed immediately after `SystemClock_Config()` (was after `MX_LoRaWAN_Init`) — watchdog covers boot/commissioning; join-wait refresh path audited OK (bench gate B4) | `stm32wlxx_it.c:102`, `reset_cause.c:21-22`, `main.c` |
| F3 | HIGH | LSE failure = reboot loop; LSECSS trip does nothing | **FIXED** — LSE failure fails over to LSI RTC clock and keeps flying (bench gate B3) | `main.c` SystemClock_Config |
| F6 / P1-8 | HIGH | Real keys in public repo; AppKey == NwkKey | **PARTIAL** — `se-identity-template.h` committed, `se-identity.h` gitignored; key rotation + git history purge is a bench/manual step before launch | `se-identity-template.h`, `.gitignore` |
| F7 | HIGH | DevEUI mismatch (default vs US915 context) | **PARTIAL** — discrepancy flagged in template; settle against Chirpstack during F6 re-provisioning | `se-identity.h:99` vs `:102` |
| F8 / N10 | HIGH | Stale GPS indistinguishable from live | **FIXED** — `gnss_stale` flag set on timeout path, cleared on real fix, flows through `sensor_t` → flash + uplink status byte | `lora_app.c`, `sys_sensors.c` |
| F9 / P1-10 / N15 | HIGH | Failed sensor reads substitute +18 °C / 50 %RH | **FIXED** — last-known-good cache + stale bits; GPS lockout treats stale temp as COLD | `sys_sensors.c`, `lora_app.c` |
| F10 / N17 | HIGH | Failed bulk conversion still packs record (zeros) | **FIXED** — early-out `continue`, packed_count tracked, zero-convertible skips packet | `lora_app.c` bulk path |
| F11 / N9 | HIGH | Flash record written before GPS refresh (stale position, current timestamp) | **FIXED** — flash write moved after post-GPS re-read, fresh timestamp | `lora_app.c` SendTxData |
| F12 / P1-16 | HIGH | No absolute time; ADR-0003 unimplemented | **FIXED** — `SysTimeSyncFromGnss()` converts NMEA DDMMYY+HHMMSS → Unix epoch, calls `SysTimeSet` on every good fix | `lora_app.c` |
| F13 | MEDIUM | Watchdog proves liveness not progress; reset cause discarded | **FIXED** — (b) reset cause at boot rides status byte; (a) progress deadman: SendTxData marks progress (RTC_BKP_DR2), main loop resets if no cycle for 3h in FLIGHT (COMMISSIONING exempt, code 6 breadcrumb). FW-4: `Deadman_Check()` now also runs inside the STOP2 chunk loop + 150-chunk bound — a dead TxTimer/Alarm-A chain can no longer sleep forever with a satisfied IWDG | `lora_app.c`, `main.c`, `reset_cause.c`, `stm32_lpm_if.c` |
| F15 / N7 | HIGH | Mark-transmitted off-by-one → perpetual retransmit | **FIXED** — clamp removed; "caught up" = last_transmitted == next_sequence | `flash_log.c` |
| F16 / N5 | HIGH | Bulk DR hardcoded DR_3; wrong SF outside US915 | **FIXED** — `DatarateFromSF()` resolves SF7/SF10 per active region (bench gate B2) | `lora_app.c` |
| F17 / P1-11 | HIGH | Status byte removed from uplink | **FIXED** — restored as byte 11 (stale bits + reset cause + mission state); compact packet now 11 B, validator updated | `payload_format.h`, `payload_encode.c` |
| F18 / N12 | MEDIUM | Battery (and solar) ADC assume VDDA = 3300 mV | **FIXED** — ratiometric via VREFINT, 3300 fallback, both battery and solar | `adc_if.c` |
| F19 / N13 | LOW | Solar charging threshold impossible (6000 mV vs 1.1 V panel) | **FIXED** — knob deleted (zero consumers), field kept as reserved to preserve struct layout | `config.h`, `config.c` |
| F20 / N16 | MEDIUM | No I2C bus recovery | **FIXED** — 9-clock SCL + STOP recovery, re-init after 3 consecutive all-fail reads | `sys_sensors.c` |
| F21 / N14 | — | Compensation table kink at −50/−55 °C | **DO-NOT-TOUCH** — bench calibration will supply correct values | `lora_app.c:601` |
| F22 / N11 | MEDIUM | GPS reconfig + PCAS00 flash-save on every boot | **FIXED** — gated to COMMISSIONING (ADR-0008) | `lora_app.c` LoRaWAN_Init |
| F23 | LOW | `TEST_UltraMinimal_STOP2()` brick trap | **FIXED** — deleted | `main.c` |
| F24 | LOW | SOS/StopJoin residue (dead EXTI3, dangerous ABP flip) | **FIXED** — EXTI3 config/handler, StopJoin, StopJoinTimer all removed | `main.c`, `stm32wlxx_it.c`, `lora_app.c` |
| F25 | LOW | LED + `HAL_Delay(50)` in `EnvSensors_Read` | **FIXED** — removed | `sys_sensors.c` |
| F26 | LOW | `flash_log.h` comment contradicts ADR-0004 | **FIXED** — header docs now describe sectors 0/1 + data from sector 2 | `flash_log.h` |
| F27 | LOW | Float printf without `-u _printf_float` | **FIXED** — three `%.1f` sites converted to integer deci-prints. FW-16: five more sites found + converted (atgm336h GPS summary x2, multiregion_h3 nearest/detection logs x3) | `lora_app.c`, `atgm336h.c`, `multiregion_h3.c` |
| FW-17 | LOW | LinkCheck log prints garbage DemodMargin/NbGateways when no LinkCheckAns received | **FIXED** — margin/gateway-count logged only when a LinkCheckAns was actually received | `lora_app.c` |
| FW-18 | LOW | Stale `LOW_POWER_DISABLE` comment/block on flight-critical flag | **FIXED** — removed | `sys_conf.h` |
| FW-6 | MEDIUM | Voltage-slope Δt amplification (tiny Δt → 10 mV ADC noise = spurious SURVIVAL) | **FIXED** — 600 s minimum Δt before recompute; returns last valid slope otherwise (new `last_slope_mv_per_hour` in `VoltageSlope_t`); covers dt==0 | `lora_app.c::CalculateVoltageSlope`, `lora_app.h` |
| FW-10 | MEDIUM | `LORAWAN_FORCE_REJOIN_AT_BOOT` brick switch (FLIGHT + virgin bank = permanent RF silence) | **FIXED** — context clear gated behind `MissionState_IsCommissioning()` (door anchored before the check), compile-time `#warning` tripwire when flag true | `lora_app.c` |
| FW-9 | MEDIUM | `FindContextSlot()` region-sentinel collision (AS923=0 matches erased-slot zeros) | **FIXED** — slots with DevAddr 0/0xFFFFFFFF skipped before the region compare | `multiregion_context.c::FindContextSlot` |
| FW-7 | MEDIUM | Pressure data dishonesty (failed MS5607 read transmits 1000.0 hPa sea-level default as real float-altitude data; sentinel check mis-counts a legit 1000.0 hPa as bus failure) | **FIXED** — last-known-good cache + `press_stale` flag mirroring F9 SHT31 pattern; real `ms_ok` boolean replaces sentinel; stale bit flows sensor_t → `FlashLog_Record_t.flags` (b0) → uplink status byte b5. Reset cause condensed 3-bit → 2-bit (b3–b4: POR/BOR+LP / IWDG / SW+PIN / FAULT) to free b5; ADR-0007 amended | `sys_sensors.c/h`, `reset_cause.c/h`, `payload_format.h`, `payload_encode.c`, `flash_log.c/h` |
| FW-8 | MEDIUM | `GNSS_EnterStandby()` comment/code contradiction (code cuts PB10+PB5 for full power-off 0µA; comments claimed PB10-HIGH hot-start ~15µA) | **FIXED** — comments in `atgm336h.c` (`GNSS_EnterStandby`, `GNSS_PowerOff`), `lora_app.c`, `stm32_lpm_if.c:193` corrected to describe actual full-power-off behavior with PCAS12 flash-persisted ephemeris. Behavior unchanged; if bench shows TTF is not hot-start-fast, restoring PB10-HIGH standby is a separate measured fix | `atgm336h.c`, `lora_app.c`, `stm32_lpm_if.c` |
| P1-17 | HIGH | Debug payloads default-ON + DR side effect | **PARTIAL** — defaults=0 done; DR save/restore pending | `payload_format.h`, `lora_app.c:1601` |
| FW-12 | MEDIUM | No frontier scan on init (header persisted every 10 records → up to 9 uncheckpointed records can reuse sequence numbers after power cut, wedging MarkRecordsTransmitted) | **FIXED** — `FlashLog_FrontierScan()` in `FlashLog_Init()`: after loading the best header, probe forward up to HEADER_UPDATE_INTERVAL slots (wrap-aware); each magic+CRC-valid record advances write_addr/record_count/oldest_addr. Erased slot reads 0xFF (magic fails fast), CRC failure = torn record at the true frontier | `flash_log.c::FlashLog_FrontierScan` |
| P2-12 / FW-14 | MEDIUM | W25Q deep-power-down commented out (dead `hw25q_ptr` plumbing) | **FIXED** — `hw25q` global extern'd from main.c; `W25Q_PowerDown()` in EnterStopMode, `W25Q_ReleasePowerDown()` in ExitStopMode after SPI re-init (tRES1 covered by in-driver 1 ms delay) | `stm32_lpm_if.c` |
| P2-13 / FW-15 | MEDIUM | VREFBUF disabled, never re-enabled (ratiometric VDDA path reads VREFINT after every STOP2 wake) | **FIXED** — `HAL_SYSCFG_EnableVREFBUF()` in ExitStopMode | `stm32_lpm_if.c` |
| P2-14 | LOW | Stale 300000ms comment | **FIXED** | `lora_app.h` |
| P2-15 | LOW | Geofence dead code, bulk TODOs, blocking delays | **OPEN** | various |
| T1 | BLOCKER | Session integrity: two-tier storage, one-way door | **FIXED** — ladder half (COMMISSIONING-only rejoin, FLIGHT RF-silence) + storage half (FW-1): Tier-1 credentials as 3 redundant CRC'd copies (pages 120–122, written once at commissioning, read-back verified, restore-repair of bad copies, never erased in flight); Tier-2 frame counters ping-pong (pages 123/124, erase-before-write, sequence-newest-wins); counter-loss degrade = Tier-1 + C6 margin; door anchors to Tier-1 presence. Bench gate B5 extended (power-cut × 50) | `lora_app.c`, `multiregion_context.c`, `STM32WLE5JCIX_FLASH.ld` (FLASH now 240K) |
| T2 | HIGH | Data honesty: stale bits + status byte | **FIXED** — stale bits (GPS/temp/hum) + status byte live; flash record carries reserved flags field for future stale bits | `sys_sensors.c`, `payload_encode.c` |
| T3 | BLOCKER | Mission state machine (COMMISSIONING/FLIGHT) | **FIXED** — door anchored to session bank, one-way transitions, join + GPS-config gated to COMMISSIONING. FW-3: `MissionState_Update()` now called each work cycle (was defined but never called — ASCENT never transitioned to FLOAT) | `mission_state.c`, `lora_app.c:1094` |
| T4 | BLOCKER | Flash ring rewrite to ADR-0004 | **FIXED** — retires P0-3, F15, F26 (bench gate B1 remains) | `flash_log.c`, `flash_log.h` |

## h3lite Region Engine (ReviewFixImplementationPlan Phase 2)

Submodule `Middlewares/Third_Party/h3lite` @ `8d15d6b` (firmware pointer bumped in `dc9e1a2`). Verification harness: `h3lite/test/{xval_pts.c, t_index.py, t_city.py, t_table.py}`.

| ID | Issue | Status | Verified at |
|----|-------|--------|-------------|
| H3-1 / H3-4 | Res-1/res-2 table entries unmatchable (no resolution in key); 12 B/entry struct | **FIXED** — packed 4 B `RegionEntry` (baseCell/res/partialIndex/regionId), `findRegion()` binary search, `h3ToRegion()` probes res3→res2→res1 first-hit-wins | `h3lite/src/h3lite.c`, `generate_lookup_table.py` |
| H3-2 | 269 duplicate / 35 conflicting keys; order-dependent region-blind compaction | **FIXED** — uniform-only compaction, deterministic conflict resolution (largest intersected area, lowest-ID tiebreak), 0.6° seaward buffer for coastal erosion | `generate_lookup_table.py` |
| H3-3 | NZ→AS923-1C, HK→CN470 assignments vs RP002 | **AUDITED, DATA FROZEN** — full discrepancy report in `docs/RegionDataAudit.md`; AS923-1B/1C/CD900-1A are non-existent plans; HK is a cell-granularity issue (needs small-territory override). Data changes pending user review | `docs/RegionDataAudit.md`, `hplans/audit_vs_rp002.py` |
| H3-5 | Unconditional `printf()` in `h3ToRegion()` hot path | **FIXED** — gated behind `H3LITE_DEBUG_PRINT` | `h3lite/src/h3lite.c` |
| H3-6(1) | `h3GetRing` latent buffer bug (no actual-count return) | **FIXED** — returns actual count, −1 on pentagon/failure; caller loops on returned count; pentagon limitation documented (846/927 rings exact, 81 clean failures, 0 wrong-content) | `h3lite/src/h3lite_neighbor.c`, `h3lite.c`, `h3lite/README.md` |
| H3-7 | `getRegionName` clamp kills CN470/EU433/CD900-1A names | **FIXED** — sizeof-based clamp + `REGION_RESTRICTED`→"RESTRICTED" guard | `h3lite/src/h3lite.c` |
| H3-8 | `REGION_RESTRICTED` (255) unreachable — no restricted-region data | **OPEN** — blocked on region-data review (needs NK GeoJSON → regionId 255, 255-always-wins in generator) | `lora_app.c::SendTxData` (branch exists) |
| H3-9 | Dead `baseCellTable` (576 B misleading data) | **FIXED** — deleted | `h3lite/src/h3lite.c`, `h3lite_faceijk.c` |

**Measured baselines (host harness):** T-INDEX 0/259,200 mismatches (0.5° global grid vs `h3` 4.5.0) · T-CITY 49/52 correct (3 fails = Wellington/Auckland/Hong Kong — region-data issues frozen per H3-3, not engine bugs) · T-TABLE 0 duplicate keys / 0 conflicts.

## Bench-Verify Checklist (hardware, before launch)

| # | Test | Status |
|---|------|--------|
| B1 | Flash: 30+ records, power cut mid-write, reboot → recovery, no sequence reuse, torn record skipped | PENDING |
| B2 | EU868/AU915/AS923 bulk at SF7 accepted with real FOpts state | PENDING |
| B3 | LSE disabled → LSI failover, keeps logging/TX, telemeters event | PENDING |
| B4 | Injected HardFault → breadcrumb, reset, fault cause in next uplink | PENDING |
| B5 | Commissioning door: power-cycle → FLIGHT; corrupt state record → still FLIGHT | PENDING |
| B6 | SHT31 unplugged → stale bit, lockout treats temp as COLD, no +18 °C | PENDING |
| B7 | Clock/UART-baud sanity after first STOP2 wake | PENDING |
| B8 | HackRF GPS sim > 18 km → ATGM336H holds fix (PCAS11 airborne) | PENDING |
| — | IWDG option-bit (IWDG_STOP) state on production board; consider IWDG_SW option byte so IWDG is hardware-enabled from reset (FW-5 follow-up) | PENDING |
| — | End-to-end energy budget (overnight solar survival) | PENDING |

## Flash Size Budget

| Build | total (text+data) | headroom (of 256KB) |
|-------|-------|---------------------|
| Post-Phase-1–6 (2026-08-01) | 239,096 | ~23 KB |
| Post-h3lite region engine (2026-08-02, `dc9e1a2`) | 191,904 | ~70 KB (of 250K) |
| Post-FW-1 two-tier storage (2026-08-02) | 192,808 | ~53 KB (of 240K — linker shrunk 250K→240K for pages 120–124) |

Region table went 65,718 B → 18,224 B (4,556 packed uint32 entries) — a 47,192 B flash saving.

Monitor after each structural phase; T1/T3/T4 add code.

## Verified good — do NOT "fix" (regression risk)

- P0-1 lat/lon encoding (comment "~550 m at equator" is wrong, reality ~611 m — courtesy fix only)
- P0-NEW PCAS11 airborne mode + PCAS00 persistence
- P0-4 Error_Handler degrade-and-continue
- C1 flash map, C4 deferred marking (modulo F15), C6 frame-counter margin
- BUG 1.3 ocean silence, BUG 1.4/1.5 voltage-floor-first ordering, BUG 3.1 NULL guard
- IWDG refresh in GPS wait loop (wrap-safe)
- PB3 solar reading path (no divider correct for ~1.1 V two-wafer panel)
- F21 cold-compensation table entries (awaiting bench measurement)
