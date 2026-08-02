# Stratosonde Firmware — Production Readiness Status Matrix

**Living document.** This file tracks *code truth only* — a row is FIXED only when the fix is in the code, not when the design is agreed. Design rationale lives in `docs/adr/`; the implementation plan and verification evidence live in `docs/FixWorkorderPlan.md`; the authoritative finding list is `archive/stratosonde-fix-workorder-2026-07-31.md`.

**Base commit:** `181f997` · **Last reconciled with code:** 2026-08-01 (post Phase 1–6 implementation; build clean, 239,096 B flash used)

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
| F1 | BLOCKER | Fault handlers are brick traps; IWDG armed late | **FIXED** — fault handlers breadcrumb (magic+code) to RTC backup reg then `NVIC_SystemReset`; cause surfaces in status byte. FW-2: RTCAPB clock + backup access now enabled before breadcrumb read/write (was silently unreadable pre-RTC-init) (bench gate B4) | `stm32wlxx_it.c:102`, `reset_cause.c:21-22` |
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
| F27 | LOW | Float printf without `-u _printf_float` | **FIXED** — three `%.1f` sites converted to integer deci-prints | `lora_app.c` |
| P1-17 | HIGH | Debug payloads default-ON + DR side effect | **PARTIAL** — defaults=0 done; DR save/restore pending | `payload_format.h`, `lora_app.c:1601` |
| P2-12 | MEDIUM | W25Q deep-power-down commented out | **OPEN** | `stm32_lpm_if.c:130-132` |
| P2-13 | MEDIUM | VREFBUF disabled, never re-enabled | **OPEN** | `stm32_lpm_if.c:187` |
| P2-14 | LOW | Stale 300000ms comment | **FIXED** | `lora_app.h` |
| P2-15 | LOW | Geofence dead code, bulk TODOs, blocking delays | **OPEN** | various |
| T1 | BLOCKER | Session integrity: two-tier storage, one-way door | **FIXED** — COMMISSIONING-only rejoin, FLIGHT RF-silence ladder live | `lora_app.c` |
| T2 | HIGH | Data honesty: stale bits + status byte | **FIXED** — stale bits (GPS/temp/hum) + status byte live; flash record carries reserved flags field for future stale bits | `sys_sensors.c`, `payload_encode.c` |
| T3 | BLOCKER | Mission state machine (COMMISSIONING/FLIGHT) | **FIXED** — door anchored to session bank, one-way transitions, join + GPS-config gated to COMMISSIONING. FW-3: `MissionState_Update()` now called each work cycle (was defined but never called — ASCENT never transitioned to FLOAT) | `mission_state.c`, `lora_app.c:1094` |
| T4 | BLOCKER | Flash ring rewrite to ADR-0004 | **FIXED** — retires P0-3, F15, F26 (bench gate B1 remains) | `flash_log.c`, `flash_log.h` |

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
| — | IWDG option-bit (IWDG_STOP) state on production board | PENDING |
| — | End-to-end energy budget (overnight solar survival) | PENDING |

## Flash Size Budget

| Build | total (text+data) | headroom (of 256KB) |
|-------|-------|---------------------|
| Post-Phase-1–6 (2026-08-01) | 239,096 | ~23 KB |

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
