# Stratosonde Firmware — Fix Workorder Implementation Plan

**Date:** 2026-08-01
**Source:** `archive/stratosonde-fix-workorder-2026-07-31.md` (workorder, supersedes raw review)
**Base commit:** `181f997`
**Status:** Static plan. For live status see `ProductionReadinessAssessment.md`. For design rationale see `docs/adr/0006-0008`.

---

## 1. Verification Report — every finding traced against code @ `181f997`

### Confirmed (mechanism verified, fix agreed)

| Finding | Evidence in code |
|---|---|
| F1 fault traps + late IWDG | `stm32wlxx_it.c`: NMI/HardFault/MemManage/BusFault/UsageFault all `while(1)`. `main.c:263`: `MX_IWDG_Init()` after `MX_LoRaWAN_Init()` (which calls `GNSS_Configure` at `lora_app.c:426`). Unprotected boot window real. |
| F2 dead wake classification | `stm32_lpm_if.c:253-254` reads WUTF/ALRAF after HAL IRQ handlers (`stm32wlxx_it.c:269,306`) already cleared them → every wake takes full exit. Naive flag fix would re-sleep over MAC events — warning heeded. |
| F3 LSE dead-end | `stm32wlxx_hal_msp.c`: RTC = `RCC_RTCCLKSOURCE_LSE` only; LSECSS NVIC enabled, handler services SSRU only. |
| F4 infinite join | `multiregion_context.c:933` `while(!g_multiregion_join_success)` self-pets IWDG. **Second path workorder misses:** `lora_app.c:1068` `SendTxData` calls `LmHandlerJoin()` when not joined. T3 gate must close both. |
| F5 only US915 joined | `multiregion_context.c:1011-1049`: EU868/AS923/AU915 join blocks commented out. |
| F6 keys | `se-identity.h:124-139`: AppKey == NwkKey == NwkSKey == AppSKey, committed publicly. AppKey/NwkKey are **live** (used at `multiregion_context.c:913-916` before every OTAA join); NwkSKey/AppSKey/DevAddr in the file are decorative ABP leftovers. |
| F7 DevEUI | Discrepancy confirmed: default `LORAWAN_DEVICE_EUI` = `…0E,09,15` vs US915 region EUI `…20,09,15` (`se-identity.h:99` vs `:102`). Which the NS holds is a bench question; fold into F6 pass. |
| F8 stale GPS | `lora_app.c:1220-1228`: on timeout, last-known position copied with `valid=true` **and** `fix_quality=GNSS_FIX_GPS` — indistinguishable downstream. (TX path does not call `GNSS_GetPosition`; it rolls its own loop.) |
| F9 cheerful defaults | `sys_sensors.c:93-95,145-147,167`: SHT31 fail → +18 °C / 50 %RH. Feeds −55 °C lockout (`lora_app.c:1016`) and `NormalizeBatteryVoltage`. |
| F10 bulk conversion | Partially disagree on mechanism: `ConvertFlashLogToHighRes` (`payload_encode.c:277`) memsets destination first, so failure ships **zeros**, not uninitialized stack. Still a data-honesty violation; same fix (early-out `continue`). |
| F11 record-before-fix | `lora_app.c`: `EnvSensors_Read` (972) → `FlashLog_WriteRecord` (1053) → GPS fix (1137+) → re-read (1340). Archived record carries previous cycle's position with current timestamp. |
| F12 no absolute time | `SysTimeSet` only called inside LoRaMac middleware (Class B/DeviceTimeAns), never by app. ADR-0003 accepted but unimplemented. |
| F14 flash ring cluster | All three confirmed: headers @ 0x0000/0x0100 ping-pong in sector 0, no erase (`flash_log.c:16-17,253-257`); `next_sequence = record_count` reuse (`:373`); reader returns at first CRC error (`:157-158`). |
| F15 off-by-one | `flash_log.c:181-183`: clamps to `next_sequence - 1`. Newest record unmarkable → perpetual retransmit. |
| F16 hardcoded DR | `lora_app.c:1441` DR_0 probe, `:1520` DR_3 bulk. DR integers region-relative: EU868 DR3 = SF9 (~115 B max) → 222 B bulk always rejected. Fix: express as SF, resolve per region at runtime; verify map against in-tree `Region*.c`. |
| F17 status byte | `payload_encode.c:99` removal comment present; ADR-0005 mandates 10+1=11 B. Byte free. |
| F18 VDDA constant | `adc_if.c:200` `* 3300 / 4096 * 2` while `SYS_GetBatteryLevel` (`:147-168`) already measures true VDDA via VREFINT. **Same bug in `SYS_GetSolarVoltage` (`:228`)** — fold into same ratiometric fix. |
| F19 solar threshold | Grepped all consumers: `solar_charging_threshold` exists only in `config.c:224` (write) and `config.h:89` (declaration). Zero readers — decorative knob. **Decision: delete it.** Raw `solar_mv` telemetry already flows; re-derive from bench data if ever needed. |
| F20 no I2C recovery | No bit-bang recovery anywhere; sensors not power-gated. Standard 9-clock SCL recovery. |
| F22 GPS config every boot | `lora_app.c:426` `GNSS_Configure` runs in `LoRaWAN_Init` every boot, ending with PCAS00 flash-save (`atgm336h.c:264-270`). Comment at `sys_sensors.c:149-150` misleading. |
| F23/F24/F25 hygiene | `TEST_UltraMinimal_STOP2` at `main.c:108` (one uncomment from boot); SOS EXTI3 then PB3 reconfigured analog (`main.c:757-761` vs `:788-792`) + dead `StopJoin`; LED + `HAL_Delay(50)` in `EnvSensors_Read` (`sys_sensors.c:187-189`). |
| F26 flash_log.h comment | Header comment describes ping-pong-in-sector-0 — contradicts ADR-0004. Fix in T4 pass. |
| F27 float printf | No `-u _printf_float` in `.cproject`/build scripts; `snprintf("%.1f…")` at `lora_app.c:1019,1032,1190` prints garbage under newlib-nano. Fix: integer prints (no linker change). |
| F28 docs lie | `ProductionReadinessAssessment.md` claims P0-5/6 FIXED while joins are commented out. Doc described aspiration, not code. |

### Do-not-touch honored
- F21 compensation table kink — left for bench calibration.
- Part 4 verified-good list spot-checked: lat/lon scaling, PCAS11 airborne (`$PCAS11,5*18`), Error_Handler degrade, BUG 1.3 ocean silence, BUG 1.4 lockout ordering, frame-counter margin (C6). Not touched.
- P0-1 courtesy: comment says "~550 m at equator" (`payload_encode.c:31`); reality ~611 m/step. One-character doc fix.

---

## 2. Design decisions (see ADRs)

- **ADR-0006** — Session integrity & one-way commissioning door (T1). Fills the gap: today keys + frame counters share ONE internal-flash page (`MultiRegionStorage_t`), single copy, single CRC — every counter save erases/rewrites the page holding the keys. No redundancy, no tier split, no door anchor.
- **ADR-0007** — Data honesty: stale bits + restored status byte (T2).
- **ADR-0008** — Minimal one-way mission state machine (T3).
- T4 flash ring: ADR-0004 already exists; implement verbatim.

## 3. Implementation phases

Single build. No `#ifdef FLIGHT_BUILD`. All gating by runtime mission state.

**Phase 1 — Small independent root-cause fixes**
1. F11 reorder `SendTxData`: GPS fix → re-read → then flash write
2. F10 early-out `continue` in bulk conversion loop
3. F18 ratiometric battery AND solar voltage via existing VREFINT path
4. F27 integer-only prints
5. F23/F24/F25 deletions (brick-trap test fn, SOS/StopJoin residue, flight-path LED/delay)
6. F19 delete `solar_charging_threshold` (zero consumers)

**Phase 2 — Data honesty (T2 / ADR-0007)**
7. Sensor last-known-good cache + stale bits (F9); lockout treats stale temp as COLD
8. GPS stale bit through flash record + uplink (F8)
9. Status byte restored as byte 11 (F17): b0 GPS stale, b1 temp stale, b2 humidity stale, b3–b5 reset cause (from `RCC->CSR`), b6–b7 mission state (F13b)

**Phase 3 — Mission state + session integrity (T3+T1 / ADR-0006/0008)**
10. Minimal state machine; door anchored to Tier-1 bank; ambiguity → FLIGHT
11. Gate ALL join paths (F4 loop + `lora_app.c:1068` rejoin) to COMMISSIONING; restore all-region commissioning joins (F5); flight degrade ladder → per-region RF silence
12. Gate GNSS_Configure + PCAS00 to COMMISSIONING (F22); fix lying comment
13. F6/F7: template-ize `se-identity.h`, `.gitignore` real keys; rotation + re-provision + re-commission is a manual bench sequence (rotate first, commission once)

**Phase 4 — Boot & fault survival**
14. F1: IWDG first in `main()`; fault handlers → breadcrumb (type, PC, CFSR/HFSR) to RTC backup regs → `NVIC_SystemReset`; boot reads breadcrumb + `RCC->CSR`, logs to flash, surfaces via status byte
15. F13a: progress deadman (ticks since last completed work cycle > ~3× interval → breadcrumb + reset; COMMISSIONING exempt)
16. F2: 25 s UTIL_TIMER tick replaces flag classification (pet IWDG, count; 12 ticks → work cycle; never re-sleep over pending MAC event)
17. F3: LSECSS failover to LSI (breadcrumb to flash, BDRST, re-init RTC on LSI, restart timer server)
18. F20: I2C bus recovery (9 SCL pulses → STOP → re-init) after N consecutive failures
19. F12: GPS UTC → `SysTimeSet` per ADR-0003; flash records carry epoch seconds (uint32)

**Phase 5 — Flash ring rewrite (T4 / ADR-0004 verbatim)**
20. Headers in own sectors, erase-before-write, erase-ahead, sequence-discontinuity frontier, skip-and-continue reader, F15 clamp fix, F26 comment fix

**Phase 6 — Multi-region radio correctness**
21. F16: SF-intent → per-region DR resolver (verify against in-tree `Region*.c`); trim `BulkTelemetryPacket_t` to ≤ 220 B so EU868 DR5 fits with zero FOpts

**Phase 7 — Documentation sync (F28)**
22. `ProductionReadinessAssessment.md` rewritten as truthful living matrix (done at plan time; updated as phases close)
23. Sync `FlashLogging.md`, `PayloadFormats.md`, `PowerManagement.md`, `GNSSModule.md`, `TransmissionModule.md`, `ErrorHandler.md`, `MultiRegionSupport.md`, `MultiRegionImplementationGuide.md`

## 4. Bench gates (acceptance, from workorder Part 3)

B1 flash power-cut recovery · B2 EU868/AU915/AS923 bulk acceptance · B3 LSE failover · B4 HardFault breadcrumb · B5 commissioning door · B6 SHT31 unplug stale behavior · B7 clock/UART after STOP2 · B8 HackRF >18 km airborne fix

---

## 5. Implementation status (2026-08-01) - CODE COMPLETE, BENCH GATES PENDING

All seven phases implemented and building clean (239,096 B flash used, ~23 KB headroom).
Status of each work item is tracked per-row in `docs/ProductionReadinessAssessment.md`
(single source of truth). Highlights:

- Phase 1-2: DONE (previously). Data-honesty stale bits, status byte, payload fixes.
- Phase 3: DONE. T1/T3 gates live - rejoin + GPS reconfig are COMMISSIONING-only;
  FLIGHT with no session = RF silence, profile keeps flying. F6/F7: se-identity-template.h
  committed, se-identity.h gitignored (key rotation + git purge = bench step).
- Phase 4: DONE except F2 (PARTIAL - chunked sleep works, UTIL_TIMER tick redesign deferred).
  F1 breadcrumb+reset fault handlers, F3 LSE->LSI failover, F12 GPS->SysTimeSet,
  F13a progress deadman (3h, COMMISSIONING exempt), F20 I2C 9-clock bus recovery.
- Phase 5: DONE. T4 flash ring: headers in separate sectors (v3), erase-before-write,
  torn-write-tolerant reader, F15 clamp fixed, F26 comments fixed.
- Phase 6: DONE. DatarateFromSF() resolves SF7/SF10 per active region.

Remaining before launch: bench gates B1-B8 + IWDG option bits + energy budget
(see ProductionReadinessAssessment.md), F6/F7 key rotation, F2 tick redesign,
P2-12/13/15 low-power refinements (optional, non-blocking).
