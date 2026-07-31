# Stratosonde Firmware — Production Readiness Assessment & Implementation Plan

**Generated:** 2026-07-30  
**Base commit:** ff1cf59 (HEAD master)  
**Source docs:** `docs/archive/stratosonde_prelaunch_worklist.md`, `docs/archive/stratosonde_future_improvements.md`  

---

## Executive Summary

Every code-checkable claim in the pre-launch worklist and future-improvements doc has been verified against the live codebase. **All reported bugs are confirmed still present.** One additional critical finding was discovered: the GNSS "high altitude mode" command (`PCAS04,5`) is actually a constellation-selection command, not the airborne dynamic-model command (`PCAS11,5`). This means the sonde will lose GPS fix above ~18 km — the classic CoCom limit failure.

This document tracks verification status, implementation plan, and serves as the living checklist through to flight readiness.

---

## Verification Matrix

| ID | Priority | Issue | Verified | Status | Files |
|----|----------|-------|----------|--------|-------|
| P0-1 | BLOCKER | Lat/lon saturates ±29.5° | ✅ `payload_encode.c:29,385-417` | **FIXED** ✅ | `payload_encode.c`, `payload_format.h` |
| P0-2 | BLOCKER | IWDG reboot-loop in STOP2 | ✅ `main.c:508-530` no IWDG_STOP option bit | **OPEN** | `main.c`, `stm32_lpm_if.c` |
| P0-3 | BLOCKER | Flash headers both in sector 0, no erase | ✅ `flash_log.c` HEADER_A=0x0000, B=0x0100 | **OPEN** | `flash_log.c`, `flash_log.h` |
| P0-4 | BLOCKER | Error_Handler bricks sonde | ✅ `main.c:804` `__disable_irq(); while(1)` | **FIXED** ✅ | `main.c` |
| P0-5/6 | BLOCKER | Multi-region disabled 3 ways + infinite join | ✅ Only US915 compiled; auto-switch=0; EU868+ commented out; infinite `while(!success)` loop | **OPEN** | `lorawan_conf.h`, `multiregion_context.h`, `multiregion_context.c` |
| **P0-NEW** | **BLOCKER** | **GNSS PCAS04 is constellation-select, NOT airborne mode; need PCAS11,5** | ✅ `atgm336h.h` defines `PCAS04,5` as "high altitude" | **FIXED** ✅ | `atgm336h.h`, `atgm336h.c` |
| P1-8 | HIGH | LoRaWAN keys in public repo | ✅ Real keys in `se-identity.h` | **OPEN** | `se-identity.h`, `.gitignore` |
| P1-10 | HIGH | Poisoned sensor defaults (18°C/50%/Nice) | ✅ `sys_sensors.c:90-94` | **OPEN** | `sys_sensors.c`, new `sensor_cache.c/h` |
| P1-11 | HIGH | 10-byte packet, status byte removed, DR flip-flop | ✅ struct=10B; DR0@:1441, DR3@:1520, default@:1601 never restored | **OPEN** | `payload_format.h`, `payload_encode.c`, `lora_app.c` |
| P1-13 | HIGH | CRC-bad record wedges bulk backlog | ✅ `flash_log.c:157` returns on error | **OPEN** | `flash_log.c` |
| P1-16 | HIGH | RTC never set from GPS; uint16 wraps 45.5d | ✅ No GPRMC→RTC push anywhere | **OPEN** | `atgm336h.c`, `payload_encode.c`, `lora_app.c` |
| P1-17 | HIGH | Debug payloads default-ON + DR side effect | ✅ `ENABLE_DEBUG_LPP=1`, `ENABLE_GNSS_DETAIL_PACKET=1` | **PARTIAL** (defaults=0; DR save/restore pending) | `payload_format.h`, `lora_app.c` |
| P2-12 | MEDIUM | W25Q deep-power-down commented out | ✅ `stm32_lpm_if.c` call commented | **OPEN** | `stm32_lpm_if.c` |
| P2-13 | MEDIUM | VREFBUF disabled, never re-enabled | ✅ Only DisableVREFBUF in app code | **OPEN** | `stm32_lpm_if.c` |
| P2-14 | LOW | Stale 300000ms="30 seconds" comment | ✅ `lora_app.h` | **FIXED** ✅ | `lora_app.h` |
| P2-15 | LOW | Geofence dead code, bulk TODOs, blocking delays | ✅ Multiple locations | **OPEN** | Various |
| P2-16 | NONE | ADC VDDA — false alarm | ✅ Already uses VREFINT | N/A | — |
| P2-17 | NONE | GPS standby — code correct | ✅ Proper PCAS12 standby | BENCH-VERIFY only | — |

---

## Implementation Phases

### Phase 0 — Security & Repo Hygiene
- [ ] **Key handling:** Convert `se-identity.h` to template with zeroed keys; real keys via untracked `se-identity-keys.h`
- [ ] **`.gitignore`:** Add key file exclusion
- [x] **Build script:** `build.ps1` + `build.bat` created and working (headless, clean build, .bin output)
- [x] **ADR decision log:** `docs/adr/0001-0005` — five foundational decisions documented
- [ ] **README:** Update from "architecture documentation" to proper project description

### Phase 1 — P0 Launch Blockers
- [x] **P0-1:** Lat/lon rescale to `deg × 32767/90` (lat), `× 32767/180` (lon) — **DONE**
- [x] **P0-NEW:** GNSS add `$PCAS11,5*18` airborne dynamic model; relabel PCAS04 as constellation-select — **DONE**
- [ ] **P0-2:** IWDG/RTC chunked sleep (wake ≤25s, refresh, compare, re-sleep)
- [x] **P0-4:** Error_Handler → log + degrade-and-continue — **DONE**
- [ ] **P0-5/6:** Enable all regions in `lorawan_conf.h`; bounded join loop; uncomment region joins; NVM erase guard

### Phase 2 — Flash Ring Integrity *(design doc → review → implement)*
- [ ] **Design doc:** `docs/FlashRingDesign.md` — header sectors, erase-before-write, sequence-discontinuity frontier, erase-ahead, boot validation
- [ ] **Owner review required before implementation**
- [ ] **Implement:** P0-3 header split + P1-13 CRC skip-not-stop

### Phase 3 — Remaining P1s  
- [ ] **P1-10:** Sensor last-known-good cache + stale bits
- [ ] **P1-11:** 11-byte packet + status byte + DR discipline (pin & verify before send)
- [x] **P1-17:** Debug flags default=0 — **DONE**; debug paths save/restore DR — pending
- [ ] **P1-16:** GPS→RTC time sync; DeviceTimeReq fallback; widen time field

### Phase 4 — P2 Secondary Fixes
- [ ] **P2-12:** Wire W25Q deep-power-down with idle guard
- [ ] **P2-13:** VREFBUF re-enable on wake
- [x] **P2-14:** Fix APP_TX_DUTYCYCLE comment — **DONE**
- [ ] **P2-15:** Document geofence table as inert; audit blocking delays; note no-downlink

### Phase 5 — Test Infrastructure
- [ ] GNSS data injection seam (mock driver boundary)
- [ ] In-flight decision breadcrumb event log

### Phase 6 — Documentation
- [ ] `docs/GroundDecoder.md` — versioned with payload format
- [ ] `docs/TimeBase.md` — GPS-master / RTC-flywheel design
- [ ] `docs/BenchVerifyChecklist.md` — hardware tests + HackRF Tier-A procedures
- [ ] `docs/FlightOneSuccessCriteria.md` — template
- [ ] `docs/QuickConnectorProtocol.md` — I2C expansion spec from future-improvements §3

---

## Bench-Verify Checklist (hardware, before launch)

| # | Test | Method | Status |
|---|------|--------|--------|
| 1 | IWDG option-bit (IWDG_STOP) state | Read option bytes on production board | PENDING |
| 2 | GPS standby current + warm reacquire time | Bench measurement vs spec | PENDING |
| 3 | VBAT routing (coin cell/supercap vs VDD) | Hardware inspection | PENDING |
| 4 | ADC accuracy after sleep (VREFBUF) | Measure known voltage post-sleep | PENDING |
| 5 | DR0/US915 max app payload = 11 bytes | Build + test on stack | PENDING |
| 6 | Sleep current with W25Q deep power-down | Before/after measurement | PENDING |
| 7 | **GNSS CoCom altitude test** | HackRF spoof >18km with PCAS11 airborne cmd | PENDING |
| 8 | End-to-end energy budget | Overnight solar survival bench campaign | PENDING |

---

## Flash Size Budget

| Build | text | data | bss | total | headroom (of 250KB) |
|-------|------|------|-----|-------|---------------------|
| Baseline (ff1cf59, US915 only) | 226,844 | 968 | 23,008 | 250,820 | ~23 KB |
| Post-Phase-0+1a (debug off, lat/lon, GNSS, Error_Handler) | 226,852 | 968 | 23,016 | 250,836 | ~23 KB |
| Post-Phase-1 (all regions) | TBD | | | | ⚠️ Watch carefully |

**Note:** Enabling EU868/AS923/AU915 adds several KB each of region-specific MAC code. Monitor after each region enable.

---

## Already Fixed in batch-A (do NOT re-report)

- Ocean silence: only `REGION_RESTRICTED` skips TX; `UNKNOWN` keeps transmitting
- GPS temp lockout ordering after `ApplyOperatingMode`
- Voltage floor <4300 mV → SURVIVAL moved to top of `SelectModeFromPredictions`
- Flash wraparound watermark clamp (`flash_log.c:123`)
- LPM leak: STOP re-enabled on GNSS-wake DMA-fail path
- IWDG NULL-guard in pre-join loop
- Frame-counter margin +interval on save; flash-record marking deferred until `OnTxData`