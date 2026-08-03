# Stratosonde Firmware — Combined Review Verification (2026-08-03)

```text
Branch:            master
Commit:            fa2c638307c97087d1403f81257614c24cbba3e1 (HEAD; h3lite-only delta vs 3dc0d321)
Toolchain:         not rebuilt for this pass (source-evidence verification only)
Build config:      n/a
Build result:      n/a — dispositions rest on direct source evidence, not compilation
Static analysis:   manual inspection + deterministic re-computation (NMEA checksum script)
Reviewer/model:    Cline (Claude) — second-pass verification
Date:              2026-08-03
```

**Sources verified:**
- `docs/archive/stratosonde_firmware_audit_confirmation.md` — findings **F-001…F-030**, architecture items **A-001…A-007**
- `docs/archive/stratosonde-firmware-review-2026-08-03.md` — findings **R01…R44** (reviewed at `3dc0d321`; HEAD differs only by an h3lite submodule commit, so all R citations were re-checked at HEAD)

**Method.** Every finding was confirmed, partially confirmed, or rejected by reading the cited code at HEAD. Vendor code (`LoRaMac.c`, region headers, CMSIS) was read where load-bearing. The R23 NMEA checksums were re-computed by script. Items that can only be fully closed on hardware are marked **bench-gate** — the code side is confirmed, the measurement is pending.

**Disposition key:** CONFIRMED / PARTIAL (real but narrower, or latent) / REJECTED / BY-DESIGN (real behavior, deliberate — needs a maintainer decision, not a fix).

---

## 1. Cross-reference map (overlap between the two reviews)

| F item | R item(s) | Relationship |
|---|---|---|
| F-001 | R22, R08, R44(main.c) | Error_Handler returns; R22/R44 are specific call-site consequences |
| F-003 | R35 | Same config CRC defect; R35 is the precise version |
| F-004 | R21 (partial) | TX/delivery semantics |
| F-005 | R19, R21 | Bulk ack-by-count; latent because conversion can't fail (R19) |
| F-006 | R13 | Read-side watermark advance + unbounded scan |
| F-007 | R12 | Header ping-pong freshness |
| F-009, F-022 | R14 | EraseAll watermark + watchdog |
| F-012 | R27 | GNSS DMA overrun |
| F-013 | R05, R42 | Pressure floor 950 hPa |
| F-014 | R16 | ADC HAL_MAX_DELAY |
| F-015 | — (extended by N-03) | 16-bit minute timestamp |
| F-024 | R06 (partial) | Altitude width/sign |
| F-025 | R19 | Archived solar = 0 |
| F-026 | R03 | Region SF/DR mapping |
| F-027 | R30 | Commissioning join unbounded |
| F-030 | R28 | Sensor success reporting |

R-items with no F counterpart: R01, R02, R04, R07–R11, R15, R17, R18, R20, R23–R26, R29, R31–R34, R36–R41, R43, R44.
F-items with no R counterpart: F-002, F-008, F-010, F-011, F-016–F-021, F-023, F-028, F-029.

---

## 2. Summary

- **F findings:** 24 confirmed (incl. 4 latent), 5 partially confirmed, 1 by-design, 0 rejected.
- **R findings:** 40 confirmed (incl. 3 latent/bench-gated), 0 rejected, 4 DOC all confirmed. One factual correction (R12 tie-break picks **B**, not A — impact unchanged).
- **New findings from this pass:** N-01 (wire endianness vs docs), N-02 (bulk doc decoder bugs), N-03 (timestamp convention mixing), N-04 (delivery semantics undocumented).
- **Readiness matrix disagreements:** `ProductionReadinessAssessment.md` rows **F3, F13, F16** are marked FIXED but the defects persist (R08, R01/R02, R03 respectively). All three have bench gates still PENDING — exactly the failure mode the R-review's process note warns about.

---

## 3. P0 findings — must fix before any flight

### R01 — Deadman corrupts RTC time base → reset loop in FLIGHT — **CONFIRMED (P0)**

**Evidence at HEAD.** `lora_app.c:1044` `#define DEADMAN_BKP_REG RTC_BKP_DR2`; `timer_if.c:100` `#define RTC_BKP_MSBTICKS RTC_BKP_DR2`. Same TAMP register. `Deadman_MarkProgress()` (`lora_app.c:1047-1052`) writes RTC *seconds* into DR2; `TIMER_IF_GetTime()` (`timer_if.c:400-422`) reads DR2 as the upper 32 bits of a 1024 Hz tick count: `ticks = (MSB<<32) + lsb; seconds = ticks >> 10`. `TIMER_IF_Init()` (`timer_if.c:201`) writes MSBticks=0 every boot, so the first `Deadman_Check()` seeds DR2 with plausible seconds (e.g. 1000); the next `TIMER_IF_GetTime()` computes ≈ `(1000 mod 1024) << 22` ≈ 4.2×10⁹ s; `now - last > DEADMAN_TIMEOUT_S (10800)` → `NVIC_SystemReset()` (`lora_app.c:1068-1073`) → loop forever. Invisible in COMMISSIONING (`lora_app.c:1065` early return). Collateral: flash record timestamps, voltage-slope `now_timestamp`, compact `timestamp_min`, and `SysTimeGet()` inside LoRaMac are all poisoned after the first deadman write.

**Fix.** One change with R02: move the deadman to a free backup register (DR3+; STM32WLE5 has 32 TAMP backup registers — confirm count) behind a single `backup_regs.h` ownership header. Do not route through TIMER_IF helpers — the register itself is the conflict.

**Tests.** Force `MISSION_ASCENT` on bench; log `TIMER_IF_GetTime()` across the first `Deadman_MarkProgress()`; confirm no reset loop and sane timestamps.

---

### R02 — All three ST-reserved backup registers are double-booked — **CONFIRMED (P0)**

**Evidence at HEAD.**

| Register | ST owner | Second claimant |
|---|---|---|
| DR0 | `RTC_BKP_SECONDS` (`timer_if.c:90`) | `MISSION_STATE_BKP_REG` (`mission_state.c:19`) |
| DR1 | `RTC_BKP_SUBSECONDS` (`timer_if.c:95`) | `RESET_CAUSE_BKP_FAULT_REG` (`reset_cause.h:32`) |
| DR2 | `RTC_BKP_MSBTICKS` (`timer_if.c:100`) | `DEADMAN_BKP_REG` (`lora_app.c:1044`) |

`SysTimeSyncFromGnss()` (`lora_app.c:1092-1112`) calls `SysTimeSet()` on **every good GPS fix** (`lora_app.c:1419`); `stm32_systime.c` writes the delta seconds→DR0 and subseconds→DR1 via `BKUPWrite_Seconds/SubSeconds` (verified). Consequences: after the first fix, `MissionState_Init()`'s `0xA55A0000` magic check fails every boot → falls to the bank anchor → `MISSION_ASCENT` (every reset in FLOAT reverts to 3 h ascent cadence, contra ADR-0008); the `0xF17B0000` fault breadcrumb is destroyed (fault resets misreported); and `MissionState_Persist()`/`Deadman` writes corrupt `SysTimeGet()` in the other direction.

**Fix.** Single `Core/Inc/backup_regs.h` owning the map: DR0–DR2 reserved for ST SysTime/timer; `BKP_MISSION_STATE=DR3`, `BKP_RESET_CAUSE=DR4`, `BKP_DEADMAN=DR5`. One commit with R01. Compile-time conflict next time instead of silent corruption.

**Tests.** Dump DR0–DR5 across a GPS fix and across reset; mission-state magic and fault breadcrumb survive; `SysTimeGet()` monotonic.

---

### R23 — Four of eight hardcoded NMEA checksums are wrong — **CONFIRMED (P0)**

**Evidence at HEAD.** Re-computed by script against `Core/Inc/atgm336h.h:165-174`:

| Constant | Sentence | In code | Correct | Status |
|---|---|---|---|---|
| `GNSS_CMD_UPDATE_RATE` | `$PCAS02,1000` | `*2B` | `2E` | **REJECTED by receiver** |
| `GNSS_CMD_SATELLITE_SYS` | `$PCAS04,7` | `*1A` | `1E` | **REJECTED** |
| `GNSS_CMD_FIX_MODE` | `$PCAS11,2` | `*1E` | `1F` | **REJECTED** (see R24 — this rejection is currently load-bearing) |
| `GNSS_CMD_STANDBY` | `$PCAS12,0` | `*1C` | `1E` | **REJECTED** |

`GNSS_SendCommand()` (`atgm336h.c:470-503`) only checks UART TX completion, so `GNSS_Configure()` reports success regardless. Impact: tri-constellation (`PCAS04,7`), 1 Hz update rate, and standby entry never apply; the module runs GPS+GLONASS from the accepted `PCAS04,5`.

**Fix.** Build sentences at runtime with the existing `GNSS_CalculateChecksum()` (`atgm336h.c:798`): `snprintf(buf, "$%s*%02X\r\n", body, cs)`. Add the review's Appendix-A script as a host test/pre-commit guard. **Must land in the same change as R24.**

---

### R24 — `$PCAS11,2` is pedestrian mode and overwrites airborne mode — **CONFIRMED (P0)**

**Evidence at HEAD.** `GNSS_Configure()` (`atgm336h.c:208-276`) sends `GNSS_CMD_AIRBORNE_MODE` (`$PCAS11,5`, checksum OK, applied) at step 3 and `GNSS_CMD_FIX_MODE` (`$PCAS11,2`) at step 6. PCAS11 is the navigation dynamic model — a single setting, last write wins; `,2` = pedestrian (CASIC table: 0 portable, 1 stationary, 2 pedestrian, 3 automotive, 4 sea, 5 airborne<1g). There is no "Auto 2D/3D" selector on PCAS11; the header comment (`atgm336h.h:170`) is fabricated. The only thing preserving airborne mode today is R23's broken checksum on step 6. Fixing R23 alone re-imposes the 18 km CoCom ceiling and ends the flight at the tropopause.

**Fix (same commit as R23).** Delete `GNSS_CMD_FIX_MODE` and its send block; comment at the PCAS11 definition: *"single value, last write wins; only `,5` is valid for this mission; never send any other PCAS11 value."* Add a grep guard. Then fix checksums.

---

### F-011 — GNSS DMA head can equal buffer size → non-terminating consumer loop — **CONFIRMED (P0, timing-rare)**

**Evidence at HEAD.** `atgm336h.c:518-519`: `dma_head = GNSS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(...)`. When NDTR==0 (readable in the window between transfer-complete and hardware reload in circular mode), `dma_head = 512`. The consumer loop (`:565-601`) advances `dma_tail = (tail+1) % 512`, so tail ∈ [0,511] and can **never** equal 512 → `while (dma_tail != dma_head)` never exits. The acquisition loop's `HAL_IWDG_Refresh` is outside this inner loop, so the hang becomes an IWDG reset ~32.8 s later. Rare (one unlucky NDTR sample) but per-flight probable over thousands of calls.

**Fix.** Minimum: `hgnss->dma_head = (GNSS_DMA_BUFFER_SIZE - dma_remaining) % GNSS_DMA_BUFFER_SIZE;`. Preferred (with R27): absolute producer/consumer counters from half/full callbacks.

**Tests.** Host-test the ring arithmetic for NDTR ∈ {512, 1, 0}; fault-inject NDTR=0 on bench.

---

## 4. F-findings matrix (F-001…F-030)

| ID | Disposition | Sev | Evidence at HEAD | Fix / notes |
|---|---|---|---|---|
| F-001 | **CONFIRMED** | Critical | `main.c` `Error_Handler()` logs and returns ("Degrade-and-continue… ADR-0001"); ~20 init call sites continue after failure (e.g. `main.c:198-202` prints "H3Lite initialized successfully" even when init failed; `main.c:206-230` proceeds to `FlashLog_Init` after W25Q failure). Mitigations exist: IWDG armed early (FW-5), deadman, breadcrumbing fault handlers. R08 shows a case (dead LSE) where continue-after-error flies with a stopped RTC. | Split fatal vs recoverable: `_Noreturn Fatal_Reset(reason)` (breadcrumb → safe outputs → bounded log flush → `NVIC_SystemReset`) for clock/flash/radio/bus init; explicit degraded mode for optional sensors. Fix the two false-success prints regardless. |
| F-002 | **CONFIRMED** | High→Low | `config.c:114-116`: `Config_Save()` returns `CONFIG_ERROR_PARAM` when `!g_config_initialized`; `Config_Init()` calls it at `:63` before the flag is set at `:70`. First-boot defaults are never persisted; the recovery path repeats every boot. Benign (defaults in RAM are valid; no flash wear — the save aborts before erase) but exactly the reported defect. | Private `Config_WriteInternal()` not gated on the public flag; set the flag after load-valid or defaults-committed. |
| F-003 | **CONFIRMED (latent)** | High→Low | `config.c:166-169, 334-338`: zeroes `crc32` then hashes `sizeof-4`; but `crc32` is at offset 8 (`config.h:67-72`), so the last 4 bytes (`reserved[12..15]`) are unprotected. Both sides agree today → validation passes; activates on any struct tail change. | Hash the full zeroed-copy struct; bump `CONFIG_VERSION`; add `_Static_assert` on size. |
| F-004 | **PARTIAL** | Critical→Med | Durable path is correct: bulk records are marked only in `OnTxData` on `LORAMAC_EVENT_INFO_STATUS_OK` (`lora_app.c:1907-1916`); `LmHandlerSend` failure leaves the watermark alone (`:1766-1769`). But: (a) the debug `PacketQueue` pops before send and ignores the status (`:1931-1941`) — compiled out by default; (b) a failed probe send drops the packet with no retry (`:1665-1668`) — acceptable for a probe, undocumented; (c) the delivery semantic is "radio TX completed" for unconfirmed uplinks — records can be marked delivered with no gateway reception (see N-04). | Document the delivery semantic; peek/submit/commit for the debug queue; optionally retry probe once on `PAYLOAD_LENGTH_ERROR` (see R04 option c). |
| F-005 | **PARTIAL (latent)** | Critical→Low | `lora_app.c:1752` sets `g_bulk_pending_mark = record_count` (records **read**), not `packed_count` (records **encoded**). Today `ConvertFlashLogToHighRes` cannot fail (R19), so packed==read and the skip path (`:1712-1719`) is dead code. If conversion ever becomes fallible, unencoded records are silently acked. | Mark exactly the sequences encoded (or make the no-skip invariant explicit and assert packed==read). Land with R19/R21. |
| F-006 | **CONFIRMED** | High | `flash_log.c:160-174`: `FlashLog_GetUnsentRecordsFIFO` advances `hlog->last_transmitted_sequence` in RAM when a record fails CRC — a read with a durable side effect (persisted at next header sync). Deliberate (T4 anti-wedge) and self-consistent with count-based ack, but violates the "scans are side-effect-free" invariant; combined with F-007 a stale header restore can re-skip records. | Accept the design but make it explicit: return skipped-corrupt positions to the caller; commit them in the same token as the TX ack (A-002). Bound the loop (R13). |
| F-007 | **CONFIRMED** | High | `flash_log.c:259`: `header.sequence = hlog->record_count`. `MarkRecordsTransmitted→SyncHeader` writes a header with unchanged `record_count` → two valid headers, equal "freshness", different `last_transmitted_seq`. Tie-break (`:353`) picks **B** on equality (the R-review said A — correction; impact identical): up to one batch of watermark progress lost → duplicate retransmission, not data loss. | Dedicated monotonic header generation, incremented every `FlashLog_WriteHeader`, wrap-safe compare. |
| F-008 | **CONFIRMED** | High | `flash_log.c:429-431`: `FlashLog_FrontierScan` returns immediately when `record_count == 0`. Fresh init writes a valid count-0 header (`:392`); records 1–9 written before the first 10-record checkpoint are then orphaned by power loss (write_addr reused, records overwritten). | Delete the early return — the scan is bounded (≤10 probes) and safe from `DATA_START`. Test: power loss after each of records 1–9. |
| F-009 | **CONFIRMED (latent)** | Critical→Med | `flash_log.c:665-692`: `FlashLog_EraseAll` resets write/oldest/count/next_sequence but **not** `last_transmitted_sequence` → new records appear already-delivered; bulk transfer permanently disabled. Also 512 sector erases × ≤400 ms with no IWDG refresh ≫ 32.76 s watchdog. Currently **uncalled** (grep-verified) → latent. | Reset the full logical state incl. watermark; IWDG refresh per sector; wire to a commissioning-only command or delete. (= R14/F-022.) |
| F-010 | **CONFIRMED** | High→Med | `flash_log.c:491`: `record.sequence = hlog->next_sequence++` executes before the flash write (`:523`); a failed write consumes a sequence number → gap. Self-heals (FIFO read CRC-skips the gap, advancing the watermark) but violates the contiguity assumption and wastes a probe cycle. | Increment only after write success (+ optional read-back). |
| F-011 | **CONFIRMED** | Critical | See §3. | `% buffer_size` minimum fix; absolute counters preferred. |
| F-012 | **CONFIRMED** | High | = R27. `atgm336h.c:828-846`: half/full callbacks only set a flag; no wrap/epoch counting; `head==tail` cannot distinguish "empty" from "one full lap behind". 512 B buffer = 533 ms at 9600 baud; R26 shows the stream is at/over capacity. Silent NMEA loss; checksum catches splices silently (no counter). | Track producer/consumer totals; on `producer-consumer > 512` declare overrun, resync at next `$`, count it, surface in status byte (ADR-0007). |
| F-013 | **CONFIRMED** | Critical (protocol) | = R05. `payload_encode.c:432-442`: `(pressure-950)/10`, negatives clamped to 0 → 900/500/200/100/50/10 hPa all encode 0; the doc decodes 0 as 950 hPa (R42). Primary altitude proxy dead for the whole flight. | See Decision D2. |
| F-014 | **CONFIRMED** | Critical→Med | = R16. `adc_if.c:276`: `HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY)`; a stalled ADC blocks until IWDG reset; same failure after reboot = reset loop. Also ~10 full ADC init/calibrate cycles per `EnvSensors_Read` (each channel read = `MX_ADC_Init`+calibrate+config+start+poll+stop+DeInit; `SYS_GetBatteryVoltage` internally re-reads VREFINT), and `EnvSensors_Read` runs twice per cycle. | Bounded timeout (10 ms) + `ADC_STATUS_TIMEOUT`; never encode 0 as valid voltage; consolidate acquisition (A-001). |
| F-015 | **CONFIRMED** | High (protocol) | `payload_format.h:93`: `uint16_t timestamp_min` — wraps every 45.5 days; `lora_app.c:1636` truncates `now_timestamp/60`. Extended by N-03: the epoch convention changes mid-mission. | See Decision D4. |
| F-016 | **CONFIRMED** | High | `lora_app.c:2142-2166`: `OnStoreContextRequest` checks erase but ignores the `FLASH_IF_Write` result; `OnRestoreContextRequest` ignores the read result; single copy, no magic/version/CRC/commit. Partially mitigated: FW-1 Tier-1/Tier-2 storage (`multiregion_context.c`) holds the region sessions redundantly — the native NVM page is the MAC's own state. | Transactional two-slot with magic/version/length/generation/CRC/commit for the native NVM page, or formally rely on Tier storage and stop storing natively. |
| F-017 | **CONFIRMED** | Medium (security) | `lora_app.c:468-469` calls `MultiRegion_DisplaySessionKeys()` on **every boot** when a US915 context exists — flight included; the function prints full AppSKey and NwkSKey hex (`multiregion_context.c:1892-1907`). Pre-join prints are commissioning-legitimate. | Gate all key display behind `MissionState_IsCommissioning()` (or a `PROVISIONING_BUILD` flag). `strings` scan of release binary as regression. |
| F-018 | **PARTIAL** | Medium | `Config_Validate` (`config.c:143-190`) has only 3 semantic checks (tx_interval range, low≤bulk, margin≤63). Unchecked: critical<low ordering, gps_timeout vs watchdog, region/SF/DR/TX-power legality, enum/bool ranges, reserved-byte normalization. 24 of 32 fields have no consumers at all (R36). | One complete semantic validator; reject CRC-valid but operationally invalid configs. Prioritize the contradictory fields (R36). |
| F-019 | **CONFIRMED** | High→Med | `Config_FlashWrite` (`config.c:385-404`): erase page, then write — single copy. Brownout between erase and write destroys the previous valid config (falls back to defaults; not fatal, but all commissioning tuning is lost). | Two slots with generation+CRC+commit (A-003), or accept defaults-fallback and document. |
| F-020 | **PARTIAL (latent)** | Medium | `main.c`: `MX_LoRaWAN_Init()` at `:185` runs before `Config_Init()` at `:241`. Currently harmless: the LoRaWAN stack is configured from compile-time defines (`LmHandlerParams`, `lora_app.c:317-327`), and the config LoRaWAN fields are dead (R36). Becomes live the moment any config field is wired into stack init. | Move `Config_Init()` before `MX_LoRaWAN_Init()` (cheap, removes the trap), or classify fields boot-only vs runtime. |
| F-021 | **CONFIRMED** | Medium | `FlashLog_ValidateHeader` (`flash_log.c:223-242`): magic+version+CRC only. No checks that write/oldest addrs are in [DATA_START, DATA_END), 64-byte aligned, count ≤ capacity, watermark ≤ next_sequence. A CRC-valid but semantically impossible header is trusted (frontier scan + record CRCs bound the blast radius). | Add semantic validation; on failure fall to the other header, else clean init. |
| F-022 | **CONFIRMED (latent)** | High | = R14/F-009 watchdog aspect: 512 × ≤400 ms erases, no IWDG refresh. Uncalled today. | Idempotent erase state machine with per-sector IWDG service, or delete. |
| F-023 | **CONFIRMED (latent)** | Medium | `atgm336h.c:284-333`: `GNSS_GetPosition()` blocks up to `timeout` with `HAL_Delay(10)` and **no IWDG refresh**. Grep-verified **no callers** — dead API. | Delete it, or give it a hard deadline + watchdog hook. |
| F-024 | **CONFIRMED** | High→Med | `payload_encode.c:167,279`: `(uint16_t)sensors->altitudeGps` — −500 m → 65036 m on the wire. Also `sys_sensors.h:46-47`: `altitudeGps int16_t` overflows at 32.77 km (mission goes to 40 km per `main.c` comments); `altitudeBar int16_t m*10` overflows at 3.28 km (R06). | Signed wire field or documented offset+saturation; widen GPS altitude to int32 in the flash record (14 reserved bytes exist) with a record-version bump. See Decision D5. |
| F-025 | **CONFIRMED** | High→Med | = R19. `payload_encode.c:288`: `highres_record->solar_voltage = 0` ("Not stored in FlashLog_Record_t yet"); `:289,296` stamp **today's** slope and power mode onto historical records. | Add `solar_mv`, `voltage_slope`, `power_mode` to `FlashLog_Record_t` (reserved space exists) or drop the fields from the wire record. Decision D5. |
| F-026 | **PARTIAL** | High | EU868/AS923 mappings verified correct against in-tree tables; **AU915 wrong** (= R03, confirmed: `DataratesAU915={12,11,10,9,8,7…}` → SF10=DR_2, SF7=DR_5; code returns DR_0/DR_3). AS923-2..4 not compiled. | Table-driven SF→DR resolver searching the region's own `Datarates*[]`; runtime `PHY_MAX_PAYLOAD` guard before send. |
| F-027 | **CONFIRMED** | High | = R30. `multiregion_context.c:1032-1054`: unbounded join wait, IWDG refreshed, spins forever; sequential US915→EU868→AS923→AU915 means commissioning can never complete at any single location; `MissionState_EnterFlight()` (`:1166`) unreachable; `all_success` discarded by caller (`lora_app.c:484`); `MultiRegion_InitializeRegionFromChirpstack()` has **no callers**. | Decision D6. |
| F-028 | **BY-DESIGN (decision needed)** | Medium | `mission_state.c:68-70`: reboot in ASCENT restarts the 3 h timer (`s_ascent_start_tick = HAL_GetTick()`), commented "intentional and documented" (FW-3 constraint). Repeated resets indefinitely defer FLOAT cadence — safe direction (more uplinks), but F-028's concern is real. | Decision D8: persist cumulative ascent elapsed (a backup register is free after R02) vs accept restart. |
| F-029 | **CONFIRMED** | High→Med | `stm32_lpm_if.c:300-363`: `PWR_ExitStopMode` re-inits DMA, I2C2, SPI2, W25Q release, VREFBUF, UART1 with **zero status checks**; a failed re-init is used blind until the next fault. Compounded by R07 (this runs every 25 s). | Subsystem capability mask + bounded recovery; fatal deps → recorded reset; optional → degraded state. |
| F-030 | **CONFIRMED** | Medium | `sys_sensors.c:303`: `EnvSensors_Read` always `return 0`. Staleness flags exist and are good (FW-7/F9), but the top-level return hides total acquisition failure; both call sites ignore it anyway. | Per-sensor status bitmask return; distinguish fresh/stale/invalid/never-acquired. |

---

## 5. R-findings matrix (R01…R44)

### P0 (covered in §3): R01, R02, R23, R24 — all CONFIRMED.

### P1

| ID | Disposition | Evidence at HEAD | Fix / notes |
|---|---|---|---|
| R03 | **CONFIRMED** | `lora_app.c:543-567` maps AU915 identically to US915. In-tree `RegionAU915.h`: `DataratesAU915={12,11,10,9,8,7,…}` → "SF10 probe" actually TXs SF12; "SF7 bulk" TXs DR_3=SF9 where max payload is 115 B (dwell0) / 53 B (dwell1) < 198 B → every AU915 bulk uplink rejected by the MAC. EU868/AS923 arms are correct. | `case LORAMAC_REGION_AU915: return (sf==10)?DR_2:DR_5;` — better: table-driven resolver + `PHY_MAX_PAYLOAD` guard. Correct the F16 row in the readiness matrix. |
| R04 | **CONFIRMED** | `sizeof(CompactTelemetryPacket_t)==11` (`payload_format.h:92-101`, asserted at `payload_encode.c:337`); `LmHandlerLinkCheckReq()` is queued before **every** probe (`lora_app.c:1652`); vendor `LoRaMac.c ValidatePayloadLength()` computes `payloadSize = lenN + fOptsLen` vs `MaxPayloadOfDatarateUS915[DR0]=11` → 11+1=12 > 11 → `PAYLOAD_LENGTH_ERROR`, probe lost, cycle completes without TX. Any pending `LinkADRAns`/`DevStatusAns` etc. breaks even a 10 B packet. | **Decision D1.** Add boot-time assertion `sizeof(packet)+worst_FOpts ≤ min over regions`. Coordinate with R05 — same bytes. |
| R05 | **CONFIRMED** | `payload_encode.c:432-442` — see F-013. `ConvertPressureToCompact(75.0f)==0`. | **Decision D2.** Regenerate decoder + `PayloadFormats.md` in the same change (R40–R42). Also revisit `humidity_5pct` (too coarse for UT/LS science). |
| R06 | **CONFIRMED** | `lora_app.c:1127` `sensor_t sensor_data;` (uninitialized); `EnvSensors_Read` (`sys_sensors.c:160-305`) never assigns `altitudeBar` (grep-verified tree-wide); `flash_log.c:503` writes it into a CRC-covered record → indeterminate data archived as authentic (ADR-0007 violation, UB). Width: `altitudeGps int16_t` overflows 32.77 km; `altitudeBar int16_t m*10` overflows 3.28 km. | `= {0}` at both call sites (necessary, not sufficient); **Decision D5** on field fate; widen/saturate with flag. |
| R07 | **CONFIRMED** | `stm32_lpm_if.c:275-276` reads `EXTI->PR1` bits 19/17. CMSIS `stm32wle5xx.h` defines `EXTI_PR1_PIF0…PIF16` then jumps to `PIF21/PIF22` — lines 17–20 are direct lines with **no pending bit**; and the wakeup timer is line **20** (`stm32wlxx_hal_rtc_ex.h` `RTC_EXTI_LINE_WAKEUPTIMER_EVENT = EXTI_IMR1_IM20`), not 19. Both tests always false → full `PWR_ExitStopMode()` peripheral re-init every 25 s (~12× per 5-min cycle): DMA, I2C, SPI, W25Q release, VREFBUF wait (R17), vcom, LED (R09). The F2 comment claims this exact symptom was fixed. | Latch wake source in `HAL_RTCEx_WakeUpTimerEventCallback` / the Alarm-A callback (timer_if USER block); test the latches; keep the 150-chunk bound. Bench: scope a GPIO in `PWR_ExitStopMode`. |
| R08 | **CONFIRMED** | `main.c:317-333` LSE→LSI failover sets `RCC_RTCCLKSOURCE_LSI`; later `HAL_RTC_MspInit` (`stm32wlxx_hal_msp.c:239-245`) unconditionally requests LSE; `HAL_RCCEx_PeriphCLKConfig` forces a **backup-domain reset** when the source differs (wiping mission state, reset cause, deadman, SysTime) and selects the dead LSE → `HAL_RTC_Init` times out → `Error_Handler()` no-ops → flying with a stopped RTC. Worse than no failover. Readiness-matrix F3 row says FIXED (bench gate B3 PENDING). | File-scope `g_rtc_clock_source` set in `SystemClock_Config`, honored in `HAL_RTC_MspInit`. Bench: kill LSE, confirm RTC runs on LSI and backup regs survive. |
| R09 | **CONFIRMED** | `stm32_lpm_if.c:122,306`: PA0 LED driven LOW on sleep entry, HIGH on every wake — in flight, contra F25/ADR-0008 (LEDs commissioning-only). On for the whole GPS window (see R25), whole TX cycle, and every 25 s chunk wake (R07). | Gate on `MissionState_IsCommissioning()`; drive PA0 analog/low in flight sleep. Measure current delta over one work cycle. |
| R10 | **CONFIRMED** | `lora_app.c:1136,1160`: `SelectModeFromPredictions` receives **normalized** voltage; the absolute floor `:815` (`< 4300 → SURVIVAL`) is defeated by up to +2700 mV of compensation at −66 °C — a real 3.6 V pack at −60 °C normalizes to ~4.4 V and never trips. Also: comp table non-monotonic (`{-40,700}` then `{-50,400}`, `:676-677`); normalization ignores `temp_stale` (`:1136`); no hysteresis (`:1161` direct assignment; `power_mode_hysteresis` config dead — R36); `slope < -5 → REDUCED` (GPS off) regardless of state of charge (`:833`). | Raw voltage to the absolute floor, normalized to slope/prediction (signature takes both); fix/re-derive the −50 °C entry + build-time monotonicity check; gate normalization on `!temp_stale`; add hysteresis + SoC term. Table-test the mode function across the temp/voltage grid. |
| R25 | **CONFIRMED (structural); bench-gate for magnitude** | `atgm336h.c:1197-1198`: `GNSS_EnterStandby` drives **both** PB10 and PB5 low → no backup rail → cold start every cycle (~30–35 s, MCU held awake by `UTIL_LPM_SetStopMode(...DISABLE)`). The PCAS12 "persists ephemeris" rationale is doubly unfounded: the checksum is wrong (R23) so it never arrives, and PCAS12 is a standby-entry command, not an ephemeris save. Comments at `atgm336h.c:1100-1105,1116-1120`, `stm32_lpm_if.c:194-198`, `lora_app.c:1243-1246` all repeat the claim. Likely the largest single power-budget item. | **Measure first** (highest-value bench test): log `ttf_ms` over 20 cycles; repeat with PB10 held HIGH / only PB5 cut; measure standby current. Then fix comments, fix or drop PCAS12, re-tune `gps_timeout_*`. |
| R26 | **CONFIRMED** | `atgm336h.h:165`: `$PCAS03,1,0,0,1,1,1,0,0` → field order nGGA,nGLL,nGSA,**nGSV**,nRMC,nVTG → GSV **enabled**. Only GSV consumer (`EncodeGNSSDetailPacket`, `lora_app.c:576`) is gated by `ENABLE_GNSS_DETAIL_PACKET 0` (`payload_format.h:41-43`). Multi-constellation GSV at 1 Hz can exceed the 9600 B/s link alone (~1050 B/s) → saturation, drives R27. RTT log at `atgm336h.c:218` says "GGA+RMC only" — false. | Set nGSV=0 in the flight mask (keep a commissioning variant); consider `PCAS01` baud raise; fix the log string. |
| R27 | **CONFIRMED** | = F-012. | See F-012. |
| R28 | **CONFIRMED** | `ms5607.c:234-237`: proceeds when `!IsInitialized` ("DIAGNOSTIC MODE"); `:315-318`: logs on `d1==0||d2==0||d1<1000||d2<1000` then **continues**; `d1==0` → `p = (-off)>>15` → large negative pressure returned as `MS5607_OK` → `sys_sensors.c:203-208` caches it as last-known-good and clears the stale flag — poisons the FW-7 mechanism at its design failure mode. (SHT31 CRC-8 checks are correct — accepted from the review; no plausibility gate there either.) | Return `MS5607_ERROR` on both branches; plausibility gate before cache accept (`1–1200 hPa`, `−90…+85 °C`); same gate for SHT31. |
| R29 | **CONFIRMED** | `w25q16jv.c:91-152`: `W25Q_Init` verifies JEDEC ID but never reads SR1 block-protect bits (`0x7C`) or SR2 SRL. With BP set: WriteEnable succeeds, PageProgram is ignored by the device, BUSY never asserts, `W25Q_WaitReady` returns OK immediately, no read-back exists anywhere → `FlashLog_WriteRecord` returns OK forever while storing nothing. Also: only 3 `initialized` references in the file — public entry points check `hw25q==NULL` but not `hw25q->initialized`; `main.c:214-230` calls the no-op `Error_Handler` on W25Q failure then runs `FlashLog_Init` on an unconfigured handle. | Read SR1 in init; clear or distinct-error on BP; log SR1/SR2; post-init write/read-back self-test reported at boot; `if (!hw25q->initialized) return W25Q_ERROR_INIT;` on public entries; make W25Q-init failure fatal (F-001 split). |
| R30 | **CONFIRMED** | = F-027. | **Decision D6.** |

### P2

| ID | Disposition | Evidence at HEAD | Fix / notes |
|---|---|---|---|
| R11 | **CONFIRMED** | `lora_app.c:1453-1457`: restricted-region early `return` precedes the flash-log write at `:1530` — over a restricted region the sample is discarded entirely; the archive exists precisely for data that can't be transmitted. | Use the existing `rf_silence` pattern (`:1228,1628`): set a flag, let GPS + re-read + flash write proceed, skip only the TX state machine. |
| R12 | **CONFIRMED** (with correction) | = F-007. Tie-break picks **B** on equality (`flash_log.c:353` else-branch), not A as the review states — impact unchanged (stale watermark → duplicate retransmission of ≤1 batch). | Dedicated monotonic header generation counter. |
| R13 | **CONFIRMED** | `flash_log.c:160-174`: corrupt-run loop bounded only by `next_sequence` — up to ~32,600 SPI reads in one call, no IWDG refresh, no iteration cap. | Cap probes per call (e.g. 256) and resume next call; refresh IWDG inside; return gathered count (caller already handles 0). |
| R14 | **CONFIRMED (latent)** | = F-009/F-022. Uncalled (grep-verified). | Fix + commissioning-gate, or delete. |
| R15 | **CONFIRMED** | `lora_app.c:1738`: `bulkData.BufferSize = sizeof(BulkTelemetryPacket_t)` — always 198 B even for 1 record (~333 ms vs ~102 ms airtime at SF7/125). Zero-fill is honest (count in header) but burns EU868 duty cycle and energy. | **Decision D3:** variable-length serialization `2 + 32*n + 4`, CRC after last record; v3 packet_type; decoder + doc in same change. |
| R16 | **CONFIRMED** | = F-014. `adc_if.c:245-289`: full init+calibrate+deinit per channel read; `SYS_GetBatteryVoltage` internally calls `SYS_GetBatteryLevel` (2 cycles); ~10 ADC cycles per `EnvSensors_Read`, which runs twice per `SendTxData`, plus direct reads and `OnRxData` — order 16–25 per work cycle. Precision note confirmed: `((level*vdda/4096)*2)` doubles truncation (`adc_if.c:206`). | Bounded poll timeout; init/calibrate once; cache VDDA per cycle; `(level*vdda*2)/4096`. Fold in A-001. |
| R17 | **CONFIRMED** | `stm32_lpm_if.c:188,340`: `HAL_SYSCFG_Disable/EnableVREFBUF` — VREFBUF is the **external** VREF+ buffer, not VREFINT (the ADC HAL enables VREFINT automatically for `ADC_CHANNEL_VREFINT`). The FW-15 rationale misidentifies it. `EnableVREFBUF` busy-waits up to the timeout for VRR and returns `HAL_TIMEOUT` — ignored; ×12 per cycle via R07. | Remove both calls (verify the package exposes VREF+ first — likely not on this board). If VREFINT quiescent matters, use `ADC->CCR VREFEN`. |
| R18 | **CONFIRMED (code); board question open** | `adc_if.c:217-242`: PB3 solar, no divider, no scaling — a 5.5 V LTO charge system both clips at VDDA and exceeds the pin's absolute-max (VDDA+0.3 V) through the ESD structures. | **Inspect the board**: if a divider exists, fix comment+scaling (telemetry under-reported by the ratio); if not, add one — telemetry is a flat ~3300 mV line whenever the panel produces, and the pin is out of spec. |
| R19 | **CONFIRMED** | = F-025. Also: the F10 "skip unconvertible" path (`lora_app.c:1712-1719`) is dead code because conversion can't fail — the F10 comment misattributes where validation lives (it's upstream in `FlashLog_VerifyRecord`). | Decision D5; fix the F10 comment to point at `FlashLog_VerifyRecord`. |
| R20 | **CONFIRMED** | `sys_sensors.c:291-292`: no-fix branch stores ST demo coordinates (43.6186, 7.0514 = Saint-Ouen, France); `flash_log.c:500-501` stores lat/lon unconditionally; only the flags byte marks them invalid — a decoder that skips flags puts the balloon in France. Also `:294`: `satellites` means "used" with a fix and "in view" without. | Store zeros (or last-known-good per F8) instead of demo coords; keep `satellites` semantics constant. |
| R21 | **CONFIRMED (latent)** | `lora_app.c:1721-1725`: `packed_count==0` → `TX_STATE_COMPLETE`, watermark never advances → every later bulk trigger re-reads the same records. Latent today (conversion infallible, R19); activates with any future fallible conversion. Interacts with F-006's CRC-skip. | Fix with F-005/R19: either advance past unconvertible records explicitly (with a counter) or make conversion infallible by contract + assert. |
| R22 | **CONFIRMED** | `payload_encode.c:332-360`: runtime, non-fatal size "validation" (`main.c:234-237` calls the no-op `Error_Handler` and flies on); `%d` used for `sizeof`. `_Static_assert` precedent exists (`config.h:135`, `flash_log.h:132`). | `_Static_assert(sizeof(CompactTelemetryPacket_t)==11, …)` etc. (update expected values with the R04/R05 rework); fix format specifiers. Fold in A-004. |
| R31 | **CONFIRMED** | `lora_app.c:1310-1311`: pre-acquisition invalidation clears only `valid`/`fix_quality`; `satellites`, `hdop`, `lat/lon` keep last cycle's values; the GGA parser skips empty tokens (`atgm336h.c:935,955,959`), so a GGA with empty sats/HDOP fields lets stale values satisfy `GNSS_IsFixGoodQuality` (`atgm336h.c:444-448`). | `memset(&hgnss.data, 0, …)` before acquisition; last-known-good lives in separate statics. |
| R32 | **CONFIRMED** | `atgm336h.c:969,975`: `if (lat_raw > 0)` / `if (lon_raw > 0)` — NMEA raw coords are non-negative (hemisphere is separate), so this is a non-zero test: crossing the equator or prime meridian silently retains the previous coordinate. Real for a circumnavigating balloon. | Track `have_lat/have_lon` from token presence, not magnitude. |
| R33 | **CONFIRMED** | `atgm336h.c:1056-1060`: `if (checksum_ptr == NULL) return true;` — a DMA-truncated sentence (no `*XX`) is accepted and parsed. | Return false when `*` absent; all ATGM336H output carries a checksum. |
| R34 | **CONFIRMED** | `atgm336h.c:853-858`: float conversion (~1 m quantization); `flash_log.h:107-108` documents "1e-7 degree resolution" (~1 cm) for the archive. Container supports it; producer doesn't. | Use `double` in the conversion (not hot-path) or correct the documented resolution. |
| R35 | **CONFIRMED** | = F-003. | See F-003. |
| R36 | **CONFIRMED** | `config.h:67-133` vs consumers: live = 5 tx_intervals + 2 gps_timeouts + `gps_temperature_lockout` (all via `Config_Get()` in `lora_app.c`). Dead = the other 24, six of which **contradict** hardcoded behavior: `gps_max_hdop_x10=25` vs `hdop<=5.0f` (`atgm336h.c:447`); `battery_critical_threshold=4000` vs `4300` (`lora_app.c:815`); `power_mode_hysteresis=10` vs none; `gps_standby_power_ua=15` vs standby removed (FW-8); `debug_lpp_enabled=1` vs `ENABLE_DEBUG_LPP 0` (`payload_format.h:38`); `battery_low_threshold=4500` used as a prediction target, not a threshold (`lora_app.c:1146`). | Wire up or delete each dead field (keep reserved padding for layout); prioritize the six contradictory ones — they will be trusted in a launch review. |
| R37 | **CONFIRMED** | `multiregion_context.c:1099` and one earlier site: `FLASH_IF_Erase((void*)0x0803F000UL, 2048)` — literal instead of `LORAWAN_NVM_BASE_ADDRESS` (`lora_app.c:125`). (Flash map itself verified correct: linker reserves 16 KB, pages 120–127 non-overlapping.) | Move the symbol + page size to a shared header; use it at both sites. |
| R38 | **CONFIRMED** | MS5607 characterized −40…+85 °C; SHT31 RH meaningless below −40 °C; float altitude is −55…−80 °C. The MS5607 second-order compensation (incl. the `temp < -1500` branch, `ms5607.c:338-344`) is correctly implemented but uncharacterized down there. No flag anywhere. | Add `press_uncharacterised`/`hum_uncharacterised` bits when T < −40 °C, carried in the flash-record flags byte (spare bits exist) and uplink status if space allows (ADR-0007; matters for publication). |
| R39 | **CONFIRMED** | `w25q16jv.c:238`: `HAL_Delay(1)` inside `W25Q_WaitReady` → `TIMER_IF_DelayMs` NOP-spin at full clock during every 45–400 ms erase (every 64 records + 2×/header write, inside the TX cycle). Units note confirmed: `sys_app.c:321-342` overrides `HAL_GetTick()` to return `TIMER_IF_GetTimerValue()` = **1024 Hz RTC ticks**, so every "ms" timeout is ~2.4 % short (margins adequate where checked; `MISSION_ASCENT_DURATION_MS` runs ~2.93 h). | `__WFI()` between polls or drop the delay; document the tick-vs-ms fact once in a header; keep it in mind when tightening any timeout. |

### DOC

| ID | Disposition | Evidence at HEAD | Fix |
|---|---|---|---|
| R40 | **CONFIRMED** | `docs/PayloadFormats.md:15,34,132-133`: Port 10 specified as **10 bytes**; reference decoder raises unless exactly 10; firmware sends **11** (status byte, F17) — the published decoder rejects every production uplink. Status byte undocumented. Stale "10-byte" claims also at `payload_format.h:9`, `lora_app.c:1611` comment; "11" at `FirmwareArchitecture.md:217`, `FlashLogging.md:122`. | Regenerate from the final struct after the R04/R05 rework — do not hand-edit now. |
| R41 | **CONFIRMED** | `PayloadFormats.md:39-40,56-78` documents `lat_100m/lon_100m` (0.0009009° units); code uses full-range int16 scaling (`payload_encode.c:32-33`: `lat = enc×90/32767`). A decoder per the doc produces a different coordinate system. Struct field names/comments (`payload_format.h:94-95`) are stale too. | Rename fields (e.g. `latitude_i16`), fix comments, regenerate doc table + decoder. |
| R42 | **CONFIRMED** | `PayloadFormats.md:42,89-96,161`: decodes pressure byte 0 as **950 hPa** — the clamped stratospheric readings (R05) report the balloon at sea level all flight. | Regenerate after D2; reserve an out-of-range sentinel. |
| R43 | **CONFIRMED** | `docs/adr/0005:14`: "LinkCheckReq: Rides FOpts header (1 byte), **separate from app payload budget**" — factually wrong; vendor `LoRaMac.c ValidatePayloadLength()` sums `lenN + fOptsLen` against the region max (verified). This premise produced R04. Same ADR: bulk "222 B" (stale, 198 since FW-20), "SF7/DR3" (DR3=SF7 only in US915), and an unfinished "Verification Required" section — the gap that let R03 through. | Amend/supersede ADR-0005 with correct FOpts accounting + the completed four-region table (R-review Appendix B is accurate against in-tree headers — verified). |
| R44 | **CONFIRMED** | All rows spot-checked at HEAD: ADR-0005 222 B/DR3 claims; `PayloadFormats.md:207` "SF7 (DR2)"; readiness-matrix **F3/F13/F16 rows marked FIXED while broken** (R08, R01/R02, R03 — bench gates B2/B3 PENDING); `lora_app.c:123-125` "128kBytes device" comment (256 KB part); `lora_app.c:1677-1681` "fall through" comment (has `break`); `stm32_lpm_if.c:148 vs :152` PB15 = I2C2_SCL vs SPI2_MOSI contradiction; `atgm336h.c:218` "GGA+RMC only"; `atgm336h.c:1293` "40 second polling loop" (60 s/config); `lora_app.c:450` "PCAS04,7 + PCAS11 + PCAS00" (seven commands; PCAS04,7 rejected); `multiregion_context.c:409-411` "live MAC counter NOT modified" (ctx **is** the RAM copy; forward-only so safe, comment wrong); `main.c:198-202` success printed on failure; `payload_format.h:96` `temperature_2deg` declared `int8_t` but clamped 0–255 (uint8 semantics; tropopause −80 °C below the −64 °C floor). | One documentation sweep after the code gates; adopt the review's process note: a matrix row can only be FIXED with a linked verification artifact (test, scope capture, log excerpt). |

---

## 6. New findings from this verification pass

### N-01 — Published wire format claims big-endian; firmware sends little-endian — **CONFIRMED (DOC/P1 for ground station)**

`PayloadFormats.md` specifies every multi-byte field as big-endian (`uint16 BE`, `int32 BE`; decoders use `'>Hhh'`, `'>Iiii'`). The firmware `memcpy`s/native-fills packed structs and transmits them raw (`lora_app.c:1648,1739`; `payload_encode.c:244-246`) — STM32 is little-endian, so the wire is LE. Even after the R40 length fix, a decoder built to the doc misreads **every** multi-byte field in both production packets. (The reviews caught the size and scaling mismatches but not this.)

**Fix — Decision D9:** treat the firmware (LE) as wire truth and regenerate the doc + reference decoder (no firmware change, no over-air break vs. any existing backend that already handles LE), **or** convert the encoders to explicit BE serialization (firmware change; breaks any LE backend). Recommend LE-as-truth unless a backend already deployed against the doc.

### N-02 — Bulk doc: power-mode enum and record-unpack bugs — **CONFIRMED (DOC/P2)**

`PayloadFormats.md:264-273` documents power modes `0=STARTUP…5=GPS_LOCKOUT`; the firmware's `OperatingMode_t` is `0=NORMAL,1=CONSERVATIVE,2=REDUCED,3=RECOVERY,4=SURVIVAL` (`lora_app.c:859-868`). The reference decoder's `struct.unpack('>Iiii', record_data[0:14])` requires 16 bytes from a 14-byte slice (would throw); altitude is then re-extracted anyway. Fold into the R40–R42 regeneration.

### N-03 — Compact timestamp mixes two epochs mid-mission — **CONFIRMED (P2, extends F-015)**

`now_timestamp` is boot-relative RTC seconds until the first good fix, then Unix epoch after `SysTimeSet` (`lora_app.c:1419,1529,1636`). `timestamp_min` therefore changes convention mid-mission with no marker; the ground station cannot distinguish "minutes since boot" from "minutes since epoch" for early records. Any F-015 fix should also carry a "time is GNSS-disciplined" indicator (a status bit is free after the R04/R05 rework) or only transmit post-fix timestamps.

### N-04 — Delivery semantics are "radio TX completed", undocumented — **CONFIRMED (P2, answers the F-audit's central question)**

Records are marked transmitted in `OnTxData` on `LORAMAC_EVENT_INFO_STATUS_OK` (`lora_app.c:1907-1916`) — i.e. the radio finished transmitting an **unconfirmed** uplink. No gateway reception, let alone backend receipt, is required. This is a defensible semantic for a balloon (confirmed uplinks cost downlink budget), but it is currently implicit. **Decision D10:** document "delivered = transmitted by radio" explicitly in `PayloadFormats.md` and the flash-log header comment, or move to confirmed uplinks for bulk (airtime cost).

---

## 7. Architecture items (A-001…A-007)

| ID | Assessment |
|---|---|
| A-001 Consolidate ADC | **Adopt** — folds into the R16/F-014 fix: one bounded acquisition (VREFINT+battery+solar) per work cycle, calibrate at boot/temperature intervals. |
| A-002 Acquire→persist→select→encode→submit→commit | **Adopt as the target invariant set** — the C4 mark-after-TX design already half-implements it; F-005/F-006/R13/R21 are the violations. Formalize with exact-sequence TX tokens when the bulk path is reworked (D3). |
| A-003 Generic two-slot persistence | **Adopt long-term, defer** — config (F-019), flash headers (F-007), LoRaWAN NVM (F-016), and Tier-1/2 each hand-roll variants today. Unify after the flight-gating gates; don't let it block them. |
| A-004 Compile-time layout asserts | **Adopt** — trivial; land with R22 (update expected sizes with the payload rework). |
| A-005 Version every wire format | **Adopt** — bulk already has `packet_type` (0x02); compact has none. Add a version/type nibble in the D1/D2 rework; maintain golden vectors shared with the backend. |
| A-006 Reduce production logging | **Adopt** — RTT volume in flight is large (every NMEA command hex-dumped, `atgm336h.c:477-489`); compile-time levels, no float formatting (already done, FW-16), no secrets (F-017). Measure flash/RAM delta. |
| A-007 Fewer LinkChecks | **Adopt** — LinkCheckReq on every probe is both an R04 trigger and airtime overhead; make it periodic (e.g. every Nth cycle, on region change, or when cached metrics go stale). |

---

## 8. Decisions & tradeoffs (maintainer call required)

**D1 — Compact packet vs US915 DR0 budget (R04).**
(a) Probe at DR_1 (SF9, 53 B) in US915-like regions — costs ~2.5 dB link budget, buys 42 B headroom; (b) stay DR_0, shrink to ≤8 B, hard-cap FOpts; (c) keep DR_0/11 B, detect `PAYLOAD_LENGTH_ERROR` and retry immediately without LinkCheckReq. *Recommendation:* **(a)+(c) hybrid** — probe at DR_1 in US915/AU915 (SF10 probe is already wrong there anyway, R03), keep DR_0-equivalent elsewhere, plus (c) as a safety net; make LinkCheck periodic (A-007). Decides the byte budget for D2.

**D2 — Pressure encoding (R05/F-013).**
(a) 1-byte logarithmic over 2–1100 hPa (~2.5 % steps); (b) steal 3 bits from `humidity_5pct` (uses 5 of 8) → 11-bit geometric/piecewise field; (c) if D1(a) frees bytes: 2-byte 0.1 hPa field (matches the bulk record's resolution — cleanest science). *Recommendation:* **(c)** if D1(a) is taken, else **(b)**. Whatever is chosen: reserve an out-of-range sentinel, version the packet, regenerate decoder+doc together (R40–R42), golden vectors both sides.

**D3 — Variable-length bulk (R15).**
Fixed 198 B vs `2+32n+4`. *Recommendation:* adopt variable-length as packet_type 0x03 with the CRC relocated after the last record; decoder branches on `payload[0]` (already the pattern). Saves ~230 ms airtime per short packet; EU868 duty-cycle relief.

**D4 — Timestamp width (F-015/N-03).**
Options: 32-bit Unix minutes; 32-bit Unix seconds; 24-bit minutes from a documented epoch; mission-relative + mission ID + wrap counter. *Recommendation:* 32-bit Unix **seconds** in any packet that has room (bulk already does); for the compact packet, spend 1 of the bytes freed by D1 on widening minutes to 24 bits (2.7-year wrap) or keep 16-bit minutes + a wrap/epoch bit — decide with D1/D2 byte budget. Always mark GNSS-disciplined time (N-03).

**D5 — Altitude & history-record fields (R06/F-024/F-025/R19).**
`altitudeBar`: compute from pressure+temperature in `EnvSensors_Read`, or delete from `sensor_t`+`FlashLog_Record_t`. `altitudeGps`: widen to int32 metres in the flash record (14 reserved bytes; bump record version) or keep int16 with saturation+flag. History self-description: add `solar_mv`, `voltage_slope`, `power_mode` to `FlashLog_Record_t` vs remove from the wire record. *Recommendation:* delete `altitudeBar` (ground computes it — that's the documented design); widen `altitudeGps` to int32; add the three fields to the flash record so bulk history is honest. One flash-record version bump covers all three.

**D6 — Commissioning completion policy (R30/F-027).**
Bound each region's join (e.g. 3 attempts / ~120 s), mark unprovisioned, continue; `MissionState_EnterFlight()` if ≥1 bank valid, stay in COMMISSIONING if none; wire a documented path (RTT command or build-time table) to `MultiRegion_InitializeRegionFromChirpstack()` for regions that can't be joined over the air; have `LoRaWAN_Init()` act on the return value. Do **not** apply the bound to autonomous flight rejoin (there is none — flight never joins, ADR-0006).

**D7 — GNSS standby rail (R25).** Bench measurement gates this: `ttf_ms` over 20 cycles, then PB10-HIGH/PB5-cut variant + standby current. Expect: keep PB10 high, cut PB5; fix or drop PCAS12; correct all "ephemeris persisted via PCAS12" comments; re-tune `gps_timeout_*` to measured warm-start TTF.

**D8 — Ascent timer persistence (F-028).** Persist cumulative ascent elapsed (backup register free after R02) vs accept restart-on-reset. *Recommendation:* persist — repeated cold-snap resets shouldn't stretch ascent cadence indefinitely; cheap once R02 lands.

**D9 — Wire endianness (N-01).** LE-as-truth + doc/decoder regeneration (no firmware change) vs BE serialization (firmware change). *Recommendation:* LE-as-truth, unless a backend was already built against the BE doc — check first.

**D10 — Delivery semantics (N-04/F-004).** Document "delivered = radio TX completed" vs confirmed bulk uplinks. *Recommendation:* document the current semantic; confirmed uplinks for bulk cost too much downlink budget for the value.

**D11 — `FlashLog_EraseAll` (F-009/R14/F-022).** Fix (watermark reset + IWDG service) and wire to a commissioning-only command, or delete. *Recommendation:* fix + commissioning-gate — a field-recovery erase capability is worth having.

**D12 — `GNSS_GetPosition` (F-023).** Delete the dead blocking API vs harden it. *Recommendation:* delete.

---

## 9. Ordering hazards (both reviews, merged)

1. **R24 before/with R23** — the wrong PCAS11,2 checksum is currently the only thing preserving airborne mode. One commit.
2. **R01+R02 together** — one backup-register map (`backup_regs.h`); fixing only the deadman leaves mission state and reset cause clobbered by `SysTimeSet()`.
3. **R04+R05+D3+N-01+R40–R43 as one payload rework** — the compact bytes, pressure encoding, bulk length, endianness, and the published decoder are one wire format; piecemeal changes produce incompatible versions. Version both packets; golden vectors.
4. **R12/F-007 before relying on F-006's skip** — the watermark side effect is only safe once header freshness is a real generation counter.
5. **Do not unmask bugs while fixing neighbors** — e.g. removing the F-006 CRC-skip before F-005's exact-identity ack exists would re-wedge bulk transfer; fixing `Error_Handler` to be fatal before R08 would turn the LSE failover bug into a boot loop.
6. **Readiness matrix** — correct F3/F13/F16 rows only when the fix lands *with* its verification artifact (bench log/scope capture), per the R-review's process note.

## 10. Merged work order

**Gate 1 — flight blockers (P0):** ① R01+R02 (backup_regs.h) ② R24→R23 (one change) ③ F-011 (one-line minimum) ④ R30/D6 (else launch-ready state is unreachable) ⑤ R08 (LSE failover honored by MspInit) ⑥ F-001 minimal split (fatal init paths → `Fatal_Reset`; fixes R08's boot-loop interaction and the false-success prints).

**Gate 2 — power budget:** ⑦ R25/D7 (measure TTF first — likely dominates everything) ⑧ R26 (GSV off) ⑨ R07 (wake-source latches) ⑩ R09 (LED gating) ⑪ R17, R39, R16/F-014+A-001 (spin/wake/ADC cost).

**Gate 3 — data integrity & coverage:** ⑫ R03 (AU915) ⑬ the payload rework (D1–D4, D9; R04/R05/R15/R40–R43/N-01/N-02/N-03) ⑭ R06+D5 (uninitialized altitude; record version bump with F-024/F-025/R19) ⑮ R28, R29 (sensor/flash silent-failure modes) ⑯ R10 (brownout floor) ⑰ F-017 (key print gating).

**Gate 4 — persistence & robustness:** ⑱ F-008 (frontier scan from zero) ⑲ F-007/R12 (header generation) ⑳ F-006/R13 (bounded scan, explicit skip reporting) + F-005/R21 (exact-identity ack) ㉑ F-009/R14/D11, F-010 ㉒ F-002, F-003/R35, F-018, F-019, F-020 (config group) ㉓ F-016 (NVM context) ㉔ F-021 (header semantics) ㉕ R11 (restricted-region archive) ㉖ R20, R31–R34 (GNSS parser correctness) ㉗ F-029 (STOP2 reinit status) ㉘ F-023/D12, F-027 policy, F-028/D8, F-030.

**Gate 5 — docs & matrix:** ㉙ R44 sweep + `PayloadFormats.md` regeneration (from the final structs, after Gate 3) ㉚ readiness-matrix corrections with verification artifacts ㉛ A-003/A-005/A-006 consolidation.

---

## 11. What was NOT verified this pass

- h3lite submodule internals and `multiregion_h3.c` (R-review Appendix C scope exclusion; HEAD's h3lite commit not re-audited).
- `flash_if.c`, `usart_if.c`, `radio_board_if.c` (RF switch/TCXO across STOP2 — flagged by the R-review as worth a dedicated look), sequencer/timer utilities, CayenneLpp.c.
- Build reproduction, host tests, and all hardware/bench gates (TTF, LSE kill, BP-bit injection, current measurements).
- `sht31.c` CRC correctness accepted from the R-review (`sht31.c:205-214`) without re-reading.
