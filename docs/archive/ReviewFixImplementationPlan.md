# Stratosonde Review-Fix Implementation Plan

**Source:** `docs/archive/stratosonde-review-handoff.md` (2026-08-02 review of firmware @ `c62a7b5` + h3lite @ `d248ad5`)
**Status of every item below:** verified against the code on 2026-08-02. File/line citations were checked by reading the code, not copied from the handoff.
**Working rules (from the handoff, restated):**
1. Each numbered item below is **one independent commit**. Implement in order; skip nothing silently.
2. Do not "improve" anything outside the item's scope. Do not remove `SEGGER_RTT` diagnostics unless the item says so.
3. Each commit also updates the item's row in `docs/ProductionReadinessAssessment.md` with the file/line where the fix lives (that document is *code truth only*).
4. h3lite is a **separate git repo** (submodule at `Middlewares/Third_Party/h3lite`). h3lite items commit there first; a final firmware-repo commit bumps the submodule pointer.
5. Respect the do-not-touch list (handoff §6): `h3lite_faceijk.c`, `h3lite_basecells.c`, F2 EXTI latch classification, C6 counter margin, C4 mark-after-TX, F15 clamp, T4 flash ring, F16 `DatarateFromSF()`, F9/T2 stale-temp lockout, F20 I2C recovery, F3 LSE→LSI failover, F18 ratiometric ADC, BUG 1.3 keep-current-on-unknown, F21 comp table, GSV bounds, W25Q timeouts.

**Confirmed policy decisions (2026-08-02, with user):**
- **H3-8 → implement restricted regions for real (option a).** Radio silence inside restricted territory (default list: North Korea). `REGION_UNKNOWN` (0) is *not* restricted — it keeps the current region and transmits (ocean policy, BUG 1.3). These must never be conflated.
- **FW-11 → investigation write-up only. NO code change.** The user does not want the LoRaWAN stack touched pending further investigation.

---

## PHASE 1 — Quick wins (firmware repo unless noted)

### 1. FW-2 — Fault breadcrumbs unreadable: enable RTCAPB clock before backup-register access
- **Goal:** make the F1/F13b fault breadcrumb path actually work; corrects two falsely-FIXED matrix rows (F1, F13b).
- **Verified evidence:** `Core/Src/reset_cause.c:15-39` `ResetCause_CaptureBoot()` calls `HAL_RTCEx_BKUPRead/Write` with no backup-access enable. It runs at `main.c:162`, before RTC init (RTC init happens inside `MX_LoRaWAN_Init()` at `main.c:179` → `HAL_RTC_MspInit` → `__HAL_RCC_RTCAPB_CLK_ENABLE()` at `stm32wlxx_hal_msp.c:249`). TAMP backup registers read 0 while RTCAPB is gated, so the `RESET_CAUSE_FAULT_MAGIC` check always fails and every fault degrades to `RESET_CAUSE_SW`. `Fault_Reset()` in `Core/Src/stm32wlxx_it.c` calls `__HAL_RCC_RTC_ENABLE()` (BDCR RTCEN — **not** RTCAPB) and only works because faults happen after RTC init.
- **Change:** at the top of `ResetCause_CaptureBoot()` add:
  ```c
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
  ```
  Add the same two lines at the top of `Fault_Reset()` in `stm32wlxx_it.c` for symmetry.
- **Constraints:** do not reorder `main.c` init sequence; do not touch the RCC->CSR flag logic.
- **Acceptance:** bench — inject HardFault → next boot reports `RESET_CAUSE_FAULT` in status byte bits 3–5; boot after that reports clean. (Bench gate B4.)
- **Files:** `Core/Src/reset_cause.c`, `Core/Src/stm32wlxx_it.c`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-2: enable RTCAPB + backup access before breadcrumb read/write`

### 2. FW-3 — `MissionState_Update()` is never called
- **Goal:** ASCENT → FLOAT transition actually happens; status byte stops reporting ASCENT forever.
- **Verified evidence:** `MissionState_Update` has exactly one hit tree-wide: its definition in `Core/Src/mission_state.c`. Zero call sites. `mission_state.h` documents the contract "call each work cycle".
- **Change:** call `MissionState_Update();` in `LoRaWAN/App/lora_app.c::SendTxData()` immediately after the existing `Deadman_MarkProgress();` line (F13a marker, verified present).
- **Constraints:** do **not** change `s_ascent_start_tick` / `HAL_GetTick()` reboot semantics — `MissionState_Init()` restarting the ascent timer on reboot is intentional and documented.
- **Acceptance:** build clean; code inspection shows one call per work cycle; status byte mission-state field transitions after `MISSION_ASCENT_DURATION_MS` (3 h) — bench confirm.
- **Files:** `LoRaWAN/App/lora_app.c`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-3: call MissionState_Update once per work cycle`

### 3. FW-4 — Chunked STOP2 loop can starve both watchdog layers
- **Goal:** a dead TxTimer/Alarm-A chain cannot sleep the device forever with a satisfied IWDG.
- **Verified evidence:** `Core/Src/stm32_lpm_if.c:228-279` — `PWR_EnterStopMode()` contains `while (1)` that refreshes IWDG every 25 s chunk (`HAL_IWDG_Refresh` at :244) and exits only on RTC Alarm A (EXTI PR1 bit 17) or a non-wakeup-timer source. `Deadman_Check()` lives in the `main()` loop (extern-declared and called in `main.c`), which never runs while stuck here.
- **Change (both parts):**
  1. Call `Deadman_Check()` inside the chunk loop immediately after `HAL_IWDG_Refresh(&hiwdg);` (it reads RTC time and self-resets; no-op in COMMISSIONING). Add the same `extern void Deadman_Check(void);` declaration pattern used in `main.c` (definition lives beside `Deadman_MarkProgress` — confirm translation unit when editing).
  2. Bound the loop: `uint32_t chunks = 0;` and `if (++chunks > 150) break;` (150 × 25 s ≈ 62.5 min > worst-case SURVIVAL cycle of 1 h).
- **Constraints:** preserve the EXTI pending-latch classification (`EXTI->PR1` bits 19/17) exactly — that F2 fix is correct and subtle.
- **Acceptance:** build clean; bench — wedge the sequencer timer, confirm reset within the deadman window.
- **Files:** `Core/Src/stm32_lpm_if.c`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-4: deadman check + chunk bound in STOP2 loop`

### 4. FW-16 — Finish F27: remaining float printf sites
- **Goal:** no `%f`-family specifiers remain (they print blanks without `-u _printf_float`); F27 becomes truly FIXED.
- **Verified evidence:** `Core/Src/atgm336h.c:533` (`HDOP:%.1f`, `Lat:%.6f Lon:%.6f`, `Alt:%.1fm`, `Speed:%.1fkm/h`) and `:545` (`HDOP:%.1f`); `Core/Src/multiregion_h3.c` APP_LOGs: `"nearest: %s (%.1f km)"` and `"(%.4f, %.4f)"`.
- **Change:** convert to integer deci/micro prints matching the existing pattern already used elsewhere in `multiregion_h3.c` (`lat_int = (int32_t)(lat * 10000); … %ld.%04ld`). HDOP → deci (`%d.%d`), lat/lon → 6 or 4 decimal fixed-point, alt/speed → deci.
- **Acceptance:** `grep -rn "%\.[0-9]*f" Core/Src LoRaWAN/App` returns no live format strings; build clean.
- **Files:** `Core/Src/atgm336h.c`, `Core/Src/multiregion_h3.c`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-16: convert remaining float printfs to fixed-point`

### 5. FW-17 — Guard LinkCheck log fields
- **Goal:** stop logging garbage `DemodMargin`/`NbGateways` when no LinkCheckAns was received.
- **Verified evidence:** `LoRaWAN/App/lora_app.c::OnRxData` reads `params->DemodMargin` / `params->NbGateways` and logs them; `bool linkcheck_received = params->LinkCheck;` exists but does not guard the log.
- **Change:** only log margin/gateway-count when `linkcheck_received` is true.
- **Files:** `LoRaWAN/App/lora_app.c`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-17: guard LinkCheck log on linkcheck_received`

### 6. FW-18 — Stale comment on flight-critical flag
- **Goal:** remove a lying comment.
- **Verified evidence:** `Core/Inc/sys_conf.h:88-92` — a USER CODE block does `#undef LOW_POWER_DISABLE` / `#define LOW_POWER_DISABLE 0` with the comment `CHANGED: 0 → 1 for baseline test`. Value is 0; comment is stale.
- **Change:** delete the stale comment (and the now-redundant `#undef`/re-`#define` block if it duplicates the value at :85 — keep the effective value `0`).
- **Files:** `Core/Inc/sys_conf.h`
- **Commit:** `FW-18: remove stale LOW_POWER_DISABLE comment`

### 7. H3-5 — Two unconditional `printf()` in flight hot path *(h3lite repo)*
- **Goal:** no raw `printf` in `h3ToRegion()`; no newlib stream/malloc pull-in; no `%llx` garbage.
- **Verified evidence:** `Middlewares/Third_Party/h3lite/src/h3lite.c:119-120` and `:124-125` — raw `printf("DEBUG h3ToRegion: …")` including `%llx`. The `H3LITE_DEBUG_PRINT` macro exists in `include/h3lite.h` (`#ifdef H3LITE_DEBUG … #else ((void)0)`). `findNearestRegions` calls `h3ToRegion` up to 36 more times per lookup.
- **Change:** wrap both calls in `H3LITE_DEBUG_PRINT(...)`. Verify the flight build does not define `H3LITE_DEBUG`.
- **Files:** `h3lite/src/h3lite.c`
- **Commit (h3lite):** `H3-5: gate debug printfs behind H3LITE_DEBUG_PRINT`

### 8. H3-9 — Delete dead `baseCellTable` *(h3lite repo)*
- **Goal:** remove 576 B of misleading dead data.
- **Verified evidence:** `h3lite/src/h3lite.c:23-48` defines `const int baseCellTable[12][12]`; only reference is `extern const int baseCellTable[12][12];` at `h3lite/src/h3lite_faceijk.c:13`; used nowhere.
- **Change:** delete both. Do not touch anything else in `h3lite_faceijk.c` (do-not-touch file otherwise).
- **Files:** `h3lite/src/h3lite.c`, `h3lite/src/h3lite_faceijk.c`
- **Commit (h3lite):** `H3-9: delete unused baseCellTable`

---

## PHASE 2 — Region engine (h3lite repo; gate each step on the acceptance tests)

### 9. Verification harness (commit first — this is the regression detector)
- **Goal:** reproduce the handoff's empirical harness so every later change is measured.
- **Change:**
  1. Add `h3lite/test/xval_pts.c` exactly as specified in handoff §1.1 (reads `lat lon` on stdin, emits `lat lon baseCell partialIndex region h3` on stderr). Build: `gcc -Wall -O2 -I./include test/xval_pts.c -L./bin -lh3lite -lm -o xval_pts`.
  2. Add `h3lite/test/t_index.py` — global grid vs `h3.latlng_to_cell(lat,lng,3)` (requires `pip install h3==4.5.0`). Must report **0 / 319,219 mismatches** before and after all Phase-2 work.
  3. Add `h3lite/test/t_city.py` — the 52-city reference set from handoff Appendix B. Baseline expectation: 36 correct / 15 Unknown / 1 wrong. Required after Phase 2: ≥ 98 % correct, 0 wrong.
  4. Add `h3lite/test/t_table.py` — static audit from handoff Appendix C (write it against the **packed** format from item 10; run against the old struct format only to record the baseline: 269 duplicate keys, 35 conflicting).
- **Acceptance:** all three scripts run; baselines recorded in `docs/ProductionReadinessAssessment.md` or the commit message.
- **Files:** `h3lite/test/xval_pts.c`, `h3lite/test/t_index.py`, `h3lite/test/t_city.py`, `h3lite/test/t_table.py`
- **Commit (h3lite):** `test: add T-INDEX/T-CITY/T-TABLE cross-validation harness`

### 10. H3-1 + H3-4 — Carry resolution in the key; pack the entry to 4 bytes
- **Goal:** make res-1/res-2 entries matchable (fixes the 31 % city failure / 75.55 % global Unknown rate) and recover ~21.9 KB flash (10,953 × 6 B → × 4 B).
- **Verified evidence:** `h3lite/include/h3lite_regions_table.h:33-37` — `RegionEntry{uint8_t baseCell; uint16_t partialIndex; RegionId regionId}` (6 B padded), no resolution. `h3lite/src/h3lite.c:111-116` — device always builds a 3-digit key (`numDigits = (res<3)?res:3`, res always 3). `generate_lookup_table.py:253,272` — dedup key includes resolution, but `:371` emits only `{bc, pi, region}`. Table visibly contains unmatchable res-1 entries (`{ 0, 1, 2 }`, `{ 0, 3, 12 }`, … at `h3lite_regions_table.c:31-34`).
- **Change:**
  1. `include/h3lite_regions_table.h`: replace the struct with the packed `uint32_t` format from handoff §H3-1 (bits [31:25] baseCell, [24:23] resolution, [22:14] partialIndex, [13:10] regionId; `RE_BASECELL/RE_RES/RE_PARTIAL/RE_REGION` macros). New `findRegion(uint8_t baseCell, uint8_t res, uint16_t partialIndex)`.
  2. `src/h3lite_regions_table.c`: regenerate — entries as packed `uint32_t`, sorted ascending by `(baseCell, resolution, partialIndex)`; `findRegion` binary-searches the 3-tuple.
  3. `src/h3lite.c::h3ToRegion`: compute digits once, probe res 3 → res 2 → res 1, first hit wins (exact code in handoff §H3-1).
  4. `generate_lookup_table.py`: emit the resolution field and the packed format (keep the existing compaction for now — H3-2 replaces it next item).
- **Constraints:** three binary searches ≈ 14 steps each — acceptable; do not add caching. Do not touch `h3lite_faceijk.c`/`h3lite_basecells.c`.
- **Acceptance:** T-INDEX still 0 mismatches; T-CITY improves to ≥ 48/52 correct (12 of 16 currently-failing cities resolve via multi-probe per the handoff's verified prediction); table ≤ ~44 KB.
- **Files:** `h3lite/include/h3lite_regions_table.h`, `h3lite/src/h3lite_regions_table.c`, `h3lite/src/h3lite.c`, `h3lite/generate_lookup_table.py`
- **Commit (h3lite):** `H3-1/H3-4: resolution-aware packed region table + most-specific-first lookup`

### 11. H3-2 — Generator: compact only uniform parents
- **Goal:** eliminate the 269 duplicate / 35 conflicting keys; stop order-dependent region-blind compaction (Hanoi → both AS923-2 and CN470; Hong Kong collapsed to CN470).
- **Verified evidence:** `generate_lookup_table.py:225-294` — `processed_cells` shared across regions; parent check `(base_cell, partial_index//8, resolution-1) in processed_cells` suppresses children regardless of region.
- **Change — rewrite `generate_lookup_table()`:**
  1. Polyfill every region at **res 3 only**; build `cell → region` across all regions.
  2. Conflicts (two regions claim the same res-3 cell): largest intersected polygon area wins; tie-break lowest region ID. **Exception: regionId 255 (restricted) always wins** (item 13). Log every resolution.
  3. Bottom-up compaction: 7 res-3 siblings → res-2 parent only if all 7 present and same region; repeat res-2 → res-1.
  4. Emit `(baseCell, resolution, partialIndex, regionId)` packed per item 10, sorted.
  5. Assert before writing: zero duplicate `(baseCell, resolution, partialIndex)` keys; fail the build otherwise.
- **Acceptance:** T-TABLE reports 0 duplicates / 0 conflicts; T-CITY ≥ 98 % with Hanoi → AS923-2; T-INDEX still 0.
- **Files:** `h3lite/generate_lookup_table.py`, regenerated `h3lite/src/h3lite_regions_table.c`
- **Commit (h3lite):** `H3-2: uniform-only compaction with deterministic conflict resolution`

### 12. H3-3 — Verify NZ / Hong Kong assignments against RP002
- **Goal:** confirm or correct the source GeoJSON for New Zealand (currently AS923-1C) and Hong Kong (currently CN470) against **LoRa Alliance RP002-1.0.4**; do not "fix" from intuition.
- **Change:** compare source GeoJSON boundaries to RP002; correct source data if it disagrees; record the decision and source in a comment at the top of `generate_lookup_table.py`. Hong Kong may resolve naturally via item 11's uniformity rule (res-2 cell containing both HK and Guangdong cannot compact).
- **Acceptance:** decision + source recorded; T-CITY expectations for Wellington/Auckland/HongKong updated from "provisional" to final in `t_city.py`.
- **Files:** `h3lite/generate_lookup_table.py`, possibly source GeoJSON, `h3lite/test/t_city.py`
- **Commit (h3lite):** `H3-3: verify NZ/HK region assignments against RP002-1.0.4`

### 13. H3-8 — Restricted regions: real data for the existing safety branch
- **Goal:** make `REGION_RESTRICTED` (255) reachable so the existing `lora_app.c::SendTxData` branch (`if (h3_region_id == REGION_RESTRICTED) … return;`) actually enforces radio silence. **User requirement: no transmission inside restricted territory (default: North Korea). `REGION_UNKNOWN` stays transmit-and-keep-current (ocean policy) — do not conflate.**
- **Verified evidence:** `h3lite/include/h3lite.h` defines `REGION_RESTRICTED 255`; generated table contains zero region-255 entries (grep-verified); generator has no restricted concept; the firmware branch exists and returns before any region-name→MAC mapping.
- **Change:**
  1. Add restricted-territory GeoJSON (North Korea; user-extensible list) to the generator's input directory, mapped to regionId **255** in `REGION_IDS`.
  2. In item-11 conflict resolution: **255 always wins**, and restricted cells never compact into a non-restricted parent (uniformity rule already prevents this if restricted siblings only compact among themselves).
  3. `getRegionName()`: 255 must not index out of bounds — after item 14's clamp it returns "Unknown" as a *name*; add an explicit `if (regionId == REGION_RESTRICTED) return "RESTRICTED";` guard so logs are honest. Enforcement keys on the ID, not the name.
  4. Firmware: no logic change needed (branch exists). Add one comment in `lora_app.c` noting the data dependency is now real.
- **Acceptance:** T-CITY-style point test inside North Korea returns 255; `SendTxData` path verified by inspection; T-TABLE clean.
- **Files:** `h3lite/generate_lookup_table.py`, restricted GeoJSON source, regenerated table, `h3lite/src/h3lite.c`, `LoRaWAN/App/lora_app.c` (comment only)
- **Commit (h3lite + firmware):** `H3-8: restricted-territory regions (id 255) with top precedence`

### 14. H3-7 — `getRegionName` clamp kills CN470/EU433/CD900-1A
- **Goal:** regions 13–15 return their real names so `H3Region_ToLoRaMacRegion` strcmp's work.
- **Verified evidence:** `h3lite/src/h3lite.c:149` `int max_region_id = 12;` while `h3lite_regions_table.c:9-27` defines `regionNames[16]` with CN470=13, EU433=14, CD900-1A=15.
- **Change:** derive the bound from the array: `#define REGION_NAME_COUNT 16` (or `sizeof(regionNames)/sizeof(regionNames[0])` where visible) and clamp to that; add the explicit 255 guard from item 13.
- **Files:** `h3lite/src/h3lite.c`
- **Commit (h3lite):** `H3-7: fix getRegionName bounds for regions 13-15`

### 15. H3-6(1) — `h3GetRing` latent buffer bug
- **Goal:** return the actual cell count so a future pentagon-capable ring can't become a stale-data read (real H3 returns 5k cells near pentagons, not 6k).
- **Verified evidence:** `h3lite/src/h3lite_neighbor.c:270` `int h3GetRing(H3Index origin, int k, H3Index *out)` returns 0/1 and always writes 6·k; `h3lite/src/h3lite.c:195-197` `findNearestRegions` checks `== 0` then loops `i < ringSize` (6·k) regardless.
- **Change:** `h3GetRing` returns the actual number of cells written (negative on failure); `findNearestRegions` loops to that count. Update the prototype in `include/h3lite.h` and the doc comment.
- **Constraints:** pentagon rotation support (H3-6(2)) is **deferred** — document the ~11 % clean-failure limitation in `h3lite/README.md` in this same commit.
- **Files:** `h3lite/src/h3lite_neighbor.c`, `h3lite/src/h3lite.c`, `h3lite/include/h3lite.h`, `h3lite/README.md`
- **Commit (h3lite):** `H3-6(1): h3GetRing returns actual count; document pentagon limitation`

### 16. Submodule pointer bump
- **Goal:** firmware repo points at the fixed h3lite.
- **Acceptance:** firmware build clean; flash usage drops by ~22 KB vs the 239,096 B baseline (record new number in the matrix's Flash Size Budget table).
- **Commit (firmware):** `h3lite: bump submodule (region engine fixes, -22KB flash)`

---

## PHASE 3 — Session integrity

### 17. FW-1 — DDR-0006 Tier-1/Tier-2 session storage
- **Goal:** eliminate the single-page erase→write window that can destroy every region's credentials (dawn brownout → permanent RF silence), and add wear leveling. Corrects the falsely-FIXED T1 matrix row.
- **Verified evidence:** `Core/Src/multiregion_context.c` — `#define MULTIREGION_FLASH_BASE_ADDR 0x0803F800` (single page 127); `FlashWriteStorage()` does `FLASH_IF_Erase` then `FLASH_IF_Write` of one `MultiRegionStorage_t` (keys **and** counters together) every `FRAME_COUNTER_SAVE_INTERVAL` (10) TXs. Flash map documented in `Core/Inc/config.h` (pages 125/126/127 used); linker reserves 6 KB (`STM32WLE5JCIX_FLASH.ld`: `FLASH … LENGTH = 250K`).
- **Change:**
  - **Tier 1 (immutable credentials per region: DevAddr, NwkSKey, AppSKey, DevEUI, region params):** written once at commissioning as **three redundant, independently CRC'd copies** in dedicated pages never erased in flight. Commissioning reads back and CRC-verifies all three after writing. Restore repairs a bad copy from a good one.
  - **Tier 2 (frame counters only):** separate area, ping-pong between two slots with erase-before-write (DDR-0004). Preserve the verified counter-margin scheme (`ctx->uplink_counter += FRAME_COUNTER_SAVE_INTERVAL` on save) exactly.
  - **Degrade ladder (per region):** restore fail → retry redundant copies → keys good/counter bad → counter = last persisted + margin → credentials unrecoverable → RF silence **in that region only**, keep logging, resume on entering a region with a valid session.
  - **Door anchoring:** virgin bank → COMMISSIONING; Tier-1 copies present → FLIGHT even if the DR0 state record is corrupt. Ambiguity resolves to FLIGHT.
  - **Flash map:** extend the reserved region (e.g. pages 120–127 = 16 KB: 3× Tier-1 pages + 2× Tier-2 slots + existing 125/126/127), shrink linker `FLASH` to `240K`, update the map comments in `config.h` and the `.ld`.
- **Constraints:** depends on item 10/16's ~22 KB saving landing first (current usage 239,096 B vs 240 K boundary). Keep C6 margin logic untouched.
- **Acceptance:** bench — power-cut during a Tier-2 save × 50 iterations: credentials survive every time; counter resumes ≥ pre-cut value. (Bench gate B5 extended.)
- **Files:** `Core/Src/multiregion_context.c`, `Core/Inc/multiregion_context.h`, `Core/Inc/config.h`, `STM32WLE5JCIX_FLASH.ld`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-1: two-tier session storage per DDR-0006 (redundant credentials + ping-pong counters)`

---

## PHASE 4 — Flight hardening

### 18. FW-5 — IWDG armed late
- **Goal:** watchdog covers boot/commissioning.
- **Verified evidence:** `MX_IWDG_Init()` at `main.c:182`, after `MX_LoRaWAN_Init()` at `:179`; the `hiwdg.Instance != NULL` guard in the join-wait loop (`multiregion_context.c`, "BUG 3.1 FIX") acknowledges the gap.
- **Change (code, this commit):** move `MX_IWDG_Init()` to immediately after `SystemClock_Config()` (`main.c:157`), and refresh IWDG in the blocking paths between there and the main loop (`leds_boot_seq()` ≈ 2 s is fine against the 32.76 s timeout; audit `MultiRegion_PreJoinAllRegions()` inter-region delays — the join wait already refreshes).
- **Bench follow-up (not code):** consider setting the IWDG_SW option byte so IWDG is hardware-enabled from reset — add to the bench checklist.
- **Files:** `Core/Src/main.c`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-5: arm IWDG immediately after clock config`

### 19. FW-6 — Voltage-slope Δt amplification
- **Goal:** a tiny Δt after a baseline shift can't amplify 10 mV of ADC noise into a spurious SURVIVAL selection.
- **Verified evidence:** `LoRaWAN/App/lora_app.c::CalculateVoltageSlope()` — `slope = Δv·3600/Δt` with baseline shifting every 7200 s; no minimum Δt guard; bulk transfer can re-arm `SendTxData` back-to-back (C7b).
- **Change:** require `time_change_sec >= 600` before recomputing; otherwise return the previously computed slope (store `last_slope_mv_per_hour` in `VoltageSlope_t`). Guard `time_change_sec == 0` explicitly.
- **Files:** `LoRaWAN/App/lora_app.c` (+ `VoltageSlope_t` definition wherever declared — confirm header when editing), `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-6: minimum delta-t for voltage slope computation`

### 20. FW-7 — Pressure data honesty (DDR-0007)
- **Goal:** MS5607 gets the same last-known-good + stale-bit treatment SHT31 got in F9; a failed read never transmits 1000.0 hPa (sea level) as real float-altitude science data.
- **Verified evidence:** `Core/Src/sys_sensors.c` — `PRESSURE_DEFAULT_VAL 1000.0f` substituted on failure with no stale flag; sentinel check `I2C_NoteResult((!th_stale) || (PRESSURE_Value != PRESSURE_DEFAULT_VAL))` also mis-counts a legitimate 1000.0 hPa reading as failure. `FlashLog_Record_t.flags` reserved field exists (`Core/Inc/flash_log.h`); uplink status byte has unused bits.
- **Change:**
  1. Add `s_last_press` / `s_have_press` cache + `press_stale` flag in `sys_sensors.c`, mirroring the SHT31 pattern exactly.
  2. Track a real `ms_ok` boolean for the read result; replace the sentinel comparison.
  3. Propagate `press_stale` through `sensor_t` → `FlashLog_Record_t.flags` → uplink status byte (next unused bit).
- **Files:** `Core/Src/sys_sensors.c`, `LoRaWAN/App/lora_app.c`, `Core/Inc/flash_log.h`, `Core/Src/payload_encode.c` (status byte bit), `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-7: pressure last-known-good + stale flag (DDR-0007)`

### 21. FW-9 — `FindContextSlot()` region-sentinel collision
- **Goal:** an AS923 (enum 0) lookup can't match an all-zeros empty slot.
- **Verified evidence:** `Core/Src/multiregion_context.c::FindContextSlot()` matches on `region`; `LORAMAC_REGION_AS923 == 0` collides with erased-slot zeros. Currently masked by downstream DevAddr/CRC guards, but a live trap for the FW-1 rewrite (item 17).
- **Change:** skip slots where `dev_addr == 0 || dev_addr == 0xFFFFFFFF` inside `FindContextSlot()`.
- **Note:** land **before** item 17 if item 17 slips.
- **Files:** `Core/Src/multiregion_context.c`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-9: skip empty slots in FindContextSlot`

### 22. FW-10 — `LORAWAN_FORCE_REJOIN_AT_BOOT` brick switch
- **Goal:** a build flashed with the flag `true` can't produce FLIGHT + virgin bank = permanent RF silence.
- **Verified evidence:** `LoRaWAN/App/lora_app.h` `#define LORAWAN_FORCE_REJOIN_AT_BOOT false`; `lora_app.c` `static bool ForceRejoin = …` → `if (ForceRejoin) { MultiRegion_ClearAllContexts(); }` during `LoRaWAN_Init()`, while `MissionState_Init()` reads the bank earlier.
- **Change:** gate the clear behind `MissionState_IsCommissioning()`; add a compile-time `#error`/static assert if the flag is `true` in a flight build.
- **Files:** `LoRaWAN/App/lora_app.c`, `LoRaWAN/App/lora_app.h`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-10: gate force-rejoin behind commissioning state`

---

## PHASE 5 — Bench-gated (code where safe; validation requires hardware)

### 23. FW-8 — GNSS standby: honest comments + bench TTF
- **Verified evidence:** `Core/Src/atgm336h.c::GNSS_EnterStandby()` cuts **both** PB10 and PB5 ("FULL POWER-OFF… 0µA"), while `stm32_lpm_if.c:193-194` comments claim "PB10 must stay HIGH for hot-start mode (~15µA backup power)". PCAS12 is standby entry; PCAS00 is the flash-save command — the "saves ephemeris before power cut" comment is doubtful.
- **Change (this commit — comments only):** correct the stale comments in `atgm336h.c`, `lora_app.c`, `stm32_lpm_if.c` to describe the actual full-power-off behavior.
- **Bench gate (decides the real fix):** log TTF across 20 consecutive cycles as-built. If not hot-starting, restore PB10-HIGH standby (separate commit after measurement). Energy stakes: ~15 s extra acquisition @ ~35 mA every 5 min ≈ 1.75 mA avg vs 15 µA backup.
- **Files:** `Core/Src/atgm336h.c`, `LoRaWAN/App/lora_app.c`, `Core/Src/stm32_lpm_if.c` (comments)
- **Commit:** `FW-8: correct GNSS standby/power comments to match behavior`

### 24. FW-12 — Flash-log frontier scan on init
- **Goal:** an unexpected reset no longer AND-corrupts up to 9 tail records (header persists every `HEADER_UPDATE_INTERVAL`=10, verified `Core/Src/flash_log.c`).
- **Change:** in `FlashLog_Init`, scan forward from the header's `write_addr` while magic+CRC pass and sequence is contiguous; adopt the true frontier (DDR-0004's sequence-discontinuity detection). Zero record loss.
- **Files:** `Core/Src/flash_log.c`, `docs/ProductionReadinessAssessment.md`
- **Commit:** `FW-12: scan to true write frontier on flash log init` (bench: power-cut mid-write recovery, extends gate B1)

### 25. FW-14 — W25Q deep power-down
- **Verified evidence:** `Core/Src/stm32_lpm_if.c:130-132` — `W25Q_PowerDown` commented out; W25Q16JV draws 1–3 mA standby vs <1 µA deep power-down.
- **Change:** wire `hw25q_ptr = &hw25q;` after `W25Q_Init()` in `main.c`; enable the power-down call; ensure `W25Q_ReleasePowerDown` runs in `PWR_ExitStopMode` before any flash access.
- **Files:** `Core/Src/stm32_lpm_if.c`, `Core/Src/main.c`, `docs/ProductionReadinessAssessment.md` (P2-12 row)
- **Commit:** `FW-14: enable W25Q deep power-down in STOP2` (bench: sleep-current measurement)

### 26. FW-15 — VREFBUF re-enable on wake
- **Verified evidence:** `HAL_SYSCFG_DisableVREFBUF()` at `stm32_lpm_if.c:187` on sleep entry; never re-enabled.
- **Change:** re-enable in `PWR_ExitStopMode` before any ADC use — or, if the F18 VREFINT ratiometric path is confirmed unaffected, delete the disable call. Decide by bench ADC reading after wake.
- **Files:** `Core/Src/stm32_lpm_if.c`, `docs/ProductionReadinessAssessment.md` (P2-13 row)
- **Commit:** `FW-15: restore VREFBUF on STOP2 exit`

---

## PHASE 6 — Cleanup

### 27. FW-13 — `vcom_Trace` transmits into the GPS UART
- **Verified evidence:** `Core/Src/usart_if.c` — `vcom_Trace` → `HAL_UART_Transmit(&huart1, …)` and `vcom_Trace_DMA` → `HAL_UART_Transmit_DMA(&huart1, …)`; USART1 is the GPS module's RX.
- **Change:** route to RTT or stub both functions.
- **Files:** `Core/Src/usart_if.c`
- **Commit:** `FW-13: stop trace output on GPS UART`

### 28. FW-19 — Rename `FlashLog_GetUnsentRecordsLIFO` (it reads FIFO since C4)
- **Verified evidence:** declared in `Core/Inc/flash_log.h`, called in `lora_app.c` bulk path.
- **Change:** rename to `FlashLog_GetUnsentRecordsFIFO` in `flash_log.h`, `flash_log.c`, `lora_app.c`.
- **Commit:** `FW-19: rename GetUnsentRecordsLIFO to FIFO`

### 29. FW-20 — Bulk packet TODOs
- **Verified evidence:** `lora_app.c` bulk path — `dummy_voltage_trend[10]`/`dummy_mode_changes[10]` (20 zero bytes per 222 B packet) and `flash_page_addr = 0` TODOs.
- **Change:** implement voltage-trend/mode-change tracking and flash page addr, **or** shrink the packet. Default: shrink (20 B at SF7 is real airtime) unless the fields are wanted.
- **Files:** `LoRaWAN/App/lora_app.c`, `Core/Inc/payload_format.h` (if packet layout changes)
- **Commit:** `FW-20: resolve bulk packet placeholder fields`

---

## INVESTIGATION-ONLY — no code change

### 30. FW-11 — AS923 sub-plan frequency collapse (write-up only, per user decision)
- **Verified finding:** `lorawan_conf.h:112` compiles `REGION_AS923_DEFAULT_CHANNEL_PLAN = CHANNEL_PLAN_GROUP_AS923_1` (offset 0). `RegionAS923.c` selects `REGION_AS923_FREQ_OFFSET` **at compile time** (AS923-2 = −1.8 MHz, AS923-3 = −6.6 MHz, AS923-4 = −5.9 MHz) and bakes it into default channels at region init. There is **no join-time sub-plan negotiation** in this stack — the dynamic parts of AS923 are dwell-time/duty-cycle MAC commands, not frequency plan. Meanwhile `multiregion_h3.c::H3Region_ToLoRaMacRegion` maps all six AS923 sub-plans to `LORAMAC_REGION_AS923`. Over Vietnam/Indonesia (AS923-2/-3) the sonde transmits AS923-1 frequencies: no gateway hears it, possibly out of band locally.
- **Action:** document the finding, the two candidate fixes ((a) conservative keep-current/RF-silence for AS923-2/-3/-4 cells; (b) runtime MIB channel reconfiguration per sub-plan), and the decision to defer pending investigation. Add a note row to `docs/ProductionReadinessAssessment.md`. **No code change.**
- **Commit:** `docs: FW-11 AS923 sub-plan investigation (deferred, no code change)`

### 31. FW-21 / FW-22 — notes only
- FW-21: equator/prime-meridian measure-zero fix drop — documented, no change.
- FW-22: solar PB3 no-divider — hardware check (panel Voc ≤ VDDA+0.3 V), not code.

---

## Matrix corrections to apply with the relevant commits
| Matrix row | Currently says | Truth (verified) | Corrected by item |
|---|---|---|---|
| T1 | FIXED | Ladder half only; storage half unimplemented | 17 (FW-1) |
| F1 | FIXED | Breadcrumb write works, read always fails (RTCAPB gated) | 1 (FW-2) |
| F13b | FIXED | Same root cause as F1 | 1 (FW-2) |
| F27 | FIXED | Float printfs remain at `atgm336h.c:533,545`, `multiregion_h3.c` | 4 (FW-16) |
| P2-12 | OPEN | confirmed OPEN | 25 (FW-14) |
| P2-13 | OPEN | confirmed OPEN | 26 (FW-15) |

## Measured baselines (regression detectors — re-record after Phase 2 and Phase 3)
```
Flash used                    239,096 B of 250 K (target after Phase 2: ~217 KB)
sizeof(RegionEntry)           6 B × 10,953 = 65,718 B  (target: 4 B × N ≈ 44 KB)
T-INDEX mismatches            0 / 319,219              (must stay 0)
T-CITY                        36 correct / 15 Unknown / 1 wrong  (target ≥ 51/52, 0 wrong)
T-TABLE                       269 dup keys / 35 conflicts        (target 0 / 0)
Global 0.5° grid Unknown      75.55 %
h3GetRing exact match         846 / 927 (81 clean failures, 0 wrong-content)
```
