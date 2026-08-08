# Stratosonde Firmware — Fix Work Order

**Repo:** https://github.com/stratosonde/firmware
**Target:** STM32WLE5JC, LoRaWAN, solar-powered long-duration balloon sonde
**Review date:** 2026-08-07
**Reviewer:** Claude (autonomous review — repo cloned, firmware built clean with `arm-none-eabi-gcc` 13.2, host tests run, P0 reproduced empirically)

**Build baseline:** `make -C Debug -j all` → `Debug/Radio_Sonde_E5_HF_EU.elf` (text 193420, data 916, bss 22936). Host tests: `make -C tests/host test`.

---

## How to use this document

Each finding below is a self-contained work item with: location, root cause, impact, and prescribed fix. Items are ordered by priority, not by file. Every fix must keep the firmware building clean and the host test suite green.

**Governing constraint:** this device cannot be reset or serviced in flight. Prefer *degrade and keep flying* over *fail fast*. A reset loop at altitude is a total mission loss; a missing log or a stale sample is not.

---

# P0 — Critical

## F-01 — Signed/unsigned division corrupts battery voltage slope

**File:** `Core/Src/power_model.c:102`

**Current code:**
```c
int16_t slope = (int16_t)((voltage_change * 3600) / time_change_sec);
```

**Root cause:** `voltage_change` is `int32_t`, `time_change_sec` is `uint32_t`. C's usual arithmetic conversions promote the *entire* expression to unsigned, so a negative numerator wraps to a huge positive value before the division.

**Verified empirically.** A reproduction harness was compiled against the real `power_model.c`. A −100 mV drop over one hour returns **+13298 mV/h** instead of −100 mV/h.

**Impact:**
- A discharging battery reads as charging. `SelectModeFromPredictions()` therefore selects `MODE_NORMAL` — the *highest* power draw — precisely when the battery is draining.
- `PredictTimeToVoltage()` returns `0xFFFF`, so the `SURVIVAL` and `RECOVERY` branches are effectively dead code.
- The only remaining protection is the hard floor at `raw_voltage_mv < 4300`.

**Fix:**
```c
int16_t slope = (int16_t)((voltage_change * 3600) / (int32_t)time_change_sec);
```

**Also required:**
1. Add a host test with a **negative** slope case. The existing test at `tests/host/test_main.c:444` only covers +36 mV, which is why this survived.
2. Add `-Wsign-conversion` to the CI build. Confirmed during review that it flags exactly this line.
3. Audit the rest of `power_model.c` for the same mixed-signedness pattern.

---

# P1 — High

## F-02 — Ring archive wrap invalidates both flash headers

**File:** `Core/Src/flash_log.c`

**Root cause:** `FlashLog_ValidateHeader()` rejects any header whose `record_count > FLASH_LOG_MAX_RECORDS` (32640). `FlashLog_WriteRecord()` increments `record_count` unbounded — the wrap is handled in validation but never in the write path.

**Impact:** At a 5-minute cadence the counter crosses the ceiling at roughly **day 113**. Both headers then fail validation, and the next reboot treats the archive as corrupt — wiping the record index *and* the TX watermark. This lands squarely inside the mission window this device is designed for.

**Fix:** Implement the ring wrap in the write path (modulo `FLASH_LOG_MAX_RECORDS`), maintaining a separate monotonic total-written counter if the absolute count is needed for telemetry. Ensure `HEADER_UPDATE_INTERVAL` (10) still produces a recoverable header after wrap.

**Test:** Host test that writes `FLASH_LOG_MAX_RECORDS + 100` records and asserts both headers still validate and the read-back index is correct.

---

## F-03 — Flash init failure causes an unbounded reset loop

**Files:** `Core/Src/main.c`, `Core/Src/reset_cause.c`

**Root cause:** On W25Q init failure, `main.c` calls `Error_Handler_Fatal(FAULT_CODE_FLASH_INIT)`, which issues `NVIC_SystemReset()`. `reset_cause.c` *clears* the breadcrumb on each boot but never *counts* resets, so there is no boot-loop escape.

**Impact:** A cold-stuck or failed flash chip at altitude produces a silent, permanent reset loop. The radio never comes up. Total mission loss from a non-essential peripheral.

**Fix:** Retry-then-fly-degraded.
1. Retry flash init 2–3 times with a short delay.
2. On continued failure, set a `flash_unavailable` flag, skip the archive, and continue to LoRaWAN init and normal transmit.
3. Add a persistent boot-attempt counter in a backup register so any future fatal path can detect and break a loop.

**Rule to apply generally:** audit all remaining `Error_Handler_Fatal()` call sites. Any that can be triggered by a peripheral rather than a core-logic invariant should degrade, not reset.

---

## F-04 — MSB tick counter zeroed on every boot

**File:** `Core/Src/timer_if.c`

**Root cause:** `TIMER_IF_Init()` unconditionally calls `TIMER_IF_BkUp_Write_MSBticks(0)` on every reset. The RTC's 32-bit counter wraps at ~48.5 days. The backup domain survives a reset, but the MSB is stomped to zero regardless.

**Impact:** After day 48.5, any reset causes the device's notion of time to jump **backward** by up to 48.5 days, corrupting every downstream time-based decision (transmit scheduling, deadman, staleness, log timestamps).

**Fix:** Only zero the MSB ticks on a true cold boot (power-on reset), detected via the RCC reset flags / backup-domain validity marker. Preserve the value across warm resets.

**Pairs with F-05** — the backward jump can directly trigger a spurious deadman fire.

---

## F-05 — Deadman does not re-seed before reset

**File:** `LoRaWAN/App/lora_app.c:806` (`Deadman_Check()`)

**Root cause:** The deadman fires `NVIC_SystemReset()` without first writing the current time into `BKP_REG_DEADMAN`. It currently only survives by an ordering accident elsewhere in the code.

**Impact:** Any change to that ordering — or the backward time jump from F-04 — produces a spurious deadman reset, or worse, a deadman reset loop.

**Fix:** Write `now` into `BKP_REG_DEADMAN` immediately before calling `NVIC_SystemReset()`. Additionally, make `Deadman_Check()` robust to a negative/backward time delta: treat `now < last_progress` as "re-seed and continue", not as an expired deadman.

---

## F-06 — Stale-GPS region switching — **DOWNGRADED to documentation-only**

**Files:** `LoRaWAN/App/lora_app.c` (`SendTxData`), `Core/Src/multiregion_context.c`

**Observed behaviour:** On GPS timeout, `SendTxData` sets `hgnss.data.valid = true` and `fix_quality = GNSS_FIX_GPS` on last-known coordinates. `latLngToRegion()` and `MultiRegion_AutoSwitchForLocation()` therefore run on a possibly-hours-old position. The DDR-0007 staleness bit exists but the region path ignores it.

**Decision (project owner, this review):** **This behaviour is intentional and is to be kept as-is.** Holding the last known region across a GPS gap is correct and safer than the alternatives. A brief GPS dropout does not mean the sonde teleported — snapping from US915 to EU868 because a fix was missed five minutes ago would be the more dangerous action. Going radio-silent on stale position is unacceptable, as it risks the entire mission.

**Action required — documentation only, no behaviour change:**
1. Add a comment at the region-selection site stating that stale-position region hold is deliberate, with the rationale above.
2. Record it as a design decision in `docs/decisions/` (next available number), consistent with DDR-0007's data-honesty framing: the stale bit governs *science data honesty*, not *region selection*.

**Do not "fix" this.** It is listed here so it is not re-flagged by a future review.

---

## F-07 — Bulk transfers pollute the flash archive

**File:** `LoRaWAN/App/lora_app.c` (`OnTxData`, `SendTxData`)

**Root cause:** `OnTxData` re-arms `SendTxData` once per bulk packet. `SendTxData` unconditionally calls `FlashLog_WriteRecord()`. The bulk path skips GPS entirely (memsets `hgnss.data`).

**Impact:**
- Up to **20 junk records per bulk cycle** with `gnss_valid = 0` written into the archive.
- Accelerates the F-02 ring wrap.
- Burns W25Q erase cycles *and* Tier-2 internal-flash erases. Internal flash is rated 10k erases; worst case reaches endurance in roughly **33 days**.

**Fix:** Gate the archive write. Only call `FlashLog_WriteRecord()` when the transmit represents a genuine, freshly-sampled telemetry record — not on bulk retransmits. Suggested approach: pass an explicit `bool archive_this_sample` through the transmit path, or set a module-level flag in the bulk re-arm path that `SendTxData` checks.

**Note:** this fix is entangled with the F-R1 refactor below. Doing the refactor first will make this a two-line change instead of a surgical patch to a 789-line function.

---

# P2 — Medium / hardening

## F-08 — Wakeup timer left armed on sleep-chunk overflow

**File:** `Core/Src/stm32_lpm_if.c:288`

`if (++chunks > 150) break;` exits the chunked-STOP2 loop but skips `HAL_RTCEx_DeactivateWakeUpTimer()`, leaving the WUT armed. Restructure so deactivation runs on every exit path.

## F-09 — Wakeup timer arm failure is silent

**File:** `Core/Src/stm32_lpm_if.c`

The return value of `HAL_RTCEx_SetWakeUpTimer_IT()` is ignored. If the timer fails to arm, the device sleeps with no scheduled wake. Check the return; on failure, fall back to a bounded busy-wait or a shorter sleep rather than an unbounded STOP2.

## F-10 — W25Q timeouts have thin cold margin

**File:** `Core/Src/w25q16jv.c`

Current timeouts vs. datasheet maxima: PAGE_PROG 5 ms (max 3 ms), SECTOR_ERASE 500 ms (max 400 ms). At −60 °C these margins are uncomfortably tight. Widen to at least 3× datasheet max — the cost of a longer timeout is negligible next to a spurious flash failure at altitude (which currently triggers F-03).

## F-11 — `W25Q_WaitReady` never refreshes the IWDG

**File:** `Core/Src/w25q16jv.c`

Currently safe only because no code path issues a CHIP_ERASE (100 s, which would watchdog-reset). Add an `HAL_IWDG_Refresh()` inside the wait loop so this stays safe if a bulk-erase path is ever added.

## F-12 — Implicit `abs` declaration

**File:** `Core/Src/ms5607.c:370`

`<stdlib.h>` is not included, so the compiler infers a prototype. Add the include.

## F-13 — Status enum confusion

**File:** `Core/Src/sht31.c:192`

`SHT31_StatusTypeDef` compared against `HAL_StatusTypeDef`. Two distinct enums that happen to overlap numerically. Use the correct type consistently.

## F-14 — No LSE clock security system

**Files:** `Core/Src/main.c` (clock config), watchdog path

If the 32.768 kHz LSE crystal dies in extreme cold, the RTC and all LoRaMac timers stall — but the main loop keeps refreshing the IWDG. The sonde appears alive while time is frozen. **This is the most dangerous P2 for a cold mission.**

**Fix:** Enable `HAL_RCCEx_EnableLSECSS()`, and add a SysTick-based liveness check that verifies the RTC is actually advancing. On LSE failure, switch to LSI and continue.

## F-15 — Last-valid position lost on reset

**File:** `LoRaWAN/App/lora_app.c`

`last_valid_lat`, `last_valid_lon`, `last_valid_alt` are function-local `static`s. A reset zeroes them, so post-reset the device briefly believes it is at 0°,0° (Null Island) — which, given F-06's deliberate stale-position hold, feeds a garbage position into region selection.

**Fix:** Park them in backup registers alongside the other survive-the-reset state, with a validity marker.

## F-16 — Unclamped casts into the log record

**File:** `LoRaWAN/App/lora_app.c` (record population)

`record.battery_mv` and `record.gnss_hdop_x10` are cast without range clamping. Casting a NaN or out-of-range float to an integer type is undefined behaviour. Clamp to the field's representable range before casting.

## F-17 — Maybe-uninitialized warning

**File:** `Core/Src/multiregion_context.c:1608`

`t2.active_slot` flagged as maybe-uninitialized. Assessed as a **likely false positive**, but initialize explicitly to silence it and remove ambiguity.

---

# Refactoring targets

Complexity is heavily concentrated. Function sizes measured across the codebase:

| Lines | Location | Function |
|---:|---|---|
| **789** | `LoRaWAN/App/lora_app.c:870–1656` | `SendTxData()` |
| **306** | `Core/Src/multiregion_context.c:464` | `MultiRegion_SwitchToRegion()` |
| **261** | `Core/Src/multiregion_context.c:1091` | `MultiRegion_PreJoinAllRegions()` |
| **190** | `Core/Src/flash_log.c:504` | `FlashLog_DeInit()` |
| **184** | `Core/Src/multiregion_context.c:907` | `MultiRegion_JoinRegion()` |
| **183** | `Core/Src/multiregion_context.c:257` | `MultiRegion_ForceSaveCurrentContext()` |
| **179** | `LoRaWAN/App/lora_app.c:431` | `LoRaWAN_Init()` |
| 126 | `Core/Src/atgm336h.c:480` | `GNSS_ProcessDMABuffer()` — *acceptable* |

`atgm336h.c` is well-factored and needs no structural work.

## F-R1 — Decompose `SendTxData()` (789 lines) — **highest priority refactor**

**File:** `LoRaWAN/App/lora_app.c:870–1656`

This is the largest function in the codebase and it is load-bearing: it is the sole place the deadman makes progress (DDR-0001, per the comment at line 787). Monolithic *and* critical is the worst combination.

**Suggested decomposition** (the coding agent may choose its own seams, but these are the natural ones):
- `AcquireSensorSample()` — GPS/pressure/temp/humidity acquisition and staleness marking
- `PopulateLogRecord()` — record construction, including the F-16 clamps
- `SelectRegionAndSession()` — region lookup and switching (preserving F-06 behaviour)
- `BuildAndQueueUplink()` — payload encode and LmHandler send
- `ArchiveSample()` — the gated flash write from F-07

The mission-state early-returns (`COMMISSIONING` join retry, `FLIGHT` with no session → RF silence) should remain at the top level of `SendTxData()` so control flow stays obvious. **Preserve the deadman progress semantics exactly** — the refactor must not change where or when the deadman is fed.

## F-R2 — Slim `MultiRegion_SwitchToRegion()` (306 lines)

**File:** `Core/Src/multiregion_context.c:464–765`

Inspected in detail. The bloat is *not* algorithmic — the real logic is small. Two causes:

**1. Debug logging — roughly a third of the function.** Bring-up scaffolding that was never removed: full context dumps, per-field logging, and byte-by-byte hex loops printing both session keys (lines 598–612).

> **Security note:** `AppSKey` and `NwkSKey` are printed in full over the serial log. This should be removed outright, not merely compiled out by default.

**Fix:** Gate all diagnostic output behind a compile-time verbosity flag; delete the key dumps entirely.

**2. A ten-step sequenced ritual with hard delays.** Reinit stack → `HAL_Delay(100)` → configure → `HAL_Delay(50)` → set keys → set DevAddr → restore counters → channel mask → `LoRaMacStart()` → `HAL_Delay(200)` → re-set DevAddr → pump `LmHandlerProcess()` in delay loops until not busy. Note DevAddr is set twice (steps 4 and 8) because it does not persist across `LoRaMacStart()`.

**Fix:** Extract the sequence into a `RestoreSessionToMac(ctx)` helper. Where possible, replace fixed `HAL_Delay()` calls with bounded poll-until-ready loops that have explicit timeouts, so the function is not carrying blind timing assumptions.

**3. Region-specific channel masks (lines 631–674)** — the only genuinely region-varying part. Make it table-driven: a small static table of `{region, mask[], mask_len}` replaces the if/else chain and makes adding a region a data change rather than a code change.

Stripping the logging and extracting the restore sequence should roughly halve this function.

## F-R3 — Clean up the commissioning-path join functions

**Files:** `Core/Src/multiregion_context.c:1091` (`MultiRegion_PreJoinAllRegions`, 261 lines), `:907` (`MultiRegion_JoinRegion`, 184 lines)

These run on the ground during `COMMISSIONING` only, so the flight-safety risk is low, but the size is unjustified. Expect the same causes as F-R2 — excess logging and inlined per-region sequencing. Apply the same treatment, reusing the `RestoreSessionToMac()` helper where the sequences overlap.

## F-R4 — Investigate `FlashLog_DeInit()` (190 lines)

**File:** `Core/Src/flash_log.c:504`

A 190-line teardown function is a strong signal it is doing more than tearing down. Inspect and split; whatever non-teardown work lives there likely belongs in its own function with a name that describes it.

---

# Not yet reviewed

The following were not covered and remain open review surface. Flagged in rough order of value:

1. **`Core/Src/atgm336h.c`** — GNSS parser (1304 lines). Has an F-011 DMA head-wrap bug history. **Highest-value remaining target.** Function map obtained; structure looks sound, but the parse paths were not audited line-by-line.
2. `Core/Src/sys_sensors.c`
3. `Core/Src/adc_if.c`
4. `Core/Src/payload_encode.c`
5. `Core/Src/config.c`
6. `Middlewares/Third_Party/h3lite` submodule (commit `6cd0a4f`)

---

# Suggested execution order

1. **F-01** — one-line fix, worst consequence. Do it first, with the negative-slope host test.
2. **F-03**, **F-14** — the two "silent total loss" failure modes.
3. **F-04 + F-05** together — they interact.
4. **F-R1** — decompose `SendTxData()`.
5. **F-07** — falls out of F-R1 nearly for free.
6. **F-02** — ring wrap, plus its host test.
7. **F-R2**, **F-R3**, **F-R4** — region and flash-log cleanup.
8. **F-08 – F-13**, **F-15 – F-17** — hardening sweep.
9. **F-06** — documentation and DDR entry only. No behaviour change.

**CI additions to land alongside:** `-Wsign-conversion`, and the negative-slope and ring-wrap host tests.

---

# Review verdict

Seven substantive findings, one genuine P0, and the majority of fixes are small and surgical. For a codebase of this ambition — multi-region session banking, chunked low-power sleep, a flash ring archive, and −60 °C operation — this is a clean bill of health. The structural debt is concentrated in two files rather than spread throughout, which makes it tractable.
