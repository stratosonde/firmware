# Stratosonde Firmware — Independent Code Review

**Repository:** `https://github.com/stratosonde/firmware`
**Commit reviewed:** `3dc0d32100aae04ffd886fb6aef691a9861a414d` ("FW-20: shrink bulk packet 222->198 B") — 2026-08-02
**Review date:** 2026-08-03
**Scope:** application code (`Core/`, `LoRaWAN/App/`, `LoRaWAN/Target/`), drivers, and `docs/`. Vendor code (HAL, CMSIS, Semtech LoRaMac) was read only where behaviour was load-bearing for a finding.

---

## How to use this document

This is a **verification worklist**, not a patch set. Every finding below is stated as a falsifiable claim with the evidence that produced it. Treat each one as a hypothesis to confirm against the actual code before changing anything.

For each finding:

1. Read the **Evidence** and open the cited file/line.
2. Run the **Verify** step. If the claim does not reproduce, mark the finding `NOT CONFIRMED` and record why — do not silently skip it.
3. Only then apply the **Fix**. Where the fix is a design choice rather than a mechanical correction, the finding says so; surface the options rather than picking one.

Line numbers are from the reviewed commit and may drift. Prefer matching on the quoted code.

### Ground rules

- **Do not batch-fix by category.** Several findings interact (see *Ordering hazards* below).
- **Do not "clean up" adjacent code** while fixing. Several existing bugs are currently masked by other bugs; removing the mask without removing the bug makes things worse.
- **Preserve the ADR contracts.** `docs/adr/0001`–`0008` encode deliberate decisions. Where a finding contradicts an ADR, the finding says which one and why — that is a decision for the maintainer, not an automatic override.
- **Flight code has no debugger.** Prefer degrade-and-continue over halt, bounded loops over unbounded ones, and explicit staleness flags over silent substitution — consistent with ADR-0001 and ADR-0007.

### Ordering hazards — read before touching anything

| Hazard | Detail |
|---|---|
| **R24 before R23** | The wrong NMEA checksums (R23) are currently the *only* thing preventing `$PCAS11,2` from overwriting airborne mode (R24). Fixing checksums alone re-imposes the GPS 18 km CoCom ceiling and ends the flight at the tropopause. Delete `GNSS_CMD_FIX_MODE` **in the same change**. |
| **R01/R02 together** | The three backup-register collisions are one problem. Fixing only the deadman (R01) leaves mission state and reset cause still clobbered by `SysTimeSet()`. |
| **R04 and R05 together** | The compact-packet size and the pressure encoding both live in the same 11 bytes. Solving them separately will produce two incompatible wire formats. |
| **R40–R43 after R04/R05** | Regenerate `docs/PayloadFormats.md` from the final struct. Do not hand-edit it now; it is already wrong in three independent ways. |

### Severity key

| | Meaning |
|---|---|
| **P0** | Loses the vehicle or the entire mission. Fix before any flight. |
| **P1** | Loses a major capability, a region, or a large fraction of the power budget. |
| **P2** | Correctness, robustness, maintainability, or data quality. |
| **DOC** | Documentation contradicts code. Ground-station or reviewer facing. |

---

## Index

| ID | Sev | Area | One line |
|---|---|---|---|
| R01 | P0 | `lora_app.c` / `timer_if.c` | Deadman writes RTC_BKP_DR2 = MSBTICKS → corrupt time base → reset loop in FLIGHT |
| R02 | P0 | backup registers | DR0/DR1/DR2 each claimed by two owners |
| R23 | P0 | `atgm336h.h` | 4 of 8 hardcoded NMEA checksums wrong → commands silently rejected |
| R24 | P0 | `atgm336h.c` | `$PCAS11,2` is pedestrian mode and overwrites airborne mode |
| R03 | P1 | `lora_app.c` | `DatarateFromSF` maps AU915 like US915 → wrong SF, bulk exceeds max payload |
| R04 | P1 | `payload_format.h` | Compact packet is 11 B; US915 DR0 limit is 11 B and FOpts counts against it |
| R05 | P1 | `payload_encode.c` | Pressure encoding floor is 950 hPa → clamps to 0 for the entire flight |
| R06 | P1 | `sys_sensors.c` / `flash_log.c` | `altitudeBar` never assigned → uninitialised stack written to flash |
| R07 | P1 | `stm32_lpm_if.c` | Chunked-sleep wake test reads unimplemented EXTI PR1 bits |
| R08 | P1 | `main.c` / `stm32wlxx_hal_msp.c` | LSE→LSI failover undone by MspInit; wipes backup domain |
| R09 | P1 | `stm32_lpm_if.c` | PA0 LED driven on every STOP2 exit, in flight |
| R10 | P1 | `lora_app.c` | Brownout floor tested against temperature-normalised voltage |
| R25 | P1 | `atgm336h.c` | GPS cold-starts every cycle; PCAS12 ephemeris claim is unfounded |
| R26 | P1 | `atgm336h.h` | GSV enabled but only consumer is compiled out; 9600 baud oversubscribed |
| R27 | P1 | `atgm336h.c` | No DMA overrun detection |
| R28 | P1 | `ms5607.c` | Returns OK on zero/implausible ADC; poisons last-known-good cache |
| R29 | P1 | `w25q16jv.c` | Block Protect bits never checked → silent write failure |
| R30 | P1 | `multiregion_context.c` | Unbounded join wait makes commissioning unfinishable |
| R11 | P2 | `lora_app.c` | Restricted-region early return skips the flash log write |
| R12 | P2 | `flash_log.c` | Header ping-pong selection uses `record_count` as sequence |
| R13 | P2 | `flash_log.c` | Unbounded corrupt-record scan, no watchdog refresh |
| R14 | P2 | `flash_log.c` | `FlashLog_EraseAll` would trip IWDG; leaves watermark inconsistent |
| R15 | P2 | `payload_encode.c` | Bulk packet always 198 B regardless of record count |
| R16 | P2 | `adc_if.c` | `HAL_MAX_DELAY` poll; ~16 full ADC init/calibrate cycles per work cycle |
| R17 | P2 | `stm32_lpm_if.c` | VREFBUF is not VREFINT; 10 ms busy-wait per wake |
| R18 | P2 | `adc_if.c` | Solar ADC has no divider; saturates at VDDA |
| R19 | P2 | `payload_encode.c` | `ConvertFlashLogToHighRes` cannot fail; stamps history with present metadata |
| R20 | P2 | `sys_sensors.c` | No-fix records store ST demo coordinates (Saint-Ouen) |
| R21 | P2 | `lora_app.c` | Bulk-transfer livelock on unconvertible records |
| R22 | P2 | `payload_encode.c` | "Compile-time" size validation is runtime and non-fatal |
| R31 | P2 | `lora_app.c` | GPS state invalidation before acquisition is incomplete |
| R32 | P2 | `atgm336h.c` | `lat_raw > 0` skips prime meridian / equator |
| R33 | P2 | `atgm336h.c` | Checksum verify returns true when `*` absent |
| R34 | P2 | `atgm336h.c` | Float coordinate math; doc claims 1e-7 deg |
| R35 | P2 | `config.c` | CRC excludes the wrong 4 bytes |
| R36 | P2 | `config.h` | 24 of 32 config fields have no consumers; several contradict hardcoded values |
| R37 | P2 | `multiregion_context.c` | Hardcoded `0x0803F000UL` instead of the symbol |
| R38 | P2 | sensors | Operated outside characterised range with no flag |
| R39 | P2 | `w25q16jv.c` | `W25Q_WaitReady` busy-spins at full clock |
| R40 | DOC | `docs/PayloadFormats.md` | Published decoder rejects every production uplink |
| R41 | DOC | `docs/PayloadFormats.md` | Documents a different coordinate encoding than the code |
| R42 | DOC | `docs/PayloadFormats.md` | Bakes in the pressure-floor bug; decodes 0 as 950 hPa |
| R43 | DOC | `docs/adr/0005` | Central premise about FOpts is factually wrong |
| R44 | DOC | various | Assorted stale claims in ADRs and readiness matrix |

---

# P0 findings

## R01 — Deadman corrupts the RTC time base and self-resets in a loop

**Severity:** P0 · **Files:** `LoRaWAN/App/lora_app.c:1044-1074`, `Core/Src/timer_if.c:100,400-422,488-499`

**Claim.** In `MISSION_ASCENT`/`MISSION_FLOAT` the device resets within milliseconds of reaching the main loop, permanently. The bug is invisible in `MISSION_COMMISSIONING` because `Deadman_Check()` returns early there.

**Evidence.**

```c
/* lora_app.c:1044 */
#define DEADMAN_BKP_REG     RTC_BKP_DR2

/* timer_if.c:100 */
#define RTC_BKP_MSBTICKS   RTC_BKP_DR2
```

Same register. `Deadman_MarkProgress()` writes RTC *seconds* into DR2. `TIMER_IF_GetTime()` reads DR2 as the MSB-ticks word:

```c
/* timer_if.c:400 */
uint32_t timerValueMSB = TIMER_IF_BkUp_Read_MSBticks();   /* reads DR2 */
ticks = (((uint64_t) timerValueMSB) << 32) + timerValueLsb;
seconds = (uint32_t)(ticks >> RTC_N_PREDIV_S);            /* RTC_N_PREDIV_S == 10 */
```

Sequence:
1. Boot. `TIMER_IF_Init()` writes MSBticks = 0 → DR2 = 0.
2. First `Deadman_Check()` sees `last == 0`, seeds DR2 with `now` (a plausible seconds value, e.g. 1000).
3. Next main-loop iteration: `TIMER_IF_GetTime()` computes `(1000 << 32 + lsb) >> 10` ≈ `1000 × 2^22` ≈ 4.19e9.
4. `now - last` ≈ 4.19e9 > `DEADMAN_TIMEOUT_S` (10800) → `NVIC_SystemReset()`.
5. Reset → step 1.

The general form is `seconds_out ≈ ((S mod 1024) << 22) + lsb>>10`, which is only benign when `S mod 1024 == 0`.

**Collateral.** Everything downstream of `TIMER_IF_GetTime()` is poisoned once the deadman has written: flash record timestamps, `now_timestamp` for the voltage slope, `timestamp_min` in the compact packet, and `SysTimeGet()` inside LoRaMac.

**Verify.**
- Confirm `RTC_BKP_DR2` resolves to the same TAMP backup register in both translation units.
- Instrument: log `TIMER_IF_GetTime()` immediately before and after the first `Deadman_MarkProgress()`.
- Force `MissionState_Get() == MISSION_ASCENT` on the bench and observe the reset loop.

**Fix.** Give the deadman storage that ST's timer layer does not own. Options, in order of preference:
1. A different backup register (DR3+ are free on this part — confirm the count for STM32WLE5JC).
2. A `__attribute__((section(".noinit")))` word in RAM. Survives reset, lost on power cycle — acceptable, since the deadman's purpose is detecting a wedged-but-powered system.

Do **not** solve this by making the deadman read/write through `TIMER_IF` helpers; the register itself is the conflict.

---

## R02 — All three ST-reserved backup registers are double-booked

**Severity:** P0 · **Files:** `Core/Src/timer_if.c:90-100`, `Core/Src/mission_state.c:19`, `Core/Inc/reset_cause.h:32`, `LoRaWAN/App/lora_app.c:1044`

**Claim.** Every backup register used by ST's SysTime/timer layer has a second application-level claimant.

| Register | ST owner (`timer_if.c`) | Second claimant |
|---|---|---|
| DR0 | `RTC_BKP_SECONDS` (SysTime seconds delta) | `MISSION_STATE_BKP_REG` |
| DR1 | `RTC_BKP_SUBSECONDS` (SysTime subseconds) | `RESET_CAUSE_BKP_FAULT_REG` |
| DR2 | `RTC_BKP_MSBTICKS` | `DEADMAN_BKP_REG` (see R01) |

**Evidence.** `SysTimeSyncFromGnss()` (`lora_app.c:1092-1112`) calls `SysTimeSet()` on **every good GPS fix**. `Utilities/misc/stm32_systime.c:238-239`:

```c
UTIL_SYSTIMDriver.BKUPWrite_Seconds( DeltaTime.Seconds );      /* → DR0 */
UTIL_SYSTIMDriver.BKUPWrite_SubSeconds( (uint32_t) DeltaTime.SubSeconds ); /* → DR1 */
```

**Consequences.**
- After the first GPS fix, `MissionState_Init()`'s magic check (`0xA55A0000`) fails every boot → falls through to the bank anchor → **`MISSION_ASCENT`**. ADR-0008 states transitions never move toward higher power; in practice every reset while in FLOAT drops back to ascent cadence for another 3 h.
- `RESET_CAUSE_FAULT` (`0xF17B0000`) is destroyed → fault resets are misreported as IWDG or POR in the uplink status byte.
- Conversely `MissionState_Persist()` writes `0xA55A000x` into SysTime's seconds delta, corrupting `SysTimeGet()`.

**Verify.** Dump DR0–DR2 before and after a GPS fix. Confirm the mission-state magic survives (it will not).

**Fix.** Relocate all three application users to registers ST does not touch, and add a single header that owns the backup-register map so a future collision is a compile-time conflict rather than a silent one:

```c
/* backup_regs.h — single source of truth */
/* DR0..DR2 are RESERVED for stm32_systime / timer_if. Do not use. */
#define BKP_MISSION_STATE   RTC_BKP_DR3
#define BKP_RESET_CAUSE     RTC_BKP_DR4
#define BKP_DEADMAN         RTC_BKP_DR5
```

Confirm the available register count for the STM32WLE5JC (TAMP backup registers) before assigning.

---

## R23 — Four of eight hardcoded NMEA checksums are wrong

**Severity:** P0 · **File:** `Core/Inc/atgm336h.h:165-175`

**Claim.** Four `$PCAS*` configuration sentences carry incorrect XOR checksums and are therefore discarded by the receiver. `GNSS_Configure()` reports success regardless, because `GNSS_SendCommand()` only checks that the UART transmit completed.

**Evidence.** Recomputed XOR over the sentence body (between `$` and `*`):

| Constant | Sentence | In code | Correct | Status |
|---|---|---|---|---|
| `GNSS_CMD_NMEA_CONFIG` | `$PCAS03,1,0,0,1,1,1,0,0` | `*02` | `02` | OK |
| `GNSS_CMD_CONSTELLATION` | `$PCAS04,5` | `*1C` | `1C` | OK |
| `GNSS_CMD_AIRBORNE_MODE` | `$PCAS11,5` | `*18` | `18` | OK |
| `GNSS_CMD_UPDATE_RATE` | `$PCAS02,1000` | `*2B` | **`2E`** | **REJECTED** |
| `GNSS_CMD_SATELLITE_SYS` | `$PCAS04,7` | `*1A` | **`1E`** | **REJECTED** |
| `GNSS_CMD_FIX_MODE` | `$PCAS11,2` | `*1E` | **`1F`** | **REJECTED** |
| `GNSS_CMD_SAVE_CONFIG` | `$PCAS00` | `*01` | `01` | OK |
| `GNSS_CMD_STANDBY` | `$PCAS12,0` | `*1C` | **`1E`** | **REJECTED** |

**Verify.** Reproduce with the script in *Appendix A*. Then confirm on hardware by observing the module's response (or lack of it) to each sentence.

**Impact.** The tri-constellation configuration (`PCAS04,7`) never applies — the receiver runs GPS+GLONASS from the earlier `PCAS04,5`. Update rate and standby commands are also never applied.

**Fix.** The driver already exposes `GNSS_CalculateChecksum()` (`atgm336h.c:798`). Build sentences at runtime:

```c
static GNSS_StatusTypeDef GNSS_SendPCAS(GNSS_HandleTypeDef *h, const char *body)
{
    char buf[64];
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
    snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, cs);
    return GNSS_SendCommand(h, buf);
}
```

**Read R24 before applying this.**

---

## R24 — `$PCAS11,2` is pedestrian mode and overwrites airborne mode

**Severity:** P0 · **File:** `Core/Src/atgm336h.c:208-276`, `Core/Inc/atgm336h.h:170`

**Claim.** `GNSS_CMD_FIX_MODE` is mislabelled and semantically wrong, and its position in `GNSS_Configure()` means that fixing R23 alone would re-impose the GPS 18 km CoCom altitude ceiling.

**Evidence.** `GNSS_Configure()` sends, in order:

```
step 3:  GNSS_CMD_AIRBORNE_MODE   $PCAS11,5*18   ← checksum OK, APPLIED
...
step 6:  GNSS_CMD_FIX_MODE        $PCAS11,2*1E   ← checksum WRONG, rejected
```

`PCAS11` is a **single setting** — the navigation dynamic model — so the last accepted value wins. The CASIC mode table is:

```
0 = portable    1 = stationary   2 = pedestrian   3 = automotive
4 = sea         5 = airborne <1g 6 = airborne <2g 7 = airborne <4g
```

There is no "Auto 2D/3D" selector on `PCAS11` at all; the header comment (`// Auto 2D/3D fix`) is fabricated. `$PCAS11,2` selects **pedestrian**.

The only reason airborne mode currently survives is the broken checksum on step 6.

**Verify.** Consult the ATGM336H / CASIC protocol document for the `PCAS11` argument table. Then, on hardware with a GPS simulator, set `PCAS11,5` followed by `PCAS11,2` and confirm the second overrides the first.

**Fix.**
1. Delete `GNSS_CMD_FIX_MODE` and its send block in `GNSS_Configure()`.
2. Add a comment at the `PCAS11` definition: *"PCAS11 is the navigation dynamic model — single value, last write wins. Only `,5` (airborne <1g) is valid for this mission. Do not send any other PCAS11 value anywhere in the codebase."*
3. Add a grep-able guard or a build check that no second `PCAS11` send exists.
4. Only then apply R23.

---

# P1 findings

## R03 — `DatarateFromSF` maps AU915 like US915

**Severity:** P1 · **File:** `LoRaWAN/App/lora_app.c:543-567`

**Claim.** Over Australia the "SF10 probe" transmits at SF12 and the "SF7 bulk" transmits at SF9, where the 198 B bulk packet exceeds the maximum payload and is rejected by the MAC.

**Evidence.** From the in-tree region headers:

```c
/* RegionUS915.h:232 */
static const uint8_t DataratesUS915[] = { 10, 9, 8,  7,  8,  0, 0, 0, ... };
/* RegionAU915.h:261 */
static const uint8_t DataratesAU915[] = { 12, 11, 10, 9, 8, 7, 8, 0, ... };
```

| | DR_0 | DR_1 | DR_2 | DR_3 | DR_4 | DR_5 |
|---|---|---|---|---|---|---|
| US915 | SF10 | SF9 | SF8 | **SF7** | SF8/500 | — |
| AU915 | SF12 | SF11 | **SF10** | SF9 | SF8 | **SF7** |

The code maps both regions identically:

```c
if (sf == 10) { case US915: case AU915: return DR_0; ... }
else if (sf == 7) { case US915: case AU915: return DR_3; ... }
```

Max payload at AU915 DR_3:

```c
/* RegionAU915.h:286 */
static const uint8_t MaxPayloadOfDatarateDwell0AU915[] = { 51, 51, 51, 115, 242, 242, ... };
/* RegionAU915.h:299 — with uplink dwell time */
static const uint8_t MaxPayloadOfDatarateDwell1AU915[] = { 0, 0, 11, 53, 125, 242, ... };
```

198 B > 115 B (dwell 0) and > 53 B (dwell 1) → every bulk uplink in AU915 fails the length check.

**Verify.** Add a temporary boot-time assertion that walks all four regions and logs `Datarates<Region>[DatarateFromSF(7)]` and `[DatarateFromSF(10)]`, asserting they equal 7 and 10 respectively.

**Fix.**

```c
case LORAMAC_REGION_AU915:  return (sf == 10) ? DR_2 : DR_5;
```

Better: replace the hand-written switch with a table lookup that resolves SF → DR by searching the region's own `Datarates*[]` array, so the mapping cannot drift from the region definition. Also add a runtime guard that queries `PHY_MAX_PAYLOAD` for the resolved DR and refuses to attempt a send that cannot fit.

**Note.** `docs/FixWorkorderPlan.md:30` (the F16 workorder) explicitly instructed *"verify map against in-tree `Region*.c`"*. That verification was completed for EU868/AS923 but not AU915.

---

## R04 — Compact packet is 11 B; the US915 DR0 budget is 11 B including FOpts

**Severity:** P1 · **Files:** `Core/Inc/payload_format.h:92-101`, `LoRaWAN/App/lora_app.c:1628-1668`

**Claim.** With `LmHandlerLinkCheckReq()` pending, the 11-byte compact packet cannot be sent at US915 DR_0 — the MAC rejects it on length and the probe uplink is lost.

**Evidence.**

```c
/* RegionUS915.h:254 */
static const uint8_t MaxPayloadOfDatarateUS915[] = { 11, 53, 125, 242, 242, ... };
```

Semtech `LoRaMac.c` `ValidatePayloadLength()` computes `payloadSize = lenN + fOptsLen` and compares against `PHY_MAX_PAYLOAD`. `LinkCheckReq` is 1 byte of FOpts → 11 + 1 = 12 > 11.

Struct is 11 bytes: `2 + 2 + 2 + 1 + 1 + 1 + 1 + 1`.

Commit `de0505f` shrank the packet to 10 bytes specifically for this. F17 restored the status byte on the reasoning that *"LinkCheck rides FOpts (ADR-0005), so the payload byte is free"* — see R43; that premise is wrong.

Even at 10 bytes the margin is exactly zero: any pending `LinkADRAns`, `DevStatusAns`, `RXParamSetupAns` or `NewChannelAns` pushes FOpts past 1 byte and the packet is dropped.

**Verify.** Instrument the return value of `LmHandlerSend()` for the compact probe and count `LORAMAC_HANDLER_PAYLOAD_LENGTH_ERROR`. Reproduce by forcing a pending MAC answer.

**Fix — this is a design decision, present the options:**

- **(a)** Probe at US915 DR_1 (SF9, 53 B) instead of DR_0. Costs ~2.5 dB link budget, buys 42 bytes of headroom.
- **(b)** Keep DR_0, shrink to ≤ 8 bytes, and accept a hard cap on FOpts.
- **(c)** Detect `PAYLOAD_LENGTH_ERROR` and retry immediately without `LinkCheckReq`, so a MAC-command collision costs the link check rather than the telemetry.

Whichever is chosen, add a boot-time check that asserts `sizeof(CompactTelemetryPacket_t) + WORST_CASE_FOPTS <= min(MaxPayloadOfDatarate[chosen DR])` across all four enabled regions.

**Coordinate with R05** — both consume the same 11 bytes.

---

## R05 — Pressure encoding floor is 950 hPa; clamps to zero for the whole flight

**Severity:** P1 · **File:** `Core/Src/payload_encode.c:432-442`

**Claim.** The compact packet's pressure field reads 0 for every sample above roughly 500 m altitude, so the primary altitude proxy is unusable for the entire mission.

**Evidence.**

```c
static uint8_t ConvertPressureToCompact(float pressure_mbar)
{
    int16_t relative_pressure = (int16_t)((pressure_mbar - 950) / 10);
    if (relative_pressure > 255) relative_pressure = 255;
    if (relative_pressure < 0) relative_pressure = 0;      /* ← everything below 950 hPa */
    return (uint8_t)relative_pressure;
}
```

Float altitude is 50–100 hPa; 40 km is ~3 hPa. All clamp to 0.

`payload_format.h:87` states *"Altitude calculated on ground station from pressure + temperature"* — impossible with this encoding.

**Verify.** Trivial: call `ConvertPressureToCompact(75.0f)` and observe 0.

**Fix — design decision.** A 1-byte linear field cannot span 3–1050 hPa usefully. Options:

- **(a) Logarithmic byte.** `idx = round(255 * ln(P/P_min) / ln(P_max/P_min))` over 2–1100 hPa gives ~2.5 % steps — about 1.4 hPa at 50 hPa (~250 m). Coarse but recovers the whole range in one byte.
- **(b) Steal bits.** `humidity_5pct` only uses 0–20 (5 bits), freeing 3 bits. An 11-bit pressure field over 3–1050 hPa in a geometric or piecewise-linear scale gives far better resolution.
- **(c) Reallocate.** If R04 pushes the probe to DR_1 (53 B), there is room for a 2-byte pressure field at 0.1 hPa.

Whatever is chosen, the ground decoder and `docs/PayloadFormats.md` must be regenerated together (R40–R42).

**Related.** `ConvertHumidityToCompact` uses 5 % steps, which is too coarse to be scientifically useful for UT/LS humidity (typical RH 1–20 %). Worth revisiting in the same change.

---

## R06 — `altitudeBar` is never assigned; uninitialised stack is written to flash

**Severity:** P1 · **Files:** `Core/Src/sys_sensors.c:160-305`, `Core/Src/flash_log.c:503`, `LoRaWAN/App/lora_app.c:1127`

**Claim.** Every flash log record contains an indeterminate value in `altitude_bar`, covered by the record CRC32 so it appears authentic. This is undefined behaviour and a direct ADR-0007 violation.

**Evidence.**

```c
/* lora_app.c:1127 — not zero-initialised */
sensor_t sensor_data;
EnvSensors_Read(&sensor_data);
```

`EnvSensors_Read()` assigns `humidity`, `temperature`, `pressure`, the stale flags, the three voltages, and (in both GNSS branches) `latitude`, `longitude`, `altitudeGps`, `satellites`, `gnss_fix_quality`, `gnss_hdop`, `gnss_valid`. It **never** assigns `altitudeBar`. Grep confirms no assignment anywhere in the tree.

```c
/* flash_log.c:503 */
record.altitude_bar = sensor_data->altitudeBar;
```

**Additional range problems in the same area:**

```c
/* sys_sensors.h */
int16_t altitudeGps;    /*!< in m */        → overflows at 32,767 m (32.77 km)
int16_t altitudeBar;    /*!< in m * 10 */   → overflows at 3,276.7 m (3.28 km)
```

`main.c:746` comments reference *"At 40 km altitude..."*. `altitudeGps` cannot represent it. `altitudeBar`'s declared scaling cannot represent anything above 3.3 km.

**Verify.** Add `-Wmaybe-uninitialized` / run a static analyser, or simply log `sensor_data.altitudeBar` immediately after `EnvSensors_Read()` across reboots and observe non-determinism.

**Fix.**
1. `sensor_t sensor_data = {0};` at both call sites — necessary but not sufficient.
2. Decide the field's fate: either compute barometric altitude from pressure + temperature in `EnvSensors_Read()`, or delete the field from `sensor_t` and `FlashLog_Record_t` (a permanently-zero placeholder is exactly what FW-20 removed from the bulk packet).
3. Widen `altitudeGps` to `int32_t` (metres) or keep `int16_t` with explicit saturation and a `alt_saturated` flag. Do not silently wrap.
4. If `altitudeBar` is kept, change the scaling from `m*10` to plain metres in `int32_t`.

---

## R07 — Chunked-sleep wake-source test reads unimplemented EXTI bits

**Severity:** P1 · **File:** `Core/Src/stm32_lpm_if.c:275-292`

**Claim.** Both wake-source tests are always false, so the loop exits after a single 25 s chunk and performs a full `PWR_ExitStopMode()` peripheral re-initialisation every 25 seconds. This is the exact symptom the F2 comment claims to have fixed.

**Evidence.**

```c
uint32_t is_wakeup_timer = (EXTI->PR1 & (1UL << 19)) != 0;
uint32_t is_alarm_a      = (EXTI->PR1 & (1UL << 17)) != 0;
```

Two independent problems:

1. **Wrong line number.** `Drivers/STM32WLxx_HAL_Driver/Inc/stm32wlxx_hal_rtc_ex.h:1231`:
   ```c
   #define RTC_EXTI_LINE_WAKEUPTIMER_EVENT   EXTI_IMR1_IM20  /* line 20, not 19 */
   ```

2. **No pending bit exists for either line.** The CMSIS header defines `EXTI_PR1_PIF0` … `EXTI_PR1_PIF16`, then jumps to `EXTI_PR1_PIF21`, `EXTI_PR1_PIF22`. Lines 17–20 are *direct* EXTI lines with no PR bit. Correspondingly, the HAL defines `__HAL_RTC_ALARM_EXTI_ENABLE_IT` / `_DISABLE_IT` / `_ENABLE_EVENT` / `_DISABLE_EVENT` but **no** `_CLEAR_FLAG` or `_GET_FLAG` macro.

Verified: the HAL IRQ handlers (`HAL_RTCEx_WakeUpTimerIRQHandler`, `HAL_RTC_AlarmIRQHandler`) only clear `RTC->SCR` flags, so the F2 diagnosis was right — but the replacement mechanism does not exist on this part.

**Cost.** With a 5-minute cadence, ~12 full wake/re-init cycles per work cycle: DMA re-init, I²C DeInit+Init, SPI DeInit+Init, W25Q release from deep power-down, `HAL_SYSCFG_EnableVREFBUF()` (see R17), `vcom_Resume()`, and the PA0 LED (see R09).

**Verify.** Toggle a GPIO in `PWR_ExitStopMode()` and scope it during a long sleep. Expect a pulse every 25 s rather than one per work cycle.

**Fix.** Latch the wake source in the callbacks, which is where the information actually is:

```c
static volatile uint8_t s_woke_wut, s_woke_alarm;

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *h) { (void)h; s_woke_wut = 1; }
/* timer_if.c already owns HAL_RTC_AlarmAEventCallback — set s_woke_alarm there
   in a USER CODE block, before UTIL_TIMER_IRQ_MAP_PROCESS(). */
```

Clear both before `HAL_PWREx_EnterSTOP2Mode()`, test them after. Keep the 150-chunk bound as belt and braces.

---

## R08 — LSE→LSI failover is undone by MspInit and wipes the backup domain

**Severity:** P1 · **Files:** `Core/Src/main.c:317-333`, `Core/Src/stm32wlxx_hal_msp.c:227-245`

**Claim.** The F3 failover does not survive RTC initialisation. A unit with a dead LSE ends up with a completely stopped RTC — worse than no failover at all — and loses its backup registers in the process.

**Evidence.** `SystemClock_Config()` on LSE failure:

```c
RCC_PeriphCLKInitTypeDef rtcClk = {0};
rtcClk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
rtcClk.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
HAL_RCCEx_PeriphCLKConfig(&rtcClk);
```

Later, `MX_RTC_Init()` → `HAL_RTC_MspInit()` unconditionally:

```c
PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
```

`HAL_RCCEx_PeriphCLKConfig` (`stm32wlxx_hal_rcc_ex.c`):

```c
if (LL_RCC_GetRTCClockSource() != PeriphClkInit->RTCClockSelection)
{
    tmpregister = READ_BIT(RCC->BDCR, ~(RCC_BDCR_RTCSEL));
    __HAL_RCC_BACKUPRESET_FORCE();     /* clears ALL backup registers */
    __HAL_RCC_BACKUPRESET_RELEASE();
    RCC->BDCR = tmpregister;
}
```

So in the failover case: RTCSEL was LSI, LSE is requested, the backup domain is reset (destroying mission state, reset cause, deadman, SysTime), and a dead LSE is selected. `HAL_RTC_Init()` then times out on INITF, `Error_Handler()` is a no-op, and the device flies with a stopped RTC — no LoRaWAN timers, no uplinks.

**Verify.** On the bench, disable/short the LSE crystal and observe boot. Confirm `HAL_RTC_Init()` returns `HAL_TIMEOUT` and that `TIMER_IF_GetTimerValue()` does not advance.

**Fix.** Record the selected RTC source in a file-scope variable in `SystemClock_Config()` and have `HAL_RTC_MspInit()` honour it:

```c
/* main.c */
uint32_t g_rtc_clock_source = RCC_RTCCLKSOURCE_LSE;   /* set to LSI in the failover path */

/* stm32wlxx_hal_msp.c */
extern uint32_t g_rtc_clock_source;
PeriphClkInitStruct.RTCClockSelection = g_rtc_clock_source;
```

Also note: on a **cold** power-on, RTCSEL is `NONE` and therefore differs from LSE, so the backup reset fires on every cold boot. That is harmless (backup registers are already lost) but worth understanding when reasoning about which resets preserve breadcrumbs. Watchdog and software resets retain RTCSEL = LSE and take no backup reset — that is the case the breadcrumb design depends on.

---

## R09 — PA0 LED driven on every STOP2 exit, in flight

**Severity:** P1 · **File:** `Core/Src/stm32_lpm_if.c:122, 306`

**Claim.** A diagnostic LED is energised during every awake period of the flight, contradicting F25 / ADR-0008 ("LEDs are COMMISSIONING-only") and consuming a meaningful fraction of the power budget.

**Evidence.**

```c
/* PWR_EnterStopMode, line 122 */
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);   /* "DIAGNOSTIC: LED OFF while sleeping" */

/* PWR_ExitStopMode, line 306 */
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);     /* "DIAGNOSTIC: LED ON while awake" */
```

PA0 is confirmed as the LED by `main.c:110` ("Turn off the actual LED on PA0") and `leds_boot_seq()`.

The LED is therefore on for the whole GPS acquisition window (up to 60 s, and see R25 — likely ~30 s of cold-start every cycle), the whole TX cycle, and — because of R07 — for the wake portion of every 25 s chunk.

**Verify.** Measure current with the LED populated vs. depopulated over one full work cycle.

**Fix.**

```c
if (MissionState_IsCommissioning()) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}
```

Better: also drive PA0 low and configure it as analog in `PWR_EnterStopMode()` when not commissioning, so it cannot leak.

---

## R10 — Brownout floor tested against temperature-normalised voltage

**Severity:** P1 · **File:** `LoRaWAN/App/lora_app.c:807-852, 1136-1160`

**Claim.** The absolute low-voltage floor that gates `MODE_SURVIVAL` is defeated precisely in the cold conditions it exists for.

**Evidence.**

```c
/* line 1136 */
uint16_t battery_mv_normalized = NormalizeBatteryVoltage(battery_mv_raw, temperature_c);
/* line 1160 */
OperatingMode_t predicted_mode = SelectModeFromPredictions(slope_mv_per_hour,
                                                          battery_mv_normalized,
                                                          time_to_critical);
```

```c
/* SelectModeFromPredictions, line 815 */
// VOLTAGE-BASED FLOOR: Emergency low voltage (LTO threshold)
// ... causing brownout. Below the absolute floor = SURVIVAL, regardless of slope.
if (current_voltage < 4300) { return MODE_SURVIVAL; }
```

Brownout is a function of **terminal** voltage, not state of charge. `NormalizeBatteryVoltage()` adds up to +2700 mV at −66 °C. A real 3.6 V pack at −60 °C normalises to ~4.4 V and never trips the floor.

**Secondary problems in the same function:**

- The compensation table is **non-monotonic**: `{-40, 700}` then `{-50, 400}`. Normalised voltage therefore decreases as the pack gets colder across that span. The entries are annotated "Approximate"; the −50 value looks like a transcription error against the −55…−66 series.
- `NormalizeBatteryVoltage()` ignores `sensor_data.temp_stale`. A frozen temperature reading silently mis-compensates.
- `SelectModeFromPredictions()` has **no hysteresis**, so mode can oscillate between cycles. `config.h` declares `power_mode_hysteresis` (default 10) with no consumer — see R36.
- `slope < -5` → `MODE_REDUCED` (GPS off) regardless of state of charge. A full 5.4 V pack discharging at 6 mV/h (≈180 h of margin) turns off the GPS.

**Verify.** Table-test `SelectModeFromPredictions()` across the temperature/voltage grid and check that the SURVIVAL region matches the actual brownout envelope of the hardware.

**Fix.**
1. Pass **raw** voltage to the absolute floor and **normalised** voltage to the slope/prediction path. Change the signature to take both.
2. Fix or re-derive the −50 °C table entry; add a build-time monotonicity check over the table.
3. Gate normalisation on `!temp_stale`; when stale, use raw and treat the result conservatively.
4. Add hysteresis (wire up the existing config field), and add a state-of-charge term so slope alone cannot disable the GPS on a full pack.

---

## R25 — GPS cold-starts every cycle; the PCAS12 ephemeris rationale is unfounded

**Severity:** P1 · **File:** `Core/Src/atgm336h.c:1109-1210`

**Claim.** Time-to-fix will be cold-start (~30–35 s typical, worse at low signal), not the "<1–5 s hot start" the code assumes. This is likely the largest single item in the power budget.

**Evidence.** Three compounding problems:

```c
/* atgm336h.c:1116 */
/* CRITICAL: Send PCAS12 standby command FIRST (while UART active).
 * FW-8: PCAS12 is the standby-entry command — it is what makes the module
 * persist ephemeris to its internal flash ... */
GNSS_StatusTypeDef cmd_status = GNSS_SendCommand(hgnss, GNSS_CMD_STANDBY);
```

1. `$PCAS12,0*1C` has the wrong checksum (R23) → the module never receives it.
2. `PCAS12` is a standby/low-power entry command. It does not persist ephemeris. `PCAS00` saves *configuration*, not orbital data.
3. Ephemeris/almanac retention on this class of receiver requires the backup rail to remain powered. `GNSS_EnterStandby()` drives **both** rails low:
   ```c
   HAL_GPIO_WritePin(hgnss->pwr_port, hgnss->pwr_pin, GPIO_PIN_RESET);  // PB10 LOW
   HAL_GPIO_WritePin(hgnss->en_port,  hgnss->en_pin,  GPIO_PIN_RESET);  // PB5  LOW
   ```

Compounding: during acquisition `UTIL_LPM_SetStopMode((1 << CFG_LPM_GNSS_Id), UTIL_LPM_DISABLE)` blocks STOP2, so the MCU is held awake for the whole window.

The FW-8 comment already anticipates this: *"Bench gate pending: if TTF is not hot-start-fast, restoring PB10-HIGH standby is a separate measured fix."*

**Verify.** This is the highest-value bench measurement in the whole review:
- Log `ttf_ms` (already captured at `lora_app.c:1332`) over 20 consecutive cycles.
- Repeat with PB10 held HIGH and only PB5 cut.
- Measure the standby current in the PB10-HIGH configuration.

**Fix.** Almost certainly: keep PB10 (backup rail) high, cut PB5 only. Then:
1. Correct the comments in `GNSS_EnterStandby()`, `stm32_lpm_if.c:194-198`, and `lora_app.c:1243-1246` — remove all "ephemeris persisted to flash via PCAS12" claims.
2. Either fix `PCAS12`'s checksum and use it for real standby, or drop the command entirely if the GPIO power cut is the mechanism.
3. Re-tune `gps_timeout_*` against measured warm-start TTF.

---

## R26 — GSV enabled but its only consumer is compiled out

**Severity:** P1 · **File:** `Core/Inc/atgm336h.h:165`, `Core/Inc/payload_format.h:41-43`

**Claim.** Roughly two-thirds of the GPS UART bandwidth is spent on sentences nothing reads, which over-subscribes the 9600 baud link and lengthens time-to-fix.

**Evidence.**

```c
#define GNSS_CMD_NMEA_CONFIG "$PCAS03,1,0,0,1,1,1,0,0*02"  // GGA + RMC + GSV + VTG
```

`PCAS03` field order is `nGGA,nGLL,nGSA,nGSV,nRMC,nVTG,nZDA,nANT` → GGA=1, GSV=1, RMC=1, VTG=1.

The only GSV consumer is `EncodeGNSSDetailPacket()` (`lora_app.c:576`), gated by:

```c
#ifndef ENABLE_GNSS_DETAIL_PACKET
#define ENABLE_GNSS_DETAIL_PACKET  0  // 0 = Default OFF for flight builds
#endif
```

Bandwidth: 9600 baud ≈ 960 B/s. Multi-constellation GSV at 1 Hz can exceed that on its own (15+ sentences × ~70 B ≈ 1050 B/s), before GGA/RMC/VTG. The link is saturated, which also drives R27.

**Verify.** Capture the raw NMEA stream for 10 s and measure bytes/s and the GSV fraction.

**Fix.** Set `nGSV=0` in the flight `PCAS03` mask (keep a commissioning variant with GSV on if the detail packet is wanted for ground testing). Consider raising the baud rate via `PCAS01` as well. Note the RTT string at `atgm336h.c:218` says "NMEA config (GGA+RMC only)" — already inconsistent with the mask.

---

## R27 — No DMA overrun detection

**Severity:** P1 · **File:** `Core/Src/atgm336h.c:510-604`

**Claim.** If the software read pointer falls more than 512 bytes behind the DMA, data is silently overwritten and the parser processes a spliced stream with no indication.

**Evidence.**

```c
uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(hgnss->huart->hdmarx);
hgnss->dma_head = GNSS_DMA_BUFFER_SIZE - dma_remaining;
...
while (hgnss->dma_tail != hgnss->dma_head) { ... }
```

There is no comparison against the half/complete callback counts, so a wrap past `dma_tail` is undetectable. `GNSS_DMA_BUFFER_SIZE` is 512; at 9600 baud that is 533 ms of buffering, and R26 shows the stream is already at or over capacity.

The NMEA checksum catches most corruption, but silently — `GNSS_ParseNMEA` logs "Checksum FAILED" and returns, with no counter and nothing in telemetry.

**Verify.** Add a counter incremented in `GNSS_DMA_RxCpltCallback`; compare the number of buffer wraps against the number of `GNSS_ProcessDMABuffer` passes.

**Fix.** Track wraps in the callbacks and detect the overrun condition explicitly. At minimum log and count it; ideally surface a bit in the status byte, consistent with ADR-0007.

---

## R28 — MS5607 returns OK on zero/implausible ADC values

**Severity:** P1 · **File:** `Core/Src/ms5607.c`

**Claim.** A failed or partial I²C read produces a wildly wrong pressure that is returned as `MS5607_OK`, cached as *last-known-good*, and marked **not stale** — defeating the FW-7 staleness mechanism at exactly the failure mode it was built for.

**Evidence.**

```c
if (d1 == 0 || d2 == 0 || d1 < 1000 || d2 < 1000) {
    SEGGER_RTT_WriteString(0, "MS5607: Warning - ADC values low\r\n");
    /* Continue anyway and see if we get reasonable values */
}
...
p = ((((int64_t)d1 * sens) >> 21) - off) >> 15;
*pressure = (float)p / 100.0f;
return MS5607_OK;
```

With `d1 == 0`, `p = (-off) >> 15` — a large negative pressure. `sys_sensors.c:203-208` then does:

```c
if (MS5607_ReadPressureAndTemperature(&hms5607, &ms_temp, &ms_press) == MS5607_OK) {
    PRESSURE_Value = ms_press;
    s_last_press = PRESSURE_Value;    /* poisoned cache */
    s_have_press = true;
    press_stale = false;              /* marked FRESH */
}
```

Same function also proceeds when `!IsInitialized`:

```c
if (!hms5607->IsInitialized) {
    SEGGER_RTT_WriteString(0, "MS5607: Sensor not initialized\r\n");
}
/* falls through and computes with whatever CalData survived a failed read */
```

`MS5607_Init()` does correctly validate the PROM CRC4 and the 0/0xFFFF coefficient cases (`ms5607.c:85-129`) — the read path just ignores the result.

**Verify.** Force `d1 = 0` (disconnect SDA mid-read or stub `MS5607_ReadADC`) and observe the reported pressure and the `press_stale` bit.

**Fix.**
1. Return `MS5607_ERROR` when `!IsInitialized`.
2. Return `MS5607_ERROR` on the implausible-ADC branch instead of continuing.
3. Add a plausibility gate before accepting into the cache: e.g. `1.0f <= P <= 1200.0f` and `-90.0f <= T <= 85.0f`. Anything outside → stale, keep the previous cached value.

Apply the same pattern to SHT31 (its CRC-8 checks are already correct — verified at `sht31.c:205-214` — but there is no value plausibility gate).

---

## R29 — W25Q Block Protect bits never checked; writes can fail silently

**Severity:** P1 · **File:** `Core/Src/w25q16jv.c`

**Claim.** If the flash's Block Protect bits are set, every program and erase silently does nothing while the driver reports success — and the flash logging subsystem would report healthy operation while storing nothing.

**Evidence.** `W25Q_Init()` reads and verifies the JEDEC ID but never reads Status Register 1's protection bits (`BP0`/`BP1`/`BP2`/`SEC`/`TB`, mask `0x7C`) or SR2's `SRL`. With BP set:

- `W25Q_WriteEnable()` sets WEL, which succeeds.
- `W25Q_PageProgram()` issues the command; the device ignores it and never asserts `BUSY`.
- `W25Q_WaitReady()` sees `BUSY == 0` immediately and returns `W25Q_OK`.
- No read-back verification exists anywhere in the driver.

Downstream, `FlashLog_Init()` would find no valid headers → clean init; `FlashLog_FrontierScan` would find nothing; `FlashLog_WriteRecord` would return `FLASH_LOG_OK` forever.

BP bits can be set by an ESD event, a brownout during a status-register write, or a part that shipped protected.

**Also:** most `W25Q_*` functions check `hw25q == NULL` but not `hw25q->initialized`. `main.c:206-230` calls `Error_Handler()` (a no-op since P0-4) on W25Q init failure and then proceeds to `FlashLog_Init()`, driving SPI against an unconfigured handle.

**Verify.** Set BP bits deliberately via `W25Q_CMD_WRITE_STATUS` and confirm that `FlashLog_WriteRecord` still returns `FLASH_LOG_OK`.

**Fix.**
1. In `W25Q_Init()`: read SR1; if `(sr1 & 0x7C) != 0`, either clear it (Write Enable + Write Status Register) or return a distinct error. Log the raw SR1/SR2 either way.
2. Add a post-init self-test: write a known record, read it back, verify. Report the result at boot. Given that flash logging is load-bearing for the science mission, this is cheap insurance.
3. Add `if (!hw25q->initialized) return W25Q_ERROR_INIT;` to the public entry points.

---

## R30 — Unbounded join wait makes commissioning unfinishable

**Severity:** P1 · **File:** `Core/Src/multiregion_context.c:1024-1054, 1078-1169`

**Claim.** `MultiRegion_PreJoinAllRegions()` cannot complete outside a lab with gateways for all four regions, so `MissionState_EnterFlight()` is never reached and the unit never leaves COMMISSIONING.

**Evidence.**

```c
/* comment at 1014 acknowledges this by design */
/* This wait loop has no timeout by design ("infinite retry until success") */
while (!g_multiregion_join_success) {
    LmHandlerProcess();
    if ((HAL_GetTick() - last_join_attempt) > retry_interval) {
        LmHandlerJoin(ACTIVATION_TYPE_OTAA, true);
        last_join_attempt = HAL_GetTick();
    }
    if (hiwdg.Instance != NULL) { HAL_IWDG_Refresh(&hiwdg); }
    HAL_Delay(250);
}
```

The IWDG is refreshed, so the loop spins forever. `MultiRegion_PreJoinAllRegions()` calls it sequentially for US915 → EU868 → AS923 → AU915. At any single physical location, at most one of those will produce a join accept.

`MultiRegion_InitializeRegionFromChirpstack()` (`:1174`) appears to be the intended manual provisioning path for the remaining regions, but **nothing calls it** — `LoRaWAN_Init()` only invokes `PreJoinAllRegions()`.

`all_success` is returned but the caller discards it.

**Verify.** Run commissioning with only a US915 gateway present and observe that the sequence never advances past EU868.

**Fix.**
1. Bound the wait: e.g. 3 join attempts or ~120 s, whichever comes first.
2. On timeout, log, mark the region unprovisioned, and **continue to the next region**.
3. Call `MissionState_EnterFlight()` if at least one bank is valid; refuse and stay in COMMISSIONING if none are.
4. Wire up a documented path (RTT command, or a build-time table) that calls `MultiRegion_InitializeRegionFromChirpstack()` for regions that cannot be joined over the air. Document it in `docs/MultiRegionImplementationGuide.md`.
5. Have `LoRaWAN_Init()` act on the return value.

---

# P2 findings

## R11 — Restricted-region early return skips the flash log write

**File:** `LoRaWAN/App/lora_app.c:1449-1453` vs `:1520-1533`

```c
if (h3_region_id == REGION_RESTRICTED) {
    SEGGER_RTT_WriteString(0, "RESTRICTED REGION DETECTED: Skipping transmission for safety\r\n");
    return;                       /* ← returns before the flash log write at :1526 */
}
```

The archive exists precisely to hold data that cannot be transmitted. Over a restricted region the sample is discarded entirely.

**Fix.** Use the existing `rf_silence` pattern (`:1228`, `:1624`) instead of an early return: set a flag, let GPS + sensor re-read + flash write proceed, then skip only the TX state machine.

---

## R12 — Header ping-pong selection uses `record_count` as the sequence

**File:** `Core/Src/flash_log.c:259, 351-365`

```c
header.sequence = hlog->record_count;   /* Use record count as sequence */
```

The field is documented as *"Header update sequence (for ping-pong selection)"*. `FlashLog_MarkRecordsTransmitted()` → `FlashLog_SyncHeader()` writes a header **without** `record_count` changing, so consecutive headers can carry identical sequences. `FlashLog_Init()` uses `>`:

```c
if (header_a.sequence > header_b.sequence) { /* pick A */ } else { /* pick B */ }
```

On a tie it picks A regardless of which is newer, losing up to one batch of `last_transmitted_seq` progress (causing duplicate retransmission).

**Fix.** Use a dedicated monotonic header-write counter, incremented on every `FlashLog_WriteHeader()` call, independent of `record_count`.

---

## R13 — Unbounded corrupt-record scan without watchdog refresh

**File:** `Core/Src/flash_log.c:159-174`

```c
while ((*actual_count) < max_count && sequence_to_read < hlog->next_sequence) {
    status = FlashLog_ReadRecord(hlog, &records[*actual_count], offset);
    if (status != FLASH_LOG_OK) {
        hlog->last_transmitted_sequence = sequence_to_read + 1;
        sequence_to_read++;
        continue;                       /* no iteration bound */
    }
    ...
}
```

A long run of corrupt records causes up to ~32,600 SPI reads in one call with no IWDG refresh.

**Fix.** Add an iteration cap (e.g. 256 probes per call) and return what was gathered; the caller already handles `actual_count == 0`. Optionally refresh the IWDG inside the loop.

---

## R14 — `FlashLog_EraseAll` would trip the watchdog and leaves state inconsistent

**File:** `Core/Src/flash_log.c:665-692`

512 sector erases at up to 400 ms each, with no IWDG refresh, against a 32.76 s timeout. It also resets `next_sequence = 0` without resetting `last_transmitted_sequence`, which would leave `FlashLog_HasUnsentData()` permanently false and disable bulk transfer forever.

Currently uncalled (grep confirms no callers outside `flash_log.c`).

**Fix.** Refresh the IWDG per sector, reset `last_transmitted_sequence = 0`, and either wire it to a commissioning-only command or delete it.

---

## R15 — Bulk packet always transmits 198 bytes

**Files:** `Core/Src/payload_encode.c:226-256`, `LoRaWAN/App/lora_app.c:1732-1742`

```c
bulkData.BufferSize = sizeof(BulkTelemetryPacket_t);   /* always 198 */
```

`EncodeBulkPacketFromRecords` zero-fills unused record slots (honest, not fabricated — `record_count` is in the header), but a single-record packet still burns full 198 B airtime: ~333 ms at SF7/125 kHz vs ~102 ms for 38 B. Matters for EU868 duty cycle and for energy.

**Fix.** Serialise variable-length: `2 + 32*record_count + 4`, with the CRC32 relocated to immediately follow the last record. Update the ground decoder and `docs/PayloadFormats.md` in the same change.

---

## R16 — `HAL_MAX_DELAY` in the flight path; ~16 ADC init cycles per work cycle

**File:** `Core/Src/adc_if.c:245-289`

```c
HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
```

Unbounded wait. A transient ADC stall becomes a watchdog reset — contrary to ADR-0001.

Separately, every channel read performs `MX_ADC_Init()` + `HAL_ADCEx_Calibration_Start()` + configure + start + poll + stop + `HAL_ADC_DeInit()`. `SYS_GetBatteryVoltage()` internally calls `SYS_GetBatteryLevel()`, so one battery read is **two** full cycles. Per `SendTxData`: two `EnvSensors_Read()` calls (5 each) + two direct reads (4) + `OnRxData` (2) ≈ 16.

Minor precision note: `((measuredLevel * vdda_mv / 4096) * 2)` doubles the truncation error; prefer `(measuredLevel * vdda_mv * 2) / 4096`.

**Fix.** Bounded timeout (e.g. 10 ms). Initialise and calibrate the ADC once, cache VDDA per work cycle instead of re-reading VREFINT for every channel.

---

## R17 — VREFBUF is not VREFINT; 10 ms busy-wait per wake

**File:** `Core/Src/stm32_lpm_if.c:188, 340`

```c
HAL_SYSCFG_DisableVREFBUF();   /* comment: "Disable VREFINT ... ~10-20µA" */
...
HAL_SYSCFG_EnableVREFBUF();    /* FW-15 */
```

VREFBUF is the **external** reference buffer (VREF+ pin). VREFINT is enabled automatically by the ADC HAL when `ADC_CHANNEL_VREFINT` is selected. The FW-15 rationale — *"the ratiometric VDDA path in SYS_GetBatteryLevel reads VREFINT after every STOP2 wake; leaving it disabled here silently degrades battery reads"* — rests on a misidentification.

`HAL_SYSCFG_EnableVREFBUF()` busy-waits up to `VREFBUF_TIMEOUT_VALUE` for the VRR bit and returns `HAL_TIMEOUT` (ignored) if VREFBUF is not usable on this package. Combined with R07 that is potentially 10 ms of full-clock spinning ×12 per work cycle.

**Fix.** Remove both calls. If VREFINT quiescent current genuinely matters, control it through the ADC common register (`ADC->CCR` `VREFEN`) — but the HAL already manages it around conversions.

**Verify first:** confirm whether STM32WLE5JC in your package exposes VREF+ at all.

---

## R18 — Solar ADC has no divider and saturates at VDDA

**File:** `Core/Src/adc_if.c:217-242`

```c
/* @note  PB3 has no voltage divider, no scaling applied */
solarVoltagemV = (uint16_t)((uint32_t)measuredLevel * vdda_mv / 4096);
```

For a 5.5 V LTO charging system, panel voltage above VDDA (~3.3 V) both clips the reading and exceeds the pin's absolute-maximum input rating (VDDA + 0.3 V), forcing current through the ESD structures.

**Verify against the board.** If a divider is present, the comment and the scaling are both wrong and solar telemetry is under-reported by the divider ratio. If no divider is present, solar telemetry is a flat line at ~3300 mV whenever the panel is actually producing — and the input is out of spec.

**Fix.** Determine which case applies, then either correct the scaling or add a divider. Either way, `sensor_data.solar_voltage` and Cayenne channel 10 currently cannot be trusted.

---

## R19 — `ConvertFlashLogToHighRes` cannot fail; stamps history with present metadata

**File:** `Core/Src/payload_encode.c:261-306`

The function only returns `false` on NULL arguments, so the F10 "skip unconvertible records" path (`lora_app.c:1709-1715`) is unreachable dead code. Record validation actually happens upstream in `FlashLog_ReadRecord()` → `FlashLog_VerifyRecord()`, which is correct — but the F10 comment misattributes where the protection lives.

More substantively:

```c
highres_record->voltage_slope = voltage_slope;   /* current cycle's value */
highres_record->power_mode    = (uint8_t)power_mode;
highres_record->solar_voltage = 0;               /* "Not stored in FlashLog_Record_t yet" */
```

Historical records in a bulk packet carry **today's** slope and mode, and a permanently-zero solar voltage — the same always-zero-placeholder pattern FW-20 removed from the bulk packet.

**Fix.** Either add `voltage_slope`, `power_mode` and `solar_mv` to `FlashLog_Record_t` (it has 14 reserved bytes) so history is self-describing, or remove the fields from `HighResTelemetryRecord_t` rather than transmitting misleading values. Update the F10 comment to point at `FlashLog_VerifyRecord()`.

---

## R20 — No-fix records store ST demo coordinates

**File:** `Core/Src/sys_sensors.c:288-301`, `Core/Src/flash_log.c:500-501`

```c
sensor_data->latitude  = (int32_t)((STSOP_LATTITUDE * MAX_GPS_POS) / 90);
sensor_data->longitude = (int32_t)((STSOP_LONGITUDE * MAX_GPS_POS) / 180);
sensor_data->gnss_valid = false;
```

`STSOP_*` are ST's demo coordinates (Saint-Ouen, France). `FlashLog_WriteRecord` stores latitude/longitude **unconditionally**, and `ConvertFlashLogToHighRes` copies them into the bulk packet. Only the flags byte marks them invalid, so any decoder that does not check the flag sees the balloon in France.

Also in the same branch: `sensor_data->satellites = hgnss.data.satellites_in_view` — the field means "satellites used" when there is a fix and "satellites in view" when there is not.

**Fix.** Store zeros (or the last-known-good position, consistent with the F8 stale-flag design) rather than demo coordinates, and keep `satellites` semantically constant.

---

## R21 — Bulk-transfer livelock on unconvertible records

**File:** `LoRaWAN/App/lora_app.c:1717-1721`

```c
if (packed_count == 0) {
    SEGGER_RTT_WriteString(0, "No records convertible - skipping bulk packet\r\n");
    g_tx_state = TX_STATE_COMPLETE;
    break;                      /* nothing marked → watermark never advances */
}
```

`FlashLog_HasUnsentData()` stays true, so every subsequent bulk trigger re-reads the same records.

Note this interacts with `FlashLog_GetUnsentRecordsFIFO`, which already advances the watermark past records that fail *CRC*. The gap is records that read fine but fail *conversion* — currently impossible (R19), so this is latent rather than active. Fix both together.

---

## R22 — "Compile-time" size validation is runtime and non-fatal

**File:** `Core/Src/payload_encode.c:332-360`, `Core/Src/main.c:232-237`

```c
if (!PayloadFormat_ValidateSizes()) {
    SEGGER_RTT_WriteString(0, "ERROR: Payload format size validation failed!\r\n");
    Error_Handler();            /* no-op since P0-4 */
}
/* falls through and flies with mismatched structs */
```

`multiregion_context.c:100-102` and `config.h:135` already use `_Static_assert` correctly — the same pattern applies here.

**Fix.**

```c
_Static_assert(sizeof(CompactTelemetryPacket_t) == 11, "compact packet size");
_Static_assert(sizeof(HighResTelemetryRecord_t) == 32, "highres record size");
_Static_assert(sizeof(BulkTelemetryPacket_t)   == 198, "bulk packet size");
```

Also note the printf uses `%d` for `sizeof` results; should be `%u` or a cast.

---

## R31 — GPS state invalidation before acquisition is incomplete

**File:** `LoRaWAN/App/lora_app.c:1308-1312`

```c
hgnss.data.valid = false;
hgnss.data.fix_quality = GNSS_FIX_INVALID;
```

`satellites`, `hdop`, `latitude` and `longitude` retain the previous cycle's values. `GNSS_IsFixGoodQuality()` reads all of them:

```c
return (hgnss->data.valid && hgnss->data.fix_quality != GNSS_FIX_INVALID &&
        hgnss->data.satellites >= 4 && hgnss->data.hdop <= 5.0f &&
        GNSS_ValidateCoordinates(...));
```

The GGA parser skips empty tokens (`if (strlen(token) > 0)`), so a GGA with an empty satellite or HDOP field lets stale values satisfy the gate.

**Fix.** `memset(&hgnss.data, 0, sizeof(hgnss.data))` before acquisition. The last-known-good position lives in separate statics (`last_valid_lat/lon/alt`), so nothing is lost.

---

## R32 — `lat_raw > 0` skips the prime meridian and equator

**File:** `Core/Src/atgm336h.c:1027-1038`

```c
if (lat_raw > 0) { hgnss->data.latitude = GNSS_ConvertToDecimalDegrees(lat_raw); ... }
if (lon_raw > 0) { ... }
```

NMEA raw coordinates are always non-negative (hemisphere is a separate field), so this is really a non-zero test. A circumnavigating balloon crossing the prime meridian silently retains the previous longitude.

**Fix.** Track a `bool have_lat / have_lon` set when the token is non-empty, and gate on that instead of magnitude.

---

## R33 — Checksum verification passes when `*` is absent

**File:** `Core/Src/atgm336h.c:1052-1058`

```c
const char *checksum_ptr = strchr(sentence, '*');
if (checksum_ptr == NULL) { return true; }   /* "No checksum present - assume valid" */
```

A sentence truncated by a DMA wrap loses its `*XX` tail and is then accepted and parsed.

**Fix.** Return `false` when no checksum is present. All ATGM336H NMEA output carries one.

---

## R34 — Float coordinate math; documented precision overstated

**File:** `Core/Src/atgm336h.c:853-859`, `Core/Inc/flash_log.h:107-108`

`GNSS_ConvertToDecimalDegrees` works in `float` and `lat_raw` is `float`, giving roughly 1 m quantisation. `FlashLog_Record_t` documents `latitude` as *"1e-7 degree resolution"* (~1 cm).

**Fix.** Either use `double` for the conversion (cheap here — it is not in a hot loop) or correct the documented resolution.

---

## R35 — Config CRC excludes the wrong four bytes

**File:** `Core/Src/config.c` (`Config_Validate`, `Config_CalculateCRC32`), `Core/Inc/config.h:67-73`

Both functions do:

```c
SystemConfig_t temp_config = *config;
temp_config.crc32 = 0;
uint32_t calculated_crc = Config_CRC32((const uint8_t*)&temp_config,
                                       sizeof(SystemConfig_t) - sizeof(uint32_t));
```

But `crc32` is at **offset 8**, not the end:

```c
uint32_t magic;    /* 0 */
uint16_t version;  /* 4 */
uint16_t size;     /* 6 */
uint32_t crc32;    /* 8  ← not last */
...
uint8_t reserved[16];   /* last */
```

Zeroing the field *and* truncating by 4 excludes `reserved[12..15]`, not the CRC. Both sides agree so validation passes today, but the last four bytes are unprotected and the comment (*"calculate excluding CRC field"*) does not describe the behaviour. This becomes a real bug the moment the struct is extended or a field is moved into the tail.

**Fix.** Since `crc32` is already zeroed in the copy, hash the full struct:

```c
Config_CRC32((const uint8_t*)&temp_config, sizeof(SystemConfig_t));
```

Bump `CONFIG_VERSION` — existing stored configs will fail validation and fall back to defaults, which is acceptable pre-launch.

---

## R36 — 24 of 32 configuration fields have no consumers

**File:** `Core/Inc/config.h:67-133`

Grepped each field outside `config.c`:

**Live (8):** `tx_interval_normal`, `tx_interval_conservative`, `tx_interval_reduced`, `tx_interval_recovery`, `tx_interval_survival`, `gps_timeout_normal`, `gps_timeout_conservative`, `gps_temperature_lockout`.

**Dead (24):** `lorawan_datarate`, `lorawan_txpower`, `lorawan_adr_enabled`, `lorawan_confirmed`, `lorawan_class_b_timeout`, `battery_low_threshold`, `battery_critical_threshold`, `bulk_battery_min_mv`, `power_mode_hysteresis`, `gps_min_satellites`, `gps_max_hdop_x10`, `gps_standby_power_ua`, `link_margin_threshold`, `gateway_count_threshold`, `max_bulk_packets`, `frame_counter_save_interval`, `bulk_timeout_ms`, `flash_log_interval`, `flash_log_enabled`, `flash_log_compression`, `flash_log_retention_days`, `debug_lpp_enabled`, `debug_gnss_detail_enabled`, `debug_lpp_interval`, `debug_rtt_level`, `debug_flags`.

Several dead fields **contradict** the hardcoded values actually in use:

| Config field (default) | Hardcoded value actually used | Where |
|---|---|---|
| `gps_max_hdop_x10 = 25` (HDOP 2.5) | `hdop <= 5.0f` | `atgm336h.c:446` |
| `battery_critical_threshold = 4000` | `4300` | `lora_app.c:815` |
| `battery_low_threshold = 4500` | `4500` (as a prediction target, not a threshold) | `lora_app.c:1146` |
| `power_mode_hysteresis = 10` | no hysteresis exists | `lora_app.c:807` |
| `gps_standby_power_ua = 15` | standby mode removed by FW-8 | — |
| `debug_lpp_enabled = 1` | `ENABLE_DEBUG_LPP` defaults to `0` | `payload_format.h:38` |

This is F19 generalised: a knob that reads plausibly and does nothing is worse than no knob, because it will be trusted during a launch review.

**Fix.** For each dead field, either wire it up or delete it (keeping reserved padding to preserve the flash layout, as F19 did). Prioritise the six contradictory ones — they are actively misleading.

---

## R37 — Hardcoded NVM address literals

**File:** `Core/Src/multiregion_context.c:938, 1099`

```c
FLASH_IF_Erase((void*)0x0803F000UL, 2048);  // LORAWAN_NVM_BASE_ADDRESS
```

The symbol exists (`lora_app.c:125`). If the NVM page moves, these erase the wrong page.

**Fix.** Move `LORAWAN_NVM_BASE_ADDRESS` and the page size to a shared header and use the symbol at both sites.

*(Note: the overall flash map is correct — the linker reserves 16 KB, `FLASH LENGTH = 240K` ends exactly at `0x0803C000`, and Tier-1/Tier-2/config/NVM pages 120–127 do not overlap. Verified.)*

---

## R38 — Sensors operated outside characterised range with no flag

**Files:** `Core/Src/ms5607.c`, `Core/Src/sht31.c`

MS5607 is specified −40 to +85 °C; SHT31 is −40 to +125 °C, and RH readings below −40 °C are essentially meaningless. Float altitude puts the payload at −55 to −80 °C. The MS5607 second-order compensation polynomial (correctly implemented, including the `temp < -1500` branch) is not characterised below −40 °C.

**Fix.** Consistent with ADR-0007, add `press_uncharacterised` / `hum_uncharacterised` bits set when `T < -40 °C`, carried in the flash record flags byte (it has spare bits) and, if space allows, the uplink status byte. This matters for publication — reviewers will ask.

---

## R39 — `W25Q_WaitReady` busy-spins at full clock

**File:** `Core/Src/w25q16jv.c:217-243`

```c
HAL_Delay(1);   /* → TIMER_IF_DelayMs → while(...) { __NOP(); } at 32 MHz */
```

During a 45–400 ms sector erase — which happens every 64 records plus twice per header write, all inside the TX cycle — the CPU spins at full MSI.

**Fix.** `__WFI()` between status polls, or drop the delay and poll directly (SPI transaction overhead already rate-limits).

**Related note on units:** `HAL_GetTick()` is overridden in `sys_app.c:321` to return `TIMER_IF_GetTimerValue()`, which is RTC ticks at **1024 Hz**, not milliseconds. Every `timeout_ms` in the codebase is therefore ~2.4 % short in real time, and every `HAL_GetTick()` delta labelled "ms" is really ticks. Margins are adequate everywhere I checked (e.g. `W25Q_TIMEOUT_SECTOR_ERASE = 500` → 488 real ms vs. a 400 ms datasheet max). Worth documenting once in a header comment rather than fixing at 40 call sites — but be aware of it when tightening any timeout.

---

# Documentation findings

## R40 — The published ground-station decoder rejects every production uplink

**File:** `docs/PayloadFormats.md:15, 32-34, 124-133`

The document specifies Port 10 as **10 bytes**, and its reference Python decoder does:

```python
raise ValueError(f"Expected 10 bytes, got {len(payload)}")
```

The firmware sends 11 (`payload_format.h:92-101`; `PayloadFormat_ValidateSizes` asserts 11). Anyone building against the published spec gets an exception on every packet.

The status byte added by F17 (byte 10 — stale bits, reset cause, mission state) is not documented anywhere in this file.

Related stale claims: `docs/FirmwareArchitecture.md:217`, `docs/FlashLogging.md:122` say 11 bytes; `docs/PayloadFormats.md` says 10; `payload_format.h:9` and the `EncodeCompactBinaryPacket` doc-comment say 10; `lora_app.c:1607` says 10.

---

## R41 — The document specifies a different coordinate encoding than the code

**File:** `docs/PayloadFormats.md:39-40, 56-73`

The document still describes `lat_100m` / `lon_100m` in 100 m units with ±3276.7 km range. `payload_encode.c:32-33` uses full-range int16 scaling per ADR-0005:

```c
#define LAT_SCALE_FACTOR  (32767.0f / 90.0f)    /* lat_deg = encoded × 90 / 32767  */
#define LON_SCALE_FACTOR  (32767.0f / 180.0f)   /* lon_deg = encoded × 180 / 32767 */
```

A decoder following the document does not produce slightly-wrong positions — it produces a different coordinate system entirely.

The struct field names and comments in `payload_format.h:94-95` are also stale: `latitude_100m` / *"Latitude in 100m resolution ... ±3276.7 km"*. Actual resolution is ~305 m in latitude and ~611 m in longitude at the equator.

`docs/FirmwareArchitecture.md:320` splits the difference badly: *"11-byte packet format... (100m resolution for coordinates)"*.

**Fix.** Rename the struct fields (e.g. `latitude_i16`), correct the comments, and regenerate the document's byte table and decoder from the struct.

---

## R42 — The document bakes in the pressure-floor bug

**File:** `docs/PayloadFormats.md:42, 89-95, 145, 161`

```
| 7 | Pressure | uint8 | 1 | 10 hPa | 950-3500 hPa | 950 + (value × 10) |
...
0x00,        # Pressure: 950+(0×10) = 950 hPa
```

So the clamped-to-zero stratospheric readings (R05) decode as **950 hPa** — the ground station reports the balloon at roughly sea level for the entire flight rather than flagging "below encodable range".

**Fix.** Regenerate after R05 is resolved. Whatever encoding is chosen, reserve a sentinel for out-of-range and document it.

---

## R43 — ADR-0005's central premise is factually wrong

**File:** `docs/adr/0005-worst-case-payload-sizing.md`

> **"LinkCheckReq:** Rides FOpts header (1 byte), separate from app payload budget"

FOpts is **not** separate. Semtech `LoRaMac.c` `ValidatePayloadLength()` computes `payloadSize = lenN + fOptsLen` and compares that against `PHY_MAX_PAYLOAD`. In the LoRaWAN specification the regional `MaxMACPayloadSize` covers the whole MACPayload including FHDR/FOpts; the published *N* values assume `FOptsLen == 0`, and every FOpts byte reduces the available application payload by one.

F17 restored the status byte on this reasoning, producing R04.

The same ADR contains an unfinished section:

> **Verification Required:** Confirm the worst-case floor holds for every enabled region (US915, EU868, AS923, AU915).

That incomplete verification is what allowed R03 (AU915 datarate mapping) through — `docs/FixWorkorderPlan.md:30` explicitly instructed *"verify map against in-tree `Region*.c`"*.

**Fix.** Amend or supersede ADR-0005 with the correct FOpts accounting, complete the four-region verification table (see *Appendix B*), and record the outcome.

---

## R44 — Assorted stale claims

| File | Claim | Reality |
|---|---|---|
| `docs/adr/0005` | Bulk packet is 222 B | 198 B since FW-20 |
| `docs/adr/0005` | "Bulk packet (SF7/DR3)" | DR3 is SF7 only in US915 |
| `docs/PayloadFormats.md:207` | Bulk sent at "SF7 (DR2)" | DR2 is SF8 in US915, SF10 in EU868 |
| `docs/ProductionReadinessAssessment.md` F3 row | LSE failover **FIXED** | Undone by `HAL_RTC_MspInit` (R08) |
| `docs/ProductionReadinessAssessment.md` F13 row | Deadman **FIXED**, cites `RTC_BKP_DR2` | That register is the collision (R01) |
| `docs/ProductionReadinessAssessment.md` F16 row | Region DR resolution **FIXED** | AU915 still wrong (R03) |
| `LoRaWAN/App/lora_app.c:125` | *"last 2 sector of a 128kBytes device"* | 256 KB part, one 2 KB page |
| `LoRaWAN/App/lora_app.c:1673-1678` | *"fall through to PROBE_SF10"* | Has a `break;` |
| `Core/Src/stm32_lpm_if.c:148 vs :152` | PB15 = I2C2_SCL / PB15 = SPI2_MOSI | Contradictory; resolve against the `.ioc` |
| `Core/Src/atgm336h.c:218` | *"NMEA config (GGA+RMC only)"* | Mask enables GGA+GSV+RMC+VTG |
| `Core/Src/atgm336h.c:1293` | *"40 second polling loop"* | `gps_timeout_ms` is 60 s / config-driven |
| `LoRaWAN/App/lora_app.c:450` | *"Sends PCAS04,7 + PCAS11 airborne + PCAS00"* | Sends seven commands; PCAS04,7 is rejected |
| `Core/Src/multiregion_context.c:409-411` | *"The live MAC counter is NOT modified — only the flash copy is advanced"* | `ctx` is the in-RAM copy; `SwitchToRegion` writes it into `nvm->Crypto.FCntList.FCntUp`. Forward-only so safe, but the comment is wrong. |
| `Core/Src/main.c:198-202` | Prints "H3Lite initialized successfully" | Prints it even when `h3liteInit()` failed, since `Error_Handler()` returns |
| `Core/Inc/payload_format.h:96` | `temperature_2deg` documented range −64…+63 °C | Declared `int8_t` but clamped 0–255 with a +64 offset (`uint8_t` semantics). Real usable range differs; the ground decoder's signedness is ambiguous. Declare as `uint8_t` and correct the range. Matters — tropopause temperatures reach −80 °C. |

---

# Appendix A — NMEA checksum verification script

```python
#!/usr/bin/env python3
"""Verify every hardcoded $PCAS* checksum in Core/Inc/atgm336h.h"""
cmds = [
    ("GNSS_CMD_NMEA_CONFIG",   "PCAS03,1,0,0,1,1,1,0,0", "02"),
    ("GNSS_CMD_CONSTELLATION", "PCAS04,5",               "1C"),
    ("GNSS_CMD_AIRBORNE_MODE", "PCAS11,5",               "18"),
    ("GNSS_CMD_UPDATE_RATE",   "PCAS02,1000",            "2B"),
    ("GNSS_CMD_SATELLITE_SYS", "PCAS04,7",               "1A"),
    ("GNSS_CMD_FIX_MODE",      "PCAS11,2",               "1E"),
    ("GNSS_CMD_SAVE_CONFIG",   "PCAS00",                 "01"),
    ("GNSS_CMD_STANDBY",       "PCAS12,0",               "1C"),
]
for name, body, claimed in cmds:
    c = 0
    for ch in body:
        c ^= ord(ch)
    ok = "OK      " if f"{c:02X}" == claimed.upper() else "MISMATCH"
    print(f"{ok} {name:24s} ${body}*{claimed}  computed={c:02X}")
```

Expected output at the reviewed commit: `PCAS02,1000`, `PCAS04,7`, `PCAS11,2`, `PCAS12,0` all MISMATCH.

**Suggested permanent guard:** add this as a pre-commit hook or a unit test so hand-edited sentences cannot regress.

---

# Appendix B — Region datarate / payload verification table

Extracted from the in-tree region headers. Any SF↔DR resolver must reproduce this.

**`Datarates<Region>[]` — DR index → spreading factor**

| DR | US915 | AU915 | EU868 | AS923 |
|---|---|---|---|---|
| 0 | SF10 | SF12 | SF12 | SF12 |
| 1 | SF9 | SF11 | SF11 | SF11 |
| 2 | SF8 | SF10 | SF10 | SF10 |
| 3 | **SF7** | SF9 | SF9 | SF9 |
| 4 | SF8/500 | SF8 | SF8 | SF8 |
| 5 | — | **SF7** | **SF7** | **SF7** |

**`MaxPayloadOfDatarate*[]` — max application payload (bytes, FOptsLen = 0)**

| DR | US915 | AU915 dwell0 | AU915 dwell1 | EU868 | AS923 dwell0 | AS923 dwell1 |
|---|---|---|---|---|---|---|
| 0 | **11** | 51 | 0 | 51 | 51 | 0 |
| 1 | 53 | 51 | 0 | 51 | 51 | 0 |
| 2 | 125 | 51 | 11 | 51 | 51 | 11 |
| 3 | 242 | **115** | **53** | 115 | 115 | 53 |
| 4 | 242 | 242 | 125 | 242 | 242 | 125 |
| 5 | — | 242 | 242 | 242 | 242 | 242 |

**Required checks for any payload change:**

1. `sizeof(CompactTelemetryPacket_t) + worst_case_FOptsLen <= MaxPayload[probe DR]` for **all four** regions.
2. `sizeof(BulkTelemetryPacket_t) <= MaxPayload[bulk DR]` for **all four** regions, dwell0 **and** dwell1.
3. Time-on-air at the bulk DR is within the AS923/AU915 400 ms dwell limit. (198 B at SF7/125 kHz ≈ 333 ms — passes, but with little margin. Recheck if the packet grows.)

---

# Appendix C — Areas not reviewed

Not examined in this pass. No claim is made about their correctness.

- `Core/Src/multiregion_h3.c` and the `Middlewares/Third_Party/h3lite` submodule
- `Core/Src/flash_if.c` — internal flash write/erase path
- `Core/Src/usart_if.c`, `sys_app.c`, `sys_debug.c`
- `Utilities/sequencer`, `Utilities/lpm`, `Utilities/timer`, `Utilities/trace`
- `LoRaWAN/App/CayenneLpp.c`
- **`LoRaWAN/Target/radio_board_if.c`** — worth a dedicated look. `RF_CTRL1`/`RF_CTRL2` are configured in `MX_GPIO_Init()` (`main.c:696-710`) *and* driven by the radio driver; TCXO control is referenced in `stm32_lpm_if.c:124-126` as "automatically managed" after manual control was removed. RF switch and TCXO handling across STOP2 entry/exit is exactly the kind of thing that produces intermittent TX failures.
- Deeper parts of `multiregion_context.c`: `CaptureCurrentContext`, `FlashReadTier1/2`, `FlashWriteTier1/2`, `FindContextSlot`, `MultiRegion_InitializeRegionFromChirpstack`
- Build system (`build.ps1`, `build.bat`), `.ioc`/CubeMX regeneration safety, linker script beyond the flash-region check

---

# Appendix D — Suggested work order

**Gate 1 — must pass before any flight**

1. R01 + R02 together (backup register map)
2. R24 then R23 together (never separately)
3. R30 (otherwise a launch-ready state is unreachable)
4. R08 (LSE failover)

**Gate 2 — power budget**

5. R25 (measure TTF first; this likely dominates everything else)
6. R26 (GSV)
7. R07 (chunked sleep)
8. R09 (LED)
9. R17, R39, R16 (spin/wake cost)

**Gate 3 — data integrity and coverage**

10. R03 (AU915)
11. R04 + R05 + R40 + R41 + R42 + R43 as one coherent payload-format change
12. R06 (uninitialised altitude)
13. R28, R29 (sensor and flash silent-failure modes)
14. R10 (brownout floor)

**Gate 4 — everything else**

15. Remaining P2 items and R44 documentation sweep.

**Recommended process note.** Two of the highest-severity findings here (R01, R08) are items the existing readiness matrix already marks **FIXED**. Consider adding a rule that a row can only be marked FIXED with a linked verification artefact — a test, a scope capture, or a log excerpt — rather than a code diff alone.
