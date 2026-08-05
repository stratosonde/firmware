# Stratosonde Firmware Review

**Repo:** `github.com/stratosonde/firmware`
**Reviewed:** 2026-08-04 (HEAD @ 2026-08-03, 50 commits)
**Scope:** ~15,637 lines of first-party C in `Core/Src` + `LoRaWAN/App`, ~9,040 lines of docs. Vendored ST HAL / LoRaMac excluded (196 of 340 tracked files). `h3lite` is an unpulled submodule — out of scope.

---

## Summary

| # | Finding | Severity | Effort |
|---|---------|----------|--------|
| 1 | Flash timestamps are boot-relative, not UTC | **P0 — fix before flight** | ~1 line |
| 2 | AS923 sub-group collapses to AS923-1 frequencies | **P1 — regulatory** | Medium |
| 3 | `SendTxData()` is 767 lines at nesting depth 6 | P2 — maintainability | Medium |
| 4 | No LICENSE, no CI, no tests, no reproducible build | P2 — project health | Low |
| 5 | 504 ungated RTT log sites on a 240 KB flash budget | P3 | Low |
| 6 | Minor: strcmp mapping, vendor-named API, stale README | P4 | Low |

---

## 1. Timestamp bug — flash records are stamped in MCU-relative seconds

### What's happening

There are two clocks in the system:

- **The hardware RTC calendar**, which counts from whenever it was last set — effectively from boot.
- **`SysTime`**, the LoRaWAN-layer time abstraction that sits on top of it.

On GPS fix, `SysTimeSyncFromGnss()` (`lora_app.c:1422`) hands the real UTC epoch to `SysTimeSet()`. But `SysTimeSet()` (`Utilities/misc/stm32_systime.c:227`) **does not set the calendar**. It reads current calendar time, subtracts it from the epoch you gave it, and stores only the *delta* into backup registers `DR0`/`DR1`. The hardware keeps ticking boot-relative. Real time is only ever reconstructed on the way out, via `SysTimeGet()`.

### Where it falls over

The flash write site uses the raw calendar:

```c
// lora_app.c:1532
now_timestamp = TIMER_IF_GetTime(&ms_unused);  // Fresh RTC seconds at write time
```

`TIMER_IF_GetTime()` reads the calendar directly. It never goes through `SysTimeGet()`, so it never applies the delta. The GPS discipline lands in those backup registers and sits there — **nothing on the archive path ever reads them.**

Every science record is stamped seconds-since-boot, presented as if it were UTC.

The tell is already in the struct comment:

```c
// flash_log.h:99
uint32_t timestamp;  // seconds since device start or epoch
```

Nobody writes "or epoch" when they know which one it is.

`payload_encode.c:159` has the same pattern.

### Fix

Use `SysTimeGet().Seconds` at the write site — it reconstructs the real epoch by applying the stored delta.

### The catch — do NOT find-and-replace

`Deadman_Check()` (`lora_app.c:1054`, `1061`) **must keep using `TIMER_IF_GetTime()`.**

```c
HAL_RTCEx_BKUPWrite(&hrtc, DEADMAN_BKP_REG, TIMER_IF_GetTime(&ms_unused));
uint32_t now = TIMER_IF_GetTime(&ms_unused);
```

The deadman needs *monotonic MCU time*. If it read disciplined time, the first GPS fix would jump the clock by decades and false-trip the watchdog instantly. Two clocks, two jobs — that separation is correct and should be documented as deliberate, not left looking like an oversight.

### Why it matters beyond the bug

Timestamp provenance is a reviewer question for AMT-class submission. "Seconds since an unspecified origin" is not a publishable time base.

---

## 2. AS923 sub-group — correct diagnosis, constrained fix

### What the code does

```c
// multiregion_h3.c:48-61
if (strcmp(name, "US915")    == 0) return LORAMAC_REGION_US915;
if (strcmp(name, "EU868")    == 0) return LORAMAC_REGION_EU868;
if (strcmp(name, "AS923-1")  == 0) return LORAMAC_REGION_AS923;
if (strcmp(name, "AS923-1B") == 0) return LORAMAC_REGION_AS923;
if (strcmp(name, "AS923-1C") == 0) return LORAMAC_REGION_AS923;
if (strcmp(name, "AS923-2")  == 0) return LORAMAC_REGION_AS923;
if (strcmp(name, "AS923-3")  == 0) return LORAMAC_REGION_AS923;
if (strcmp(name, "AS923-4")  == 0) return LORAMAC_REGION_AS923;
...
```

All six AS923 variants flatten to one enum. `h3lite` knows which sub-group you're over; the radio config can't consume it.

### Why it's built that way

**This is not laziness in the mapping — the ST stack has no separate enum for the sub-groups.** There is nowhere in the stack's region model for the sub-group to live. The mapping is boxed in by the stack's type system.

### But the stack doesn't discard it

The stack models sub-groups as a **frequency offset on the same channel plan**, not as separate regions:

```c
// lorawan_conf.h:106-112
 *  - CHANNEL_PLAN_GROUP_AS923_1     (Default. Freq offset =  0.0 MHz / 915-928 MHz)
 *  - CHANNEL_PLAN_GROUP_AS923_2     (Freq offset = -1.80 MHz / 915-928 MHz)
 *  - CHANNEL_PLAN_GROUP_AS923_3     (Freq offset = -6.60 MHz / 915-928 MHz)
 *  - CHANNEL_PLAN_GROUP_AS923_4     (Freq offset = -5.90 MHz / 917-920 MHz)
 *  - CHANNEL_PLAN_GROUP_AS923_1_JP  (Freq offset =  0.0 MHz / 920.6-923.4 MHz)

#define REGION_AS923_DEFAULT_CHANNEL_PLAN   CHANNEL_PLAN_GROUP_AS923_1
```

That offset is **pinned at compile time**. A float over AS923-2/-3/-4 transmits on AS923-1 frequencies, off by −1.8 to −6.6 MHz. Out of band.

### Fix direction

Not a new enum. Carry the sub-group as **data** alongside the enum, and set the channel plan / offset at runtime on region switch.

**Open question to resolve:** is `REGION_AS923_DEFAULT_CHANNEL_PLAN` reachable at runtime, or is the offset baked into `RegionAS923` at compile time in a way that can't be rewritten without patching the vendored stack? That determines whether this is a small change or a fork.

`docs/RegionDataAudit.md` is already circling this.

---

## 3. `SendTxData()` — decomposition, and why splitting alone isn't enough

`lora_app.c:1117-1883` — 767 lines, nesting depth 6, doing eight jobs:

power modelling · temperature compensation · GPS lockout · timer reprogramming · region switching · flash logging · a four-state TX machine · two debug paths

The seams are already visible in your own comments, so it's carve-able, not a rewrite.

### But splitting is necessary, not sufficient

If the eight jobs still share a pile of locals, you've spread the mess across more functions. **The real fix is separating deciding from doing.**

Today it interleaves: compute power budget → act → check temperature → act → pick region → act.

Instead: run all decisions first into one plain struct, then hand it to a dumb executor.

### The transmit plan struct

Pure data, no logic:

| Field | Purpose |
|---|---|
| `should_transmit` | Single go/no-go bit. GPS lockout, dead battery, too cold all resolve here. |
| `target_region` | Which region enum to use. |
| `channel_plan` / `sub_group` | (see finding #2) |
| `data_rate` / `tx_power` | How hot to key up. |
| `needs_region_switch` | Whether to switch before transmitting. |
| `should_log_record` | Whether this cycle writes to flash. |
| `next_wake_interval` | What to reprogram the timer to. |
| `veto_reason` | *Why* a skipped cycle was skipped. |

Five or six real fields, all set by code that touches **no hardware**.

### The decide half — invert the pyramid

The go/no-go is nested six deep because it's written as a pyramid: *if GPS good, then if battery good, then if warm enough, then transmit.* Every condition wraps the next.

Flip it: **list the noes, not the yeses.** A flat run of guard clauses, first veto wins, fall straight through:

```c
if (!gps_valid)        { plan.should_transmit = false; plan.veto = VETO_NO_FIX;     return plan; }
if (batt_mv < BATT_MIN){ plan.should_transmit = false; plan.veto = VETO_BATTERY;    return plan; }
if (temp_c  < TEMP_MIN){ plan.should_transmit = false; plan.veto = VETO_COLD;       return plan; }
```

No pyramid, because each check exits on its own instead of wrapping what follows.

**Record *why*, not just *that*.** A skipped cycle becomes "skipped — battery below threshold" instead of a silent gap. That's the same doctrine as DDR-0007: a gap is honest, a fantasy default is not.

### Veto ordering

Two rules: **dependencies first, then cheapest-and-safest among what's left.**

1. **Battery** — cheapest read, and if you're brownout-risk you don't want to wake GPS or key the radio at all.
2. **Temperature** — must resolve before GPS, because…
3. **GPS** — you already treat a stale or cold temp reading as a GPS lockout condition. GPS depends on a validated temp, so it goes last.

### What it collapses to

- **Decide function:** ~50-60 lines, no nesting past one level, reads top-to-bottom like a checklist.
- **Executor:** ~150-200 lines. Keying the radio, region switch, flash write is real work — but it's flat and boring, each step its own small function.

Total line count is similar. **The tangle is what's gone.** One 700-line knot becomes a 50-line brain plus dumb hands.

### The payoff loops back to finding #4

That decide function is **testable on a host with zero hardware.** Feed it fake battery and temp values, assert the go bit and the veto reason. That's the wedge for the entire test suite.

---

## 4. Reproducibility gap

### No LICENSE

Confirmed: no `LICENSE` file at repo root. The project describes itself as open-source; without a license, **nobody can legally use, fork, or contribute to it.** Default copyright is all-rights-reserved. This is the cheapest fix on the list and blocks the citizen-science participation model (SkyQuest) at the door.

### Nothing verifiable off your machine

- No CI.
- No unit tests.
- `Debug/` is gitignored, so the CubeMX-generated Makefile isn't tracked — **a fresh clone can't build.**
- `build.ps1` hardcodes `C:\ST` paths.

### Host-testable pure logic that exists *today*

No refactor required for any of these:

- `DaysFromCivil()`
- `NormalizeBatteryVoltage()`
- `CalculateVoltageSlope()`
- CRC16 / CRC32
- Payload encoders (`payload_encode.c`)
- H3 → region mapping (`multiregion_h3.c`)

### Recommendation

1. Add a LICENSE (MIT or Apache-2.0 given the hardware/citizen-science posture).
2. Host harness: compile the pure-logic files with the host compiler, minimal assert-based tests.
3. GitHub Actions: `arm-none-eabi-gcc` build + run the host tests. Track the Makefile.

---

## 5. RTT logging on a 240 KB flash budget

**504 confirmed SEGGER RTT call sites** across `Core/Src` and `LoRaWAN/App`, none compile-time gated.

Linker budget (`STM32WLE5JCIX_FLASH.ld`): FLASH 240 K, RAM 64 K + 32 K.

Format strings and call overhead are real estate you'll want back as the expansion port work lands.

### Fix

Wrap them in a `SONDE_LOG()` macro that compiles to nothing in a flight build:

```c
#ifdef SONDE_FLIGHT_BUILD
  #define SONDE_LOG(...)  ((void)0)
#else
  #define SONDE_LOG(...)  SEGGER_RTT_printf(0, __VA_ARGS__)
#endif
```

Keep every log line for the bench; pay nothing for them in the air. Pairs naturally with the dev/flight flag architecture already decided.

---

## 6. Minor items

**String-compare region mapping.** `multiregion_h3.c:48-61` matches on region *names*. If `h3lite` renames a region upstream, the match silently fails and you fall through to "keep current region." A rename in a submodule shouldn't be able to quietly strand a balloon. Folding this into the lookup table (finding #2) fixes it — and a `_Static_assert` on table length vs. enum count catches drift at compile time.

**`MultiRegion_InitializeRegionFromChirpstack()`** bakes a vendor name into what is otherwise a generic API. Cosmetic today; it will itch the first time the network server changes.

**README is stale.** Still describes the repo as architecture-docs-only. Off by ~15,000 lines of firmware.

---

## What's genuinely good

**Fault survival is the strongest part of this codebase.**

- **Chunked STOP2 sleep with EXTI-latch inspection** (`stm32_lpm_if.c:277-281`). This found a real silicon-level behaviour where `WUTF`/`ALRAF` were already cleared by IRQ handlers, so every 25 s wake was taking the full exit path. That's a genuinely hard bug to see.
- **LSE → LSI clock failover.** Keeps flying at ~1% drift instead of entering a reboot loop. Correct trade for a flight article.
- **IWDG armed immediately after `SystemClock_Config`** (FW-5) — no unguarded window during init.
- **Deadman check with a belt-and-braces 150-chunk bound.**
- **`backup_regs.h` single ownership map** after the DR0-DR2 collision. Exactly the right response to that class of bug.
- **Data honesty enforced in code (DDR-0007):** stale bits set at read and propagated record → uplink; stale temp treated as cold for GPS lockout; failed conversions skipped rather than zero-filled.
- **`_Static_assert` on the 64-byte record size.**
- **`TEST_UltraMinimal_STOP2()` deleted outright** rather than left behind a flag. Deleting a brick function is the right call.

---

## What's unique

**The DDR series (`docs/decisions/`, DDR-0001…0012) states doctrine, not just decisions.** "Forward progress always." "A gap is honest; a fantasy default is not." "Fail safe, not fail sunny." And critically — **the code cites the DDR at the enforcement point.** That closes the loop from principle → decision → enforcement → traceable reference. Most industrial ADR practice never gets there.

**Three-state readiness model** (`docs/ProjectStatus.md`): ✅ DONE / 🟡 CODE-DONE-BENCH-PENDING / ⬜ OPEN, with the rule that nothing goes ✅ without a linked bench artifact. That's the discipline that separates a flight article from a demo.

**Multi-region session banking** — two-tier redundant credentials with ping-pong counters, allowing region switch without rejoin. This is a real contribution to the pico-balloon problem space and is publishable on its own.

### One caution

Doc-to-code ratio is ~0.58. Right now that's an asset. It will flip to maintenance debt. `docs/FixWorkorderPlan.md`, `docs/ReviewFixImplementationPlan.md`, and `docs/CombinedReviewVerification-2026-08-03.md` read like process artifacts that belong in issues, not in `docs/`. Keep the DDRs and the design docs; let the work-tracking live where work-tracking lives.

---

## Suggested order of work

1. **LICENSE file.** Ten minutes, unblocks everything social about the project.
2. **Timestamp fix** at the write site, with a comment documenting why `Deadman_Check()` deliberately keeps MCU time.
3. **Region lookup table**, replacing the strcmp ladder and carrying the sub-group as data.
4. **Determine whether AS923 channel plan is runtime-settable** — this gates the real fix.
5. **Host test harness** on existing pure-logic functions + CI.
6. **`SONDE_LOG()` gate.**
7. **`SendTxData()` split into decide/execute**, with the decide half covered by tests from step 5.
