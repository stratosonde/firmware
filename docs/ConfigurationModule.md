# Configuration Module

## Overview

The Configuration Module (`Core/Src/config.c`, API in `Core/Inc/config.h`) holds
`SystemConfig_t`: the flash-persisted system configuration in a **dedicated
2 KB internal-flash page**, with magic/version/size/CRC32 validation and a
defaults fallback. It replaces scattered `#define`s for the knobs that are
actually configurable — and honestly annotates the fields that are **reserved**
(no live consumer) so nobody trusts a decorative knob (R12/#197).

## Storage

Internal flash page 125 (`CONFIG_FLASH_ADDRESS` 0x0803E800, one full 2 KB erase
page). The STM32WLE5 erases in 2 KB pages, so the region must own its page —
this module previously lived at 0x0803FC00, **inside** page 127, and every
`Config_Save` erased the entire LoRaWAN multiregion context store (DevAddr,
session keys, frame counters) with it.

Internal flash page map (256 KB device, mirrored in `multiregion_context.c` —
FR-21/#102: any new page user updates both):

| Page | Address | Owner |
|------|---------|-------|
| 120-122 | 0x0803C000+ | Tier-1 LoRaWAN credentials, copies A/B/C (FW-1/DDR-0018) |
| 123-124 | 0x0803D800+ | Tier-2 frame counters, ping-pong slots A/B |
| 125 | 0x0803E800 | **System configuration (this module)** |
| 126-127 | 0x0803F000+ | LoRaWAN NVM context + slot B ping-pong (F-016/#54) |

## What Is Actually Configurable

`SystemConfig_t` (packed, 8-byte aligned for `FLASH_IF_Write` — FR-04/#81: the
write is rejected *after* erase if the length isn't 64-bit aligned, so the
`_Static_assert` is load-bearing):

| Group | Fields | Status |
|-------|--------|--------|
| TX intervals | `tx_interval_{normal,conservative,reduced,recovery,survival}` (ms; 5/10/15/30/60 min) | **ACTIVE** — power-model cadence inputs; ceiling `CONFIG_MAX_TX_INTERVAL_MS` = 2 h (DR-04/#240) |
| LoRaWAN params | datarate, txpower, ADR, confirmed, class-B timeout | **RESERVED** (R12/#197) — no consumer; session radio params persist per-region in the Tier-2 bank (R11/#196) |
| Power thresholds | `battery_low_threshold`, `battery_critical_threshold`, hysteresis | `battery_critical_threshold` is **ACTIVE** as the first-flight raw-battery admission minimum (default/effective floor 3800 mV, PWR-SIMPLIFY Gate B 2026-08-24). A legacy lower persisted value still loads but is clamped to 3800 mV at use; stricter values remain effective. Other legacy fields retain layout. |
| Bulk gate | `bulk_battery_min_mv` (default 5000), `max_bulk_packets` (20), `bulk_timeout_ms` (60 s) | **ACTIVE** (§6b) |
| GPS admission | `gps_temperature_lockout` | **ACTIVE** as the first-flight minimum temperature (default −60 °C, PWR-SIMPLIFY Gate A 2026-08-24); it is an admission threshold, not a GNSS runtime lockout |
| GPS | `gps_timeout_{normal,conservative}`, `gps_min_satellites` (4), `gps_max_hdop_x10` (25), `gps_standby_power_ua` (15) | Active/quality knobs — see #286 (def-val entry points must agree) and #284 (acceptance predicate) |
| Adaptive TX | `link_margin_threshold` (15 dB), `gateway_count_threshold` (2) | SF7 elevation gates |
| Counters | `frame_counter_save_interval` (default 10) | **ACTIVE** (`CfgFrameCounterSaveInterval`) — H-02: the restore margin must follow the configured value, so `Config_Init` runs **before** `MX_LoRaWAN_Init` (#273) |
| Flash logging | interval, enabled, compression, retention_days | archive policy |
| Debug | LPP/GNSS-detail enables, intervals, RTT level, flags | compiled out of flight builds with `ENABLE_DEBUG_LPP=0` |
| Solar | `reserved_solar` | deleted (F19): the 6000 mV default could never trip on the ~1.1 V panel and had zero consumers |

## Derived Values

- `ConfigGetDeadmanTimeoutS()` (S-04/#228): the deadman watchdog timeout is
  **derived**, not configured — `max(CONFIG_DEADMAN_FLOOR_S = 3 h,
  3 × survival cadence)`. A fixed 3 h timeout against a configurable 2 h
  survival cadence gave a 1.5× margin where 3× was intended.

## API

| Function | Purpose |
|----------|---------|
| `Config_Init` | Load from flash, fall back to defaults (must precede `MX_LoRaWAN_Init`, H-02/#273) |
| `Config_Get` | Read-only pointer; **NULL before init** — callers use macro defaults (§6b single-accessor pattern) |
| `Config_Load` / `Config_LoadDefaults` | Reload / factory defaults (defaults are not saved until `Config_Save`) |
| `Config_Save` | Erase page 125 + program — see immutability below |
| `Config_Validate` | Range/dependency checks (enforces the DR-04 ceiling) |
| `Config_UpdateParameter` | Validated single-field update with CRC refresh |
| `Config_GetStats` / `Config_PrintCurrent` | Read/write/CRC-failure counters; RTT dump |
| `ConfigGetDeadmanTimeoutS` | Derived deadman timeout (S-04) |

## Immutability in Flight (the load-bearing rule)

**`Config_Save` has no callers outside `config.c`** — configuration is written
once at commissioning and never in flight. This is what makes the single-slot
erase-then-write persistence acceptable: there is a torn-write window between
erase and program, and it is only tolerable because a reset inside it can only
happen on the bench (#198, documented as the IMMUTABILITY ASSUMPTION at the
save site; a host regression test fails if any new caller appears — R14).
Upgrading to an atomic slot scheme is tracked as **#282** (needs host tests
first per the test-first rule).

Invalid or corrupted configuration (bad magic/version/size/CRC) falls back to
compiled defaults — the system always boots with sane values.

## Cross-References

- `Core/Inc/backup_regs.h` — the backup-domain ownership map (sibling rule to
  the flash page map)
- DDR-0018 (credentials), DDR-0010 (persistence classification)
- `docs/PowerManagement.md` — first-flight admission and the remaining
  power-model thresholds
