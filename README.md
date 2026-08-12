# Stratosonde Firmware

Flight firmware for the Stratosonde — an ultra-lightweight, solar-powered radiosonde for long-duration autonomous stratospheric flight. Collects atmospheric and positional data, archives it to onboard flash, and transmits over LoRaWAN with multi-region geofencing.

**Start here:** [`docs/ProjectStatus.md`](docs/ProjectStatus.md) — what is done, what is bench-pending, what is open.
**Docs index:** [`docs/README.md`](docs/README.md) · **Work tracking:** [GitHub issues](https://github.com/stratosonde/firmware/issues) · **Design doctrine:** [`docs/decisions/`](docs/decisions/) (DDR-0001…0012)

## Hardware

- **MCU:** STM32WLE5JC (ARM Cortex-M4 + integrated LoRa radio)
- **GNSS:** ATGM336H-5N31 (UART1, power-switched)
- **Pressure:** MS5607 · **Temp/Humidity:** SHT31
- **Flash:** W25Q16JV (science archive)
- **Power:** solar panel + battery, STOP2-based duty cycling

## Architecture highlights

- **Mission state machine** (COMMISSIONING → FLIGHT, one-way door; DDR-0008) — joins and GPS reconfig only on the bench; flight never rejoins (DDR-0006)
- **Multi-region session banking** — two-tier redundant credentials with ping-pong frame counters; region switch without rejoin (h3lite geofence engine)
- **Flash ring** — erase-before-write, torn-write-tolerant headers, frontier scan (DDR-0004)
- **Data honesty in code** (DDR-0007) — stale bits propagated sensor → flash → uplink; failed reads never become fantasy defaults
- **Fault survival** — IWDG armed at boot, LSE→LSI RTC failover, deadman progress check, chunked STOP2 sleep, backup-register ownership map (`Core/Inc/backup_regs.h`)

## Repository layout

| Path | Contents |
|---|---|
| `Core/`, `LoRaWAN/` | First-party firmware (~15.6k lines C) |
| `Drivers/`, `Middlewares/`, `Utilities/` | Vendored ST HAL / LoRaMac / h3lite submodule |
| `docs/` | Status, decision records, architecture & protocol specs ([index](docs/README.md)) — historical reviews/ledgers live in git history only |

## Build

STM32CubeIDE / `arm-none-eabi-gcc` project (`.ioc`: `Radio_Sonde_E5_HF_EU.ioc`). Helper scripts: `build.ps1` / `build.bat`. Reproducible-build and CI work is tracked as [issue #46](https://github.com/stratosonde/firmware/issues/46).

### Flight build (logging stripped)

The default `Debug/` build is a **bench** build: all `SONDE_LOG` RTT output is compiled in. For flight, define `SONDE_FLIGHT_BUILD` (gate in `Core/Inc/sonde_log.h`, #47) to compile every debug/log line out. **First-class target (F-009/#209):**

```powershell
.\build.ps1 -Flight
```

This injects the macro into the generated compile fragments with a loud-failure gate (a missed fragment aborts the build), builds from scratch, verifies the embedded `SONDE_BUILD:flight` marker in the binary, restores the fragments to bench default, and writes `dist/flight/` artifacts (`.bin`/`.elf`/`.map`) plus a manifest (git SHA, dirty status, toolchain version, SHA-256 hashes, timestamp, defines). On Linux CI the equivalent sed-injection recipe remains:

```sh
cd Debug
find . -name subdir.mk -exec sed -i 's/-DDEBUG -DCORE_CM4/-DDEBUG -DSONDE_FLIGHT_BUILD -DCORE_CM4/' {} +
make clean && make -j all
```

Measured effect (A-006 / #80, arm-none-eabi-gcc 10.3.1, `-Os`): **flash text −22.4 KB** (192,016 → 169,536 B, −11.7%), data/bss unchanged (RTT control-path buffer intentionally retained). Flight build compiles clean; the only warnings are benign "unused variable" for locals referenced solely in compiled-out log lines.
