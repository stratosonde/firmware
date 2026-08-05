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
| `docs/` | Status, decision records, architecture & protocol specs ([index](docs/README.md)) |
| `docs/archive/` | Point-in-time reviews and superseded process docs |

## Build

STM32CubeIDE / `arm-none-eabi-gcc` project (`.ioc`: `Radio_Sonde_E5_HF_EU.ioc`). Helper scripts: `build.ps1` / `build.bat`. Reproducible-build and CI work is tracked as [issue #46](https://github.com/stratosonde/firmware/issues/46).
