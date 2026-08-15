# System Module

## Overview

There is no `System_*` API module — system orchestration is **main.c plus the
wake cycle in `lora_app.c`**, held together by four small, real mechanisms:
the mission lifecycle (`mission_logic.c`/`mission_state.c`), the STOP2
low-power path (`stm32_lpm_if.c`), the capability ledger (`sys_caps.h`), and
the backup-domain ownership map (`backup_regs.h`). This document describes
that actual architecture; an earlier version described a fictional
`System_Init`/`System_SetState`/`System_HandleError` module.

## Lifecycle (one-way, SI-002)

```
PRE-COMMISSIONED → COMMISSIONED_PRE-FLIGHT → ASCENT → FLOAT
```

Every transition is one-way and latch-based (provisioning latch C-01/#270;
pressure launch detector STAB-06/#153 + R3-10/#222 two-evidence latch; float
latch). **Float is terminal** (DDR-0002 §19 INV-LIFE-011) — the reversible
Ascending/Float/LowPower triangle in the old doc never existed. Low power is
not a lifecycle state; it is the power model's mode (see PowerManagement.md),
orthogonal to lifecycle.

## Boot Order (main.c — order is load-bearing)

1. RTT buffer + build marker (F12/#173: the marker is genuinely referenced so
   `--gc-sections` can't strip it)
2. `ResetCause_CaptureBoot` — classify this reset, record fault codes
3. `Config_Init` **before** `MX_LoRaWAN_Init` (H-02/#273: the frame-counter
   restore margin must follow the configured save interval)
4. `MX_LoRaWAN_Init` → `MultiRegion_Init` (Tier-1/2 restore)
5. `leds_boot_seq()` — commissioning-only boot blink (R09/DDR-0002)
6. Sequencer start → the wake cycle runs as tasks, not a superloop

## The Wake Cycle

Each scheduled wake is one early admission decision and one bounded epoch:
plan (power model + veto) → GNSS (if enabled) → post-acquisition env re-sample
(LT-03/#271) → archive → probe/burst (see TransmissionModule.md) → sleep.
Opportunistic archive work yields to live science epochs (SI-012). Sleep is
STOP2 via `stm32_lpm_if.c` with the wakeup timer; LT-05/#275 degraded
capability handling on STOP2 re-init failure.

## Supervision

- **IWDG** independent watchdog (fault code F6/#171 if init fails)
- **Deadman** (`Deadman_Check` in lora_app.c, run pre-sleep): resets the unit
  if no productive work cycle completed within the derived timeout
  (`ConfigGetDeadmanTimeoutS` = max(3 h, 3 × survival cadence), S-04/#228);
  **no-op in COMMISSIONING** (DDR-0020)
- **Reset cause + fault codes** (`reset_cause.h`): boot classification,
  fault codes in a backup register (magic 0xF17B), boot-attempt counter

## Capability Degrade (sys_caps.h, F-014/#207)

Four capabilities — FLASH, GNSS, SENSORS, RADIO. `SysCaps_MarkFailed` /
`SysCaps_Available`: a subsystem that fails its bounded recovery degrades
**only its dependent capability** (SI-013); everything else continues. This
replaced speculative "limp mode" designs — there is no limp mode.

## Persistent-State Ownership

`backup_regs.h` is the backup-domain ownership map (R01/R02 Gate-1): every
backup register has exactly one writer, documented in one table, mirrored by
the internal-flash page map in `config.h` (FR-21/#102: new users update both).

## Cross-References

- `docs/ARCHITECTURE-OVERVIEW.md` — the system-level narrative
- `docs/SYSTEM-INVARIANTS.md` — the contract this orchestration serves
- DDR-0002 (lifecycle), DDR-0010 (persistence), DDR-0020 (supervision)