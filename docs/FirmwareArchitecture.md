# Stratosonde Firmware Architecture (Module Map)

> **As-built module map.** For the system-level narrative read
> `ARCHITECTURE-OVERVIEW.md`; for the product contract read
> `SYSTEM-INVARIANTS.md`; normative detail lives in `decisions/`. An earlier
> version of this document was a design-phase proposal whose invented modules
> and state machines never shipped — every "module" below names its real code.

## How the Code Is Actually Organized

Three layers, top to bottom:

1. **Application** — `LoRaWAN/App/lora_app.c`: the wake-cycle executor
   (`SendTxData`), burst FSM, region policy, GNSS epoch orchestration.
2. **Pure decision halves** (host-testable, zero hardware — the R47/#44 /
   R49/#46 pattern): `Core/Src/transmit_plan.c` (transmit-cycle decide half),
   `Core/Src/power_model.c` (power modes), `Core/Src/mission_logic.c`
   (launch/float detectors), `Core/Src/payload_format.c` (wire encoders).
3. **Drivers + services** — `Core/Src/`: `atgm336h.c` (GNSS), `ms5607.c` /
   `sht31.c` / `adc_if.c` + `sys_sensors.c` (environment), `flash_log.c` +
   `w25q16jv.c` (archive), `multiregion_context.c` + `multiregion_h3.c`
   (regions), `config.c` (configuration), `stm32_lpm_if.c` (STOP2),
   `reset_cause.c` / `sys_caps.c` / `backup_regs.h` (supervision).

## Module → Reality Map

| "Module" | Real code | Doc |
|----------|-----------|-----|
| System | `main.c` boot order + wake cycle + `mission_logic.c`/`mission_state.c` + `stm32_lpm_if.c` + `sys_caps.h` + `backup_regs.h` | [SystemModule.md](SystemModule.md) |
| Power Management | `power_model.c` + `transmit_plan.c` (cadence/veto) | [PowerManagement.md](PowerManagement.md) |
| GNSS | `atgm336h.c` (event-driven, no blocking API) | [GNSSModule.md](GNSSModule.md) |
| Environmental Sensors | `sys_sensors.c` over `ms5607.c`/`sht31.c`/`adc_if.c` | [EnvironmentalSensors.md](EnvironmentalSensors.md) |
| Flash Logging | `flash_log.c` + `w25q16jv.c` | [FlashLogging.md](FlashLogging.md) |
| Transmission | `lora_app.c` executor + `transmit_plan.c` + `multiregion_context.c` | [TransmissionModule.md](TransmissionModule.md) |
| Region Lookup | `multiregion_h3.c` over the h3lite submodule | [RegionLookup.md](RegionLookup.md) |
| LED Status | one PA0 boot blink in `main.c` — no module | [LEDStatus.md](LEDStatus.md) |
| Configuration | `config.c`, internal flash page 125 | [ConfigurationModule.md](ConfigurationModule.md) |
| Error Handler | distributed: reset_cause + sys_caps + deadman + honesty bits | [ErrorHandler.md](ErrorHandler.md) |

## The Wake Cycle (the one diagram that matters)

```
scheduled epoch (start-to-start cadence, SI-012)
  → DecideTransmitPlan (pure: power mode, cadence, veto)
  → [veto?] → record WHY (flags b5-b7) → sleep
  → GNSS epoch (if enabled; bounded, provenance-first R3-02)
  → post-acquisition env re-sample (LT-03: same-moment record)
  → archive 64 B record (flash ring, v5 watermarks)
  → confirmed probe → [no ACK] → sleep
                    → [ACK] → link-gated unconfirmed burst (one-pass walker)
  → STOP2 sleep (deadman checked pre-sleep, commissioning-exempt)
```

## Cross-Cutting Mechanisms (no module owns them)

- **Lifecycle**: one-way PRE-COMMISSIONED → COMMISSIONED_PRE-FLIGHT → ASCENT
  → FLOAT (SI-002); mission_logic + backup-domain latches.
- **Honesty**: every sensor channel plausibility-gated + stale bit (R28/#136);
  every non-go cycle veto-coded (DR-06/#241).
- **Supervision**: IWDG + derived deadman (S-04/#228) + reset-cause/fault
  codes in backup registers.
- **Capability degrade**: `sys_caps.h` — FLASH/GNSS/SENSORS/RADIO degrade
  independently (SI-013); no limp mode, no probation.
- **Persistence tiers**: backup regs (ownership map) · internal flash pages
  120-127 (Tier-1 credentials, Tier-2 counters, config, LoRaWAN NVM) ·
  external NOR (science archive).

## Testing

13-suite host baseline (`tests/host/`, CI-green) covering the pure halves
behaviourally and the hardware-coupled files by regression scans; ARM build
gate via STM32CubeIDE make. Bench/HIL evidence is tracked in
`requirements/flight1-validation-readiness-checklist.md`.