# Power Management System

## Overview (PWR-SIMPLIFY, 2026-08-24)

The firmware implements a **two-gate admission + fixed-cadence** power model,
optimized for extreme-temperature operation (down to −60 °C). The previous
voltage-slope predictive ladder (slope tracking, mode selection,
hysteresis, time-to-target predictions, backup-register persistence) was
deleted: its thresholds assumed a warm, solar-charged envelope, and inside
that envelope every chamber run showed the model degrading and then
latching (see `docs/temp/stratosonde-power-simplification.md` for the
evidence and the traded-away goals).

**What decides safety now — exactly two hard gates, checked once per wake
before GNSS, archive, probe, or live telemetry** (`FirstFlightWakeAdmitted`
in `LoRaWAN/App/lora_app.c`, policy in `Core/Src/first_flight_policy.c`):

| Gate | Input | Default | Meaning |
|------|-------|---------|---------|
| **A — temperature** | fresh temperature | **−60 °C** (`gps_temperature_lockout`) | Below this, or on a stale/invalid temperature, the wake retries at the survival cadence. Sits on the measured knee of the corrected bench CSV: a 30 s fix still clears the 3.3 V GPS floor at −60 °C on a full pack (+0.14 V) and fails at −65 °C. Revisit after the low-SoC sag re-measurement. |
| **B — raw battery** | fresh raw battery (mV) | **3800 mV** (`CONFIG_FIRST_FLIGHT_BATTERY_FLOOR_MV`, clamped up by `ConfigGetFirstFlightBatteryMinMv`) | A brownout-protection / GPS-admission floor just above the LTO discharge cutoff (~1.8 V/cell). It does NOT guarantee a fix in the cold + low-SoC corner: a fix that browns the receiver there returns an honest no-fix record. |

Both gates are one number each, named and commented with their provenance
(bench CSV + operator choice). A rejected admission schedules
`tx_interval_survival` (60 min retry) and performs no GNSS, no archive
write, no probe, no live telemetry.

**Cadence:** a single configured target — `tx_interval_normal` (default
5 min) — subject to the mission-phase override and the outage backoff
(below). `tx_interval_conservative/reduced/recovery` are reserved for
flash layout compatibility and are no longer read.

---

## What survived, deliberately

### Mission cadence override (DDR-0002)

When the power model is admitted (mode pinned NORMAL), the mission phase
still overrides the interval: **ASCENT = 10 s**, **FLOAT = 5 min**.
COMMISSIONING gets no override. The plan's `power_mode` is pinned
`MODE_NORMAL`; the override gate always passes, so the mission-phase
target decides the cadence.

### Outage backoff (network politeness — kept, F-10/#267)

The **no-ACK backoff** (`OutageBackoff_Interval`) is NOT power management:
it throttles cadence ×2/×4/×8 at 1 h/4 h/12 h without a confirmed ACK,
capped at `CONFIG_MAX_TX_INTERVAL_MS`, and resets on any ACK. In a metal
chamber it is the only thing that changes cadence — read `power_mode` off
received packets to discriminate it from the old SURVIVAL latch.

### SoC temperature normalization (telemetry only)

`NormalizeBatteryVoltage` in `Core/Src/power_model.c` maps raw pack voltage
to a 25 °C-equivalent level for **state-of-charge telemetry**. It is not an
input to any safety decision — the admission gates read raw values. The
compensation table itself is the surviving knot table (see "Battery
Specifications" below). Known issue: the −40…−55 °C span is non-monotonic
(LT-06/#248, red-gated as the LT-06 pair; recalibration tracked separately).

---

## Battery Specifications (LTO 2S)

### LTO (Lithium Titanate Oxide) Characteristics

| Voltage | Capacity | State |
|---------|----------|-------|
| **5.5V** | 100% | Fully charged |
| **5.2V** | 80% | Still looks full (flat curve) |
| **5.0V** | 60% | Normal operation |
| **4.8V** | 40% | Getting low |
| **4.6V** | 20% | Low battery |
| **4.5V** | 5% | Critical (mostly dead) |
| **4.3V** | 1% | Emergency |
| **3.5V** | 0% | Totally dead |

### LTO vs LiPo

| Property | LTO (Your System) | LiPo (Common) |
|----------|-------------------|---------------|
| Voltage range | 5.5V → 3.5V (2.0V) | 4.2V → 3.0V (1.2V) |
| Discharge curve | Very flat | Non-linear |
| Slope visibility | Excellent | Poor (steep at ends) |
| Cycle life | 10,000+ cycles | 500-1000 cycles |
| Cold performance | Excellent | Poor |

### Cold-temperature terminal voltage (bench CSV, corrected −55/−60 swap)

| Temperature | Vmax (No Load) | GNSS-loaded (30 s fix) |
|-------------|----------------|------------------------|
| +25°C | 5.24 V | 5.04 V |
| −40°C | 5.10 V | 4.36 V |
| −55°C | 5.02 V | 3.58 V |
| −60°C | 4.98 V | 3.44 V |
| **−65°C** | 4.98 V | **3.10 V** (below the 3.3 V GPS floor) |
| −70°C | 4.96 V | 2.40 V |

The −60/−65 °C knee is Gate A's evidence. Sag values are at **full charge**;
sag near empty (LTO internal resistance rises sharply at low SoC + cold) is
the pending measurement before flight lock.


---

## Telemetry Channels

> **Debug-only view (R44):** These CayenneLPP channels ride Port 2, which is
> **compiled out of flight builds** (`ENABLE_DEBUG_LPP=0`,
> `payload_format.h`). In production the same quantities travel in the
> **Port 11 archive record** — `voltage_slope` (int16 mV/h, now the
> `SLOPE_MV_H_NOT_COMPUTED` sentinel −32768), `power_mode`, and
> `battery_voltage`/`solar_voltage` fields ([PayloadFormats.md](PayloadFormats.md)).

| Channel | Name | Unit | Notes |
|---------|------|------|-------|
| 11 | Voltage Slope | mV/h | **Reserved** — written as −32768 (PWR-SIMPLIFY: ladder deleted; layout preserved) |
| 12 | Time to Target | hours | **Reserved** — same sentinel |
| 14 | Operating Mode | enum | 0=NORMAL (pinned) … 4=SURVIVAL (enum retained for telemetry) |

The **archive record v6 wire layout is unchanged** (34 bytes, offset 24 =
`voltage_slope`); decoders read −32768 as "not computed", never as a slope.

---

## Decision Cascade (what one wake does)

1. **Read fresh temperature + raw battery.**
2. **Gate A:** temperature below −60 °C, or stale/invalid → **retry** at
   `tx_interval_survival` (60 min); no science record, no live telemetry.
3. **Gate B:** raw battery below 3800 mV, or stale/zero → **retry** as above.
4. **Admitted:** `DecideTransmitPlan` returns the fixed cadence
   (`tx_interval_normal`), `MODE_NORMAL`, GNSS always budgeted (nonzero
   timeout). The only conditional branch is the RF-silence veto
   (`VETO_RF_SILENCE` when FLIGHT with no valid LoRaWAN session).
5. **Mission phase** overrides the cadence (ASCENT 10 s / FLOAT 5 min).
6. **Outage backoff** scales the interval ×2/×4/×8 as no-ACK time grows.
7. GPS fix, sensor re-sample, archive write (veto recorded in flags),
   transmit.

---

## Design Philosophy

### Simple > Complex

- Two named constants decide safety; one number each, with the bench
  measurement in the comment. No slope estimator, no hysteresis state
  machine, no predictions, no backup-register persistence of power state.
- mV/hour was dropped with the ladder; SoC is a level, not a rate.
- Deliberate: *"you only do that extra complication, not to be fancy, but
  only if you need to."*

### Honest > Clever

- A fix that browns the receiver in the cold+empty corner returns an
  honest no-fix record (stale bits), not a fabricated fix.
- The archive record's reserved slope slot carries a named sentinel, not
  a fabricated zero.

---

## Summary

**Current Implementation (post PWR-SIMPLIFY):**
- ✅ **Two hard admission gates** — temperature (−60 °C) + raw battery
  (3800 mV), each a named constant with bench provenance
- ✅ **Fixed configured cadence** (`tx_interval_normal`), mission-phase
  override, network-politeness outage backoff (kept)
- ✅ **SoC normalization for telemetry only** (never a safety input)
- ✅ **Wire-compatible**: archive record v6 layout unchanged; reserved
  slope slot = `SLOPE_MV_H_NOT_COMPUTED`

**System Characteristics:**
- **Radically simplified**: the ladder, slope tracking, hysteresis,
  predictions, and backup-register persistence are deleted
- **Temperature Aware**: one admission threshold, bench-derived
- **Battery Protective**: one admission floor, bench-derived
- **Honest**: stale bits and a sentinel, never fabricated data
