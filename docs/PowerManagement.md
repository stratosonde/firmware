# Power Management System

## Overview

The firmware implements a **real-time adaptive power management system** optimized for extreme temperature operation (-65°C to +25°C). The system continuously monitors battery voltage slope and adjusts transmission intervals and GPS usage to optimize battery life — downgrading immediately when conditions deteriorate, upgrading only after consecutive confirmation (F8/#172).

**Key Features:**
- Temperature-compensated voltage measurements (accurate across -65°C to +25°C)
- Real-time voltage slope calculation (+charging, -discharging in mV/hour)
- Five adaptive operating modes (5min to 60min intervals)
- Direction-aware time-to-critical and time-to-full predictions (STAB-08/#155)
- **Immediate downgrades, hysteresis-confirmed upgrades** (F8/#172, S-E/#214)

---

## Operating Modes

| Mode | TX Interval | GPS | When Used (`SelectModeFromPredictions`) |
|------|-------------|-----|-------------|
| **NORMAL** | 5 min | ✅ Yes | Fast charging: slope > +20 mV/h |
| **CONSERVATIVE** | 10 min | ✅ Yes | Stable/slow charge (slope > 0, and the default) |
| **REDUCED** | 15 min | ✅ Yes for admitted science | Slow discharge: slope < -5 mV/h |
| **RECOVERY** | 30 min | ✅ Yes for admitted science | slope < -15 mV/h **and** time-to-critical < 12 h |
| **SURVIVAL** | 60 min cadence/retry | ✅ Yes if admitted | Very fast discharge prediction; below-floor admission instead sleeps with no science record or live telemetry |

The evaluation order is: raw-voltage floor first (a cold-compensated voltage must
never mask a real brownout — normalization feeds slope/prediction only), then the
two predictive emergencies, then plain slope thresholds.

**First-flight admission:** the persisted `gps_temperature_lockout` (default
−55 °C) and `battery_critical_threshold` (default 4300 mV) are minimum fresh
temperature/raw-battery inputs. Below either threshold, or on stale/invalid
admission data, the wake schedules `tx_interval_survival` and performs no GNSS,
archive, probe, or live telemetry. Above both thresholds, REDUCED/RECOVERY keep
their cadence preferences but admitted science always enables GNSS with a
nonzero budget. New/default configurations use 4300 mV. Older persisted lower
values remain loadable for flash compatibility, but the first-flight accessor
clamps their effective admission threshold to the existing 4300 mV raw
electrical floor; a configured value above 4300 mV remains effective.

**Mission cadence override (DDR-0002):** when the power model is healthy
(NORMAL/CONSERVATIVE), mission phase overrides these intervals — ASCENT = 10 s,
FLOAT = 5 min. REDUCED, RECOVERY, and admitted SURVIVAL retain the table cadence;
the same SURVIVAL interval is used after rejected admission. None is a deliberate
GNSS-less science mode.

---

## Temperature Compensation

### LTO Battery Characteristics at Low Temperature

The system uses **LTO (Lithium Titanate Oxide)** batteries with extreme temperature performance:

| Temperature | Vmax (No Load) | Notes |
|-------------|----------------|-------|
| +25°C | 5.50V | Reference (room temperature) |
| -55°C | 5.07V | -430mV drop |
| -60°C | 4.70V | -800mV drop |
| **-65°C** | **3.33V** | **-2170mV drop** (massive!) |

**Without compensation at -65°C:** Raw voltage of 3.33V would trigger SURVIVAL mode even if battery is fully charged!

### Temperature Normalization

**Algorithm:**
```c
normalized_voltage = measured_voltage + temperature_compensation
```

**Example at -65°C:**
- Measured: 3330mV (looks critical!)
- Compensation: +2170mV (from lookup table)
- Normalized: 5500mV (actually fully charged!)

**Result:** Voltage slope calculation uses normalized values, providing accurate charge/discharge rates regardless of temperature.

---

## Voltage Slope Calculation

### 2-Hour Baseline Window

The system tracks voltage change over a rolling 2-hour window:

```c
slope_mV_per_hour = (current_voltage - baseline_voltage) × 3600 / time_seconds
```

**Baseline Shift:** Every 2 hours, the baseline moves forward to the current measurement, creating a sliding window.

### Typical Slope Values

| Scenario | Slope (mV/h) | Meaning |
|----------|--------------|---------|
```

**Example:**
- Baseline: 4800mV at 10:00am
- Current: 4820mV at 12:00pm (2 hours later)
- Slope = (4820 - 4800) × 3600 / 7200 = **+10 mV/h** (charging)

### Baseline Update

Every **2 hours** (7200 seconds), the baseline shifts forward:
- New baseline voltage = current voltage
- New baseline timestamp = current timestamp

This creates a **sliding 2-hour window** that adapts to changing conditions.

### Window Duration by Mode

| Mode | TX Interval | Time to Fill Window |
|------|-------------|-------------------|
| NORMAL | 5 min | 12 samples × 5min = 60 min |
| CONSERVATIVE | 10 min | 12 samples × 10min = **120 min** |
| REDUCED | 15 min | 12 samples × 15min = 180 min |
| RECOVERY | 30 min | 12 samples × 30min = 360 min |
| SURVIVAL | 60 min | 12 samples × 60min = 720 min |

**Note:** The window measures change over the full duration (e.g., 2 hours in CONSERVATIVE mode).

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

---

## Telemetry Channels

> **Debug-only view (R44):** These CayenneLPP channels ride Port 2, which is **compiled out of flight builds** (`ENABLE_DEBUG_LPP=0`, `payload_format.h`). In production the same quantities travel in the **Port 11 archive record** — `voltage_slope` (int16 mV/h), `power_mode`, and `battery_voltage`/`solar_voltage` fields ([PayloadFormats.md](PayloadFormats.md)). This table documents the bench/debug encoding.

| Channel | Name | Unit | Scaling | Description |
|---------|------|------|---------|-------------|
| 11 | Voltage Slope | mV/h | ÷10 | Charging rate (+) or discharge rate (-) |
| 12 | Time to Target | hours | Direct | Hours until critical (-) or full (+) |
| 14 | Operating Mode | enum | Direct | 0=NORMAL ... 4=SURVIVAL |

### Voltage Slope Encoding

**Firmware:**
```c
CayenneLppAddAnalogInput(11, (float)slope_mv_per_hour / 10.0f);
```

**Example:**
- Internal: `slope = 1198 mV/h`
- Sent: `119.8` (÷10)
- Cayenne LPP: `11980` (×100, signed 16-bit)
- Decoded: `119.8`
- Ground station display: `1198 mV/h` (×10)

**Why ÷10?** Allows slopes up to ±3,276 mV/h (s igned int16 with 0.01 resolution).

---

## Example: Winter Day in Calgary

### Real-Time Response Timeline

```
Time      Battery  Solar  Slope      Mode         Action
--------  -------  -----  ---------  -----------  ----------------------
5:00pm    5200mV   0mV    -          CONS         Sunset - start discharge
6:00pm    5195mV   0mV    -5 mV/h    CONS         Slow discharge
11:00pm   5170mV   0mV    -6 mV/h    REDUCED      Slope < -5mV/h → immediate switch
6:00am    5125mV   0mV    -5 mV/h    REDUCED      Still dark, discharging
9:00am    5120mV   1500mV 0 mV/h     CONS         Sunrise! Slope stabilizing
10:00am   5125mV   2800mV +8 mV/h    CONS         Gentle charging detected
12:00pm   5145mV   3200mV +20 mV/h   CONS         Charging window (slope building)
1:00pm    5155mV   3400mV +25 mV/h   NORMAL       Slope > 20mV/h, confirmed over consecutive cycles (F8)
4:00pm    5192mV   3600mV +22 mV/h   NORMAL       Peak charging continues
6:00pm    5200mV   1000mV +8 mV/h    CONS         Slope < 20mV/h → immediate downgrade
8:00pm    5195mV   0mV    -3 mV/h    CONS         Light discharge
11:00pm   5185mV   0mV    -7 mV/h    REDUCED      Slope < -5mV/h → immediate switch
```

### Responsiveness: asymmetric by design

The system responds **asymmetrically** to changing slope conditions (F8/#172):
- **Downgrades are immediate**: slope < -5 mV/h switches to REDUCED on the next
  decision; emergency conditions (raw V < 4.3 V, or slope < -30 mV/h with
  time-to-critical < 6 h) force SURVIVAL instantly. Battery protection never
  waits.
- **Upgrades require consecutive confirmation**: one ADC sample of noise
  (~10 mV) over the slope window is ~60 mV/h — larger than every mode
  threshold — so a raw proposal chatters. A higher-power mode must be proposed
  on consecutive evaluations, separated in time (F-4/#179: a same-timestamp
  burst re-arm cannot confirm an upgrade), and with a consistent target
  (S-E/#214: a changed upgrade target restarts the streak).

### Hysteresis state (RAM-only)

`VoltageSlope_t` carries the hysteresis machine alongside the slope window:
`committed_mode`, `upgrade_streak`, `hyst_last_ts`, `hyst_last_proposal`. Like
the slope history it is intentionally RAM-only — after reset the system simply
re-earns any upgrade from the conservative default.

---

## Voltage Slope Examples

### Typical Values

| Scenario | Slope | Meaning |
|----------|-------|---------|
| **Night (idle)** | -3 to -8 mV/h | Slow self-discharge + MCU power |
| **Night (cold)** | -15 to -25 mV/h | Cold temp + higher MCU consumption |
| **Day (weak sun)** | +5 to +15 mV/h | Trickle charging (winter/cloudy) |
| **Day (good sun)** | +20 to +50 mV/h | Active charging (summer/clear) |
| **Day (peak sun)** | +80 to +150 mV/h | Peak charging (noon, summer, clear) |

### Unrealistic Values (Bugs)

| Value | Likely Cause |
|-------|--------------|
| >±500 mV/h | Buffer initialization error (comparing against zeros) |
| Wide oscillation | ADC noise or measurement glitches |
| Always zero | Timestamp not incrementing |

---

## Decision Cascade

Mode selection is **not** a voltage-level matrix. The model uses exactly one
voltage test (the raw 4.3 V floor) plus slope and direction-aware predictions
(`SelectModeFromPredictions` in `Core/Src/power_model.c`, evaluated in this
order — first match wins):

| # | Condition | Mode | Why |
|---|-----------|------|-----|
| 1 | raw voltage < 4300 mV | **SURVIVAL** | Absolute LTO floor; reads the **raw** voltage so cold compensation can't mask a brownout (R10/#37, BUG 1.5) |
| 2 | slope < -30 mV/h **and** time-to-critical < 6 h | **SURVIVAL** | Depleting fast, critical imminent |
| 3 | slope < -15 mV/h **and** time-to-critical < 12 h | **RECOVERY** | Moderate depletion, limited time |
| 4 | slope < -5 mV/h | **REDUCED** | Slow depletion |
| 5 | slope > +20 mV/h | **NORMAL** | Fast charging (subject to upgrade hysteresis, F8/#172) |
| 6 | slope > 0 mV/h | **CONSERVATIVE** | Gentle charging / stable |
| 7 | (default) | **CONSERVATIVE** | Stable or slight discharge |

Note what this means: at 4.6 V with a flat slope the model stays CONSERVATIVE —
there is no mid-level voltage derating. Between the 4.3 V floor and full charge,
only **slope and time-to-critical** move the mode. The selected mode then passes
through the hysteresis machine: downgrades apply immediately, upgrades need
consecutive time-separated confirmations of the same target.

**Voltage override at <4.3V (raw) forces SURVIVAL mode regardless of slope or history.**

---

## Key Insights

### 1. Slope and Time-to-Critical Decide; Voltage Only Floors

- **Any voltage ≥ 4.3V raw**: slope thresholds and direction-aware predictions
  decide the mode (see Decision Cascade)
- **Below 4.3V raw**: the voltage floor dominates — SURVIVAL regardless of slope

### 2. Time-to-Critical is Most Important

The system prioritizes **avoiding shutdown** over maximizing data collection
(predictive emergencies both require an actual discharge slope — a static low
reading does not panic the system):

```
If (slope < -30 mV/h AND time_to_critical < 6 hours):
    → SURVIVAL cadence (60 min; admitted wakes still require GNSS)

If (slope < -15 mV/h AND time_to_critical < 12 hours):
    → RECOVERY cadence (30 min; admitted wakes still require GNSS)
```

This ensures the device **stays alive** even in worst-case scenarios.

### 3. The System is Asymmetric

| Direction | Behavior |
|-----------|----------|
| **Charging** | Gradual mode upgrades as slope proves sustained |
| **Discharging** | Immediate downgrades when critical threshold hit |

This **errs on the side of caution** - quick to protect, slow to trust.

---

## Troubleshooting

### Problem: Slope shows 0.0 mV/h constantly

**Cause:** First sample or BaselineTimestamp = 0
**Fix:** Wait 10-20 minutes for second sample

### Problem: Slope shows huge values (>1000 mV/h)

**Cause:** Comparing real voltage against uninitialized memory (zeros)
**Fix:** Firmware now initializes both baseline and current to first sample

### Problem: Time to Target always 0.0h

**Possible causes:**
1. Battery at exactly 4.5V or 5.5V (already at target)
2. Slope is zero (stable voltage)
3. Moving away from both targets (e.g., at 5.0V, discharging)

**Expected behavior (STAB-08/#155):** the predict functions return an explicit
`PredictionState_t` — `PRED_AT_OR_PAST`, `PRED_STABLE`, `PRED_MOVING_AWAY`, or
`PRED_REACHABLE` — and the hours output is meaningful only for `PRED_REACHABLE`
(clamped at 9999 h). The old API folded "already past" and "never" into the
same sentinel; the current one cannot.

### Problem: Mode never changes

**Check real-time conditions:**
- Downgrades apply on the next decision when slope thresholds are crossed
- Upgrades need consecutive confirmed proposals (F8/#172) — a single good
  cycle is not enough, by design
- Raw-voltage override (<4.3V raw, R10/#37) forces SURVIVAL mode instantly

**Expected behavior:**
- Slope > +20mV/h → NORMAL mode (after confirmation)
- Slope < -5mV/h → REDUCED mode
- Slope < -15mV/h and time-to-critical < 12h → RECOVERY mode
- Slope < -30mV/h and time-to-critical < 6h → SURVIVAL mode

---

## Design Philosophy

### Real-Time > Historical

- **Current conditions decide** - Real-time voltage slope analysis
- **Asymmetric adaptation** - Downgrades within 1 decision cycle; upgrades after consecutive confirmation (F8/#172)
- **Deliberate upgrade lag** - A single noisy sample must not unlock higher power draw; protection never waits, optimism must be earned

### Conservative > Aggressive

- Default mode: CONSERVATIVE
- Immediate protection when voltage/slope conditions deteriorate
- Gradual upgrades when charging conditions prove sustained
- Better to transmit less than to die early

### Simple > Complex

- 2-value baseline tracking plus a small RAM-only hysteresis machine (`VoltageSlope_t`: baseline, current, last slope, committed mode, upgrade streak) instead of a 12-slot buffer (74 bytes)
- mV/hour instead of mAh/hour (no calibration needed)
- Linear slope calculation (works well with LTO's flat discharge curve)
- Real-time decision making without complex historical analysis

---

## Future Enhancements

### Considered but NOT Implemented

1. **Dual-timescale (short + long-term slope)** - Adds complexity, lag is feature not bug
2. **mAh/hour energy tracking** - Requires calibration, mV/hour is sufficient
3. **Instantaneous mode switching** - Causes thrashing; upgrades are hysteresis-confirmed instead (F8/#172)
4. **Outlier rejection/clamping** - Not needed with proper initialization

### Potential Additions

1. **Flash logging of daily profiles** - Track performance over weeks/months
2. **Seasonal calibration** - Different thresholds for summer vs winter
3. **Altitude compensation** - Solar efficiency varies with altitude
4. **Historical trend analysis** - Optional long-term performance tracking

---

## Summary

**Current Implementation:**
- ✅ **Real-time voltage slope tracking** with 2-hour baseline window
- ✅ **Temperature compensation** for accurate measurements (-65°C to +25°C)
- ✅ **LTO battery optimization** with correct voltage thresholds (4.3V critical, 5.5V full)
- ✅ **Asymmetric mode switching** — immediate downgrades, hysteresis-confirmed upgrades (F8/#172, S-E/#214)
- ✅ **Emergency protection** with raw-voltage override (<4.3V raw forces SURVIVAL mode, R10/#37)

**System Characteristics:**
- **Simplified Architecture**: baseline/current tracking plus a small RAM-only hysteresis machine instead of complex circular buffers
- **Asymmetric Responsive**: downgrades within 1 decision cycle; upgrades earned over consecutive cycles
- **Temperature Aware**: voltage compensation plus one first-flight FULL-or-SLEEP temperature admission threshold
- **Battery Protective**: Conservative defaults with immediate emergency response
- **Telemetry Rich**: Channels 11, 12, 14 provide slope, time-to-target, and mode data

**Result:**
- ✅ Voltage slopes are **realistic and stable** (typically ±10-50 mV/h)
- ✅ System **protects immediately and upgrades cautiously** as battery conditions change
- ✅ **LTO chemistry fully supported** with appropriate thresholds
- ✅ Code is **simple and maintainable** with clear real-time decision logic
- ✅ **Battery protection prioritized** over data collection frequency
