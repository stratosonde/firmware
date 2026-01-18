# Power Management System

## Overview

The firmware implements a **real-time adaptive power management system** optimized for extreme temperature operation (-65°C to +25°C). The system continuously monitors battery voltage slope and immediately adjusts transmission intervals and GPS usage to optimize battery life.

**Key Features:**
- Temperature-compensated voltage measurements (accurate across -65°C to +25°C)
- Real-time voltage slope calculation (+charging, -discharging in mV/hour)
- GPS temperature lockout (supercap hardware constraint at -55°C)
- Five adaptive operating modes (5min to 60min intervals)
- Predictive time-to-critical and time-to-full calculations
- **Immediate mode switching** based on real-time conditions

---

## Operating Modes

| Mode | TX Interval | GPS | Temperature | When Used |
|------|-------------|-----|-------------|-----------|
| **NORMAL** | 5 min | ✅ Yes* | >-55°C | Fast charging (slope > +20 mV/h) |
| **CONSERVATIVE** | 10 min | ✅ Yes* | >-55°C | Stable/slow charge (default mode) |
| **REDUCED** | 15 min | ❌ No | Any | Slow discharge (slope < -5 mV/h) |
| **RECOVERY** | 30 min | ❌ No | Any | Moderate discharge (slope < -15 mV/h) |
| **SURVIVAL** | 60 min | ❌ No | Any | Critical voltage or fast discharge |

*GPS automatically disabled if temperature < -55°C (supercap fails)

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
1:00pm    5155mV   3400mV +25 mV/h   NORMAL       Slope > 20mV/h → immediate upgrade
4:00pm    5192mV   3600mV +22 mV/h   NORMAL       Peak charging continues
6:00pm    5200mV   1000mV +8 mV/h    CONS         Slope < 20mV/h → downgrade
8:00pm    5195mV   0mV    -3 mV/h    CONS         Light discharge
11:00pm   5185mV   0mV    -7 mV/h    REDUCED      Slope < -5mV/h → immediate switch
```

### Real-Time Responsiveness

The system **immediately responds** to changing slope conditions:
- **Discharge detection**: Slope < -5mV/h triggers REDUCED mode within 1 transmission cycle
- **Fast charging**: Slope > +20mV/h immediately upgrades to NORMAL mode
- **Emergency conditions**: Voltage < 4.3V or time-to-critical < 6h forces SURVIVAL mode instantly

### No Lag by Design

Unlike historical systems, this **real-time approach** ensures:
- **Immediate protection** when battery conditions deteriorate
- **Quick optimization** when charging conditions improve
- **Responsive adaptation** to rapidly changing weather/solar conditions

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

## Voltage-Specific Behavior

### Real-Time Decision Matrix

The system behavior changes based on **current voltage** and **slope direction**:

#### At 5.5V (Fully Charged)
| Slope | Time to Critical | Time to Full | Mode Decision | Reason |
|-------|------------------|--------------|---------------|---------|
| +20 mV/h | N/A (charging) | 0h (at target) | CONSERVATIVE | Already full, can't charge more |
| 0 mV/h | N/A (stable) | 0h (at target) | CONSERVATIVE | Stable and full |
| -10 mV/h | 100h | N/A (discharging) | CONSERVATIVE | Lots of capacity, slow discharge OK |

#### At 5.2V (80% Capacity)
| Slope | Time to Critical | Time to Full | Mode Decision | Reason |
|-------|------------------|--------------|---------------|---------|
| +20 mV/h | N/A | 15h | **NORMAL** | Fast charging, upgrade mode |
| +5 mV/h | N/A | 60h | CONSERVATIVE | Slow charging, stay cautious |
| 0 mV/h | N/A | N/A | CONSERVATIVE | Stable, good capacity |
| -10 mV/h | 70h | N/A | CONSERVATIVE | Slow discharge, lots of time |

#### At 5.0V (60% Capacity - Normal Operation)
| Slope | Time to Critical | Time to Full | Mode Decision | Reason |
|-------|------------------|--------------|---------------|---------|
| +30 mV/h | N/A | 17h | **NORMAL** | Fast charging, plenty of time |
| +10 mV/h | N/A | 50h | CONSERVATIVE | Gentle charging |
| 0 mV/h | N/A | N/A | CONSERVATIVE | Stable at mid-range |
| -10 mV/h | 50h | N/A | CONSERVATIVE | Slow discharge, monitor |
| -20 mV/h | 25h | N/A | REDUCED | Moderate discharge, slow down |

#### At 4.8V (40% Capacity - Getting Low)
| Slope | Time to Critical | Time to Full | Mode Decision | Reason |
|-------|------------------|--------------|---------------|---------|
| +20 mV/h | N/A | 35h | **NORMAL** | Charging back up |
| 0 mV/h | N/A | N/A | CONSERVATIVE | Stable but low capacity |
| -10 mV/h | 30h | N/A | REDUCED | Discharging with limited time |
| -30 mV/h | 10h | N/A | **RECOVERY** | Fast discharge (15 mV/h threshold) |

#### At 4.6V (20% Capacity - Low Battery)
| Slope | Time to Critical | Time to Full | Mode Decision | Reason |
|-------|------------------|--------------|---------------|---------|
| +20 mV/h | N/A | 45h | CONSERVATIVE | Charging but from low point |
| 0 mV/h | N/A | N/A | REDUCED | Stable but very low |
| -10 mV/h | 10h | N/A | **RECOVERY** | Limited time to critical |
| -30 mV/h | 3h | N/A | **SURVIVAL** | Emergency! <6h to critical |

#### At 4.5V (5% Capacity - Critical)
| Slope | Time to Critical | Time to Full | Mode Decision | Reason |
|-------|------------------|--------------|---------------|---------|
| +10 mV/h | N/A | 100h | REDUCED | Charging from critical, be cautious |
| 0 mV/h | 0h (at target) | N/A | **RECOVERY** | At critical threshold |
| -10 mV/h | Already critical | N/A | **SURVIVAL** | Below threshold |

#### At 4.3V (1% Capacity - Emergency)
| Slope | Time to Critical | Time to Full | Mode Decision | Reason |
|-------|------------------|--------------|---------------|---------|
| Any | Below threshold | N/A | **SURVIVAL** | Voltage override! |

**Voltage override at <4.3V forces SURVIVAL mode regardless of slope or history.**

---

## Key Insights

### 1. Voltage Level Matters More at Extremes

- **Above 5.0V**: Slope is primary factor (plenty of capacity buffer)
- **4.5-5.0V**: Both voltage and slope matter (moderate capacity)
- **Below 4.5V**: Voltage dominates (critical threshold triggering)

### 2. Time-to-Critical is Most Important

The system prioritizes **avoiding shutdown** over maximizing data collection:

```
If (time_to_critical < 6 hours):
    → SURVIVAL mode (60 min intervals, no GPS)
    
If (time_to_critical < 12 hours):
    → RECOVERY mode (30 min intervals, no GPS)
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

**Expected behavior:**
- At 4.8V discharging → Negative hours (time to critical)
- At 5.2V charging → Positive hours (time to full)
- At 5.0V stable → 0h (not moving toward either)

### Problem: Mode never changes

**Check real-time conditions:**
- Mode changes **immediately** when slope thresholds are crossed
- Emergency voltage override (<4.3V) forces SURVIVAL mode instantly
- Time-to-critical calculations trigger mode changes in real-time

**Expected behavior:**
- Slope > +20mV/h → NORMAL mode
- Slope < -5mV/h → REDUCED mode  
- Slope < -15mV/h → RECOVERY mode
- Time-to-critical < 6h → SURVIVAL mode

---

## Design Philosophy

### Real-Time > Historical

- **Current conditions decide immediately** - Real-time voltage slope analysis
- **Responsive adaptation** - Mode changes within 1 transmission cycle
- **No lag by design** - Battery protection happens in real-time when conditions change

### Conservative > Aggressive

- Default mode: CONSERVATIVE
- Immediate protection when voltage/slope conditions deteriorate
- Gradual upgrades when charging conditions prove sustained
- Better to transmit less than to die early

### Simple > Complex

- 2-value tracking (12 bytes) instead of 12-slot buffer (74 bytes)
- mV/hour instead of mAh/hour (no calibration needed)
- Linear slope calculation (works well with LTO's flat discharge curve)
- Real-time decision making without complex historical analysis

---

## Future Enhancements

### Considered but NOT Implemented

1. **Dual-timescale (short + long-term slope)** - Adds complexity, lag is feature not bug
2. **mAh/hour energy tracking** - Requires calibration, mV/hour is sufficient
3. **Instantaneous mode switching** - Causes thrashing, defeats the purpose
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
- ✅ **Immediate mode switching** based on real-time slope and voltage conditions
- ✅ **Emergency protection** with voltage override (<4.3V forces SURVIVAL mode)

**System Characteristics:**
- **Simplified Architecture**: 2-value tracking (12 bytes) instead of complex circular buffers
- **Real-Time Responsive**: Mode changes within 1 transmission cycle of threshold crossing
- **Temperature Aware**: GPS lockout at -55°C, voltage compensation to -65°C
- **Battery Protective**: Conservative defaults with immediate emergency response
- **Telemetry Rich**: Channels 11, 12, 14 provide slope, time-to-target, and mode data

**Result:**
- ✅ Voltage slopes are **realistic and stable** (typically ±10-50 mV/h)
- ✅ System **responds immediately** to changing battery conditions
- ✅ **LTO chemistry fully supported** with appropriate thresholds
- ✅ Code is **simple and maintainable** with clear real-time decision logic
- ✅ **Battery protection prioritized** over data collection frequency
