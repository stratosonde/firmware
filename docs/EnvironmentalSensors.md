# Environmental Sensors

## Overview

Environmental sensing is the `sys_sensors.c` aggregation layer over three
drivers — `ms5607.c` (pressure/temperature), `sht31.c` (temperature/humidity),
`adc_if.c` (battery/solar/regulator) — plus GNSS merge from `atgm336h.c`.
Its product is `sensor_t` (`Core/Inc/sys_sensors.h`): one coherent observation
where **every field carries its own freshness** (DDR-0003 data honesty).

## Hardware

- **MS5607** pressure + temperature, I2C2 (PA11 SDA / PA12 SCL; driver range
  check 1–1200 hPa, -90…+85 °C; compensation includes the `temp < -1500`
  very-low-temperature branch most drivers omit)
- **SHT31** temperature + humidity, I2C2
- **ADC** battery, solar panel, and VDDA/regulator rails
- **GNSS** fields merge from the ATGM336H handle (no I2C traffic —
  `EnvSensors_MergeGnss`, FR-15/#96)

## The Honesty Contract (T2/DDR-0003)

```c
typedef struct {
  float pressure, temperature, humidity;
  int32_t latitude, longitude;   // binary scaled
  int32_t altitudeGps;           // int32 since D5/#35 (float alt >32767 m overflowed int16)
  /* altitudeBar DELETED (D5/#35): never assigned; backend computes
     barometric altitude from pressure+temperature */
  uint8_t satellites, gnss_fix_quality;  float gnss_hdop;  bool gnss_valid;
  float battery_voltage, regulator_voltage, solar_voltage;
  uint8_t temp_stale, hum_stale, press_stale, gnss_stale, batt_stale;
} sensor_t;
```

A stale bit means "last-known-good (or default), **not** a live read". The
bits land in the flash record's flags byte (b0–b4) and the wire
`sensor_quality` byte. **Omission is preferable to fabrication** (SI-004).

## Plausibility-Gate-Before-Cache (R28/#36, #136)

Every channel follows the same discipline — a corrupt read must never poison
the last-known-good cache that stale mode later serves:

1. Driver-level gate: MS5607 rejects zero/tiny ADC words and out-of-range
   results as hard errors (never "calculate anyway").
2. Aggregation gate: `sys_sensors.c` range-checks **before** the cache accepts
   (SHT31 −90…+85 °C / 0–100 %; MS5607 1–1200 hPa, defense in depth).
3. Battery (`#136`): the one sensor feeding the power state machine has the
   same gate + LKG cache + stale bit in `adc_if.c` (`SYS_GetBatteryVoltage`,
   `SYS_BatteryIsStale`) — a rejected read serves the cache; with no history
   it returns 0, the conservative direction (raw < 4300 mV floor → SURVIVAL,
   never a fantasy full battery).
4. **Solar is the exception**: no gate, no stale bit → tracked as **#279**.

`EnvSensors_Read` returns a freshness bitmask (`ENV_SENSORS_FRESH_*`,
F-030/#59); `EnvSensors_MarkGnssStale` / `EnvSensors_GnssIsStale` carry GNSS
provenance (F-3/#178 — a stale position may **inhibit** RF but never **switch**
region, DDR-0015).

## Same-Moment Coherence (LT-03/#271)

The transmit cycle reads sensors twice by design:

1. **Pre-acquisition** read — feeds the power/plan decision.
2. **Post-acquisition re-sample** — after the GNSS fix (acquisition can run up
   to 255 s; at 5 m/s ascent that is ~300 m of altitude skew), so the archived
   record pairs same-moment environment and position (SI-006). Timestamp is
   taken fresh at write time.

FR-15 (#96) had removed this re-read as "no new information"; LT-03 refuted
that across the acquisition gap — the code comment explicitly warns not to
re-apply FR-15's reasoning. `EnvSensors_MergeGnss` remains the no-I2C way to
merge a fix into an existing sample.

## What This Module Does NOT Do

- **No altitude calculation**: barometric altitude is derived backend-side
  from pressure+temperature (documented design; `altitudeBar` deleted D5/#35).
- **No ascent detection**: launch/float detection is the pressure-reference
  state machine in `mission_logic.c` (STAB-06/#153, R3-10/#222), not a sensor
  concern.
- **No per-mode sampling table**: cadence comes from the mission phase and
  power model (ASCENT 10 s, FLOAT 5 min, power-mode intervals) — there are no
  sensor-local measurement schedules.
- **No sensor power gating GPIOs**: the I2C sensors idle at µA; power strategy
  is the wake-cycle cadence itself.

## Error Handling

Bounded retries at the driver level; a failed read serves the stale cache and
sets the honesty bit; an uninitialized sensor is a hard error, never an I2C
probe with garbage calibration (R28). A stuck bus gets the F20 9-clock + STOP
recovery (`I2C_BusRecover`), with per-read bus-health tracking
(`I2C_NoteResult`). Recovery is aggressive, bounded, and forgetful (SI-013):
normal operation resumes immediately on the next good read — no probation, no
escalation.

## Cross-References

- `docs/PowerManagement.md` — cadence authority
- `docs/GNSSModule.md` — the fix pipeline that merges here
- DDR-0003 (honesty), DDR-0009 (bounded recovery), DDR-0023 (integrity)