# LoRaWAN Payload Formats - Stratosonde Telemetry

## Overview

The Stratosonde firmware transmits data using multiple LoRaWAN packet formats optimized for different purposes. This document provides complete specifications for backend implementation.

---

## LoRaWAN Port Assignments

| Port | Packet Type | Size | Purpose | Status |
|------|-------------|------|---------|--------|
| **2** | CayenneLPP | Variable | Human-readable debug format | Development only |
| **3** | GNSS Detail | Variable | Satellite tracking data | Development only |
| **10** | Compact Binary | 11 bytes | Production telemetry (SF10) | **PRODUCTION** |
| **11** | Bulk Binary | Variable (v7, `0x07`: `6 + 40n`) | Core science archive transfer (SF7) | **PRODUCTION** |

### Debug Packet Control

Debug packets (ports 2 and 3) are **compiled out of flight builds** (defaults
below); they are opt-in for bench/commissioning builds:
```c
#define ENABLE_DEBUG_LPP           0  // 0 = Default OFF for flight builds
#define ENABLE_GNSS_DETAIL_PACKET  0  // 0 = Default OFF for flight builds
#define DEBUG_LPP_TX_INTERVAL      5  // Send debug every 5th TX (when enabled)
```

---

## PORT 10: Mission Heartbeat v2 (PRODUCTION)

### Description
Ultra-compact **11-byte** heartbeat for maximum-range transmission (SF10; SF9 in US915/AU915 per D1). **All multibyte fields are little-endian** (D9 — LE is wire truth; earlier revisions of this document incorrectly described big-endian). Wire format version: **v2** (2026-08-06, issue #33). v1 shared port and length but is not decodable as v2 — v1 never flew; discriminate by deployment epoch and the golden vectors in `tests/host/test_main.c`.

### Packet Structure (11 bytes, heartbeat v2)

| Offset | Field | Type | Size | Resolution | Range | Description |
|--------|-------|------|------|------------|-------|-------------|
| 0 | Timestamp | uint16 LE | 2 | 1 minute | 0-45.5 days | Minutes since epoch (wraps; see status bit 5) |
| 2 | Latitude | int16 LE | 2 | ~300 m | ±90° | Full-range scale: deg = value × 90 / 32767 |
| 4 | Longitude | int16 LE | 2 | ~550 m | ±180° | Full-range scale: deg = value × 180 / 32767 |
| 6 | Temperature | int8 | 1 | 2°C | -64 to +63.5°C | (value - 64) × 2 |
| 7 | Pressure + Humidity | uint16 LE | 2 | 1 hPa / 5% | 0-2046 hPa / 0-100% | bits 0-10 pressure hPa (2047 = invalid); bits 11-15 humidity 5%-units (31 = invalid) |
| 9 | Battery | uint8 | 1 | 50 mV | 0-12.75V | value × 0.050 |
| 10 | Status v2 | uint8 | 1 | — | — | bit table below |

Status v2 bits:

| Bits | Meaning |
|------|---------|
| 0 | GPS stale (position is last-known-good) |
| 1 | Temperature stale |
| 2 | Humidity stale |
| 3 | Pressure stale |
| 4 | RTC GNSS-disciplined this cycle (timestamp is UTC-traceable) |
| 5 | timestamp has wrapped its 45.5-day uint16 range |
| 6-7 | Mission state: 0=COMMISSIONING, 1=ASCENT, 2=FLOAT, 3=reserved (matches `MissionState_t`, STAB-10/#157) |

**Note**: Altitude is NOT transmitted - it must be calculated on the backend from pressure + temperature using the barometric formula.

### Encoding Formulas

#### Timestamp (uint16, minutes since epoch)
```
encoded_value = (unix_seconds / 60) & 0xFFFF
```
Wraps every 65,535 minutes (~45.5 days). Use context from previous packets to handle wraparound.

Stamped only via `Payload_TimestampMinutesNow()` (SysTime UTC epoch, GNSS-disciplined).
SP-11/SP-12 (#250): previously the TX path stamped RTC-tick uptime minutes and the encoder's
`timestamp_min == 0` sentinel substituted epoch minutes - one field, two clock domains. RTC
uptime never reaches the wire now; 0 is simply a legal value at the wrap boundary.

#### Latitude / Longitude (int16 LE, full-range scale)
```python
# Encoding (firmware):
lat_enc = clamp(round(lat_deg * 32767 / 90),  -32768, 32767)
lon_enc = clamp(round(lon_deg * 32767 / 180), -32768, 32767)

# Decoding (backend):
lat_deg = struct.unpack('<h', payload[2:4])[0] * 90.0 / 32767.0
lon_deg = struct.unpack('<h', payload[4:6])[0] * 180.0 / 32767.0
```

#### Temperature (int8, 2°C resolution with offset)
```python
# Encoding (firmware):
encoded_value = round(temp_celsius / 2.0) + 64

# Decoding (backend):
temp_celsius = (byte_value - 64) × 2.0
```

#### Pressure + Humidity (uint16 LE packed, bytes 7-8)
```python
press_hum = struct.unpack('<H', payload[7:9])[0]
press_raw = press_hum & 0x07FF          # bits 0-10
hum_raw   = (press_hum >> 11) & 0x1F    # bits 11-15
pressure_hpa = None if press_raw == 0x07FF else float(press_raw)  # 1 hPa units; stratospheric-useful (v1 collapsed below 950 hPa)
humidity_pct = None if hum_raw == 0x1F else hum_raw * 5.0         # 5% units
```

#### Battery Voltage (uint8, 50 mV resolution)
```python
# Encoding (firmware):
encoded_value = round(voltage_volts / 0.050)

# Decoding (backend):
voltage_volts = payload[9] * 0.050
```

### Python Decoder Example (heartbeat v2)

```python
import struct

MISSION_STATES = ["COMMISSIONING", "ASCENT", "FLOAT", "RESERVED"]  # STAB-10 (#157): matches MissionState_t

def decode_heartbeat_v2(payload: bytes) -> dict:
    """Decode 11-byte heartbeat v2 from LoRaWAN Port 10 (little-endian)."""
    if len(payload) != 11:
        raise ValueError(f"Expected 11 bytes, got {len(payload)}")

    timestamp_min = struct.unpack('<H', payload[0:2])[0]
    lat_enc, lon_enc = struct.unpack('<hh', payload[2:6])
    temp_raw = payload[6]
    press_hum = struct.unpack('<H', payload[7:9])[0]
    battery_raw = payload[9]
    status = payload[10]

    press_raw = press_hum & 0x07FF
    hum_raw = (press_hum >> 11) & 0x1F

    return {
        'timestamp_minutes': timestamp_min,
        'latitude': lat_enc * 90.0 / 32767.0,
        'longitude': lon_enc * 180.0 / 32767.0,
        'temperature_c': (temp_raw - 64) * 2.0,
        'pressure_hpa': None if press_raw == 0x07FF else float(press_raw),
        'battery_v': battery_raw * 0.050,
        'humidity_pct': None if hum_raw == 0x1F else hum_raw * 5.0,
        'gps_stale': bool(status & 0x01),
        'temp_stale': bool(status & 0x02),
        'hum_stale': bool(status & 0x04),
        'press_stale': bool(status & 0x08),
        'time_gnss_disciplined': bool(status & 0x10),
        'timestamp_wrapped': bool(status & 0x20),
        'mission_state': MISSION_STATES[(status >> 6) & 0x03],
    }

# Example usage
if __name__ == "__main__":
    # ts=1234 min, 45.0N, 114.0W, 25C, 1013 hPa, 45%, 5.0V, GNSS-disciplined
    example = bytes([
        0xD2, 0x04,  # Timestamp: 1234 minutes (LE)
        0x00, 0x40,  # Latitude: 16384 -> 16384*90/32767 = 45.0 deg (LE)
        0x50, 0xAE,  # Longitude: -20912 -> -114.48 deg (LE)
        0x4C,        # Temperature: (76-64)*2 = 24C... (see golden vector for exact)
        0x00, 0x00,  # press_hum placeholder
        0x64,        # Battery: 100*0.05 = 5.0V
        0x10         # Status: GNSS-disciplined time
    ])
    # Authoritative byte-level vectors: tests/host/test_main.c prints
    # "GOLDEN heartbeat-v2:" from the real firmware encoder.
    decoded = decode_heartbeat_v2(example)
    print("Heartbeat v2 decoded:")
    for key, value in decoded.items():
        print(f"  {key}: {value}")
```

### Altitude Calculation (Backend)

Since altitude is not transmitted, calculate it from pressure and temperature:

```python
import math

def calculate_altitude_from_pressure(pressure_hpa: float, temp_c: float, sea_level_pressure: float = 1013.25) -> float:
    """
    Calculate altitude from barometric pressure using hypsometric formula
    
    Args:
        pressure_hpa: Station pressure in hPa
        temp_c: Temperature in Celsius
        sea_level_pressure: Reference sea level pressure (default 1013.25 hPa)
        
    Returns:
        Altitude in meters
    """
    # Hypsometric formula (international standard atmosphere)
    temp_k = temp_c + 273.15
    altitude_m = ((sea_level_pressure / pressure_hpa) ** (1/5.257) - 1) * temp_k / 0.0065
    
    return altitude_m

# Example
altitude = calculate_altitude_from_pressure(886.4, 17.9)
print(f"Altitude: {altitude:.1f} m")  # Should be ~1078m (Calgary elevation)
```

---

---

## PORT 20: Version Report (PRODUCTION)

### Description

Firmware and wire-format announce frame (A-005/STAB-11/F-09 — #79/#158/#266).
Because firmware cannot be updated in flight, the wire-schema discriminator is the
**known firmware version**, announced explicitly once at commissioning and once at
first-flight admission — not deployment folklore. The backend maps firmware
version to the expected wire layout for every subsequent frame.

### Packet Structure (12 bytes, little-endian)

| Offset | Field | Type | Size | Description |
|---|---|---|---|---|
| 0 | magic | u8 | 1 | `0x56` 'V' |
| 1 | fw major | u8 | 1 | firmware semantic version |
| 2 | fw minor | u8 | 1 | |
| 3 | fw patch | u8 | 1 | |
| 4 | format version | u8 | 1 | heartbeat wire version (e.g. 2) |
| 5 | stage | u8 | 1 | bit0 = commissioning, bit1 = flight admission |
| 6–9 | mission minutes | u32 LE | 4 | `Payload_TimestampMinutesNow()` basis |
| 10–11 | CRC16 | u16 LE | 2 | CRC-16CCITT over bytes 0–9 |

Announced on `LORAWAN_VERSION_PORT` (20) via the packet queue, unconfirmed —
acknowledge-or-lose is the same as any diagnostic frame; the next commissioning
wake or flight start re-announces if the queue was full.

---


## PORT 11: Core Science Archive (PRODUCTION)

### Description
Bulk transfer of historical high-resolution records at SF7, sent when link quality is good and battery is sufficient. Records are FIFO order (oldest unsent first). **All multibyte fields are little-endian** (D9 — earlier revisions of this document incorrectly described big-endian; N-01/N-02). The decoder branches on `payload[0]`:

| `payload[0]` | Format | Length |
|---|---|---|
| `0x01` | Legacy v1, 222 B fixed (historical only) | 222 |
| `0x02` | Legacy v2, 198 B fixed (FW-20) | 198 |
| `0x03` | Variable, no record identity (CI-only <1 day, never deployed — superseded) | 2+32n+4 |
| `0x04` | v4, variable `6 + 32n + 4` with base sequence (never deployed — superseded by v5, FR-07/#87) | 42-202 |
| `0x05` | v5, variable `6 + 36n` with per-record explicit sequence (FR-07/#87; **never deployed** — superseded by v6 before the first real burst, STAB-04/#151) | 42-186 |
| `0x06` | v6, variable `6 + 38n` with per-record sequence + data-honesty provenance (STAB-04/#151; superseded by v7) | 44-196 |
| `0x07` | **Current v7, variable** `6 + 40n` with per-record sequence + provenance + dual battery (resting + loaded) | 46-166 |

### Packet Structure v7 (variable length)

#### Header (2 bytes)

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 0 | Packet Type | uint8 | 1 | 0x07 = variable-length + per-record identity + provenance + dual battery |
| 1 | Record Count | uint8 | 1 | n records (1-4) |

n complete 40-byte wire records follow immediately at offset 2, each being
`sequence uint32 LE` + the 36-byte v7 high-resolution record (layout below,
explicitly little-endian on the wire). The packet ends with a 4-byte CRC32
(LE) over all preceding bytes: total length `6 + 40n`.

**Identity is explicit per record (FR-07, #87).** v4 derived identity
implicitly (`base_seq + i`), which misattributed every record after a skipped
one (corrupt flash slot or failed conversion compacting the candidate array).
v5/v6 serialize each record's own flash sequence, so the ground-side
`(DevEUI, sequence)` dedup key (DDR-0005) is correct for ANY subset of the
archive, and the sender's watermark commit point is exactly
`sequence_of(last packed record) + 1` (FR-08, #91).

**Provenance survives the archive hop (STAB-04, #151).** v6 adds the
`sensor_quality` and `veto_reason` bytes so a stale/fallback measurement
stored honestly in flash can never arrive at the backend looking fresh
(DDR-0007), and the flags byte's power-mode field is the HISTORICAL mode
(STAB-05, #152).

The firmware queries the runtime payload budget before each packet (`LoRaMacQueryTxPossible` — current DR plus pending FOpts) and packs as many complete records as fit, walking the archive **newest-to-oldest** (the one-pass recovery walker, DDR-0005); records cut by the budget remain pending and are retransmitted next cycle (at-least-once, DDR-0005).

### Packet Structure v4 (variable length, SUPERSEDED — never deployed)

#### Header (6 bytes)

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 0 | Packet Type | uint8 | 1 | 0x04 = variable-length + identity |
| 1 | Record Count | uint8 | 1 | n records (1-6) |
| 2 | Base Sequence | uint32 LE | 4 | Flash sequence of the FIRST record — record i has identity `base_seq + i` (DDR-0005; backend dedups on (DevEUI, sequence)) |

n complete 32-byte records follow immediately at offset 6 (layout below, explicitly little-endian on the wire). The packet ends with a 4-byte CRC32 (LE) over all preceding bytes: total length `6 + 32n + 4`.

#### High-Resolution Records (n × 36 bytes in v7; n × 34 bytes in v6; n × 32 bytes in v5 and earlier)

Each v7 record is 36 bytes, little-endian:

| Offset | Field | Type | Size | Resolution | Range | Description |
|--------|-------|------|------|------------|-------|-------------|
| 0 | Timestamp | uint32 LE | 4 | 1 second | Full range | Unix timestamp (seconds) |
| 4 | Latitude | int32 LE | 4 | 1e-7° | Full | GPS binary format |
| 8 | Longitude | int32 LE | 4 | 1e-7° | Full | GPS binary format |
| 12 | Altitude | uint16 LE | 2 | 1 meter | 0-65535m | GPS altitude |
| 14 | Temperature | int16 LE | 2 | 0.1°C | ±3276.7°C | temp × 10 |
| 16 | Humidity | uint16 LE | 2 | 0.1% | 0-6553.5% | humidity × 10 |
| 18 | Pressure | uint16 LE | 2 | 0.1 hPa | 0-6553.5 hPa | pressure × 10 |
| 20 | Battery Voltage | uint16 LE | 2 | 1 mV | 0-65.535V | **LOADED** — post-GNSS, receiver hot |
| 22 | Battery Rest | uint16 LE | 2 | 1 mV | 0-65.535V | **v7: RESTING** — pre-GNSS, receiver off |
| 24 | Solar Voltage | uint16 LE | 2 | 1 mV | 0-65.535V | Millivolts |
| 26 | Voltage Slope | int16 LE | 2 | 1 mV/hour | ±32.767 V/h | Reserved (sentinel −32768, PWR-SIMPLIFY) |
| 28 | Satellites | uint8 | 1 | 1 | 0-255 | GPS satellite count |
| 29 | HDOP | uint8 | 1 | 0.1 | 0-25.5 | HDOP × 10 |
| 30 | Power Mode | uint8 | 1 | enum | 0-7 | Operating mode at write time |
| 31 | Flags | uint8 | 1 | bitfield | - | Status flags (table below) |
| 32 | Sensor Quality | uint8 | 1 | bitfield | - | per-sensor staleness (b0-b4) + battery load-phase (b5-b6) |
| 33 | Veto Reason | uint8 | 1 | enum | 0-6 | TransmitVeto_t at write time (0 = none) |
| 34 | CRC16 | uint16 LE | 2 | - | - | CRC16/MODBUS over record bytes 0-33 (v6: bytes 0-31, CRC at 32) |

**Dual battery load phases.** `Battery Voltage` (offset 20) is the *loaded*
sample taken after GNSS acquisition (receiver hot, the heaviest real load);
`Battery Rest` (offset 22) is the *resting* sample taken at cycle start before
GNSS (receiver off, no TX) — the value Gate A/B admission decides on. Together
they yield the sag-vs-temperature curve used to derive the Gate B floor. The
`sensor_quality` byte carries the load-phase provenance in bits 5-6: `0` = rest
only, `1` = loaded only, `2` = both (expected). See
`SENSOR_QUALITY_LOAD_*` in `Core/Inc/payload_format.h`.

#### Status Flags (byte 29 of each record)

| Bit | Mask | Field | Description |
|-----|------|-------|-------------|
| 0 | 0x01 | GPS Valid | GPS fix valid (1) or invalid (0) |
| 1-4 | 0x1E | Satellite Count | Satellite count 0-15 (duplicates byte 26) |
| 5-7 | 0xE0 | Power Mode | Operating mode 0-7 at WRITE time (STAB-05/#152; duplicates byte 28 deliberately) |

#### Sensor Quality (byte 30 of each v6 record, STAB-04/#151)

| Bit | Mask | Field | Description |
|-----|------|-------|-------------|
| 0 | 0x01 | Pressure stale | Last-known-good MS5607 value |
| 1 | 0x02 | Temperature stale | Last-known-good SHT31 value |
| 2 | 0x04 | Humidity stale | Last-known-good SHT31 value |
| 3 | 0x08 | GNSS stale | Last-known-good position |
| 4 | 0x10 | Battery stale | Last-known-good ADC value (#136) |
| 5-7 | 0xE0 | Reserved | 0 |

#### Veto Reason (byte 31 of each v6 record, STAB-04/#151)

| Value | Meaning |
|------:|---------|
| 0 | VETO_NONE — cycle was a full go |
| 1 | VETO_TEMP_STALE — **DEPRECATED (RV-08/#164, DDR-0021): never produced by current firmware** (stale temperature now falls back to the raw battery reading); retain decoding for historical records |
| 2 | VETO_TEMP_LOCKOUT — **DEPRECATED (RV-08/#164, DDR-0021): never produced** (the cold lockout is removed); retain decoding for historical records |
| 3 | VETO_RF_SILENCE — FLIGHT with no valid session (DDR-0018) |
| 4 | VETO_RESTRICTED_REGION — regulatory RF prohibition |
| 5 | VETO_GPS_LOSS — GPS-loss silence, position stale beyond the 24 h budget (DR-06/#241, DDR-0015) |
| 6 | VETO_PRELAUNCH_QUIET — commissioned-but-not-launched quiet watch (DR-06/#241, DDR-0002/0018) |

#### Trailer (4 bytes)

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 2+38n (v6) / 2+36n (v5) / 6+32n (v4) | CRC32 | uint32 LE | 4 | CRC32/IEEE over all preceding bytes |

### Legacy v2 (packet_type 0x02, fixed 198 B)

Same header with type `0x02`, record count 1-6, six fixed 32-byte record slots at offsets 2..193 (unused slots zero), CRC32 at bytes 194-197 over bytes 0-193. v1 (`0x01`, 222 B) additionally carried three permanently-zero placeholder blocks (Flash Page Addr 4 B at offset 2, Voltage Trend 10 B and Mode Changes 10 B at 198-217) and its records started at offset 6 — retain decoding only while historical data requires it.

### Power Mode Enum

STAB-10 (#157) audit fix — the previous table (STARTUP/.../CRITICAL/GPS_LOCKOUT)
never matched the firmware. These are the `OperatingMode_t` values in
`Core/Inc/power_model.h`:

| Value | Mode | Description | TX Interval |
|-------|------|-------------|-------------|
| 0 | NORMAL | Normal operation | 300s (5 min) |
| 1 | CONSERVATIVE | Battery saving | 600s (10 min) |
| 2 | REDUCED | Power-model cadence preference; admitted science still acquires GNSS | 900s (15 min) |
| 3 | RECOVERY | Power-model cadence preference; admitted science still acquires GNSS | 1800s (30 min) |
| 4 | SURVIVAL | Slowest cadence preference; also used for low-admission retry | 3600s (60 min) |

Mission cadence (DDR-0002) overrides these only when the power model is
healthy (NORMAL/CONSERVATIVE): ASCENT = 10 s, FLOAT = 5 min.

### Python Decoder Example (v6 + legacy)

```python
import struct

RECORD_SIZE = 32          # v5 and earlier record
V6_RECORD_SIZE = 34       # v6 record (adds sensor_quality + veto_reason)
V5_RECORD_WIRE = 36       # seq u32 LE + 32-byte record (v5, never deployed)
V6_RECORD_WIRE = 38       # seq u32 LE + 34-byte record (v6)

def decode_archive_packet(payload: bytes) -> dict:
    """
    Decode core science archive packet from LoRaWAN Port 11 (LITTLE-ENDIAN).

    Branches on payload[0]:
      0x06 = v6 variable length: 6 + 38n bytes, per-record sequence + provenance (current firmware)
      0x05 = v5 variable length: 6 + 36n bytes, per-record sequence (never deployed)
      0x04 = v4 variable length: 6 + 32n + 4 bytes, base_seq header (never deployed)
      0x03 = v3 variable without identity (never deployed; superseded)
      0x02 = v2 legacy fixed 198 B
      0x01 = v1 legacy fixed 222 B (historical only)
    """
    if len(payload) < 2:
        raise ValueError("payload too short")

    version = payload[0]
    if version == 0x06:
        count = payload[1]
        expected = 2 + V6_RECORD_WIRE * count + 4
        if len(payload) != expected:
            raise ValueError(f"v6: expected {expected} bytes for {count} records, got {len(payload)}")
        base_seq = None  # identity is per-record in v6
        header_len = 2
        crc_off = len(payload) - 4
    elif version == 0x05:
        count = payload[1]
        expected = 2 + V5_RECORD_WIRE * count + 4
        if len(payload) != expected:
            raise ValueError(f"v5: expected {expected} bytes for {count} records, got {len(payload)}")
        base_seq = None  # identity is per-record in v5
        header_len = 2
        crc_off = len(payload) - 4
    elif version == 0x04:
        count = payload[1]
        expected = 6 + RECORD_SIZE * count + 4
        if len(payload) != expected:
            raise ValueError(f"v4: expected {expected} bytes for {count} records, got {len(payload)}")
        base_seq = struct.unpack('<I', payload[2:6])[0]
        header_len = 6
        crc_off = len(payload) - 4
    elif version == 0x03:
        count = payload[1]
        expected = 2 + RECORD_SIZE * count + 4
        if len(payload) != expected:
            raise ValueError(f"v3: expected {expected} bytes for {count} records, got {len(payload)}")
        base_seq = None
        header_len = 2
        crc_off = len(payload) - 4
    elif version == 0x02:
        if len(payload) != 198:
            raise ValueError(f"v2: expected 198 bytes, got {len(payload)}")
        count = min(payload[1], 6)
        base_seq = None
        header_len = 2
        crc_off = 194
    elif version == 0x01:
        if len(payload) != 222:
            raise ValueError(f"v1: expected 222 bytes, got {len(payload)}")
        count = min(payload[1], 6)
        base_seq = None
        header_len = 6
        crc_off = 218
    else:
        raise ValueError(f"Unknown archive packet type 0x{version:02X}")

    result = {'packet_type': version, 'record_count': count,
              'base_seq': base_seq, 'records': []}

    if version == 0x06:
        stride = V6_RECORD_WIRE
    elif version == 0x05:
        stride = V5_RECORD_WIRE
    else:
        stride = RECORD_SIZE
    for i in range(count):
        wire = payload[header_len + i * stride: header_len + (i + 1) * stride]
        if version in (0x05, 0x06):
            seq = struct.unpack('<I', wire[0:4])[0]
            rec = wire[4:]
        else:
            seq = (base_seq + i) if base_seq is not None else None
            rec = wire
        timestamp = struct.unpack('<I', rec[0:4])[0]
        lat, lon = struct.unpack('<ii', rec[4:12])
        alt = struct.unpack('<H', rec[12:14])[0]
        temp = struct.unpack('<h', rec[14:16])[0]
        humidity, pressure = struct.unpack('<HH', rec[16:20])
        bat_mv, solar_mv = struct.unpack('<HH', rec[20:24])
        slope = struct.unpack('<h', rec[24:26])[0]
        sats, hdop, mode, flags = struct.unpack('BBBB', rec[26:30])
        if version == 0x06:
            sensor_quality, veto_reason = struct.unpack('BB', rec[30:32])
            crc16 = struct.unpack('<H', rec[32:34])[0]
            crc16_cover = rec[0:32]
        else:
            sensor_quality, veto_reason = None, None  # provenance lost on the wire (STAB-04)
            crc16 = struct.unpack('<H', rec[30:32])[0]
            crc16_cover = rec[0:30]

        result['records'].append({
            'sequence': seq,
            'timestamp': timestamp,
            'latitude': lat * (90.0 / 8388607.0),
            'longitude': lon * (180.0 / 8388607.0),
            'altitude_m': alt,
            'temperature_c': temp / 10.0,
            'humidity_pct': humidity / 10.0,
            'pressure_hpa': pressure / 10.0,
            'battery_mv': bat_mv,
            'solar_mv': solar_mv,
            'voltage_slope_mv_per_hour': slope,
            'satellites': sats,
            'hdop': hdop / 10.0,
            'power_mode': mode,
            'gps_valid': bool(flags & 0x01),
            'sensor_quality': sensor_quality,      # v6: b0-b4 stale bits, None on older formats
            'veto_reason': veto_reason,            # v6: TransmitVeto_t, None on older formats
            'crc16_valid': calculate_crc16(crc16_cover) == crc16,
        })

    result['crc32'] = struct.unpack('<I', payload[crc_off:crc_off + 4])[0]
    result['crc32_valid'] = calculate_crc32(payload[0:crc_off]) == result['crc32']
    return result

def calculate_crc16(data: bytes) -> int:
    """CRC16-MODBUS"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 0x0001 else crc >> 1
    return crc

def calculate_crc32(data: bytes) -> int:
    """CRC32 (IEEE 802.3)"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 0x00000001 else crc >> 1
    return ~crc & 0xFFFFFFFF

# Example usage
if __name__ == "__main__":
    # Authoritative byte-level vectors: tests/host/test_main.c prints
    # "GOLDEN bulk-v5 (2 records, seqs=256,257):" from the real firmware encoder.
    # Build a minimal 1-record v5 packet (2 + 36 + 4 = 42 bytes):
    example = bytearray(42)
    example[0] = 0x05  # v5
    example[1] = 0x01  # 1 record
    struct.pack_into('<I', example, 2, 256)          # record sequence (explicit, FR-07)
    struct.pack_into('<I', example, 6, 1737848000)   # Timestamp (LE)
    struct.pack_into('<i', example, 10, 4768932)     # Lat binary
    struct.pack_into('<i', example, 14, -10633288)   # Lon binary
    struct.pack_into('<H', example, 18, 1078)        # Altitude
    struct.pack_into('<h', example, 20, 179)         # Temp: 17.9 C
    struct.pack_into('<H', example, 22, 301)         # Humidity: 30.1%
    struct.pack_into('<H', example, 24, 8864)        # Pressure: 886.4 hPa
    struct.pack_into('<H', example, 26, 5606)        # Battery: 5606 mV
    struct.pack_into('<H', example, 28, 1189)        # Solar: 1189 mV
    struct.pack_into('<h', example, 30, 0)           # Slope: 0 mV/h
    example[32] = 9    # Satellites
    example[33] = 16   # HDOP: 1.6
    example[34] = 1    # Power mode: NORMAL
    example[35] = 0x13 # Flags: GPS valid, 9 sats
    struct.pack_into('<H', example, 36, calculate_crc16(bytes(example[6:36])))
    struct.pack_into('<I', example, 38, calculate_crc32(bytes(example[0:38])))

    decoded = decode_archive_packet(bytes(example))
    print(f"Archive packet: type 0x{decoded['packet_type']:02X}, {decoded['record_count']} records")
    for i, rec in enumerate(decoded['records']):
        print(f"  Record {i+1}: {rec['latitude']:.6f}, {rec['longitude']:.6f} @ {rec['altitude_m']}m, "
              f"{rec['temperature_c']:.1f}C, {rec['pressure_hpa']:.1f} hPa, crc16 {'OK' if rec['crc16_valid'] else 'FAIL'}")
    print(f"  Packet CRC32: {'OK' if decoded['crc32_valid'] else 'FAIL'}")
```

---

## PORT 2: CayenneLPP (DEVELOPMENT ONLY)

### Description
Standard CayenneLPP format for human-readable debugging. **Not used in production** - disable via `ENABLE_DEBUG_LPP=0`.

### Typical Channels

| Channel | Type | Data | Size |
|---------|------|------|------|
| 0 | GPS | Latitude, Longitude, Altitude | 11 bytes |
| 1 | Temperature | Temperature (0.1°C) | 4 bytes |
| 2 | Barometer | Pressure (0.1 hPa) | 4 bytes |
| 3 | Humidity | Humidity (0.5%) | 3 bytes |
| 4 | Analog | Battery voltage | 4 bytes |
| 5 | Analog | Solar voltage | 4 bytes |

**Total**: ~30 bytes typical

See [Cayenne LPP specification](https://developers.mydevices.com/cayenne/docs/lora/#lora-cayenne-low-power-payload) for details.

---

## PORT 3: GNSS Detail Packet (DEVELOPMENT ONLY)

### Description
Detailed satellite tracking and speed telemetry. **Not used in production** - disable via `ENABLE_GNSS_DETAIL_PACKET=0`.

**See [GNSSDetailPacket.md](GNSSDetailPacket.md) for complete specification.**

Typical size: 40-60 bytes (8-12 satellites with speed data)

---

## Transmission Patterns (from RTT Logs)

### Normal Operation Cycle

1. **Initial transmission** (Port 10):
   - Sends compact 11-byte heartbeat (v2) at SF10 (DR0)
   - Includes LinkCheckReq MAC command
   - TX count: 1

2. **Debug packets** (optional, every 5th cycle):
   - Port 2: CayenneLPP (~30 bytes)
   - Port 3: GNSS Detail (~50 bytes)

3. **Bulk transfer trigger** (Port 11):
   - Activated when: margin ≥15dB AND gateways ≥2 AND battery ≥5.0V
   - Sends variable-length v4 archive packets (`6 + 32n + 4` bytes) at SF7 (DR3), packed to the runtime payload budget (`LoRaMacQueryTxPossible`); n ≤ 6
   - Up to 20 packets per session
   - Clears flash backlog (ACK-gated watermark, DDR-0005)

### Example from RTT Log

```
=== SendTxData START ===
GPS FIX! Lat=51.163508 Lon=-114.066568 Alt=1078.3m Sats:9 HDOP=1.6 (took 2990ms)

Encoding 10-byte compact binary packet...
Compact: T=0m Lat=51.163504 Lon=-57.033276 Temp=17.87C P=231.12 H=30.13% Bat=5596mV Sats=9 Mode=1
Requesting LinkCheck...
Sending 10-byte compact packet at SF10 (DR0) on port 10

OnTxData Callback: Status=0, Datarate=DR0, TxPower=5, Channel=11, UplinkCounter=3

OnRxData Callback:
LinkCheckAns: Margin=20dB, Gateways=3
Link quality: margin=20dB (>=15), gateways=3 (>=2) -> GOOD
Battery: 5576mV (>=5000) -> GOOD
CONDITIONS MET: Triggering bulk transfer mode!

Bulk transfer mode: packet 1/20
Retrieved 1 unsent records from flash
Encoding 198-byte bulk packet with 1 records...
Bulk packet: Type=2 Records=1 CRC32=0xAB9C...
```

---

## Error Handling

### CRC Validation

**All production packets include CRC checksums:**

- **Port 10** (Compact): No CRC (too small), rely on LoRaWAN MIC
- **Port 11** (Bulk): CRC32 for packet + CRC16 for each record

**Backend should:**
1. Validate LoRaWAN MIC (handled by network server)
2. Validate packet CRC32 (bulk packets only)
3. Validate individual record CRC16 (bulk packets only)
4. Log and flag packets with CRC errors

### Invalid Data Handling

**GPS Invalid:**
- Port 10: latitude=0, longitude=0
- Port 11: gps_valid flag = 0 in status flags

**Out-of-Range Values:**
- Temperature: -64°C to +63°C (compact), ±3276.7°C (bulk)
- Pressure: 950-3500 hPa (compact), 0-6553.5 hPa (bulk)
- Battery: 0-12.75V (compact), 0-65.535V (bulk)

Values outside these ranges indicate sensor errors.

SP-14 (#251): NaN/negative inputs to the COMPACT temperature/battery converters encode as
range-bottom (0 raw units); only pressure/humidity have explicit sentinels (2047/31). Treat
temperature=raw-0 or battery=0 as sensor-error evidence, and prefer the heartbeat status
byte's stale bits for data-quality judgements.

---

## Related Documentation

- [GNSSDetailPacket.md](GNSSDetailPacket.md) - Port 3 specification
- [TransmissionModule.md](TransmissionModule.md) - LoRaWAN transmission logic
- [FlashLogging.md](FlashLogging.md) - Flash storage format
- [PowerManagement.md](PowerManagement.md) - Operating modes and intervals

---

## Backend Implementation Checklist

- [ ] Port 10 decoder with altitude calculation
- [ ] Port 11 decoder with CRC validation
- [ ] Port 2 CayenneLPP parser (optional)
- [ ] Port 3 GNSS detail parser (optional)
- [ ] Timestamp wraparound handling
- [ ] GPS coordinate validation
- [ ] Sensor range checking
- [ ] CRC error logging
- [ ] Bulk transfer session tracking
- [ ] Power mode interpretation

---

## Changelog

### 2026-08-06 (archive v4 / confirmed delivery, issue #34, DDR-0005)
- **Archive v4** (`packet_type 0x04`): header gains `base_seq` (uint32 LE) — record i identity = base_seq + i; backend dedups on (DevEUI, sequence). Length now `6 + 32n + 4`. `0x03` existed <1 day in CI only, never deployed.
- **Confirmed delivery**: opportunity-probe heartbeat and all archive packets are confirmed uplinks; watermark commits only on `McpsConfirm.AckReceived` (at-least-once; lost ACK → duplicate retransmission → backend dedup, never a gap). LinkCheckReq moved from the probe to the first archive packet; its LinkCheckAns gates burst continuation.

### 2026-08-06 (heartbeat v2 / archive v3, issue #33)
- **Port 10 heartbeat v2**: pressure/humidity merged into one packed uint16 LE (bits 0-10 = 1 hPa units, 0-2046 valid / 2047 invalid sentinel — stratospheric-useful; bits 11-15 = 5% humidity, 31 sentinel). Status byte v2: b3 pressure-stale, b4 RTC GNSS-disciplined (N-03), b5 timestamp-wrap (D4); condensed reset cause removed from the wire. Endianness corrected to little-endian throughout (D9; earlier BE documentation was wrong, N-01).
- **Port 11 archive v3** (`packet_type 0x03`): variable-length `2 + 32n + 4` with explicit LE serialization; records packed to the runtime payload budget (`LoRaMacQueryTxPossible`). v2 (`0x02`, 198 B) and v1 (`0x01`, 222 B) documented as legacy.
- **Probe DR (D1, superseded 2026-08-18)**: heartbeat at SF10 in EVERY region. The 11-byte heartbeat is sized exactly for US915 DR0, so SF9's headroom bought nothing and cost ~2.5 dB of link budget. Accepted residual: a queued MAC answer (e.g. DevStatusAns) cannot fit beside 11 B inside DR0's dwell-limited budget and drops that one probe cycle; the next wake recovers. (Earlier text read "SF9 in US915/AU915, SF10 elsewhere".)
- **COMM-TX (2026-08-18, DDR-0002 §7 / BR-LIFE-004)**: commissioning (status-byte mission bits = 00) transmits privacy-safe telemetry on the normal wake cadence - the port-10 compact probe (SF10 confirmed) plus one port-11 v6 packet carrying a single live record (SF7, unconfirmed, best effort, sent from the probe's confirm after RX2). Horizontal position is WITHHELD on both: `latitude_100m`/`longitude_100m` are zeroed post-encode; the live record's int32 lat/lon are zeroed and its CRC16 re-sealed. Flags stay honest (a valid fix with a withheld position, not "no fix"). Live records carry `seq = record timestamp` (UTC seconds) - a live record has no flash sequence and no watermark is committed; the backend's (device, seq) dedup still applies. A (0,0) position with mission state COMMISSIONING reads as "withheld", never as a fix at Null Island.
- Golden vectors printed by the firmware host tests (`tests/host/test_main.c` → `GOLDEN heartbeat-v2`, `GOLDEN bulk-v3`).

### 2026-01-25
- Initial comprehensive documentation
- Added complete decoder examples
- Documented all encoding formulas
- Added CRC validation details
- Included real RTT log examples
