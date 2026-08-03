# LoRaWAN Payload Formats - Stratosonde Telemetry

## Overview

The Stratosonde firmware transmits data using multiple LoRaWAN packet formats optimized for different purposes. This document provides complete specifications for backend implementation.

---

## LoRaWAN Port Assignments

| Port | Packet Type | Size | Purpose | Status |
|------|-------------|------|---------|--------|
| **2** | CayenneLPP | Variable | Human-readable debug format | Development only |
| **3** | GNSS Detail | Variable | Satellite tracking data | Development only |
| **10** | Compact Binary | 10 bytes | Production telemetry (SF10) | **PRODUCTION** |
| **11** | Bulk Binary | 198 bytes (v2) | Historical data transfer (SF7) | **PRODUCTION** |

### Debug Packet Control

Debug packets (ports 2 and 3) can be disabled via compile-time flags:
```c
#define ENABLE_DEBUG_LPP           1  // 0 = Disable CayenneLPP
#define ENABLE_GNSS_DETAIL_PACKET  1  // 0 = Disable GNSS detail
#define DEBUG_LPP_TX_INTERVAL      5  // Send debug every 5th TX
```

---

## PORT 10: Compact Binary Packet (PRODUCTION)

### Description
Ultra-compact 10-byte packet optimized for maximum range transmission at SF10 (DR0). This format leaves room for MAC commands (like LinkCheckReq) in the FOpts field, enabling adaptive transmission strategies.

### Packet Structure (10 bytes)

| Offset | Field | Type | Size | Resolution | Range | Description |
|--------|-------|------|------|------------|-------|-------------|
| 0 | Timestamp | uint16 BE | 2 | 1 minute | 0-45.5 days | Minutes since Unix epoch (wraps) |
| 2 | Latitude | int16 BE | 2 | ~100m | ±3276.7 km | Latitude in 100m units |
| 4 | Longitude | int16 BE | 2 | ~100m | ±3276.7 km | Longitude in 100m units |
| 6 | Temperature | int8 | 1 | 2°C | -64 to +63°C | (value - 64) × 2 |
| 7 | Pressure | uint8 | 1 | 10 hPa | 950-3500 hPa | 950 + (value × 10) |
| 8 | Battery | uint8 | 1 | 50 mV | 0-12.75V | value × 0.050 |
| 9 | Humidity | uint8 | 1 | 5% | 0-100% | value × 5 |

**Note**: Altitude is NOT transmitted - it must be calculated on the backend from pressure + temperature using the barometric formula.

### Encoding Formulas

#### Timestamp (uint16, minutes since epoch)
```
encoded_value = (unix_seconds / 60) & 0xFFFF
```
Wraps every 65,535 minutes (~45.5 days). Use context from previous packets to handle wraparound.

#### Latitude (int16, ~100m resolution)
```python
# Encoding (firmware):
lat_degrees = lat_binary × (90.0 / 8388607.0)
lat_100m = round(lat_degrees / 0.0009009)  # 0.0009009° ≈ 100m at equator
encoded_value = clamp(lat_100m, -32768, 32767)

# Decoding (backend):
lat_100m = struct.unpack('>h', bytes[2:4])[0]  # signed big-endian
lat_degrees = lat_100m × 0.0009009
```

#### Longitude (int16, ~100m resolution)
```python
# Encoding (firmware):
lon_degrees = lon_binary × (180.0 / 8388607.0)
lon_100m = round(lon_degrees / 0.0009009)
encoded_value = clamp(lon_100m, -32768, 32767)

# Decoding (backend):
lon_100m = struct.unpack('>h', bytes[4:6])[0]  # signed big-endian
lon_degrees = lon_100m × 0.0009009
```

#### Temperature (int8, 2°C resolution with offset)
```python
# Encoding (firmware):
encoded_value = round(temp_celsius / 2.0) + 64

# Decoding (backend):
temp_celsius = (byte_value - 64) × 2.0
```

#### Pressure (uint8, 10 hPa resolution from 950 hPa base)
```python
# Encoding (firmware):
encoded_value = round((pressure_hPa - 950) / 10.0)

# Decoding (backend):
pressure_hPa = 950 + (byte_value × 10.0)
```

#### Battery Voltage (uint8, 50 mV resolution)
```python
# Encoding (firmware):
encoded_value = round(voltage_volts / 0.050)

# Decoding (backend):
voltage_volts = byte_value × 0.050
```

#### Humidity (uint8, 5% resolution)
```python
# Encoding (firmware):
encoded_value = round(humidity_percent / 5.0)

# Decoding (backend):
humidity_percent = byte_value × 5.0
```

### Python Decoder Example

```python
import struct
from datetime import datetime, timezone

def decode_compact_packet(payload: bytes) -> dict:
    """
    Decode 10-byte compact telemetry packet from LoRaWAN Port 10
    
    Args:
        payload: 10-byte packet from LoRaWAN
        
    Returns:
        dict with decoded telemetry data
    """
    if len(payload) != 10:
        raise ValueError(f"Expected 10 bytes, got {len(payload)}")
    
    # Unpack all fields (big-endian)
    timestamp_min, lat_100m, lon_100m = struct.unpack('>Hhh', payload[0:6])
    temp_raw, pressure_raw, battery_raw, humidity_raw = struct.unpack('BBBB', payload[6:10])
    
    # Decode values
    result = {
        'timestamp_minutes': timestamp_min,
        'latitude': lat_100m * 0.0009009,
        'longitude': lon_100m * 0.0009009,
        'temperature_c': (temp_raw - 64) * 2.0,
        'pressure_hpa': 950 + (pressure_raw * 10.0),
        'battery_v': battery_raw * 0.050,
        'humidity_pct': humidity_raw * 5.0
    }
    
    return result

# Example usage
if __name__ == "__main__":
    # Example from RTT log: Lat=51.163504 Lon=-114.066276 (Calgary)
    # This is encoded as compact binary
    example = bytes([
        0x00, 0x00,  # Timestamp: 0 minutes (example)
        0xE6, 0x30,  # Latitude: 58928 × 0.0009009 ≈ 53.09°
        0x82, 0xA8,  # Longitude: -32088 × 0.0009009 ≈ -28.91°
        0x4D,        # Temperature: (77-64)×2 = 26°C
        0x00,        # Pressure: 950+(0×10) = 950 hPa
        0x70,        # Battery: 112×0.05 = 5.6V
        0x06         # Humidity: 6×5 = 30%
    ])
    
    decoded = decode_compact_packet(example)
    print("Compact Packet Decoded:")
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

## PORT 11: Bulk Binary Packet (PRODUCTION)

### Description
198-byte packet (v2, FW-20) for efficient bulk transfer of historical data at SF7 (DR2). Contains up to 6 high-resolution records. Transmitted when link quality is good (margin ≥15dB, gateways ≥2) and battery is sufficient (≥5.0V). Records are FIFO order (oldest unsent first).

> **FW-20 layout change:** the v1 222-byte layout carried three permanently-zero
> placeholder fields (Flash Page Addr 4 B, Voltage Trend 10 B, Mode Changes 10 B
> = 24 B of SF7 airtime per packet). v2 (packet_type 0x02) deletes them and
> shrinks the packet to 198 B. Ground decoders must branch on `payload[0]`:
> `0x01` = legacy 222 B v1 (decode below only for archival logs), `0x02` = v2.

### Packet Structure v2 (198 bytes)

#### Header (2 bytes)

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 0 | Packet Type | uint8 | 1 | Format version (0x02 = v2 FIFO, no placeholders) |
| 1 | Record Count | uint8 | 1 | Number of records (1-6) |

#### High-Resolution Records (192 bytes = 6 × 32 bytes)

Each record is 32 bytes:

| Offset | Field | Type | Size | Resolution | Range | Description |
|--------|-------|------|------|------------|-------|-------------|
| 0 | Timestamp | uint32 BE | 4 | 1 second | Full range | Unix timestamp (seconds) |
| 4 | Latitude | int32 BE | 4 | 1e-7° | Full | GPS binary format |
| 8 | Longitude | int32 BE | 4 | 1e-7° | Full | GPS binary format |
| 12 | Altitude | uint16 BE | 2 | 1 meter | 0-65535m | GPS altitude |
| 14 | Temperature | int16 BE | 2 | 0.1°C | ±3276.7°C | temp × 10 |
| 16 | Humidity | uint16 BE | 2 | 0.1% | 0-6553.5% | humidity × 10 |
| 18 | Pressure | uint16 BE | 2 | 0.1 hPa | 0-6553.5 hPa | pressure × 10 |
| 20 | Battery Voltage | uint16 BE | 2 | 1 mV | 0-65.535V | Millivolts |
| 22 | Solar Voltage | uint16 BE | 2 | 1 mV | 0-65.535V | Millivolts |
| 24 | Voltage Slope | int16 BE | 2 | 1 mV/hour | ±32.767 V/h | Charge rate |
| 26 | Satellites | uint8 | 1 | 1 | 0-255 | GPS satellite count |
| 27 | HDOP | uint8 | 1 | 0.1 | 0-25.5 | HDOP × 10 |
| 28 | Power Mode | uint8 | 1 | enum | 0-7 | Operating mode |
| 29 | Flags | uint8 | 1 | bitfield | - | Status flags |
| 30 | CRC16 | uint16 BE | 2 | - | - | Record integrity check |

#### Status Flags (byte 29 of each record)

| Bit | Mask | Field | Description |
|-----|------|-------|-------------|
| 0 | 0x01 | GPS Valid | GPS fix valid (1) or invalid (0) |
| 1-4 | 0x1E | Satellite Count | Satellite count 0-15 (duplicates byte 26) |
| 5-7 | 0xE0 | Power Mode | Operating mode 0-7 (duplicates byte 28) |

#### Trailer (4 bytes)

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 194 | CRC32 | uint32 BE | 4 | Packet integrity check (over bytes 0-193) |

> v1 records started at offset 6; in v2 they start at offset 2 (`2 + i*32`).
> v1's `flash_page_addr`/`voltage_trend`/`mode_changes` fields are gone —
> record identity comes from each record's timestamp + sequence.

### Power Mode Enum

| Value | Mode | Description | TX Interval |
|-------|------|-------------|-------------|
| 0 | STARTUP | Initial boot | - |
| 1 | NORMAL | Normal operation | 300s (5 min) |
| 2 | CONSERVATIVE | Battery saving | 600s (10 min) |
| 3 | REDUCED | Low battery | 900s (15 min) |
| 4 | CRITICAL | Critical battery | 1800s (30 min) |
| 5 | GPS_LOCKOUT | GPS disabled | As configured |

### Python Decoder Example

```python
import struct
from typing import List, Dict

def decode_bulk_packet(payload: bytes) -> dict:
    """
    Decode bulk telemetry packet from LoRaWAN Port 11 (v2 198 B; v1 legacy 222 B)
    
    Args:
        payload: bulk packet from LoRaWAN (branch on payload[0]: 0x01 v1, 0x02 v2)
        
    Returns:
        dict with header, records array, metadata, and CRC validation
    """
    # FW-20: v2 is 198 B (packet_type 0x02); v1 legacy is 222 B (0x01)
    version = payload[0]
    if version == 0x02:
        assert len(payload) == 198, f"Expected 198 bytes (v2), got {len(payload)}"
        header_len = 2
    elif version == 0x01:
        assert len(payload) == 222, f"Expected 222 bytes (v1), got {len(payload)}"
        header_len = 6
    else:
        raise ValueError(f"Unknown bulk packet version 0x{version:02X}")
    
    result = {}
    
    # Parse header
    result['packet_type'] = version
    result['record_count'] = payload[1]
    if version == 0x01:
        result['flash_page_addr'] = struct.unpack('>I', payload[2:6])[0]
    
    # Parse records (up to 6, each 32 bytes)
    result['records'] = []
    for i in range(min(result['record_count'], 6)):
        offset = header_len + (i * 32)
        record_data = payload[offset:offset+32]
        
        # Unpack record
        timestamp, lat, lon, alt = struct.unpack('>Iiii', record_data[0:14])
        alt = struct.unpack('>H', record_data[12:14])[0]  # Re-extract as uint16
        temp, humidity, pressure = struct.unpack('>hhH', record_data[14:20])
        bat_mv, solar_mv, slope = struct.unpack('>HHh', record_data[20:26])
        sats, hdop, mode, flags = struct.unpack('BBBB', record_data[26:30])
        crc16 = struct.unpack('>H', record_data[30:32])[0]
        
        # Verify record CRC
        calc_crc = calculate_crc16(record_data[0:30])
        crc_valid = (calc_crc == crc16)
        
        record = {
            'timestamp': timestamp,
            'latitude': lat * (90.0 / 8388607.0),  # Convert binary to degrees
            'longitude': lon * (180.0 / 8388607.0),  # Convert binary to degrees
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
            'crc16': crc16,
            'crc16_valid': crc_valid
        }
        
        result['records'].append(record)
    
    # Parse trailer + verify packet CRC
    if version == 0x01:
        result['voltage_trend'] = list(payload[198:208])
        result['mode_changes'] = list(payload[208:218])
        result['crc32'] = struct.unpack('>I', payload[218:222])[0]
        calc_packet_crc = calculate_crc32(payload[0:218])
    else:
        result['crc32'] = struct.unpack('>I', payload[194:198])[0]
        calc_packet_crc = calculate_crc32(payload[0:194])
    result['crc32_valid'] = (calc_packet_crc == result['crc32'])
    
    return result

def calculate_crc16(data: bytes) -> int:
    """CRC16-MODBUS"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def calculate_crc32(data: bytes) -> int:
    """CRC32 (IEEE 802.3)"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x00000001:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return ~crc & 0xFFFFFFFF

# Example usage
if __name__ == "__main__":
    # Create example v2 packet (header + 1 record, FW-20 layout)
    example = bytearray(198)
    
    # Header
    example[0] = 0x02  # Packet type (v2 FIFO, no placeholders)
    example[1] = 0x01  # 1 record
    
    # Record 1 (32 bytes at offset 2)
    struct.pack_into('>I', example, 2, 1737848000)  # Timestamp
    struct.pack_into('>i', example, 6, 4768932)  # Lat binary
    struct.pack_into('>i', example, 10, -10633288)  # Lon binary
    struct.pack_into('>H', example, 14, 1078)  # Altitude
    struct.pack_into('>h', example, 16, 179)  # Temp: 17.9°C
    struct.pack_into('>H', example, 18, 301)  # Humidity: 30.1%
    struct.pack_into('>H', example, 20, 8864)  # Pressure: 886.4 hPa
    struct.pack_into('>H', example, 22, 5606)  # Battery: 5606 mV
    struct.pack_into('>H', example, 24, 1189)  # Solar: 1189 mV
    struct.pack_into('>h', example, 26, 0)  # Slope: 0 mV/h
    example[28] = 9  # Satellites
    example[29] = 16  # HDOP: 1.6
    example[30] = 1  # Power mode: NORMAL
    example[31] = 0x13  # Flags: GPS valid, 9 sats
    # Record CRC16 would be calculated here (bytes 32:34)
    
    # Packet CRC
    crc32 = calculate_crc32(bytes(example[0:194]))
    struct.pack_into('>I', example, 194, crc32)
    
    # Decode
    decoded = decode_bulk_packet(bytes(example))
    print(f"Bulk packet: {decoded['record_count']} records")
    for i, rec in enumerate(decoded['records']):
        print(f"\nRecord {i+1}:")
        print(f"  GPS: {rec['latitude']:.6f}, {rec['longitude']:.6f} @ {rec['altitude_m']}m")
        print(f"  Temp: {rec['temperature_c']:.1f}°C")
        print(f"  Pressure: {rec['pressure_hpa']:.1f} hPa")
        print(f"  Battery: {rec['battery_mv']} mV")
        print(f"  CRC16: {'OK' if rec['crc16_valid'] else 'FAIL'}")
    print(f"\nPacket CRC32: {'OK' if decoded['crc32_valid'] else 'FAIL'}")
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
   - Sends compact 10-byte packet at SF10 (DR0)
   - Includes LinkCheckReq MAC command
   - TX count: 1

2. **Debug packets** (optional, every 5th cycle):
   - Port 2: CayenneLPP (~30 bytes)
   - Port 3: GNSS Detail (~50 bytes)

3. **Bulk transfer trigger** (Port 11):
   - Activated when: margin ≥15dB AND gateways ≥2 AND battery ≥5.0V
   - Sends 198-byte bulk packets at SF7 (DR3) (v2, FW-20)
   - Up to 20 packets per session
   - Clears flash backlog

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

### 2026-01-25
- Initial comprehensive documentation
- Added complete decoder examples
- Documented all encoding formulas
- Added CRC validation details
- Included real RTT log examples
