# GNSS Detail Packet Format Documentation

## Overview
The GNSS Detail packet provides comprehensive satellite tracking and 3D speed telemetry data. This packet is transmitted on **LoRaWAN Port 3** alongside the standard Cayenne LPP environmental sensor data on Port 2.

## Packet Structure

### Header (3 bytes)
| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | Version | uint8 | Packet format version (currently 0x01) |
| 1 | GPS Count | uint8 | Number of GPS satellites |
| 2 | Other Count | uint8 | Combined GLONASS + BeiDou satellite count |

### GPS Satellites (variable length)
For each GPS satellite (count from byte 1):
| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| N+0 | PRN | uint8 | GPS PRN number (1-32) |
| N+1 | SNR | uint8 | Signal-to-noise ratio (dBHz, 0-99) |

### GLONASS Satellites (variable length)
For each GLONASS satellite:
| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| N+0 | PRN | uint8 | GLONASS PRN number (65-96) |
| N+1 | SNR | uint8 | Signal-to-noise ratio (dBHz, 0-99) |

### BeiDou Satellites (variable length)
For each BeiDou satellite:
| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| N+0 | PRN | uint8 | BeiDou PRN number (201-237) |
| N+1 | SNR | uint8 | Signal-to-noise ratio (dBHz, 0-99) |

### Speed Data (12 bytes)
| Bytes | Field | Type | Resolution | Range | Description |
|-------|-------|------|------------|-------|-------------|
| N+0..1 | Ground Speed | uint16 (BE) | 0.1 km/h | 0-6553.5 km/h | Horizontal ground speed |
| N+2..3 | Vertical Speed | int16 (BE) | 0.01 m/s | ±327.68 m/s | Climb (+) or descent (-) rate |
| N+4..5 | 3D Speed | uint16 (BE) | 0.1 km/h | 0-6553.5 km/h | Total 3D velocity |
| N+6..7 | Track | uint16 (BE) | 0.1° | 0-359.9° | Course over ground (true north) |
| N+8..9 | HDOP | uint16 (BE) | 0.01 | 0-655.35 | Horizontal dilution of precision |

### Fix Metadata (2 bytes)
| Byte | Field | Type | Description |
|------|-------|------|-------------|
| N+10 | Fix Quality | uint8 | 0=Invalid, 1=GPS, 2=DGPS |
| N+11 | Satellites Used | uint8 | Number of satellites in fix solution |

## Decoding Example (Python)

```python
def decode_gnss_detail(payload):
    """
    Decode GNSS detail packet from LoRaWAN Port 3
    
    Args:
        payload: bytes object containing the packet
        
    Returns:
        dict with decoded GNSS telemetry
    """
    if len(payload) < 3:
        return None
        
    idx = 0
    result = {}
    
    # Header
    result['version'] = payload[idx]
    idx += 1
    result['gps_count'] = payload[idx]
    idx += 1
    result['other_count'] = payload[idx]
    idx += 1
    
    # GPS satellites
    result['gps_satellites'] = []
    for i in range(result['gps_count']):
        if idx + 2 > len(payload):
            break
        sat = {
            'prn': payload[idx],
            'snr': payload[idx + 1]
        }
        result['gps_satellites'].append(sat)
        idx += 2
    
    # GLONASS satellites
    result['glonass_satellites'] = []
    glonass_count = 0
    while idx + 2 <= len(payload):
        prn = payload[idx]
        if prn < 65 or prn > 96:
            break  # Not GLONASS PRN
        sat = {
            'prn': prn,
            'snr': payload[idx + 1]
        }
        result['glonass_satellites'].append(sat)
        glonass_count += 1
        idx += 2
        if glonass_count >= result['other_count']:
            break
    
    # BeiDou satellites
    result['beidou_satellites'] = []
    beidou_count = 0
    remaining = result['other_count'] - glonass_count
    while idx + 2 <= len(payload) and beidou_count < remaining:
        prn = payload[idx]
        if prn < 201:
            break  # Not BeiDou PRN
        sat = {
            'prn': prn,
            'snr': payload[idx + 1]
        }
        result['beido u_satellites'].append(sat)
        beidou_count += 1
        idx += 2
    
    # Speed data
    if idx + 12 <= len(payload):
        result['ground_speed_kmh'] = int.from_bytes(payload[idx:idx+2], 'big') * 0.1
        idx += 2
        
        result['vertical_speed_ms'] = int.from_bytes(payload[idx:idx+2], 'big', signed=True) * 0.01
        idx += 2
        
        result['speed_3d_kmh'] = int.from_bytes(payload[idx:idx+2], 'big') * 0.1
        idx += 2
        
        result['track_degrees'] = int.from_bytes(payload[idx:idx+2], 'big') * 0.1
        idx += 2
        
        result['hdop'] = int.from_bytes(payload[idx:idx+2], 'big') * 0.01
        idx += 2
    
    # Fix metadata
    if idx + 2 <= len(payload):
        result['fix_quality'] = payload[idx]
        result['satellites_used'] = payload[idx + 1]
    
    return result


# Example usage
if __name__ == "__main__":
    # Example packet: 3 GPS sats, 2 BeiDou sats, with speed data
    example = bytes([
        0x01,           # Version
        0x03,           # 3 GPS satellites
        0x02,           # 2 other (BeiDou)
        # GPS sats
        0x0F, 0x28,     # PRN 15, SNR 40 dBHz
        0x0A, 0x2D,     # PRN 10, SNR 45 dBHz
        0x17, 0x25,     # PRN 23, SNR 37 dBHz
        # BeiDou sats
        0xC9, 0x22,     # PRN 201, SNR 34 dBHz
        0xCA, 0x20,     # PRN 202, SNR 32 dBHz
        # Speed data
        0x00, 0x64,     # Ground speed: 10.0 km/h
        0xFF, 0x9C,     # Vertical speed: -1.0 m/s (descending)
        0x00, 0x65,     # 3D speed: 10.1 km/h
        0x05, 0xDC,     # Track: 150.0 degrees
        0x00, 0x78,     # HDOP: 1.20
        # Fix metadata
        0x01,           # GPS fix
        0x05            # 5 satellites used
    ])
    
    decoded = decode_gnss_detail(example)
    print("Decoded GNSS Detail:")
    print(f"  GPS satellites: {len(decoded['gps_satellites'])}")
    for sat in decoded['gps_satellites']:
        print(f"    PRN {sat['prn']}: SNR {sat['snr']} dBHz")
    print(f"  BeiDou satellites: {len(decoded['beidou_satellites'])}")
    for sat in decoded['beidou_satellites']:
        print(f"    PRN {sat['prn']}: SNR {sat['snr']} dBHz")
    print(f"  Ground speed: {decoded['ground_speed_kmh']:.1f} km/h")
    print(f"  Vertical speed: {decoded['vertical_speed_ms']:.2f} m/s")
    print(f"  3D speed: {decoded['speed_3d_kmh']:.1f} km/h")
    print(f"  Track: {decoded['track_degrees']:.1f}°")
    print(f"  HDOP: {decoded['hdop']:.2f}")
    print(f"  Fix quality: {decoded['fix_quality']}")
    print(f"  Satellites used: {decoded['satellites_used']}")
```

## Packet Size Calculation

Typical packet sizes:
- **Minimum**: 17 bytes (header + no sats + speed data + metadata)
- **Typical**: 40-60 bytes (8-12 satellites with speed data)
- **Maximum**: ~150 bytes (20+ satellites from multiple constellations)

All packets fit well within DR2 payload limit (125 bytes for US915).

## Visualization Use Cases

### 1. Satellite Sky Plot
Use PRN, elevation, and azimuth (from extended data) to plot satellite positions relative to the receiver.

### 2. Signal Quality Analysis
Plot SNR values per satellite to assess antenna performance during ascent.

### 3. Speed/Altitude Profile
Combine with Port 2 altitude data to create ascent rate profiles:
- Ground speed vs altitude
- Vertical speed (climb rate) vs altitude
- 3D velocity vector visualization

### 4. Constellation Performance
Compare GPS vs BeiDou satellite counts and SNR to evaluate multi-constellation performance at high altitude.

## Notes

- **Big-endian** byte order used for multi-byte values
- **VTG sentence** provides ground speed (more accurate than RMC)
- **Vertical speed** calculated from altitude changes between readings
- **3D speed** computed as `sqrt(ground_speed² + vertical_speed²)`
- **PRN ranges**: GPS (1-32), GLONASS (65-96), BeiDou (201-237)

## Related Documentation
- [GNSSModule.md](GNSSModule.md) - GNSS driver implementation
- [TransmissionModule.md](TransmissionModule.md) - LoRaWAN packet transmission
