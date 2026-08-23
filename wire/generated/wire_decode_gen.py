"""GENERATED FILE -- DO NOT HAND-EDIT.
Source of truth: firmware/wire/wire_schema.json. Regenerate with:
    python wire/tools/gen_python_wire.py
Pure, stdlib-only decoders for the sonde wire frames. Little-endian (D9).
"""
import base64
import struct
import zlib


def decode_heartbeat(data_base64):
    """Decode heartbeat (port 10, v2, 11 bytes LE). Returns dict or None."""
    try:
        p = base64.b64decode(data_base64)
    except Exception:
        return None
    if len(p) != 11:
        return None
    r = {}
    r['timestamp_min'] = struct.unpack_from('<H', p, 0)[0]
    r['latitude_raw'] = struct.unpack_from('<h', p, 2)[0]*90/32767
    r['longitude_raw'] = struct.unpack_from('<h', p, 4)[0]*180/32767
    r['temperature_raw'] = (p[6]-64)*2
    r['pressure_raw'] = ((struct.unpack_from('<H', p, 7)[0] >> 0) & 0x7FF)
    r['humidity_raw'] = ((struct.unpack_from('<H', p, 7)[0] >> 11) & 0x1F)*5
    r['battery_raw'] = p[9]*0.050
    r['gps_stale'] = bool(((p[10] >> 0) & 0x1))
    r['temp_stale'] = bool(((p[10] >> 1) & 0x1))
    r['humidity_stale'] = bool(((p[10] >> 2) & 0x1))
    r['pressure_stale'] = bool(((p[10] >> 3) & 0x1))
    r['time_gnss_disciplined'] = bool(((p[10] >> 4) & 0x1))
    r['timestamp_wrapped'] = bool(((p[10] >> 5) & 0x1))
    r['mission_state'] = ((p[10] >> 6) & 0x3)
    return r


def decode_bulk_packet_v6(data_base64):
    """Decode a bulk archive burst (port 11, v6). Envelope [0x06][count][records][crc32]."""
    try:
        p = base64.b64decode(data_base64)
    except Exception:
        return None
    if len(p) < 6 or p[0] != 0x06:
        return None
    count = p[1]
    if len(p) != 6 + 38 * count:
        return None
    body, crc_got = p[:-4], struct.unpack_from('<I', p, len(p) - 4)[0]
    r = {'packet_type': p[0], 'count': count,
         'crc32_valid': (_crc32(body) == crc_got), 'records': []}
    for i in range(count):
        base = 2 + i * 38
        seq = struct.unpack_from('<I', p, base)[0]
        rec = _decode_archive_record_v6(p[base + 4: base + 38])
        rec['sequence'] = seq
        r['records'].append(rec)
    return r


def _decode_archive_record_v6(b):
    """Decode one 34-byte v6 archive record."""
    r = {}
    r['timestamp'] = struct.unpack_from('<I', b, 0)[0]
    r['latitude_deg'] = struct.unpack_from('<i', b, 4)[0] * 90.0 / 8388607.0
    r['longitude_deg'] = struct.unpack_from('<i', b, 8)[0] * 180.0 / 8388607.0
    r['altitude_m'] = struct.unpack_from('<H', b, 12)[0]
    r['temperature_degC'] = struct.unpack_from('<h', b, 14)[0] * 0.1
    r['humidity_pct'] = struct.unpack_from('<H', b, 16)[0] * 0.1
    r['pressure_hPa'] = struct.unpack_from('<H', b, 18)[0] * 0.1
    r['battery_mV'] = struct.unpack_from('<H', b, 20)[0]
    r['solar_mV'] = struct.unpack_from('<H', b, 22)[0]
    r['voltage_slope_mVh'] = struct.unpack_from('<h', b, 24)[0]
    r['satellites'] = b[26]
    r['hdop'] = b[27] * 0.1
    r['power_mode'] = b[28]
    r['flags'] = b[29]
    r['sensor_quality'] = b[30]
    r['veto_reason'] = b[31]
    crc_got = struct.unpack_from('<H', b, 32)[0]
    r['crc16'] = crc_got
    r['crc16_valid'] = (_crc16_modbus(b[:32]) == crc_got)
    return r



def decode_version_report(data_base64):
    """Decode version_report (port 20, v1, 12 bytes LE). Returns dict or None."""
    try:
        p = base64.b64decode(data_base64)
    except Exception:
        return None
    if len(p) != 12:
        return None
    r = {}
    r['magic'] = p[0]
    r['fw_major'] = p[1]
    r['fw_minor'] = p[2]
    r['fw_patch'] = p[3]
    r['heartbeat_format_version'] = p[4]
    r['stage'] = p[5]
    r['mission_minutes'] = struct.unpack_from('<I', p, 6)[0]
    r['crc16'] = struct.unpack_from('<H', p, 10)[0]
    _calc = _crc16_ccitt_false(p[:10])
    _got = struct.unpack_from('<H', p, 10)[0]
    r['crc_valid'] = (_calc == _got)
    return r



def _crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def _crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc


def _crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF
