#!/usr/bin/env python3
"""Generate wire_decode_gen.py from wire_schema.json.

Produces a pure, dependency-free Python decoder for each frame. The ground
station VENDORS the generated file (do not hand-edit it); this generator is the
only writer. Phase 3 of the wire-schema flow.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCHEMA = os.path.join(HERE, "..", "wire_schema.json")
OUT = os.path.join(HERE, "..", "generated", "wire_decode_gen.py")

HDR = '''"""GENERATED FILE -- DO NOT HAND-EDIT.
Source of truth: firmware/wire/wire_schema.json. Regenerate with:
    python wire/tools/gen_python_wire.py
Pure, stdlib-only decoders for the sonde wire frames. Little-endian (D9).
"""
import base64
import struct
import zlib

'''


def _read(typ, off):
    if typ in ("u8", "i8"):
        return ("p[%d]" % off) if typ == "u8" else ("struct.unpack_from('<b', p, %d)[0]" % off)
    fmt = {"u16": "<H", "i16": "<h", "u32": "<I", "i32": "<i"}[typ]
    return "struct.unpack_from('%s', p, %d)[0]" % (fmt, off)


def _scale(raw, fld):
    s = fld.get("scale")
    if not s:
        return raw
    return s.replace("x", raw)


def _crcfn(algo):
    return {"crc16-modbus": "_crc16_modbus", "crc16-ccitt-false": "_crc16_ccitt_false",
            "crc32-iso-hdlc": "_crc32"}[algo]


def gen_frame(fr):
    name = fr["name"]
    if "record_length" in fr:
        return ""
    if name == "bulk_packet_v6":
        return gen_bulk(fr)
    total = fr["total_length"]
    L = []
    L.append("def decode_%s(data_base64):" % name)
    L.append('    """Decode %s (port %s, v%s, %d bytes LE). Returns dict or None."""'
             % (name, fr["f_port"], fr["format_version"], total))
    L.append("    try:")
    L.append("        p = base64.b64decode(data_base64)")
    L.append("    except Exception:")
    L.append("        return None")
    L.append("    if len(p) != %d:" % total)
    L.append("        return None")
    L.append("    r = {}")
    for fld in fr["fields"]:
        off, typ, nm = fld["offset"], fld["type"], fld["name"]
        if "bitfield" in fld:
            word = _read(typ, off)
            for bf in fld["bitfield"]:
                mask = (1 << bf["bits"]) - 1
                v = "((%s >> %d) & 0x%X)" % (word, bf["lsb"], mask)
                if bf["bits"] == 1 and "scale" not in bf and "enum" not in bf:
                    L.append("    r[%r] = bool(%s)" % (bf["name"], v))
                else:
                    L.append("    r[%r] = %s" % (bf["name"], _scale(v, bf)))
            continue
        raw = _read(typ, off)
        if "sentinel" in fld:
            s = fld["sentinel"]["value"]
            L.append("    _v = %s" % raw)
            L.append("    r[%r] = None if _v == %d else %s" % (nm, s, _scale("_v", fld)))
        else:
            L.append("    r[%r] = %s" % (nm, _scale(raw, fld)))
    for integ in fr.get("integrity", []):
        cov_hi = int(integ["cover"].split("..")[1]) + 1  # inclusive end -> python slice
        L.append("    _calc = %s(p[:%s])" % (_crcfn(integ["algo"]), cov_hi))
        L.append("    _got = struct.unpack_from('<H', p, %d)[0]" % integ["offset"])
        L.append("    r['crc_valid'] = (_calc == _got)")
    L.append("    return r")
    L.append("")
    return "\n".join(L)


def gen_bulk(fr):
    return '''def decode_bulk_packet_v6(data_base64):
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
    r['crc16_valid'] = (_crc16_modbus(b[:32]) == crc_got)
    return r

'''


CRC_HELPERS = '''
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
'''


def main():
    with open(SCHEMA) as f:
        schema = json.load(f)
    parts = [HDR]
    for fr in schema["frames"]:
        body = gen_frame(fr)
        if body:
            parts.append(body + "\n")
    parts.append(CRC_HELPERS)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write("\n".join(parts))
    print("wrote %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())

