#!/usr/bin/env python3
"""Run every golden vector in wire_schema.json through the GENERATED decoder.

Gate 2: proves the schema + generated decoder agree with the authoritative byte
vectors (which the firmware host suite also prints). Any drift between schema,
decoder, or bytes fails here. Exit 0 = clean.
"""
import base64
import importlib.util
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCHEMA = os.path.join(HERE, "..", "wire_schema.json")
GEN = os.path.join(HERE, "..", "generated", "wire_decode_gen.py")

failures = []


def err(m):
    failures.append(m)
    print("FAIL: %s" % m)


def load_gen():
    spec = importlib.util.spec_from_file_location("wire_decode_gen", GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# Map schema expect-keys -> generated decoder output keys, with a tolerance for floats.
KEYMAP = {
    "heartbeat": {
        "timestamp_min": "timestamp_min",
        "latitude_deg": "latitude_raw",
        "longitude_deg": "longitude_raw",
        "temperature_degC": "temperature_raw",
        "pressure_hPa": "pressure_raw",
        "humidity_pct": "humidity_raw",
        "battery_voltage_V": "battery_raw",
        "mission_state": "mission_state",
        "gps_stale": "gps_stale",
    },
    "version_report": {
        "magic": "magic", "fw_major": "fw_major", "fw_minor": "fw_minor",
        "fw_patch": "fw_patch", "heartbeat_format_version": "heartbeat_format_version",
        "stage": "stage", "mission_minutes": "mission_minutes", "crc16": "crc16",
    },
}


def close(a, b):
    if isinstance(b, float) or isinstance(a, float):
        try:
            return abs(float(a) - float(b)) < 1e-4
        except (TypeError, ValueError):
            return False
    return a == b


def main():
    if not os.path.exists(GEN):
        err("generated decoder missing; run gen_python_wire.py first")
        return 1
    gen = load_gen()
    with open(SCHEMA) as f:
        schema = json.load(f)

    nvec = 0
    for fr in schema["frames"]:
        name = fr["name"]
        gvs = fr.get("golden_vectors", [])
        if not gvs:
            continue
        fn = getattr(gen, "decode_%s" % name, None)
        if fn is None:
            err("%s: generated decoder has no decode_%s" % (name, name))
            continue
        km = KEYMAP.get(name, {})
        for gv in gvs:
            nvec += 1
            raw = bytes.fromhex(gv["bytes_hex"])
            b64 = base64.b64encode(raw).decode()
            out = fn(b64)
            if out is None:
                err("%s: decoder returned None for golden vector" % name)
                continue
            for ekey, evalv in gv["expect"].items():
                okey = km.get(ekey, ekey)
                oval = out.get(okey, "<missing>")
                if not close(oval, evalv):
                    err("%s.%s: schema expects %r, decoder gave %r"
                        % (name, ekey, evalv, oval))
            # integrity must validate on a well-formed vector
            if fr.get("integrity") and not out.get("crc_valid", True):
                err("%s: crc_valid False on golden vector" % name)

    print("check_schema_vectors: %d vectors, %d failures" % (nvec, len(failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
