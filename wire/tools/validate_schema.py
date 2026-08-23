#!/usr/bin/env python3
"""Validate wire_schema.json internal consistency.

Gate 1 of the wire-schema flow: the schema must be self-consistent before any
codegen or drift-check trusts it. Pure stdlib. Exit 0 = clean, 1 = failures.
"""
import json
import sys
import os

SCHEMA_PATH = os.path.join(os.path.dirname(__file__), "..", "wire_schema.json")

TYPE_SIZE = {"u8": 1, "i8": 1, "u16": 2, "i16": 2, "u32": 4, "i32": 4}
KNOWN_ALGOS = {"crc16-modbus", "crc16-ccitt-false", "crc32-iso-hdlc"}

failures = []


def err(msg):
    failures.append(msg)
    print("FAIL: %s" % msg)


def parse_cover(cover):
    """'0..31' -> (0, 31); '0..len-5' / 'len-4' markers are envelope-relative."""
    if cover is None:
        return None
    if ".." not in str(cover):
        return None
    lo, hi = str(cover).split("..", 1)
    try:
        lo_i = int(lo)
    except ValueError:
        return None
    return (lo_i, hi)  # hi may be 'N' or 'len-K' (symbolic)


def main():
    with open(SCHEMA_PATH) as f:
        schema = json.load(f)

    if schema.get("schema") != "sonde-wire/1":
        err("schema tag must be 'sonde-wire/1', got %r" % schema.get("schema"))

    frames = schema.get("frames", [])
    if not frames:
        err("no frames defined")
    names = set()
    for fr in frames:
        name = fr.get("name", "<unnamed>")
        if name in names:
            err("duplicate frame name %r" % name)
        names.add(name)

        if "f_port" not in fr:
            err("%s: missing f_port" % name)
        fields = fr.get("fields", [])
        if not fields:
            err("%s: no fields" % name)

        # Bitfield sub-bit checks + fixed-frame overlap/length checks.
        if fr.get("length_rule") == "fixed" or "record_length" in fr:
            declared = fr.get("total_length", fr.get("record_length"))
            span = [False] * declared
            for fld in fields:
                off = fld.get("offset")
                typ = fld.get("type")
                if off is None:
                    err("%s.%s: missing offset" % (name, fld.get("name")))
                    continue
                if typ == "array":
                    continue  # variable-length frames aren't fixed
                size = TYPE_SIZE.get(typ)
                if size is None:
                    err("%s.%s: unknown type %r" % (name, fld.get("name"), typ))
                    continue
                if off + size > declared:
                    err("%s.%s: offset %d + %d overruns declared length %d"
                        % (name, fld.get("name"), off, size, declared))
                for b in range(off, off + size):
                    if b < declared and span[b]:
                        err("%s.%s: byte %d overlaps another field"
                            % (name, fld.get("name"), b))
                    if b < declared:
                        span[b] = True
                # Bitfield sanity
                for bf in fld.get("bitfield", []):
                    if bf.get("lsb", 0) + bf.get("bits", 0) > size * 8:
                        err("%s.%s bitfield %s overruns its word"
                            % (name, fld.get("name"), bf.get("name")))
            # No gaps in a fixed frame (every byte accounted for)
            if not all(span):
                gaps = [i for i, v in enumerate(span) if not v]
                err("%s: undeclared gap bytes at %s" % (name, gaps))

        # Integrity algo must be known
        for integ in fr.get("integrity", []):
            if integ.get("algo") not in KNOWN_ALGOS:
                err("%s: unknown integrity algo %r" % (name, integ.get("algo")))

        # Golden vectors: input length must match the frame length rule
        for gv in fr.get("golden_vectors", []):
            hexs = gv.get("bytes_hex", "")
            raw = bytes.fromhex(hexs)
            if fr.get("length_rule") == "fixed":
                if len(raw) != fr["total_length"]:
                    err("%s: golden vector %d bytes != total_length %d"
                        % (name, len(raw), fr["total_length"]))
            elif "record_length" in fr and name.endswith("record_v6"):
                pass  # record golden vectors optional
            if not gv.get("expect"):
                err("%s: golden vector missing 'expect' map" % name)

    print("validate_schema: %d frames, %d failures" % (len(frames), len(failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
