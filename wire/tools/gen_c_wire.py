#!/usr/bin/env python3
"""Generate Core/Inc/wire_format_gen.h from wire_schema.json.

Emits C constants (lengths, offsets, masks, CRC selectors) derived from the
schema. The firmware encoder stays hand-written; a host test _Static_asserts
these generated constants against the real payload_format.h / version_report.h
values, so a C layout change that isn't reflected in the schema fails the build.
Phase 2 of the wire-schema flow.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCHEMA = os.path.join(HERE, "..", "wire_schema.json")
OUT = os.path.join(HERE, "..", "..", "Core", "Inc", "wire_format_gen.h")


def find(frames, name):
    return next(f for f in frames if f["name"] == name)


def field(fr, name):
    return next(f for f in fr["fields"] if f["name"] == name)


def main():
    with open(SCHEMA) as f:
        schema = json.load(f)
    fr = schema["frames"]
    hb = find(fr, "heartbeat")
    rec = find(fr, "archive_record_v6")
    vr = find(fr, "version_report")
    bulk = find(fr, "bulk_packet_v6")

    ph = field(hb, "press_hum")
    press_bf = ph["bitfield"][0]
    hum_bf = ph["bitfield"][1]
    vr_integ = vr["integrity"][0]
    rec_integ = rec["integrity"][0]
    bulk_integ = bulk["integrity"][0]

    def crcsel(algo):
        return {"crc16-modbus": 0, "crc16-ccitt-false": 1, "crc32-iso-hdlc": 2}[algo]

    out = []
    out.append("/* GENERATED FILE -- DO NOT HAND-EDIT.")
    out.append(" * Source of truth: wire/wire_schema.json. Regenerate: python wire/tools/gen_c_wire.py")
    out.append(" * Constants asserted against the real firmware headers by test_main.c (wire-schema drift gate). */")
    out.append("#ifndef WIRE_FORMAT_GEN_H")
    out.append("#define WIRE_FORMAT_GEN_H")
    out.append("")
    out.append("/* heartbeat (port 10) */")
    out.append("#define WIRE_HEARTBEAT_LEN %dU" % hb["total_length"])
    out.append("#define WIRE_HEARTBEAT_VERSION %dU" % hb["format_version"])
    out.append("#define WIRE_HB_OFF_TIMESTAMP %dU" % field(hb, "timestamp_min")["offset"])
    out.append("#define WIRE_HB_OFF_LAT %dU" % field(hb, "latitude_raw")["offset"])
    out.append("#define WIRE_HB_OFF_LON %dU" % field(hb, "longitude_raw")["offset"])
    out.append("#define WIRE_HB_OFF_TEMP %dU" % field(hb, "temperature_raw")["offset"])
    out.append("#define WIRE_HB_OFF_PRESSHUM %dU" % field(hb, "press_hum")["offset"])
    out.append("#define WIRE_HB_OFF_BATT %dU" % field(hb, "battery_raw")["offset"])
    out.append("#define WIRE_HB_OFF_STATUS %dU" % field(hb, "status")["offset"])
    out.append("#define WIRE_HB_PRESS_LSB %dU" % press_bf["lsb"])
    out.append("#define WIRE_HB_PRESS_BITS %dU" % press_bf["bits"])
    out.append("#define WIRE_HB_PRESS_INVALID %dU" % press_bf["sentinel"]["value"])
    out.append("#define WIRE_HB_HUM_LSB %dU" % hum_bf["lsb"])
    out.append("#define WIRE_HB_HUM_BITS %dU" % hum_bf["bits"])
    out.append("#define WIRE_HB_HUM_INVALID %dU" % hum_bf["sentinel"]["value"])
    out.append("")
    out.append("/* archive record v6 (port 11) */")
    out.append("#define WIRE_ARCHIVE_RECORD_LEN %dU" % rec["record_length"])
    out.append("#define WIRE_REC_OFF_SQ %dU" % field(rec, "sensor_quality")["offset"])
    out.append("#define WIRE_REC_OFF_VETO %dU" % field(rec, "veto_reason")["offset"])
    out.append("#define WIRE_REC_OFF_CRC16 %dU" % field(rec, "crc16")["offset"])
    out.append("#define WIRE_REC_CRC_COVER %dU /* bytes covered by record crc16 */" % (int(rec_integ["cover"].split("..")[1]) + 1))
    out.append("#define WIRE_REC_CRC_ALGO %dU /* 0=modbus */" % crcsel(rec_integ["algo"]))
    out.append("")
    out.append("/* bulk envelope v6 (port 11) */")
    out.append("#define WIRE_BULK_TYPE %dU" % field(bulk, "packet_type")["const"])
    out.append("#define WIRE_BULK_OVERHEAD 6U /* type+count+crc32 */")
    out.append("#define WIRE_BULK_RECORD_STRIDE 38U /* seq u32 + 34B record */")
    out.append("#define WIRE_BULK_CRC_ALGO %dU /* 2=crc32-iso-hdlc */" % crcsel(bulk_integ["algo"]))
    out.append("")
    out.append("/* version report (port 20) */")
    out.append("#define WIRE_VERSION_LEN %dU" % vr["total_length"])
    out.append("#define WIRE_VERSION_MAGIC %dU" % field(vr, "magic")["const"])
    out.append("#define WIRE_VR_OFF_CRC16 %dU" % field(vr, "crc16")["offset"])
    out.append("#define WIRE_VR_CRC_COVER %dU" % (int(vr_integ["cover"].split("..")[1]) + 1))
    out.append("#define WIRE_VR_CRC_ALGO %dU /* 1=ccitt-false */" % crcsel(vr_integ["algo"]))
    out.append("")
    out.append("#endif /* WIRE_FORMAT_GEN_H */")

    with open(OUT, "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print("wrote %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
