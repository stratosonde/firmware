# Wire format single source of truth

`wire_schema.json` is the **canonical** description of every frame on the air.
The firmware encoder is the authority; this file is updated **in the same commit**
as any wire change, and the derived artifacts + both repos' decoders are
regenerated/validated from it. This eliminates the hand-mirrored layout that
caused the port-10/11 decode drift.

## Files
- `wire_schema.json` — the schema (frames, fields, offsets, scaling, bitfields, CRCs, golden vectors).
- `tools/validate_schema.py` — internal-consistency gate (offsets/lengths/CRC covers/vector lengths).
- `tools/gen_python_wire.py` — generates `generated/wire_decode_gen.py` (pure Python decoders).
- `tools/gen_c_wire.py` — generates `../../Core/Inc/wire_format_gen.h` (C constants).
- `tools/check_schema_vectors.py` — runs every golden vector through the generated Python decoder.
- `generated/wire_decode_gen.py` — GENERATED (do not hand-edit); vendored into the ground repo.
- `../../Core/Inc/wire_format_gen.h` — GENERATED (do not hand-edit); asserted against the real
  firmware headers by `_Static_assert`s in `tests/host/test_main.c` (the firmware-side drift gate).

## Make targets (from tests/host)
- `make wire` — validate schema + run golden vectors (part of `make check`, so it's in CI).
- `make wiregen` — regenerate the C header + Python decoder after editing the schema.
- `make wirecheck` — regenerate + validate + `git diff --exit-code` the artifacts (fails if you
  edited the schema but forgot to regenerate).

## How to change a wire format
1. Edit `wire_schema.json` (and the firmware encoder + ground mapping as needed).
2. `make -C tests/host wiregen` to regenerate artifacts.
3. Add/refresh a golden vector in the schema.
4. `make -C tests/host wirecheck` — must be clean (artifacts in sync, vectors pass).
5. `make -C tests/host check` — the `_Static_assert` drift gate proves the C layout matches.
6. Re-vendor `wire/generated/wire_decode_gen.py` into the ground repo and commit it there.

The three CRC conventions are deliberately distinct and pinned in the schema:
record CRC16 = MODBUS, version-frame CRC16 = CCITT-FALSE, bulk trailer CRC32 = ISO-HDLC (zlib).
