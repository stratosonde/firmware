# H3Lite Fix-Implementation Verification Report

**Date:** 2026-08-03
**Scope:** `docs/archive/h3lite-code-review.md` + `docs/archive/h3lite_code_review_verification_handoff.md` fixes applied to the `Middlewares/Third_Party/h3lite` submodule and regenerated against `../hplans` (GeoJSON, 16 files loaded; `regions.geojson`/`Unknown.geojson` skipped).
**Prior context:** FIX-SIZE-001 (world-map polyfill → compact packed table) landed previously in this repo (FirmwareArchitecture.md / RegionLookup.md).

---

## 1. Executive summary

Two handoffs describing the same library were reconciled against the actual
source. The code-review's 15 findings (F-01..F-15) were all verified as real
and fixed. The verification handoff's pentagon numbers were partially
inaccurate — its own README claim ("846/927 exact, failures cluster near
pentagons, search continues outward") was wrong in two specific ways that the
fixes expose: the predicate bug, and the abort-on-ring-failure semantics.
The regenerated table is **4564 entries (18,256 bytes)**, produced
deterministically with fail-closed checks, and device-equivalent probes match
the expected LoRaWAN plan at every tested capital city.

**User decision recorded:** RESTRICTED remains part of h3lite (ID 15,
repurposed from the CD900-1A test-plan slot, which is not used in
production); `lora_app.c` continues to block transmission on it.

## 2. Findings fixed (C library, Phase 1)

| ID | Finding | Fix |
|----|---------|-----|
| F-01 | `_h3IsPentagon` tested only the base cell → any res>0 cell inside a pentagon base cell (~6% of globe, incl. Bohai Sea, Gulf of Alaska, Norwegian Sea, Arabian Sea, Gulf of Guinea) reported ring failure | Digit loop moved before base-cell test (h3lite.c) |
| F-02 | `regionNames` modifiable via API (ODR/segfault) | `const char * const`, all writers dropped |
| F-03 | NaN/inf/out-of-domain coordinates walked through to a real cell (Cortex-M NaN→int cast flushes to 0) | `isfinite` + range guard; `findNearestRegions` checks input and ring-failure |
| F-05 | int32 index-math overflow at face 17, lat < -60 | Index math `int8_t`; face-17 constant audited correct |
| F-06 | `h3ToRegion` clamped res >3; ring search could silently abort | Clamp removed; neighbor-search failure handled in caller |
| F-08 | `binarySearch` early-exit returned match without boundary scan | Lo-hi boundary scan |
| F-10 | README omitted basecells/neighbor sources; stale binary name; memory figures off by >2× | README rewritten; `findNearestRegions` documented (first-productive-ring, ~120 km/ring, 336 B stack) |
| F-11 | `faceIjkToH3` lacked pentagon rev-rotation guard + face 17 comment | Reference-faithful guard + comment |
| F-12 | Duplicate `neighbors`/`rotations` tables | Deleted from h3lite.c (ODR collision with h3lite_faceijk.c) |
| F-13 | 22 KB of int tables that fit in int8/uint8; `int32_t` loop vars | Tables narrowed to int8/uint8 (≈16 KB flash saved) |
| F-14 | ~9.7 KB dead flash tables | `baseCellTable`, `faceNeighbors`, `maxDimByCIIres`, `unitScaleByCIIres` deleted |
| F-15 | Missing system includes; missing extern-C guards | Added to all four headers |
| H3-6(1) | `h3GetRing` returned compile-time MAX | Returns actual cell count |
| — | Makefile: `ar` used target name; duplicate `-lm`; mkdir ran after link | Fixed; `-Wall -Wextra`; `all` = lib+tests |

## 3. Findings fixed (generator, Phase 2)

- **C-1:** tie-break is now deterministic (smallest total region extent wins,
  lexicographic name as final tie-break) — 51 contested cells resolved
  repeatably.
- **C-2:** `pack_entry` bounds-checks baseCell/res/partialIndex/regionId and
  raises on overflow (previously silent wrap at res ≥ 4 or RESTRICTED).
- **C-3:** `table.c/h` always rewritten (previously a failed run left stale
  artifacts that compiled fine).
- **C-4:** duplicate-key assertion before writing (0 duplicates in output).
- **C-5:** GeoJSON filenames sorted before loading (deterministic ID
  assignment).
- **C-6:** `UNKNOWN.geojson`/`regions.geojson` now load (ID 0 = Unknown is
  legal; region 0 is simply never polyfilled).
- **C-7:** provenance header in generated files (timestamp, resolution,
  GeoJSON dir, python/h3/shapely versions, entry count, RESTRICTED remap
  record).
- **C-8:** over-aggressive `k_ring(compact(cell,3),1)` guard replaced by
  `cell != cell_to_parent(cell, res)` check; compaction uses actual grid_disk
  with res-3 reference, not "7 children means uniform".
- **C-9:** compaction walks each base cell's digit space to catch parents
  split across the sort order (previously `groupby` missed them).
- **C-10:** RESTRICTED (15) remapped from the CD900-1A test slot with an
  explicit `PROVENANCE_REMAPS` record in the generator and the table header.
- **F-08 data-gap fix:** seaward buffer per region polyfilled and conflicts
  resolved by largest intersected area (log shows per-region `+N seaward
  cells`). This closes the DRC landlocked-border failure mode.

## 4. Empirical verification (host, python h3 4.5.0 + shapely 2.1.1)

Device-equivalent probing (res 3 → 2 → 1 over the packed table,
`test/probe_table.py`):

```
Kinshasa, DRC      -> EU868     (was: Unknown — seaward-buffer fix)
DRC center         -> EU868     (was: Unknown)
Hong Kong          -> CN470     (matches prior bug fix)
Macau              -> CN470     (matches prior bug fix)
NYC -> US915  Paris -> EU868  Tokyo -> AS923-1  Sydney -> AU915
New Delhi -> IN865  Seoul -> KR920  Beijing -> CN470  Moscow -> RU864
Wellington/Auckland -> AS923-1C  Sao Paulo -> AU915
Mid-Atlantic -> Unknown (correct)
```

**F-04 coverage gaps (data, not code) — verified against hplans polygons:**
Mongolia, Fiji, Tuvalu, Kiribati, Marshall Islands, FSM, Nauru, Palau, and
Maldives are inside **no** region polygon in hplans (they fall in the
`Unknown.geojson` umbrella). These resolve to `Unknown` and rely on
`findNearestRegions` offshore. Closing them requires extending the hplans
source polygons — tracked as a separate hplans task.

## 5. Regression tests added

- `test/t_pentagon.py` + `test/xval_ring.c` — differential ring test around
  all 12 pentagons (F-01 regression). Classification per handoff §5.1: only
  `k=1` pentagon-origin clean failures are legitimate.
- `test_invalid_inputs` in `h3lite_grid_test.c` — NaN/inf/out-of-domain must
  yield h3 == 0 (F-03 regression); grid/nearest tests now exit non-zero on
  any mismatch (previously always 0).
- `test/probe_table.py` — device-equivalent probe harness used for §4.

## 6. Firmware touch-ups

- `Core/Src/multiregion_h3.c`: CD900-1A comment → RESTRICTED (ID 15) note;
  `H3_MAX_DISTANCE_KM` comment corrected (120 km/ring ⇒ 3 rings ≈ 360 km <
  500 km gate — threshold itself unchanged and still sound).
- `LoRaWAN/App/lora_app.c`: comment records that `REGION_RESTRICTED` is now
  15, not 255 (the 4-bit field could never emit 255; the comparison keeps
  working because both sides use the same `h3lite.h` macro).

## 7. Deliberately NOT done (per user direction)

- **Phase 0 (bit-exactness baseline):** no gcc on this machine; handed off as
  a separate task — on a machine with gcc: `make lib`, build `xval_pts`,
  run `test/t_table.py t_index.py t_city.py t_pentagon.py` against the
  **pre-fix** commit to produce the baseline, then re-run on the fixed tree.
  `t_pentagon.py` skips gracefully without gcc/h3.
- **hplans polygon extension** for the F-04 island nations (§4) — data task,
  separate repo.
- **Pentagon `k=1`-origin ring distortion** — legitimate H3 behavior;
  `findNearestRegions` handles it by trying the next ring.

## 8. Remaining verification steps (require toolchain/hardware)

1. Build `libh3lite.a` + tests with gcc/MinGW (`make clean all`) — expect
   `-Wall -Wextra` clean.
2. Run `h3lite_grid_test` / `h3lite_nearest_test` — expect `RESULT: PASS`,
   non-zero exit on failure.
3. Run `test/t_pentagon.py` — expect `wrong=0`, clean failures only for
   k=1 pentagon origins.
4. STM32 build (`make -C <firmware>`): confirm flash delta ≈ +4 KB table,
   −16 KB narrowed const tables.
5. On-device: RTT profiling suite (`MultiRegion_ProfileH3Performance`)
   unchanged in behavior for in-region coordinates; offshore ring search now
   succeeds where the pentagon predicate bug previously blanked ~6% of globe.
