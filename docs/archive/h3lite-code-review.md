# H3Lite — Code Review

**Repository:** https://github.com/stratosonde/h3lite
**Commit reviewed:** `8d15d6b41eedbb3c41141bde4afb1a2ffe5334f3` ("docs: README pentagon limitation + current table size/format/deps", 2026-08-02)
**Reference used:** `h3` Python package 4.5.0
**Host toolchain:** gcc, x86-64, `-Wall -Wextra -O3`

---

## 0. How to use this document

This is written for an agent that has the repository checked out and can build and run it.

Every finding below has an **ID**, a **severity**, a **file/line anchor**, the **evidence** that produced it, and a **verification step** you can run to confirm the finding still reproduces before you change anything.

**Ground rules:**

1. **Confirm before fixing.** Each finding has a `Verify:` block. Run it. If it does not reproduce, say so and stop — do not "fix" something that isn't there. Several findings were measured empirically and the numbers may have moved.
2. **Do not regress the geometry engine.** `latLngToH3` is currently bit-exact against reference H3. That is the single most valuable property this library has. Any change must keep `T-INDEX: 0 / 259200 mismatches`.
3. **Section 8 lists hypotheses that were tested and found FALSE.** Read it before you start. Do not implement fixes for them.
4. Severity meanings:
   - **P0** — wrong radio region selected, or undefined behaviour reachable from normal input.
   - **P1** — undefined behaviour not currently observable, or a defect that breaks documented integration.
   - **P2** — size, maintainability, test quality, documentation.

---

## 1. Executive summary

The core coordinate engine is **correct**. `latLngToH3` matches reference H3 bit-for-bit at resolutions 0–4 over 100,000 random-on-sphere points and the full 259,200-point 0.5° global grid, including both poles, both sides of the antimeridian, and sub-epsilon coordinates. `h3GetRing` never returns a *wrong* ring — only exact rings or clean failures. The packed table is well-formed: 4,556 entries, zero duplicate keys, zero conflicting keys, correctly sorted for binary search, with sane uniform-only compaction.

The defects are at the edges: the **table data** (which decides what frequency you transmit on), one **substantive logic bug** in the pentagon predicate, several **UB/portability hazards** that behave differently on the STM32 target than on the host, and roughly **26% of flash spent on `int` where `int8_t` would do**.

Highest-value items, in order:

| ID | Severity | One-line |
|---|---|---|
| **F-01** | P0 | `_h3IsPentagon` tests only the base cell → `findNearestRegions` is inert over ~6% of the globe |
| **F-02** | P0 | Conflict tie-break "lowest region ID" hands the entire DRC to EU868 instead of CD900-1A |
| **F-03** | P0 | `latLngToH3` accepts NaN; UB, and Cortex-M behaves differently from x86 |
| **F-04** | P0 | Mongolia has zero coverage; Fiji + 7 island states have zero coverage |
| **F-05** | P1 | `baseCellData` declared with two different types across TUs (ODR violation) |
| **F-12** | P2 | ~13 KB recoverable by narrowing base-cell tables to `int8_t` |

---

## 2. What is verified correct (do not change)

Establish this baseline before touching anything.

```bash
make lib
cd test && python3 t_table.py && python3 t_index.py
```

Expected:

```
T-TABLE: OK (4556 entries, no duplicates, no conflicts)
T-INDEX: 0 / 259200 mismatches
```

Also confirmed during review:

- `latLngToH3` exact vs `h3.latlng_to_cell` at res 0, 1, 2, 3, 4 — 20,000 random points each, 0 mismatches.
- Edge coordinates exact: `(90,0)`, `(-90,0)`, `(90,180)`, `(-90,-180)`, `(0,±180)`, `(0,0)`, `(1e-15,1e-15)`.
- `h3GetRing` produced **0 wrong-content rings** across 8,000 origins × k=1..6 (48,000 cases). Where it succeeds it is set-identical to `h3.grid_ring`.
- Table compaction is sound: 95 res-1 / 771 res-2 / 3,690 res-3 entries. The res-1 entries are large uniform ocean/interior blocks and are legitimate.
- The res-3→res-2→res-1 probe order in `h3ToRegion` is correct given uniform-only compaction (a parent exists only if all children were present and identical, so no key can exist at two resolutions).

---

## 3. P0 — Correctness

### F-01 — `_h3IsPentagon` tests only the base cell

**File:** `src/h3lite_neighbor.c`, `static bool _h3IsPentagon(H3Index h)`

```c
static bool _h3IsPentagon(H3Index h) {
    int baseCell = H3_GET_BASE_CELL(h);
    return _isBaseCellPentagon(baseCell);   // missing the digit test
}
```

Reference H3 requires the base cell to be a pentagon **and** every indexing digit to be zero. Without the second condition, the entire ~4.2 Mkm² pentagon *base cell* is reported as "a pentagon". `h3GetRing` rejects on its first line:

```c
if (_h3IsPentagon(origin)) return -1;
```

so the ring never starts. **Every ring k=1..6 fails for any origin within roughly 1000 km of a pentagon centre** — not just the ring that contains the pentagon. `findNearestRegions` therefore has no fallback at all in those areas.

I instrumented all six failure sites in `h3GetRing`. At 120 km, 300 km and 600 km from a pentagon centre, **100%** of k=1 failures were `origin is pentagon`. At 1500 km everything works — that is the base-cell radius, not the pentagon-cell radius.

Of 614 k=1 ring failures across 8,000 random res-3 origins, **603 were this predicate**; only 11 were genuine pentagon distortion.

**Fix:**

```c
static bool _h3IsPentagon(H3Index h) {
    return _isBaseCellPentagon(H3_GET_BASE_CELL(h)) &&
           !_h3LeadingNonZeroDigit(h);
}
```

`_h3LeadingNonZeroDigit` is already defined `static` in the same file.

**Measured effect (8,000 origins):**

| k | exact before → after | clean-fail before → after | wrong |
|---|---|---|---|
| 1 | 7351 → **7989** | 649 → **11** | 0 → 0 |
| 2 | 7205 → 7974 | 795 → 26 | 0 → 0 |
| 3 | 7051 → 7948 | 949 → 52 | 0 → 0 |
| 4 | 6918 → 7918 | 1082 → 82 | 0 → 0 |
| 5 | 6753 → 7873 | 1247 → 127 | 0 → 0 |
| 6 | 6562 → 7828 | 1438 → 172 | 0 → 0 |

Residual failures are the genuine deferred pentagon-rotation limitation.

**Practical effect.** The twelve res-3 pentagon centres:

| base cell | lat / lng | surroundings |
|---|---|---|
| 4 | 64.700N 10.536E | Norwegian Sea, off Trøndelag |
| 14 | 50.103N 143.478W | Gulf of Alaska |
| 24 | 39.100N 122.300E | **Bohai Sea, enclosed by China** |
| 38 | 23.718N 67.132W | Atlantic NE of Puerto Rico |
| 49 | 10.447N 58.158E | Arabian Sea |
| 58 | 2.301N 5.245W | Gulf of Guinea |
| 63 | 2.301S 174.755E | Gilbert Islands |
| 72 | 10.447S 121.842W | SE Pacific |
| 83 | 23.718S 112.868E | off NW Australia |
| 97 | 39.100S 57.700W | off Buenos Aires |
| 107 | 50.103S 36.522E | Southern Ocean |
| 117 | 64.700S 169.464W | Southern Ocean |

Fallback recovery for points 60–450 km out, before → after:

```
Bohai Sea / China :  0/10  ->  10/10
Norwegian Sea     :  0/28  ->  28/28
off NW Australia  :  0/56  ->  31/56
off Argentina     :  0/70  ->  33/70
```

The two enclosed seas go from total failure to full recovery. Globally the fallback rescues 51.5% of Unknown points after the fix (45.5% before); the remainder is genuine mid-ocean with nothing inside 6 rings, which is a table-coverage limit, not a pentagon one.

**Verify:**
```bash
# Build both, compare rings against reference h3 at several k.
# Before the fix you should see ~8% clean failures at k=1 and ~18% at k=6,
# concentrated inside the 12 pentagon base cells.
python3 verify_equivalence.py <pre-fix-tree> <post-fix-tree>
```

**Documentation consequence.** The README's *Known limitations* paragraph must be rewritten. Its "846/927 exact ring matches, 81 clean failures, ~11% of sampled origins" figures were measured with the broken predicate, and its claim that

> `findNearestRegions()` treats this as "no ring" and searches the next ring outward, so nearest-region lookup still works

is **false** — the next ring outward failed too.

---

### F-02 — Conflict tie-break loses nested national plans

**File:** `generate_lookup_table.py`, `generate_lookup_table()`, step 2

```python
areas.sort(key=lambda t: (-t[0], t[1]))  # area desc, id asc
winner = areas[0][1]
```

For a cell wholly interior to a country that also falls inside an overlapping continental catch-all polygon, **both** regions have full-cell intersection area. The areas tie, so the lowest region ID wins.

**Observed:** every point probed inside the DRC — Kinshasa, Lubumbashi, Kisangani, Kasai, northern interior — returns **EU868 (ID 2)**, not CD900-1A (ID 15). CD900-1A has **2 entries in the entire table**, both on base cell 57, for a 2.34 Mkm² country.

Per-region entry counts for context:

```
EU868 1305 | AU915 766 | AS923-1 548 | US915 528 | RU864 462 | IN865 271
CN470  225 | AS923-2 194 | AS923-3 103 | AS923-1C 58 | EU433 36 | AS923-1B 34
KR920   21 | AS923-4 3 | CD900-1A 2
```

**Fix:** tie-break on *specificity* — smallest total region extent wins — not lowest ID.

```python
region_extent = {rid: u.area for rid, u in unions.items()}
...
areas.append((area, region_extent[rid], rid))
areas.sort(key=lambda t: (-t[0], t[1], t[2]))   # intersected area desc,
                                                # total extent asc, id asc
winner = areas[0][2]
```

**Verify (after regenerating):** Kinshasa (−4.44, 15.27), Lubumbashi (−11.66, 27.48) and Kisangani (0.52, 25.20) should all return CD900-1A, and CD900-1A's entry count should be on the order of 150–200 rather than 2.

---

### F-03 — `latLngToH3` accepts NaN / ±inf / out-of-domain input

**File:** `src/h3lite.c`, `latLngToH3`

There is no input validation. A NaN reaches `(int)x1` / `(int)x2` in `_hex2dToCoordIJK` (`src/h3lite_faceijk.c`). Converting a NaN to `int` is **undefined behaviour**, and critically it is **not consistent across the platforms this library targets**:

- **x86-64 SSE:** yields `INT_MIN`. The later `MAX_FACE_COORD` range check catches it, so `latLngToH3` returns 0. This is what the host tests observe — by luck.
- **Cortex-M `VCVT`:** flushes NaN to **0**. Then `m1 = m2 = 0`, `r1 = r2 = NaN`, every comparison in the branch chain is false, control falls through to `h->i = m1 + 1; h->j = m2 + 1;` → coord `{1,1,0}` on face 0 → **a valid H3 index**, which will resolve to a real region and configure real frequencies.

A GPS module that reports a lost fix as NaN would therefore select a plausible-looking wrong region on the device while testing clean on the host. Out-of-range latitudes are also accepted: `(91, 0)`, `(-91, 0)` and `(1000, 2000)` all produce valid-looking indexes today.

**Fix:** at the top of `latLngToH3`, after the resolution check:

```c
if (!isfinite(lat) || !isfinite(lng) ||
    lat < -90.0 || lat > 90.0 || lng < -180.0 || lng > 180.0) {
    return 0;
}
```

Requires `<math.h>`, already included.

**Verify:**
```c
double bad[][2] = {{NAN,NAN},{NAN,0},{0,NAN},{INFINITY,0},{1000,2000},{-91,0},{91,0},{0,181}};
/* every one must give h3 == 0 and region == 0 */
```

**Related:** `findNearestRegions` does not guard `h3 == 0`. If the conversion fails it proceeds to `h3GetRing(0, k, ...)`, which walks `baseCellNeighbors[0][...]` off an all-zero index and returns meaningless cells. Add:

```c
if (h3 == 0) {
    return result;
}
```

immediately after the `latLngToH3` call.

---

### F-04 — Missing region coverage

Probed with `test/xval_pts.c` at res 3.

**Mongolia: entirely Unknown.** Four widely separated interior points (Ulaanbaatar 47.89/106.91, west 48.0/92.0, east 47.0/114.0, Gobi 44.0/104.0) all return region 0. A 1.56 Mkm² landmass — this cannot be explained by cell granularity or by any projection effect.

**Fiji entirely Unknown**, as are Tuvalu, Kiribati, Marshall Islands, FSM, Nauru, Palau, and Maldives.

Capital-city sweep, Unknown results: `Suva, Funafuti, Tarawa, Majuro, Palikir, Yaren, Ngerulmud, Male, Ulaanbaatar`.

**Important:** the two obvious explanations were tested and **ruled out** — see §8. The remaining leading hypothesis is that these regions are simply absent from the source GeoJSON. This cannot be confirmed from the repository because the GeoJSON inputs are not committed (see F-08).

**Action:** determine whether these regions exist in the `hplans` input set. If they do, instrument `load_regions()` / `convert_region_to_h3()` to log per-region cell counts and find where they are dropped. If they do not, that is an upstream data gap to document rather than a code fix.

---

### F-05 — Hong Kong and Macau assigned CN470

Hong Kong (22.32, 114.17) and Macau (22.19, 113.54) both return CN470. Both are separate LoRaWAN jurisdictions. `test/t_city.py` already flags Hong Kong. This is likely the same nesting problem as F-02 (Chinese polygon overlapping the SAR polygons) — re-test after fixing F-02 before doing anything else.

---

### F-06 — The repository's own regression test fails at HEAD

```
$ cd test && python3 t_city.py
T-CITY: 49 correct / 0 Unknown / 3 wrong (94.2% correct)
  FAIL Wellington      expected=AU915    got=AS923-1C
  FAIL Auckland        expected=AU915    got=AS923-1C
  FAIL HongKong        expected=AS923    got=CN470
T-CITY: FAIL (target: >=51 correct, 0 wrong)
```

The docstring calls the NZ and HK expectations "PROVISIONAL pending H3-3 (RP002 verification)" but the assertion is hard. Resolve the RP002 question and fix either the table or the expectations. A permanently-failing committed test stops functioning as a signal.

---

### F-07 — Border granularity is undocumented

A res-3 cell is ~12,400 km², roughly 120 km across. Any point within ~60 km of a plan boundary can be assigned to the wrong side. Concrete example: 66.0N / 170.0W (Little Diomede, US territory) returns **RU864**. That is defensible — the cell straddles both Diomedes — but the README should state the limit explicitly. For a device that acts on the answer, this is the difference between "approximate" and "trustworthy".

---

## 4. P1 — Undefined behaviour and integration

### F-08 — `baseCellData` declared with two different types

**Files:** `src/h3lite_basecells.c` (definition), `src/h3lite_neighbor.c` (declaration)

```c
/* basecells.c */  const BaseCellData baseCellData[122];       /* 28 B/entry */
/* neighbor.c  */  extern const FaceIJK baseCellData[NUM_BASE_CELLS];  /* 16 B/entry */
```

`neighbor.c` then reads `baseCellData[oldBaseCell].face`. With a 16-byte stride against a 28-byte definition, every index past 0 reads the wrong memory. This is an ODR violation and formally UB. Confirmed by LTO:

```
src/h3lite_neighbor.c:24: warning: type of 'baseCellData' does not match original declaration
  note: type 'const struct BaseCellData' should match type 'const struct FaceIJK'
```

**Currently unobservable.** I built a corrected version and diffed rings across 15,000 cases: zero difference. The only consumer is `_baseCellIsCwOffset(...)` inside the pentagon branch of `h3NeighborRotations`, and `h3GetRing` discards pentagon results. **But F-01's fix widens the set of ring traversals that reach that branch, and pentagon rotation support is on the roadmap.** Fix this alongside F-01, not after.

**Fix:** move `BaseCellData` (and `BaseCellRotation`) into `include/h3lite_faceijk.h`, declare the tables once there, delete the private copies.

**Verify:**
```bash
for f in src/*.c; do gcc -Wall -Wextra -flto -O2 -Iinclude -c $f -o /tmp/$(basename $f).o; done
gcc -flto -O2 /tmp/*.o -lm -shared -o /tmp/lib.so   # must emit no type-mismatch warning
```

### F-09 — Relative includes break the documented integration

Every source uses `#include "../include/h3lite.h"`. The README tells integrators to copy `include/` onto their include path and `src/*.c` into their sources — which breaks that path. Flatten to `#include "h3lite.h"` and rely on `-I`.

### F-10 — README's STM32 file list does not link

README §"STM32 Integration / Step 1" lists `h3lite.c`, `h3lite_faceijk.c`, `h3lite_regions_table.c`. It omits **`h3lite_basecells.c`** and **`h3lite_neighbor.c`**, both required. Following the README verbatim produces undefined-symbol errors.

### F-11 — No `extern "C"` guards

Zero occurrences across `include/`. STM32CubeIDE projects with a `main.cpp` will not link.

---

## 5. P2 — Size

### F-12 — Base-cell tables use `int` for byte-sized values

All values fit in a signed byte: base cells 0–121 plus the 127 sentinel, rotations −1..5, faces 0–19, ijk 0–2, direction digits 0–7.

Measured (`nm --print-size`, x86-64 `-O3`):

| symbol | before | after | saved |
|---|---:|---:|---:|
| `faceIjkBaseCells` | 4,320 | 1,080 | 3,240 |
| `baseCellNeighbors` | 3,416 | 854 | 2,562 |
| `baseCellData` | 3,416 | 854 | 2,562 |
| `baseCellNeighbor60CCWRots` | 3,416 | 854 | 2,562 |
| `faceNeighbors` (dead, see F-14) | 1,600 | 0 | 1,600 |
| `NEW_DIGIT_II/III`, `NEW_ADJUSTMENT_II/III` | 784 | 196 | 588 |
| `maxDimByCIIres`, `unitScaleByCIIres` (dead) | 136 | 0 | 136 |
| `regionLookup` | 18,224 | 18,224 | 0 |
| **static data total** | **36,720** | **23,470** | **13,250** |
| **library total (text+data+bss)** | **62,970** | **46,736** | **16,234** |

**Implementation note that keeps the diff tiny:** introduce byte-width mirrors with the *same brace nesting* —

```c
typedef struct { int8_t i, j, k; } CoordIJK8;
typedef struct { int8_t face; CoordIJK8 coord; } FaceIJK8;
typedef struct { FaceIJK8 homeFijk; int8_t isPentagon; int8_t cwOffsetPent[2]; } BaseCellData;
typedef struct { uint8_t baseCell; int8_t ccwRot60; } BaseCellRotation;
```

All 122 `baseCellData` initializer lines and all 540 `faceIjkBaseCells` entries then compile **unchanged** — only the type names in the declarations move.

### F-13 — README memory figures are ~2× low

README claims Core Code ~2–4 KB, Lookup Tables ~18 KB, **Total ~20–22 KB**. Actual static data alone is **~36.5 KB**; the five objects total **~63 KB** text+data on x86-64 `-O3`. Update after F-12 lands.

### F-14 — Dead code, all externally linked

Defined, exported, never called anywhere in the repository:

`_adjustOverageClassII` (and its only consumers `maxDimByCIIres`, `unitScaleByCIIres`, `faceNeighbors`), `_upAp3`, `_downAp3`, `_downAp3r`, `_ijkToHex2d`, `_v2dMag`, `_v2dAlmostEquals`, `_v2dIntersect`, `_geoAzDistanceRads`, and the degrees-taking wrapper `geoToFaceIjk` (confusable with `_geoToFaceIjk`, which takes radians).

Because these are non-`static` externs in a static library, they survive unless the integrator links with `--gc-sections` — which the README never mentions. Either delete them or make them `static`, and document the linker flag.

### F-15 — `regionNames` lands in RAM

`const char* regionNames[16]` is an array of `const char*`, not a `const` array. Verified with `-fno-pie` (bare-metal-like):

```
before:  D regionNames    <- .data, 64 B of RAM on 32-bit + startup copy
after:   R regionNames    <- .rodata
```

Change to `const char* const regionNames[16]` in both the header and the generator template.

### F-16 — Region ID field is full

`regionId` is 4 bits and IDs 1–15 are all allocated. **KZ865, CN779, AS923-1A and AS923-1D cannot be added without a format change.** Ten bits `[9:0]` are reserved and zero, so the headroom exists — but decide the migration now, while it is cheap. Note `REGION_RESTRICTED` (255) can never be represented in the table either (see F-22).

---

## 6. P2 — Tests, build, generator

### F-17 — `h3lite_grid_test.c` carries a stale private name table

**File:** `test/h3lite_grid_test.c`, `print_statistics()`

This is the exact bug that was fixed in `h3lite.c` as H3-7 but never propagated. A private 13-entry `region_names[]` omits AS923-1B and AS923-1C, so every ID ≥ 5 is mislabelled, and `for (i = 0; i <= 12; i++)` means CN470, EU433 and CD900-1A never appear at all.

Observed output vs truth:

```
printed "KR920:  2 points"   -> actually AS923-3
printed "RU864:  1 points"   -> actually KR920
printed "CN470:  9 points"   -> actually IN865
printed "EU433: 40 points"   -> actually RU864
CN470 / EU433 / CD900-1A     -> never printed
```

**Fix:** call `getRegionName()` and iterate to `ARRAY_SIZE(stats->region_counts)`. Also guard the `100.0 * x / stats->valid_h3` divisions against `valid_h3 == 0`.

### F-18 — C tests never fail

Neither `h3lite_grid_test` nor `h3lite_nearest_test` returns a non-zero exit code. `make test` always "passes" regardless of results. The Python harnesses (`t_table`, `t_index`, `t_city`) do this correctly — match them. `test/xval_pts.c` is not in the Makefile at all.

### F-19 — Makefile

- No header dependency tracking. Add `-MMD -MP` to `CFLAGS` and `-include $(OBJ_DIR)/*.d`. Given how macro-driven this library is, editing `h3lite_constants.h` currently rebuilds nothing.
- `STM32_CFLAGS` hardcodes `-DSTM32F411xE` and `-DUSE_HAL_DRIVER`. The stated target is the STM32WLE5 — a different part. `-DUSE_HAL_DRIVER` is meaningless for a standalone library.
- `make test`'s `@echo "\n..."` prints a literal `\n` under `/bin/sh`.

### F-20 — Generator robustness

**File:** `generate_lookup_table.py`

- **Polygon holes are dropped.** `convert_region_to_h3` uses only `poly.exterior.coords` and passes no holes to `h3.LatLngPoly`. Enclaves — Lesotho in South Africa, Vatican and San Marino in Italy — get filled solid. Fix: `h3.LatLngPoly(exterior, *holes)` built from `poly.interiors`.
- **`pack_entry` masks silently.** `region_id & 0x0F` turns a new region 16 into 0 = Unknown; `resolution & 0x03` turns a res-4 table into res 0. Raise `ValueError` instead — the field widths are far too tight for silent truncation.
- **`SEAWARD_BUFFER_DEG = 0.6  # ~60 km` is only true in longitude near the equator.** At 60°N it is ~33 km. Buffer in a projected CRS or scale by `cos(lat)`.
- **Conflict areas are planar degrees².** Antimeridian-crossing cell polygons from `_cell_polygon()` get nonsensical areas, making conflict resolution near the dateline unreliable.
- **`regions[region_name] = geoms` overwrites** when two files substring-match the same region key. Use `setdefault(...).extend(...)`.
- **`max_id` sizing assumes contiguous IDs.** A gap in `REGION_IDS` misaligns the emitted `regionNames[]`.
- **`load_regions(directory="..")` default contradicts `main()`'s `'../hplans'`.** The function default is dead and misleading.
- **`best = None` is dead.** The duplicate-key check uses `assert`, which vanishes under `python -O`.

### F-21 — Table has no provenance and cannot be reproduced

The GeoJSON inputs are not in the repository, and the generated `.c`/`.h` carry no source list, no timestamp, no `h3` version. There is currently **no way to distinguish "the generator dropped Mongolia" from "Mongolia was never in the input"** — which blocks root-causing four of the six data-level P0s.

**Action:** commit the GeoJSON set (or pin the upstream `hplans` revision), and emit a provenance header containing UTC timestamp, `h3.__version__`, resolution, and a per-file SHA-256 of every input.

### F-22 — Stale / misleading declarations

- **`analyze_regions.py` is actively harmful.** It still emits the pre-H3-1 legacy struct format:
  ```c
  { 15, 234, REGION_AU915 }, // Sydney
  ```
  Anyone following its "sample patch" would corrupt the packed table. It also references a non-existent `h3lite_test.c` and is framed around a long-fixed Sydney bug. Delete or port it.
- **`regions_h3_conversion.py` and `test/visual_grid_test.py` run `pip install` at import time**, at module scope, before `--help` is parsed.
- **`harness_common.run_points()`** consumes `points` to build `data`, then calls `len(list(points))` **twice**. Correct for a list, silently broken for a generator, and two extra O(n) copies of 259,200 tuples.
- **`REGION_RESTRICTED` (255) is unreachable.** It is defined, `getRegionName` handles it, and a code comment says enforcement keys on it — but the generator has no restricted-territory concept and the 4-bit field cannot hold 255. Either implement it or mark it explicitly as a reserved, unimplemented hook.
- **`RES0_U_GNOMONIC` is defined as `2.6181773447340`.** In reference H3 that constant is `0.38196601125010500003`; `2.618…` is its inverse (and `INV_RES0_U_GNOMONIC` already holds `2.61803398874989588842`). Unused today, so harmless — but a trap.
- **Unused constants:** `MAX_REGIONS` (stale at 12; there are 16), `INVALID_FACE`, `NUM_PENTAGONS`, `NUM_HEX_VERTS`, `NUM_PENT_VERTS`, `EARTH_RADIUS_KM`, `M_SIN60`, `M_COS60`, `M_RSQRT7`, `NUM_DIGITS`, `IJ`, `KI`.
- **`H3LITE_DEBUG_PRINT` expands to `printf`** but `h3lite.h` does not include `<stdio.h>`. Defining `H3LITE_DEBUG` breaks the build.
- **`src/h3lite.c` includes `<stdio.h>`, `<string.h>`, `<stdlib.h>`**, none of which it uses.

### F-23 — `findNearestRegions` distance model

```c
result.regions[...].distanceKm = k * 65.0;   // "~65km per ring at resolution 3"
```

Res-3 mean hexagon edge is ~69 km, so centre-to-centre spacing is √3 × 69 ≈ **120 km**. The estimate is low by ~45%. The nearest-neighbour test's "No regions found within 3 rings (~195km)" should read ≈ 360 km. If `distanceKm` is used anywhere to decide whether to trust a nearest-region answer, that threshold is off by nearly a factor of two.

Separately: the function breaks out of the ring loop on the first productive ring, so it returns whatever that ring found and never compares against a genuinely closer region reachable via a different ring. That is a defensible design, but it should be documented as "first ring with any hit" rather than "nearest".

---

## 7. Suggested order of work

1. **F-01 + F-08 together.** The pentagon predicate fix widens the code paths that touch `baseCellData`; fix the type mismatch in the same change. Add `-flto` to CI so the class of bug cannot recur.
2. **F-03.** Cheapest fix, largest on-target safety value. Include the `h3 == 0` guard in `findNearestRegions`.
3. **F-21.** Commit or pin the GeoJSON. Everything in F-02/F-04/F-05 is guesswork until inputs are reproducible.
4. **F-02.** Fix the tie-break, regenerate, re-check the DRC and the SARs.
5. **F-04.** With inputs available, trace where Mongolia and Fiji are lost.
6. **F-09 / F-10 / F-11.** Integration surface; small and independent.
7. **F-12 / F-14 / F-15.** Size. Mechanical; use the same-brace-shape trick so the diff stays reviewable.
8. **F-17 / F-18 / F-19.** Make the test suite capable of failing.
9. **F-13 / F-22 / F-23 and the README rewrite.**

---

## 8. Hypotheses tested and found FALSE — do not implement

These were plausible and were investigated. Each was **disproved empirically**. Do not spend effort on them.

### ✗ "Missing antimeridian handling causes the Fiji / Kiribati / Tuvalu gaps"

`generate_lookup_table.py` has no dateline splitting, while `regions_h3_conversion.py` has `check_crosses_idl()` and `split_polygon_at_dateline()`. This looked like the smoking gun. **It is not.**

`h3.h3shape_to_cells` in h3 4.5.0 already handles dateline-crossing `LatLngPoly` correctly. Tested with a Fiji-shaped ring (176.9E → 178.2W) and a Russia-shaped ring (19E → 169W): polyfill results are **set-identical** with and without splitting (48 and 1,932 cells respectively), Moscow and Anadyr are both inside the Russia ring, and mid-Atlantic correctly stays outside. Adding splitting is harmless defence-in-depth but fixes nothing observed.

### ✗ "Res-3 polyfill granularity causes the Fiji gap"

Also disproved. A 0.6° seaward buffer around Viti Levu + Vanua Levu takes the polyfill from 3 cells to 6 cells and **does cover Suva**. Islands of that size are rescued by the existing buffer.

**Conclusion:** the F-04 gaps are most likely missing source data, not a generator defect. Confirm via F-21 before writing any code.

### ✗ "The `baseCellData` type mismatch is producing wrong results today"

It is real UB and must be fixed, but at the reviewed commit it changes nothing observable — 15,000 ring cases diffed identical. Fix it for correctness and for what F-01 unlocks, not because it is currently corrupting output.

---

## 9. Regression harness

`verify_equivalence.py` (provided alongside this document) builds a pre-change and post-change tree and asserts:

1. `latLngToH3` + `latLngToRegion` byte-identical over ~216,200 points (200k random-on-sphere + a dense 2° global grid).
2. `h3GetRing` for k=1..6 over 8,000 origins: **wrong count must be 0**, clean failures must not increase, exact matches must not decrease. (Not "identical" — the F-01 fix deliberately converts failures into correct rings.)
3. Agreement with reference `h3` stays at 0 mismatches.
4. Non-finite and out-of-domain input returns 0.

```bash
python3 verify_equivalence.py <old_tree> <new_tree>
```

Requires `gcc` and `pip install h3==4.5.0`. A reference run of the full patch set produced:

```
[PASS] index+region identical over 216200 points
[PASS] h3GetRing k=1: exact 7351->7989, clean-fail 649->11, wrong 0->0
[PASS] h3GetRing k=2: exact 7205->7974, clean-fail 795->26, wrong 0->0
[PASS] h3GetRing k=3: exact 7051->7948, clean-fail 949->52, wrong 0->0
[PASS] h3GetRing k=4: exact 6918->7918, clean-fail 1082->82, wrong 0->0
[PASS] h3GetRing k=5: exact 6753->7873, clean-fail 1247->127, wrong 0->0
[PASS] h3GetRing k=6: exact 6562->7828, clean-fail 1438->172, wrong 0->0
[PASS] reference h3 agreement: 0 mismatches over 216200 points
[PASS] non-finite / out-of-domain input returns 0 (intended change; old build was UB)

RESULT: all checks passed
```

A reference patch implementing F-01, F-03, F-08, F-09, F-11, F-12, F-14, F-15, F-17, F-19, F-20 and F-21 is available as `h3lite-review-patch.diff`. It is not required — reimplement independently if you prefer — but the numbers above are what a correct implementation should reproduce.
