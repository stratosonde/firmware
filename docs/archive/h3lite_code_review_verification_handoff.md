# H3Lite Code Verification and Remediation Handoff

**Repository:** https://github.com/stratosonde/h3lite  
**Audit basis:** Static inspection of the `main` branch as available on 2026-08-02  
**Audience:** Coding LLM / software engineer performing an independent code-level verification  
**Primary target:** STM32WLE5-class embedded firmware performing automatic LoRaWAN region selection  
**Important use case:** `findNearestRegions()` / ring search is used routinely, not merely as an optional diagnostic feature.

---

## 1. Assignment

Independently inspect the entire repository, reproduce or reject every finding below, implement only confirmed fixes, and return a final verification report.

Do **not** assume this document is correct. For each item:

1. Locate the exact code.
2. Classify it as:
   - **Confirmed**
   - **Already fixed**
   - **Not reproducible**
   - **Intentional design**
   - **Needs product decision**
3. Explain the real runtime consequence.
4. Make the smallest safe correction.
5. Add an automated regression test.
6. Run the complete test suite and include the commands and results.
7. Record the exact Git commit SHA reviewed.

Preserve the project's embedded constraints:

- no heap allocation in the runtime library;
- compact flash representation;
- predictable execution time;
- primary lookup resolution is H3 resolution 3;
- STM32-specific build support is useful and does not need to be replaced with a generic build system;
- avoid broad rewrites when a narrow fix is sufficient.

---

## 2. Required first steps

```bash
git clone https://github.com/stratosonde/h3lite.git
cd h3lite
git rev-parse HEAD
git status --short
git log -1 --date=iso --format=fuller
```

Create an isolated Python environment and use the H3 version expected by the repository's differential test:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install "h3==4.5.0" shapely tqdm
```

Run the existing tests before changing anything:

```bash
make clean
make test
python test/t_index.py
python test/t_city.py
python test/t_table.py
```

Capture all output. Do not treat printed `PASS`/`FAIL` text as authoritative unless the process exit status also reflects failure.

---

## 3. Product decisions already made

### 3.1 Restricted-region policy

A restricted area should be represented as a region-like value. The application layer will decide that it must not transmit.

However, verify the encoding carefully:

- `REGION_UNKNOWN` is currently `0`.
- The generated region IDs currently occupy **every value from 1 through 15**.
- The packed entry currently gives the region ID four bits.
- Therefore the four-bit namespace has **no unused code**.
- The packed `uint32_t` does have **ten currently reserved bits**, so widening the field is possible without increasing table size.

Choose and document one of these policies:

#### Option A — repurpose an existing four-bit ID

Use one existing ID, probably ID 15, as `RESTRICTED`.

This preserves the exact bit layout but removes or remaps the existing region using that ID. At present ID 15 is `CD900-1A`.

Use this only if that region is deliberately unsupported.

#### Option B — widen the field to five bits

Preserve IDs 1–15 and assign:

```c
#define REGION_RESTRICTED 16
```

Consume one of the ten reserved bits. This is the smallest format change that preserves every current region.

A possible layout is:

```text
bits [31:25] baseCell
bits [24:23] resolution
bits [22:14] partialIndex
bits [13:9]  regionId
bits [8:0]   reserved
```

The binary-search key remains bits `[31:14]`, so `RE_KEY_MASK` may remain unchanged. Verify this rather than assuming it.

#### Option C — retain the public value 255

Use eight region bits, for example bits `[13:6]`, leaving six reserved bits. This preserves:

```c
#define REGION_RESTRICTED 255
```

This is slightly more future-proof but wider than presently required.

### 3.2 Generator polygon holes

The current regulatory source polygons reportedly do not intentionally contain exclusion holes. Hole support is therefore lower priority, but it is still cheap to implement correctly and should not be broken by geometry repair.

### 3.3 Build portability

Do not spend time replacing the STM32-specific build merely for portability. The project is allowed to target STM32. The useful build fix is automatic header dependency tracking and reliable test targets.

### 3.4 Pentagon behavior

Pentagon behavior is important because nearest-region ring search is used frequently. It must be tested and either made correct or explicitly handled as an error. Do not dismiss it as theoretical.

---

# 4. Findings to verify

## F-01 — Restricted value is not encodable in the current table

**Priority:** Critical  
**Status from static inspection:** Confirmed design mismatch

### Evidence

`include/h3lite.h` defines:

```c
#define REGION_UNKNOWN      0
#define REGION_RESTRICTED   255
```

Current generator mapping uses all IDs 1–15:

```python
REGION_IDS = {
    "US915": 1,
    ...
    "CD900-1A": 15,
}
```

Current packed format:

```python
(region_id & 0x0F) << 10
```

Current unpacking:

```c
#define RE_REGION(e) ((RegionId)(((e) >> 10) & 0x0F))
```

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/include/h3lite.h
- https://github.com/stratosonde/h3lite/blob/main/generate_lookup_table.py

### Consequence

A generated table entry cannot return `255`. If `255` is passed to the packer, it truncates to 15 and becomes indistinguishable from the current ID-15 region.

### Required action

Implement the selected policy from Section 3.1.

Also decide and test:

- whether a direct lookup inside a restricted cell returns `RESTRICTED`;
- whether a restricted region is returned by `findNearestRegions()` when the device is outside it;
- whether restricted cells should block all searching when the current cell is restricted;
- whether restricted cells should be excluded as offshore nearest-transmission candidates.

The safest semantic split is usually:

- current cell is restricted → hard block transmission;
- restricted region found only as a nearby candidate → do not attempt that plan; continue looking for an allowed candidate if product policy permits.

### Acceptance tests

1. Pack and unpack the restricted value without truncation.
2. `getRegionName(RESTRICTED)` returns `"RESTRICTED"`.
3. A known restricted test cell returns the restricted ID.
4. Unknown/ocean remains distinct from restricted.
5. Application-facing tests prove no transmission plan is selected for a restricted current cell.
6. Table audit accepts the new bit layout.
7. Every existing valid region ID still decodes correctly.

---

## F-02 — Nonfinite and invalid geographic inputs are not rejected

**Priority:** High  
**Status from static inspection:** Confirmed

### Evidence

`latLngToH3()` validates only resolution before passing values through trigonometry and eventually into integer coordinate conversion.

It does not reject:

- `NaN`;
- positive or negative infinity;
- latitude outside `[-90, 90]`.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/src/h3lite.c

### Consequence

NaN can propagate into floating-point calculations and ultimately reach floating-to-integer conversion. That can produce undefined or implementation-dependent behavior. Invalid latitude can produce nonsensical indexes.

### Required action

At the public boundary:

```c
if (!isfinite(lat) || !isfinite(lng)) {
    return 0;
}
if (lat < -90.0 || lat > 90.0) {
    return 0;
}
```

Choose and document longitude behavior:

- reject longitude outside `[-180, 180]`; or
- normalize longitude modulo 360 consistently.

Match official H3 behavior where practical.

### Acceptance tests

Test at least:

```text
NaN latitude
NaN longitude
+Infinity
-Infinity
latitude 90
latitude -90
latitude 90 + epsilon
latitude -90 - epsilon
longitude 180
longitude -180
longitude 540
longitude -540
```

Run under UBSan.

---

## F-03 — Generator accepts resolutions incompatible with the packed table

**Priority:** Critical for table generation  
**Status from static inspection:** Confirmed

### Evidence

The CLI accepts any integer:

```python
parser.add_argument('--resolution', type=int, ...)
```

But:

- resolution has only two packed bits;
- `extract_h3_components()` retains at most three child digits;
- runtime lookup probes only resolutions 3, 2, and 1;
- the compaction logic and comments are designed around resolution 3.

At resolution 4, `(resolution & 0x03)` becomes zero and the fourth H3 digit is discarded.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/generate_lookup_table.py
- https://github.com/stratosonde/h3lite/blob/main/src/h3lite.c

### Consequence

`--resolution 4` can generate collisions, malformed keys, or entries that runtime lookup cannot use correctly.

### Required action

For the current design, fail immediately unless resolution is exactly 3:

```python
if args.resolution != 3:
    parser.error("The current packed H3Lite region table supports resolution 3 only")
```

Alternatively, redesign the entire representation and runtime probe path, but do not imply that arbitrary resolution already works.

### Acceptance tests

- `--resolution 3` succeeds.
- `--resolution 2`, `4`, negative values, and excessive values fail with nonzero status and do not modify generated output files.
- Generated table contains only resolutions 1–3 after compaction.

---

## F-04 — Polygon conversion can silently distort or omit regulatory coverage

**Priority:** High  
**Status from static inspection:** Confirmed

### Evidence

When conversion fails, the generator tries:

1. reversed coordinates;
2. simplified geometry;
3. buffered geometry;
4. convex hull.

If all approaches fail, it prints a message and continues.

The convex hull can cover territory that was never in the original polygon. Simplification and buffering can also alter a boundary.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/generate_lookup_table.py

### Consequence

The generated regulatory table can look valid while containing enlarged, altered, or missing territory.

### Required action

Make generation fail closed:

1. validate each input geometry;
2. use a deliberate geometry-repair operation such as Shapely `make_valid` where available;
3. preserve all polygon components and holes;
4. compare repaired area and bounds with the original;
5. abort on unresolved conversion errors;
6. do not use convex hull as an automatic fallback;
7. write output atomically only after every validation passes.

If a lossy fallback is retained for development, require an explicit CLI flag such as `--allow-lossy-repair`, print a prominent warning, and record it in the output manifest.

### Acceptance tests

- malformed polygon causes nonzero exit;
- existing generated files are unchanged after failure;
- convex-hull expansion is never silently used;
- valid repaired geometry produces expected cells;
- all skipped features are counted and cause failure by default.

---

## F-05 — Interior polygon rings are ignored

**Priority:** Medium / low for current dataset  
**Status from static inspection:** Confirmed

### Evidence

The generator creates:

```python
h3.LatLngPoly(exterior)
```

but does not pass `poly.interiors`.

### Consequence

Any hole in a polygon is filled as though it were part of the region.

### Required action

Construct the H3 polygon with outer and inner rings:

```python
outer = [(lat, lon) for lon, lat in poly.exterior.coords]
holes = [
    [(lat, lon) for lon, lat in ring.coords]
    for ring in poly.interiors
]
shape = h3.LatLngPoly(outer, *holes)
```

Use the exact constructor contract for the installed `h3==4.5.0`.

### Acceptance test

Create a synthetic donut polygon. The outer ring must be assigned and the center hole must remain unassigned.

---

## F-06 — Region-file loading is nondeterministic and can overwrite geometry

**Priority:** High for reproducible generation  
**Status from static inspection:** Confirmed

### Evidence

The loader uses unsorted `os.listdir()` and later assigns:

```python
regions[region_name] = geoms
```

If multiple filenames map to the same region name, later files replace earlier files. Filesystem enumeration order can differ.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/generate_lookup_table.py

### Consequence

Islands, territories, or split source files may be lost. Generated output may differ between systems.

### Required action

```python
files = sorted(
    f for f in os.listdir(directory)
    if f.endswith(".geojson")
)

regions.setdefault(region_name, []).extend(geoms)
```

Also:

- reject duplicate IDs with ambiguous policy;
- log every source file contributing to each region;
- generate a manifest with input filenames and SHA-256 hashes.

### Acceptance tests

- two files for the same region are merged;
- reversing directory creation order does not change generated output;
- repeated runs produce byte-identical C and header files.

---

## F-07 — Buffering and overlap areas are computed in longitude/latitude degrees

**Priority:** Medium to high, depending on regulatory accuracy requirements  
**Status from static inspection:** Confirmed design risk

### Evidence

The generator uses:

```python
SEAWARD_BUFFER_DEG = 0.6
g.buffer(SEAWARD_BUFFER_DEG)
unions[rid].intersection(cpoly).area
```

on geographic longitude/latitude coordinates.

### Consequence

A degree is not a constant physical distance. Longitude scale changes with latitude, and planar calculations are problematic near the antimeridian and poles. The phrase “~60 km” is only approximate and latitude-dependent.

### Required action

Evaluate one of these approaches:

1. **H3-native expansion:** polyfill true cells, then use one-cell H3 grid expansion for maritime coverage.
2. **Projected geometry:** transform each geometry to a suitable equal-area/distance CRS before buffering and area comparison.
3. **Explicit policy:** use a regulatory priority table for contested cells rather than area measured in raw degrees.

Do not alter this blindly. Compare the resulting table against the current table and inspect all changed boundary cells.

### Acceptance tests

- antimeridian region test;
- high-latitude region test;
- equivalent physical buffer at low and high latitude;
- deterministic conflict winner;
- visual diff of every changed cell.

---

## F-08 — Pentagon ring traversal can skip the genuinely nearest region

**Priority:** Critical because ring search is used routinely  
**Status from static inspection:** Confirmed limitation; real operational impact must be measured

### Evidence

`h3GetRing()` returns a negative value when:

- origin is a pentagon;
- traversal reaches a pentagon;
- ring closure fails due to distortion.

`findNearestRegions()` ignores a negative ring result and continues to the next `k`.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/src/h3lite_neighbor.c
- https://github.com/stratosonde/h3lite/blob/main/src/h3lite.c
- official H3 traversal behavior: https://h3geo.org/docs/api/traversal/

Official H3 distinguishes a fast unsafe ring traversal from a safe fallback. `gridRingUnsafe` may fail on pentagonal distortion; `gridRing` uses a more robust method.

### Consequence

If an inner ring fails, the code may later report a farther region while missing a closer region in the failed ring. The README claim that nearest lookup “still works” must be proven, not assumed.

### Required action

Build a complete pentagon differential test first.

At resolution 3:

1. obtain all 12 pentagons from official H3;
2. gather origins in a disk around each pentagon;
3. compare H3Lite ring results for `k = 0..6` against official safe `grid_ring`;
4. record:
   - exact matches;
   - clean failures;
   - partial or wrong-content output;
5. test `findNearestRegions()` using a reference implementation based on safe traversal.

If failures can change the selected nearest region, implement a safe fallback. A compact breadth-first traversal using a fixed-size visited array may be acceptable for maximum radius 6 and avoids heap allocation.

Do not merely skip the failed ring.

### Acceptance criteria

- no wrong-content rings;
- no missed nearer region in nearest-region lookup;
- deterministic output around all 12 pentagons;
- bounded stack and execution time;
- documented maximum memory use;
- reference comparison covers all origins near all pentagons, not a small sample.

---

## F-09 — `findNearestRegions()` semantics do not necessarily return three nearest regions

**Priority:** High  
**Status from static inspection:** Confirmed

### Current behavior

- If the current cell has a region, it returns immediately with one result.
- During offshore search, it stops after the first ring containing any region.
- If that ring contains only one distinct region, it returns one, even if a second and third region exist in the next ring.
- `distanceKm` is `k * 65.0`, not a measured distance.
- Results within one ring are in traversal/discovery order, not sorted by physical distance.

### Consequence

The API name and documentation can be interpreted more strongly than the implementation guarantees.

### Required product decision

Choose one contract:

#### Contract A — first matching ring

Rename/document the function as returning distinct regions found in the first ring that contains any match.

#### Contract B — up to three nearest region plans

Continue searching until three distinct allowed regions are found or `maxRings` is exhausted. Sort by:

1. ring distance;
2. measured center-to-center distance, if required;
3. stable region ID tie-break.

For routine communication fallback, Contract B is likely more useful.

### Distance field

Either:

- rename it to `approximateDistanceKm`;
- return ring distance only; or
- calculate a real great-circle distance to a representative cell center/boundary.

### Acceptance tests

- first matching ring has one region; next ring has two others;
- duplicate region cells are deduplicated;
- restricted region candidate behavior follows product policy;
- results are deterministic;
- max ring 0, negative, 1, and 6;
- no result;
- current cell assigned;
- current cell restricted.

---

## F-10 — Public ring API lacks validation and output capacity

**Priority:** Medium  
**Status from static inspection:** Confirmed API weakness

### Evidence

```c
int h3GetRing(H3Index origin, int k, H3Index *out);
```

The caller is responsible for allocating `6 * k` cells. The function does not receive capacity. It also does not visibly reject:

- `out == NULL`;
- `k < 0`;
- invalid origin index.

For negative `k`, loop structure may return a nonsensical result rather than an error.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/include/h3lite_faceijk.h
- https://github.com/stratosonde/h3lite/blob/main/src/h3lite_neighbor.c

### Required action

At minimum:

```c
if (out == NULL || k < 0 || origin == 0) {
    return H3LITE_ERROR;
}
```

Prefer a capacity-aware API if it remains public:

```c
int h3GetRing(
    H3Index origin,
    int k,
    H3Index *out,
    size_t capacity
);
```

Or make it private if only `findNearestRegions()` needs it.

### Acceptance tests

- null output;
- negative `k`;
- zero origin;
- undersized output;
- exact capacity;
- `k == 0`;
- `k == 6`;
- pentagon failure does not expose partially written data as valid.

---

## F-11 — `h3ToRegion()` does not fully validate arbitrary H3 indexes

**Priority:** Medium  
**Status from static inspection:** Likely confirmed; inspect complete call path

### Evidence

`h3ToRegion()` checks only initialization and nonzero input before extracting:

- resolution;
- base cell;
- first three digits.

It does not visibly validate:

- cell mode;
- reserved/high bits;
- base cell range;
- resolution range;
- unused digits;
- deleted pentagon subsequence.

### Consequence

A fabricated or corrupted 64-bit value may match a valid compact table key.

### Required action

Either:

1. add `h3liteIsValidCell()` and validate public input; or
2. make `h3ToRegion()` internal/trusted-input-only and expose only coordinate lookup.

Match the subset of official `isValidCell()` required for resolutions 0–4.

### Acceptance tests

Generate invalid indexes covering each malformed field and prove they return unknown/error without table lookup.

---

## F-12 — The default test target omits the strongest tests

**Priority:** High  
**Status from static inspection:** Confirmed

### Evidence

`make test` runs only:

- `h3lite_grid_test`;
- `h3lite_nearest_test`.

It does not run:

- `test/t_index.py`;
- `test/t_city.py`;
- `test/t_table.py`.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/Makefile

### Required action

Add a host verification target, for example:

```make
test: c_tests py_tests

py_tests:
	python3 test/t_index.py
	python3 test/t_city.py
	python3 test/t_table.py
```

Pin dependencies in a requirements file.

Add CI that runs on every push and pull request.

### Acceptance criteria

One command runs all required correctness gates and returns nonzero on any failure.

---

## F-13 — C tests print failures but may still exit successfully

**Priority:** High  
**Status from static inspection:** Confirmed

### Evidence

The grid test prints `PASS` or `FAIL` for known cities but `main()` returns zero regardless of failed comparisons. The nearest test is mainly diagnostic output and has few or no assertions.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/test/h3lite_grid_test.c
- https://github.com/stratosonde/h3lite/blob/main/test/h3lite_nearest_test.c

### Consequence

CI can be green despite visible failed test output.

### Required action

Count failures and return nonzero:

```c
return failures == 0 ? 0 : 1;
```

Convert nearest tests into expected-result assertions.

### Additional stale test issue

`h3lite_grid_test.c` has a hardcoded region-name array that no longer matches the current full ID map. Verify every test-side ID-to-name mapping and remove duplicate mappings where possible.

---

## F-14 — Python differential harness consumes iterators twice

**Priority:** Medium  
**Status from static inspection:** Confirmed

### Evidence

`run_points(points)` iterates `points` to build input, then calls `list(points)` later to check length. A generator will be exhausted.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/test/harness_common.py

### Required action

Materialize once:

```python
points = list(points)
data = "".join(...)
...
if len(out) != len(points):
    ...
```

### Acceptance test

Pass a generator expression and verify all rows are processed.

---

## F-15 — Initialization is mutable global state but performs no real initialization

**Priority:** Low to medium  
**Status from static inspection:** Confirmed

### Evidence

`h3liteInit()` only sets a static Boolean. Coordinate conversion works without initialization, while `h3ToRegion()` returns unknown if initialization was forgotten.

### Consequence

A missed initialization call becomes indistinguishable from an ocean/no-region result. That is a poor failure mode for regulatory selection.

### Required action

Prefer removing initialization entirely if no initialization is required.

If initialization is retained, return a distinct error rather than `REGION_UNKNOWN`, and enforce the rule consistently.

### Acceptance tests

- every public function before initialization;
- after initialization;
- repeated initialization;
- missing initialization cannot silently look like ocean coverage.

---

## F-16 — Build does not track header dependencies

**Priority:** Medium  
**Status from static inspection:** Confirmed

### Evidence

Object rules depend only on `.c` files.

### Consequence

Changing a header or generated table header may leave stale object files until a clean rebuild.

### Required action

Add compiler-generated dependencies:

```make
CFLAGS += -MMD -MP
STM32_CFLAGS += -MMD -MP

DEPS = $(OBJECTS:.o=.d) $(STM32_OBJECTS:.o=.d)
-include $(DEPS)
```

Adapt to the actual Makefile variable names.

Do not unnecessarily remove STM32-specific targets.

---

## F-17 — Documentation and constants are stale

**Priority:** Medium  
**Status from static inspection:** Confirmed in several places

Verify and correct at least:

- `MAX_REGIONS` is still 12 while current generated IDs extend to 15.
- README project structure omits current base-cell and neighbor sources.
- README STM32 integration list may omit required files.
- README claims fixed-point options although the visible implementation uses `double`.
- README stack estimate appears too low: `H3Index ringCells[42]` alone is 336 bytes.
- comments say nearest ring distance is 1–3 while runtime clamps to 6;
- resolution claims imply broader support than the current packed region table can safely provide;
- README says initialization is mandatory although it is a no-op and is inconsistently enforced;
- generated-table bit-layout documentation must change if restricted encoding changes.

Relevant source:

- https://github.com/stratosonde/h3lite/blob/main/README.md
- https://github.com/stratosonde/h3lite/blob/main/include/h3lite_constants.h
- https://github.com/stratosonde/h3lite/blob/main/include/h3lite.h

### Acceptance criteria

Documentation matches measured code and memory behavior. Do not state performance or memory numbers without a reproducible measurement command and target configuration.

---

## F-18 — Internal implementation APIs are exposed publicly

**Priority:** Low to medium  
**Status from static inspection:** Confirmed

### Evidence

`include/h3lite_faceijk.h` exposes many underscore-prefixed internal functions and structures, as well as neighbor traversal internals.

### Consequence

The public API becomes difficult to stabilize and allows malformed intermediate data.

### Required action

Separate:

- installed public API;
- private internal headers;
- test-only internals.

Make file-local helpers `static` where possible.

Do this after correctness fixes, not as an invasive first change.

---

## F-19 — Legacy and visualization utilities are inconsistent with the current format

**Priority:** Low  
**Status from static inspection:** Confirmed examples

### Evidence

`test/visualize_table.py` still parses legacy struct entries:

```text
{ baseCell, partialIndex, regionId }
```

while the current generated table contains packed `uint32_t` values.

Other scripts install Python packages at runtime and contain older processing paths.

Relevant sources:

- https://github.com/stratosonde/h3lite/blob/main/test/visualize_table.py
- https://github.com/stratosonde/h3lite/blob/main/test/visual_grid_test.py
- https://github.com/stratosonde/h3lite/blob/main/regions_h3_conversion.py
- https://github.com/stratosonde/h3lite/blob/main/analyze_regions.py

### Required action

Classify each script as:

- supported;
- development-only;
- obsolete.

Update or archive obsolete scripts. Do not allow scripts to install packages automatically during normal execution. Use pinned requirements.

---

## F-20 — Generated regulatory data lacks a complete provenance manifest

**Priority:** High for auditability  
**Status from static inspection:** Confirmed

### Required action

Generate or commit a manifest containing:

```text
generator Git SHA
input dataset name/version
every input filename
SHA-256 for each input
h3 Python version
Shapely version
generation options
generation date
output entry count
output SHA-256
number of conflicts
number of repairs
number of skipped features
```

The build should fail if the generated C/header does not match the manifest or source data.

This matters because the table is regulatory decision data, not merely a cache.

---

## F-21 — License and attribution should be audited against upstream H3

**Priority:** Medium  
**Status from static inspection:** Requires legal/maintainer verification

H3Lite is not merely the full H3 library repackaged, but several algorithms and tables are explicitly ported or based on Uber H3. Upstream H3 is Apache-2.0 and includes both `LICENSE` and `NOTICE`.

Official upstream:

- https://github.com/uber/h3
- https://github.com/uber/h3/blob/master/LICENSE
- https://github.com/uber/h3/blob/master/NOTICE

### Required action

For every substantially copied or adapted file:

1. identify the upstream source file and version/commit;
2. retain applicable copyright and license notices;
3. mark that the file was modified;
4. include any required upstream NOTICE material;
5. keep the repository Apache-2.0 license;
6. document provenance in a `THIRD_PARTY_NOTICES.md` or equivalent.

This is a maintainership checklist, not legal advice.

---

## F-22 — Debug macro may require `<stdio.h>` in the public header path

**Priority:** Low  
**Status from static inspection:** Likely confirmed

`h3lite.h` expands `H3LITE_DEBUG_PRINT` to `printf` when debugging is enabled, but the header itself does not include `<stdio.h>`.

### Required action

Either:

- include `<stdio.h>` when `H3LITE_DEBUG` is defined; or
- avoid a public-header `printf` macro and route diagnostics through a user-supplied callback.

### Acceptance test

Compile a minimal consumer that includes only `h3lite.h` with `-DH3LITE_DEBUG -Werror`.

---

# 5. Required pentagon test plan

This test is mandatory because nearest-region search is operationally important.

## 5.1 Ring-level differential test

Using `h3==4.5.0`:

```python
pentagons = h3.get_pentagons(3)
```

For each pentagon:

1. generate origins in `h3.grid_disk(pentagon, 6)`;
2. for each origin and each `k` from 0 through 6:
   - calculate the official safe ring with `h3.grid_ring(origin, k)`;
   - call a small H3Lite C harness that invokes `h3GetRing(origin, k, out)`;
   - compare sets and counts;
   - record negative returns separately.

The test must fail on:

- wrong cell content;
- duplicates;
- invalid H3 cells;
- a successful return with the wrong count;
- partial output treated as valid.

## 5.2 Nearest-region reference test

Implement a host-side reference:

1. start at the query H3 cell;
2. use official safe rings or `grid_disk`;
3. apply the same generated region table;
4. collect region IDs according to the selected product contract;
5. compare with `findNearestRegions()`.

Include points:

- centered on every pentagon;
- in every cell within several rings of every pentagon;
- near region boundaries where a failed ring could change the result;
- over ocean;
- inside restricted areas;
- outside but near restricted areas.

## 5.3 Fixed-memory safe fallback

If a safe fallback is needed, measure the upper bound.

For disk radius `k`, the ordinary maximum cell count is:

```text
1 + 3k(k + 1)
```

At `k = 6`, that is 127 cells.

A fixed array of 127 `H3Index` values requires 1016 bytes before distance/visited metadata. Determine whether that is acceptable on the real task stack. A ring-by-ring BFS may reduce or reshape the storage requirement, but prove it correct and bounded.

Do not claim `<100 bytes` stack if the implementation uses hundreds of bytes.

---

# 6. Generator verification suite

Create unit/integration tests for:

1. valid Polygon;
2. valid MultiPolygon;
3. polygon with hole;
4. invalid/self-intersecting polygon;
5. empty geometry;
6. unsupported geometry type;
7. two files contributing to one region;
8. two regions claiming the same cell;
9. deterministic tie;
10. antimeridian crossing;
11. high-latitude polygon;
12. missing directory;
13. malformed JSON;
14. no matching region files;
15. unsupported resolution;
16. restricted-region cells;
17. output write failure;
18. repeat generation is byte-identical.

The generator must return nonzero on incomplete output and must not leave half-written production files.

---

# 7. Runtime sanitizer and compiler checks

Create a host debug build with strict warnings:

```bash
gcc \
  -std=c99 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow \
  -Wstrict-prototypes -Wmissing-prototypes -Werror \
  -fsanitize=address,undefined \
  -fno-omit-frame-pointer -O1 -g \
  -Iinclude \
  test/<test-harness>.c \
  src/h3lite.c \
  src/h3lite_faceijk.c \
  src/h3lite_basecells.c \
  src/h3lite_neighbor.c \
  src/h3lite_regions_table.c \
  -lm \
  -o bin/h3lite_sanitize_test
```

Adjust warnings only where justified. Do not silence real conversion or bounds problems globally.

Also compile the STM32 target and report:

```bash
arm-none-eabi-size bin/libh3lite_stm32.a
```

For meaningful function-level stack analysis, use compiler stack-usage output where supported:

```text
-fstack-usage
```

Report the worst-case stack for:

- `latLngToH3`;
- `latLngToRegion`;
- `h3GetRing`;
- `findNearestRegions`.

---

# 8. Expected strong points to preserve

The review should not turn into an unnecessary rewrite. Preserve the project's good properties:

- compact static lookup table;
- binary search with no runtime heap;
- mixed-resolution compaction only for complete uniform child sets;
- deterministic conflict tie-break, once geometry math is validated;
- global H3-index differential test against official H3;
- explicit documentation of pentagon limitations;
- separation of offline geometry generation from embedded lookup;
- bounded ring radius;
- simple C99 runtime.

The core coordinate-to-H3 conversion may already be very good. The existing `T-INDEX` test claims a zero-mismatch baseline over 259,200 global points at resolution 3. Independently rerun it before and after every core geometry change.

---

# 9. Recommended implementation order

## Phase 1 — safety and correctness

1. Resolve restricted-region encoding and semantics.
2. Reject nonfinite/invalid coordinates.
3. reject unsupported generator resolutions.
4. make generator fail closed and remove automatic convex hull fallback.
5. make file loading deterministic and merge repeated region files.
6. integrate all strong tests into one command.
7. make C tests return failure status.

## Phase 2 — nearest-search correctness

1. build exhaustive pentagon differential tests;
2. define exact `findNearestRegions()` contract;
3. implement safe fallback if failed rings can change results;
4. define restricted candidate handling;
5. correct or rename distance reporting.

## Phase 3 — data quality

1. preserve holes;
2. replace raw-degree buffer/area logic or justify it with measured error;
3. add data provenance manifest;
4. add atomic generation and reproducibility checks.

## Phase 4 — cleanup

1. header dependency tracking;
2. remove or redesign no-op initialization;
3. validate arbitrary H3 input or restrict API;
4. move internal APIs to private headers;
5. update stale documentation and utilities;
6. complete license/NOTICE audit.

---

# 10. Final report format

Return a Markdown report containing this table:

| ID | Classification | Severity | Files changed | Test added | Result |
|---|---|---:|---|---|---|
| F-01 | Confirmed / etc. | Critical | ... | ... | ... |

Then include:

## Commit reviewed

```text
<full SHA>
```

## Changes made

A concise explanation per commit or patch.

## Tests run

Show command, exit code, and summarized output.

## Remaining risks

Only real unresolved risks, with operational consequence.

## Memory and timing

Measured host and STM32 data, clearly labelled by platform and build flags.

## Table reproducibility

Input manifest, output hashes, entry count, and whether regeneration is byte-identical.

## Final recommendation

Choose exactly one:

- ready for production use;
- ready with documented limitations;
- not ready until listed blockers are fixed.

Do not give a numerical score without evidence.

---

# 11. Concise source map

| Area | Main files |
|---|---|
| Public API | `include/h3lite.h` |
| Constants and H3 bit layout | `include/h3lite_constants.h` |
| Internal coordinate and neighbor declarations | `include/h3lite_faceijk.h` |
| Coordinate-to-H3 and region lookup | `src/h3lite.c` |
| Face/IJK conversion | `src/h3lite_faceijk.c` |
| Base-cell data | `src/h3lite_basecells.c` |
| Ring traversal | `src/h3lite_neighbor.c` |
| Generated region data | `src/h3lite_regions_table.c`, `include/h3lite_regions_table.h` |
| Generator | `generate_lookup_table.py` |
| Build | `Makefile` |
| Global H3 differential test | `test/t_index.py` |
| City/table tests | `test/t_city.py`, `test/t_table.py` |
| Shared harness | `test/harness_common.py`, `test/xval_pts.c` |
| Diagnostic C tests | `test/h3lite_grid_test.c`, `test/h3lite_nearest_test.c` |
| Legacy/visual tools | `test/visualize_table.py`, `test/visual_grid_test.py`, `regions_h3_conversion.py`, `analyze_regions.py` |

---

## Bottom line

H3Lite has a sound embedded concept and a strong core differential test, but the coding verification should focus on four operational issues:

1. restricted areas must have an actually encodable, unambiguous value;
2. the offline generator must fail closed and be reproducible;
3. unsupported resolutions must be rejected rather than silently truncated;
4. nearest-region search must be proven correct around H3 pentagons because the application depends on it routinely.

Do not rewrite the project wholesale. Verify, patch narrowly, and prove each fix with automated tests.
