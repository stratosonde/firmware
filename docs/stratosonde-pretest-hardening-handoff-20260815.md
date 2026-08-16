# Stratosonde firmware — pre-test hardening and release-pipeline handoff

**Repository:** `stratosonde/firmware`  
**Reviewed baseline:** `3da210017f6a62771aaf49c465c52e1201e45002` (`3da2100`)  
**h3lite submodule:** `e16481dc02d718eabfc347df1c8d86cdb02d5518`  
**Review date:** 2026-08-15  
**Input reconciled:** `stratosonde-review-20260815.md`  
**Purpose:** implementation-ready coding handoff for the final code, build, lint,
test, and release-pipeline hardening pass before sustained hardware qualification.

---

## 1. Executive decision

The refactor was worthwhile. The pure modules and their contract tests materially
improved the project. The next step is **not** another broad decomposition campaign,
but it is also **not yet time to freeze the current source**.

This is the right point for one bounded pre-test hardening pass because real HIL,
cold, power-cut, and endurance testing has not started. Complete the following before
declaring the first hardware-test release candidate:

1. Correct the four live control-policy defects in this document.
2. Replace the fragile build/test mechanics that could invalidate qualification.
3. Eliminate the compiler-warning and DDR-warning baselines.
4. Add scoped lint and full host sanitizer coverage.
5. Convert the highest-consequence adapter scans into executable linked tests.
6. Produce the debug and flight artifacts through one deterministic, non-mutating
   build path.
7. Tag the exact code, submodule, configuration, and generated data used for HIL.

Then freeze **architecture and control flow**. Do not expect the first bench run to
freeze battery constants: cold characterization exists specifically to determine
those values. Battery profiles and restricted-region data should be made explicitly
versioned inputs now, so test results can change data rather than reopen control-flow
code. After those inputs are updated, cut a new release candidate and repeat the
affected qualification.

Current assessment remains:

- engineering/architecture: approximately **8/10**;
- maintainability: approximately **7.5/10**;
- first-flight readiness: approximately **5/10 — NO-GO**;
- readiness to begin serious HIL: **after this bounded hardening pass**.

---

## 2. Product behavior that this work must preserve

The coding agent must treat these as acceptance constraints, not suggestions.

1. **Simple first-flight energy admission.** Read battery and temperature first.
   If either sample is stale/invalid, or either value is below its characterized
   minimum, stop the work cycle and return to survival-cadence sleep. Do not add a
   complicated partial-energy mode before Flight 1.
2. **Fail soft after admission.** Once a wake is admitted, a GNSS timeout, humidity
   failure, pressure failure, or other individual subsystem failure must not discard
   unrelated science. Preserve per-field stale/error semantics, log the record when
   storage is available, and independently decide whether RF is authorized.
3. **Strict GNSS authority.** Only the configured accepted-fix predicate may refresh
   trusted position, position age, or persisted last-known-good position. A weak
   position may be retained as explicitly stale diagnostic data. Valid GNSS time may
   be accepted independently from position quality if the time fields pass their own
   validation.
4. **RF authorization is final-state authorization.** A fresh location that requires
   a region change may transmit only after the active MAC region actually equals the
   detected region. Busy, failed, rolled-back, unjoined, or sessionless switching is
   RF-silent for that wake while local logging continues.
5. **Stale position may inhibit but never switch.** The 24-hour stale-position limit
   remains. An accepted fresh fix may restore eligibility in the same wake.
6. **No keys means no transmission.** Provisioning/commissioning gates remain fail
   closed.
7. **Confirmed compact probe behavior remains.** No ACK means no historical burst;
   sleep. An ACK may permit the configured full-resolution/current-data behavior.
8. **Current science outranks historical recovery.** Archive recovery must never
   delay the next current-science deadline.
9. **Records remain immutable.** Preserve record identity, raw/fundamental values,
   stale flags, and append-only recovery semantics.
10. **Fatal faults reset and retry indefinitely.** Do not restore the former
    five-boot escape path.
11. **Do not silently change the wire protocol.** No new diagnostic bits or fields may
    be added to the 11-byte or high-resolution payload without an explicit versioned
    protocol change and decoder update.

---

## 3. Evidence reproduced on the reviewed baseline

### 3.1 Builds and tests

- `master` and `origin/master` both resolved to `3da2100` during review.
- `make -C tests/host all` exited 0.
- The suite reported approximately 803,624 checks/model events. Of these, 734,060
  are TX-FSM shadow comparisons, not independent test cases.
- Eight known failures are converted to a green result using `EXPECT_UNFIXED=1`:
  LT-06 ×2, GEO-01/GEO-04 ×3, and PWR-02 ×3.
- The existing `sanitize` target instruments only the 292-check core runner.
- As a review experiment, every current host executable was rebuilt and run with
  ASan+UBSan, `-fno-sanitize-recover=all`, and leak detection disabled only for the
  ptraced review environment. All suites completed with zero sanitizer diagnostics.
  This proves the expansion is feasible; it does not prove HAL-bound target code.
- Current CI successfully builds both ARM configurations and verifies the embedded
  `SONDE_BUILD:flight` marker.

### 3.2 Current warning population

A clean host `all` build emits **40 warning lines**. The accurate breakdown is:

- **19 unique production-code sites**:
  - 11 integer-address-to-pointer casts in `multiregion_context.c`;
  - 6 signed/unsigned slot conversions in `multiregion_context.c`;
  - 2 conversions in `atgm336h.c`.
- **13 unique test-harness sites**.
- The two `atgm336h.c` sites are compiled into five test executables, producing eight
  duplicate warning lines. That is why 32 unique sites produce 40 emitted lines.

There is no `-Werror` gate. The ARM first-party build currently uses only `-Wall`.

### 3.3 Source-text test population

Fresh counts on `3da2100` are:

- 129 `slurp(` occurrences across 13 C test files;
- 340 `strstr(` occurrences across the same 13 files;
- 7 dated/review-session test source filenames.

These scans retain historical value, but they are not behavioral proof. In particular,
the 734,060-event FSM shadow test proves the pure FSM agrees with its model after the
input structure has been populated. It cannot prove the production adapter populated
that structure correctly.

### 3.4 Build-tree hygiene

- Running the host suite creates roughly two dozen executables directly under
  `tests/host/`.
- `.gitignore` ignores only `tests/host/test_runner*`, so the other executables dirty
  the working tree.
- `Debug/` contains 74 tracked support files; at least 50 are unused
  `Debug/Reference/` or `Debug/archive/` fragments.
- The flight CI job edits every `Debug/**/subdir.mk` with `sed` to inject
  `SONDE_FLIGHT_BUILD`. The PowerShell build performs the same mutation and attempts
  to restore the files afterward.
- CI uses `ubuntu-latest` and installs the unpinned distribution
  `gcc-arm-none-eabi` package. It verifies the marker but does not retain the ELF,
  BIN, map, test summary, or manifest as workflow artifacts.

### 3.5 Documentation drift

- `FINDINGS.md` still identifies an older baseline in its header and reports the old
  115-site source-scan inventory.
- `R2_TEST_MAP.md` calls `make all` the post-fix gate, although `all` still invokes
  three `EXPECT_UNFIXED` gates.
- The flight-readiness checklist is annotated against an older SHA and still labels a
  same-wake GPS-loss failure that was subsequently fixed.
- The DDR checker exits 0 with 99 ambiguity warnings, including new source files.

These are not merely editorial defects: qualification evidence must identify the code
that was actually tested.

---

## 4. Reconciliation of the submitted MT-series review

The submitted review is strong and should be retained as an engineering record, but
the following corrections supersede it.

| Item | Disposition | Corrected conclusion |
|---|---|---|
| Six extracted pure modules | **Accept** | Real improvement in testability; approximately 616 pure-logic lines. |
| TX adapter seam coverage (MT-01) | **Accept, revise implementation** | The defect class is open. Builders must live in a small production-compiled adapter module, not remain static inside unhostable `lora_app.c`. |
| Duplicated FSM stale predicates (MT-02) | **Accept** | `TxFsm_Dispatch()` should call `TxFsm_ProbeStale()` and `TxFsm_BurstStale()` directly. No behavior change. |
| FSM reach-ins (MT-03) | **Accept with wording correction** | There are 19 direct field accesses on 18 lines. Semantic accessors are useful, but they do not make the public structure truly opaque. Do not claim complete encapsulation. |
| Sanitizer gap (MT-04) | **Accept, replace build sketch** | Do not combine suites with separate `main()` functions into one executable. Rebuild every existing suite separately under sanitizer flags in a separate output directory. |
| Admission result collapse (MT-05) | **Partly accept** | Seven causes collapse into one enum and log reason. The current 11-byte telemetry does **not** transmit that enum. Improve internal reason codes/RTT diagnostics; do not invent three payload bits. |
| DDR ambiguity (MT-06) | **Accept, change remedy** | Do not add a new `(canonical)` annotation dialect. Resolve every legacy reference, retire the overlapping aliases, and make ambiguous/retired references hard failures. |
| `lora_app.c` size (MT-07) | **Accept** | It remains 3,038 lines. The review's “71 static functions” is actually 71 lines beginning with `static`; it includes objects and duplicate prototypes/definitions. The central point—coordination complexity remains—is valid. |
| Warning baseline (MT-08) | **Accept with corrected counts** | 40 emitted lines, 32 unique sites, 19 unique production sites. The submitted category counts accidentally included four harness pointer casts as firmware warnings. |
| No static analysis (MT-09) | **Accept** | Add scoped static analysis after warning cleanup. |
| `REGION_UNKNOWN` vocabulary (MT-10) | **Accept with policy clarification** | Return `GEO_PERMISSION_UNKNOWN` for `REGION_UNKNOWN`, but keep the action explicit. Current confirmed product behavior may hold the latched region over open ocean; UNKNOWN must not be mislabeled as PERMITTED. |
| Region table population | **Correct** | The generated table contains **4,564**, not 19,112, entries. IDs 1–14 are populated; IDs 0 and 15 have zero entries. `REGION_RESTRICTED` is still empty. |
| GEO-02 comment closure | **Partly reopen** | The `lora_app.c` comment was corrected, but `h3lite.c:41-43` still falsely claims its Atlantic probe proves UNKNOWN can never be unmapped land. One probe cannot prove global coverage. |
| Cold power consequence | **Correct** | The power-model defect is real, but with the default −55 °C first-flight admission floor, a −62.3 °C wake exits before `SelectModeFromPredictions()`. Do not describe the active path as compensation → SURVIVAL → GPS-loss veto. The raw-floor/model issue becomes active if the admission threshold is lowered based on testing. |
| “Freeze structural refactoring now” | **Replace** | Perform this bounded pipeline/adapter/control-policy pass now, then freeze architecture at the HIL release-candidate tag. Do not start another broad module-splitting campaign. |

The submitted review's numerical rating of 6/10 is not inherently wrong; it combines
code quality and flight readiness into one number. Keep the two axes separate instead.

---

## 5. Live behavior defects to fix before the HIL release candidate

These are behavior changes. Each requires a red test commit followed by a distinct fix
commit. Do not hide a behavior change inside build cleanup or refactoring.

### BEH-01 — Remove the all-fresh science-package abort

**Current code:**

- `Core/Src/first_flight_policy.c:21-27`
- `LoRaWAN/App/lora_app.c:2393-2401`
- `LoRaWAN/App/lora_app.c:2433-2448`

`FirstFlightPolicy_GnssPackagePresent()` and
`FirstFlightPolicy_PackageComplete()` cause an admitted wake to return without a live
record when GNSS/time or any environmental field is stale. One failed humidity channel
therefore discards valid pressure, temperature, battery, time history, and the fact of
the failure itself. A GNSS timeout also prevents local science logging, contradicting
the primary mission and fail-soft invariants.

**Required behavior:**

```text
sample temperature + battery
        |
        +-- invalid/stale/below floor --> park TX FSM; survival sleep
        |
        `-- admitted --> attempt GNSS and remaining sensors
                         --> mark each field fresh/stale honestly
                         --> append current record if flash is available
                         --> compute final RF authorization
                         --> transmit only if authorized
```

Remove these predicates as abort gates. They may be retained only as record-quality or
diagnostic predicates that do not decide whether unrelated observations exist.

**Red tests:**

1. admitted energy + humidity stale → archive current record; humidity stale bit set;
2. admitted energy + pressure stale → archive other fields;
3. admitted energy + GNSS timeout + trusted stale position within legal age → archive;
4. same but RF unauthorized → archive yes, transmit no;
5. battery or temperature admission failure → archive no, transmit no, survival retry;
6. bulk continuation remains independent from live-package freshness.

The test must exercise production decision code, not grep for return statements.

### BEH-02 — Do not promote weak GNSS position to trusted position

**Current code:** `LoRaWAN/App/lora_app.c:1275-1297`.

When the configured accepted-fix predicate fails but `GNSS_HasPosition()` is true, the
code currently updates:

- `last_valid_lat/lon/alt`;
- `s_last_fresh_fix_s`;
- persistent `LastPos`;
- `have_previous_fix`;
- the GNSS stale flag to false.

`GNSS_HasPosition()` checks presence and range, not the configured satellite, HDOP,
and fix-quality thresholds. This still contaminates the regulatory position-age
authority and means issue #284 is not fully corrected.

**Required behavior:**

- A weak/basic position must not update trusted `LastPos` or position-freshness time.
- It must not clear GNSS staleness.
- It may remain in the current diagnostic sample with stale/weak provenance.
- Valid date/time may discipline the RTC through a separate time-validity decision;
  never use successful time discipline as proof of accepted position quality.

**Red tests:** poor HDOP, too few satellites, invalid fix quality, and partial sentence
state must each prove that trusted position and age remain unchanged. A boundary-valid
accepted fix must prove that all authoritative updates occur together.

### BEH-03 — Fail closed when a required region switch does not complete

**Current code:**

- `LoRaWAN/App/lora_app.c:1487-1528`
- `Core/Src/multiregion_context.c:871-922`

If a fresh position requires a new joined region but the MAC is busy, switching fails,
or rollback restores the old session, `SelectRegionAndSession()` logs the result and
can continue to TX on the old plan.

**Required invariant:**

```text
fresh known detected region differs from active region
    => RF allowed only if active region == detected region after switch attempt
```

Busy, failed, rolled-back, unjoined, and sessionless outcomes must archive locally and
set RF silence for the wake. Retry at the next fresh fix. A successful rollback is good
recovery of the old session; it is not authorization to use that session at the new
location.

Change `SelectRegionAndSession()` from a `void` logging helper into an explicit result
or make its final authorization output unavoidable. Add linked tests for:

- same region;
- successful switch;
- target unjoined;
- MAC busy;
- switch error + rollback success;
- switch error + rollback failure;
- stale position never switches;
- restricted result always silences.

### BEH-04 — Preserve UNKNOWN as a distinct result

**Current code:** `Core/Src/region_policy.c:13-20` treats every valid coordinate that is
not RESTRICTED as PERMITTED, including `REGION_UNKNOWN`.

Change the pure result to:

```c
if (!coordinates_valid || h3_region_id == REGION_UNKNOWN) {
    return GEO_PERMISSION_UNKNOWN;
}
```

The caller must then implement the confirmed action explicitly:

- RESTRICTED → archive, RF silence;
- UNKNOWN/open-ocean policy → hold the latched region under the documented limits;
- known supported region → require an active matching session;
- invalid/no-authority position → stale-position policy, never a new switch.

This change is about truthful policy state. It need not darken open-ocean operation.
Correct the false global-coverage claim in `h3lite.c` at the same time.

### BEH-05 — Resolve newest-first recovery or exclude it from Flight 1

**Current code:** `Core/Src/flash_log.c:198-219` services
`[tx_high_water, next_sequence)` ascending before its descending recovery walker. A
fresh outage therefore begins oldest-first despite the newest-first mission intent.

Choose one Flight-1 disposition before the RC:

1. **Recommended minimum Flight-1 profile:** preserve local logging and current
   full-resolution data, but compile historical backlog bursts out of the flight
   profile. Keep the feature compiled and tested in the development profile.
2. **If historical recovery will fly:** correct the traversal/watermark semantics and
   add ring-wrap, reset-resume, corruption-skip, partial-packet, and current-science
   preemption tests before HIL.

Do not delete the archive. Only historical radio recovery is a stretch objective.

### BEH-06 — Do not guess the cold power fix

The PWR and LT tests correctly expose an unqualified fixed raw-voltage floor and a
non-monotonic compensation table. However, the replacement must come from the actual
Nichicon 100 mAh flight cells under cold load.

Before the bench campaign, separate policy from characterized data:

- define one immutable, versioned `PowerProfile` input containing temperature knots,
  OCV/load thresholds, hysteresis, and profile identity;
- make `power_model.c` consume that profile without embedding a new guessed table;
- preserve the current values in an explicitly named `UNQUALIFIED_LEGACY` profile
  until measurements replace them;
- include the selected profile ID/hash in the release manifest;
- test monotonic interpolation, both transition directions, hysteresis, bounds, and
  corrupt/missing profile fallback.

Bench data will still change firmware bytes. The point is that it should change a
reviewable data profile, not control flow.

### PRETEST-DEC-01 — Resolve the primary launch-readiness input

`MissionState_EnterFlight()` exists, but `Core/Inc/mission_state.h:114-117` states that
no free GPIO is assigned for the intended button/arming hook. The current implemented
path is automatic pressure-departure detection. Before the HIL RC, choose and document
one real Flight-1 arrangement:

1. assign and implement a physical arming/readiness input and test the complete
   fix + ACK + sensor-health indication path; or
2. explicitly approve pressure-only launch entry for Flight 1 and revise the mission,
   checklist, and requirements so they no longer claim an unimplemented button gate.

Do not leave the requirement saying “button” while the release silently relies only on
pressure.

### PRETEST-DEC-02 — Freeze the Flight-1 region profile

The full platform may retain automatic multi-region support. For the actual Flight-1
configuration, make one explicit choice before qualification:

1. **Recommended minimum-risk profile:** lock the known commissioned home/launch region
   and keep automatic switching enabled only in the development/HIL profile; or
2. fly automatic switching, in which case BEH-03, the non-empty restricted dataset,
   region-session commissioning, and boundary-crossing HIL are mandatory release
   gates.

This is a build-profile choice, not permission to delete multi-region functionality.

---

## 6. Final bounded maintainability work

### MAINT-01 — Add production-compiled adapter builders

Create a small module such as:

```text
Core/Inc/lora_app_adapters.h
Core/Src/lora_app_adapters.c
tests/host/test_lora_app_adapters.c
```

It must have no HAL, LoRaMac, flash, timer, or global dependencies. Builders should
consume raw domain values, not pre-derived booleans whose polarity can already be
wrong. Example:

```c
typedef struct {
    MissionState_t mission_state;   /* raw enum, not mission_ascent bool */
    int tx_status;
    bool ack_received;
    bool battery_good;
    bool has_cache;
    uint8_t max_bulk_packets;
} AppTxConfirmSnapshot_t;

TxFsmConfirmInput_t AppAdapters_BuildTxConfirm(
    const AppTxConfirmSnapshot_t *snapshot);
```

Cover at least:

- `TxFsmConfirmInput_t`;
- `TxFsmCycleInput_t`;
- `TxFsmRxInput_t`;
- region-policy inputs/final authorization;
- first-flight admission inputs.

Production `lora_app.c` must call these exact builders. Host tests must link the real
adapter object. Retire the corresponding literal-string scans only after the linked
test is green.

### MAINT-02 — Reuse the FSM's exported stale predicates

Replace the duplicate stale calculations inside `TxFsm_Dispatch()` with calls to
`TxFsm_ProbeStale()` and `TxFsm_BurstStale()`. The shadow suite should remain bit-for-
bit behaviorally identical.

### MAINT-03 — Add semantic FSM queries

Add narrowly named queries such as:

```c
bool TxFsm_InBulk(const TxFsm_t *fsm);
bool TxFsm_WaitingForProbeAck(const TxFsm_t *fsm);
uint8_t TxFsm_BulkPacketsSent(const TxFsm_t *fsm);
```

Replace raw state reads where the caller only asks one of those questions. Keep raw
state capture only where the complete enum is genuinely needed for transition logging.
Do not describe this as full opacity while `TxFsm_t` remains public.

### MAINT-04 — Stop creating new source-scan tests

Classify current tests as:

- `contract`: linked pure-module behavior;
- `integration`: linked production objects with fakes;
- `structural`: source/build/document scans;
- `characterization`: deliberate known failures awaiting data;
- `hardware`: HIL/bench evidence.

Migrate in consequence order:

1. RF authorization and region switching;
2. GNSS authority and stale-position age;
3. TX adapter polarity and state transitions;
4. NVM/flash/power-cut behavior;
5. comments, log wording, and documentation anchors.

Never delete a structural scan in the same commit that first introduces its behavioral
replacement. Land the behavioral test, prove it fails on the old defect, fix it, then
retire the scan in a later cleanup commit.

### MAINT-05 — Keep large post-flight splits deferred

Do not split `multiregion_context.c` or broadly rewrite `lora_app.c` merely to reduce
line count in this pass. The targeted cycle-decision and adapter seams above are
justified because they close live test blind spots. General decomposition can wait
until after Flight 1.

---

## 7. Build and test pipeline cleanup

### PIPE-01 — Make test builds out-of-tree and leave Git clean

Place executables and objects under configuration-specific paths:

```text
tests/host/build/gcc/
tests/host/build/clang/
tests/host/build/asan-ubsan/
tests/host/build/coverage/
```

Ignore only `tests/host/build/`, not broad source-name patterns.

Acceptance:

```bash
git status --porcelain          # empty before
make -C tests/host clean all
git status --porcelain          # still empty
```

Distinct directories are mandatory because Make does not know that a changed CFLAGS
string invalidates an existing executable.

### PIPE-02 — Give targets honest names

Provide these stable entry points, ideally through a small root `Makefile` that wraps
the existing systems:

```text
make check                 all mandatory host tests; no expected-failure masking
make characterization      exact known failures awaiting owner data
make contracts             pure-module contracts
make integration           linked production/fake tests
make structural            remaining source/build/document checks
make sanitize              all host executables rebuilt under ASan+UBSan
make lint                  compiler lint + cppcheck + workflow/style checks
make arm-debug             clean debug ARM build
make arm-flight            clean flight ARM build + marker proof + package
make release-gate          every software gate; refuses known-failure masking
```

`all` should be an alias for `check`, not a green wrapper around eight red checks.
During characterization, print the exact expected failure IDs and counts. At release,
`EXPECT_UNFIXED` must be absent or the release gate must fail.

Compile test executables in parallel if desired, but run them serially with clear suite
headers and per-suite timeouts so output remains deterministic.

### PIPE-03 — Sanitize every suite separately

Retain each suite's own executable and `main()`. Reuse its real source list with:

```make
SAN_FLAGS := -fsanitize=address,undefined \
             -fno-omit-frame-pointer \
             -fno-sanitize-recover=all
```

Run from the separate sanitizer build directory with CI settings equivalent to:

```text
ASAN_OPTIONS=halt_on_error=1:detect_leaks=1
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
```

Leak detection may be disabled only for a documented local ptrace/debugger limitation;
keep it enabled in CI.

### PIPE-04 — Remove flight-build source mutation

Do not use `sed` or PowerShell replacement to edit `subdir.mk` files.

Minimal migration path—no CMake rewrite required:

1. Add a centralized make variable such as `PROJECT_CPPFLAGS` to every **active** C
   compile recipe.
2. Debug build passes no flight define.
3. Flight build invokes a clean build with
   `PROJECT_CPPFLAGS=-DSONDE_FLIGHT_BUILD`.
4. Add a checker that derives active fragments from `Debug/Makefile` and fails if any
   active C recipe omits `$(PROJECT_CPPFLAGS)`.
5. Keep the embedded debug/flight marker proof.
6. Require `git diff --exit-code` after both builds.

This keeps the Cube-generated build for now while removing mutation and restoration
risk. Defer a full CMake migration until after Flight 1.

### PIPE-05 — Prune unused generated build fragments

Remove tracked `Debug/Reference/**`, `Debug/archive/**`, and other fragments not
included by `Debug/Makefile`, after proving no active include or object-list reference
uses them. Keep only the generated support files required for a fresh clone to build.

Add a build-manifest checker that confirms every production `.c` file appears exactly
once in the ARM build and no reference/archive/test source appears. This is more useful
than trusting a large force-added generated tree.

### PIPE-06 — Pin and record the release environment

- Replace `ubuntu-latest` with an explicit runner image.
- Pin the approved ARM compiler release and verify its download/package checksum.
- Pin Cppcheck, Clang/clang-format, and actionlint versions.
- Pin external GitHub Actions to reviewed full commit SHAs.
- Record the complete compiler version, flags, linker script hash, git SHA, submodule
  SHA, config hash, power-profile hash, and region-data hash.

Local auto-discovery of the newest CubeIDE toolchain can remain a developer convenience,
but it must not produce the authoritative flight artifact.

### PIPE-07 — Retain release evidence

Upload, at minimum:

```text
flight .bin
flight .elf
flight .map
debug .elf/.map
SHA-256 manifest
arm-none-eabi-size output
largest-symbol report
stack-usage report (.su summary)
host test summary
sanitizer log
lint/static-analysis reports
```

The exact binary tested and flashed must be downloadable from the successful release
workflow. Do not rebuild locally after sign-off and call it equivalent.

### PIPE-08 — Add deterministic and cross-compiler checks

- Run all host tests with pinned GCC.
- Run module contracts and integration tests with pinned Clang.
- Use fixed, printed random seeds for merge gates.
- Optionally run additional random seeds on a scheduled job; print the failing seed so
  it is reproducible.
- Set `LC_ALL=C` and `TZ=UTC` for deterministic logs.
- Add a timeout around each executable.
- Generate coverage for linked production modules as a report. Do not use a global
  percentage gate while many tests merely scan source text.

---

## 8. Lint and warning policy

### 8.1 Yes—add lint, in this order

### Step 1: compiler warnings are the primary C linter

Fix the existing 32 unique sites manually and review each change. Do not blanket-disable
the warning classes.

Recommended fixes:

- represent flash addresses as `uintptr_t` before conversion to pointers;
- use unsigned slot/index types where negative is not a valid state, or range-check
  before explicit narrowing;
- cast the NMEA checksum character through `uint8_t` intentionally;
- convert guarded NMEA lengths to `size_t` only after bounds checks;
- correct the harness `size_t`/`ptrdiff_t` comparisons and fake-address types.

Then:

- host production and harness code: `-Wall -Wextra -Wsign-conversion -Werror`;
- add useful warning classes incrementally, one at a time, after clearing each new
  population;
- ARM first-party code: the same reviewed warnings and `-Werror`;
- vendor/ST code: compile separately without turning third-party warnings into errors.

Do not let `-Werror` depend on whether a file happens to be included by a host suite.
The ARM first-party build is the authoritative coverage for HAL-bound sources.

### Step 2: Cppcheck first

Cppcheck is the sensible first static analyzer because it can analyze the existing
tree without first replacing the build system. Run it twice using the real first-party
include paths and defines:

- debug configuration;
- flight configuration with `SONDE_FLIGHT_BUILD`.

Start with `warning`, `performance`, and `portability` as candidate gates. Run `style`
and inconclusive checks as a report until triaged.

Do **not** immediately suppress the initial report. For every finding, choose one:

1. real defect → red test where feasible, then fix;
2. clearer code removes the warning → fix;
3. proven false positive → narrow external suppression with reason and owner;
4. accepted risk → explicit issue/waiver, not a silent suppression.

Prefer `tools/cppcheck-suppressions.txt`; do not scatter inline suppressions throughout
flight source. After triage, `--error-exitcode=1` gates new unsuppressed findings. Model
the 32-bit target, but remember that no generic platform model is a substitute for the
actual ARM compiler.

### Step 3: formatting and workflow lint

- Add a pinned `.clang-format` configuration.
- Enforce formatting only on newly added/changed first-party C/H lines or files.
- Do not mass-format the complete firmware tree immediately before qualification.
- Add `actionlint` for `.github/workflows/*.yml`.
- Run `python -m py_compile tools/*.py`; add a Python linter only if the tools directory
  grows enough to justify another dependency.

### Step 4: defer clang-tidy and MISRA

Clang-tidy is valuable but wants an accurate compilation database. Generate one from
the stabilized build before adding it. Do not create a second hand-maintained set of
defines merely to make clang-tidy run. A MISRA C:2012 program is appropriate for the
long-term platform but is a post-Flight-1 engineering program, not this hardening pass.

### 8.2 What lint does not prove

Lint would not have detected:

- reversed `mission_ascent` adapter polarity;
- an empty restricted-region dataset;
- an unqualified battery model;
- weak GNSS being treated as regulatory authority;
- transmitting on a rolled-back old region.

Those require behavioral tests, generated-data validation, and HIL. Lint raises the
mechanical floor; it does not establish mission correctness.

---

## 9. DDR and documentation cleanup

### DOC-01 — Retire the overlapping DDR alias namespace

The manifest currently permits IDs 0014–0021 to be both canonical records and retired
aliases. This produces 99 warnings and allows ambiguity into new files.

Required migration:

1. For each reference, determine the intended record from context.
2. Rewrite legacy-meaning references to the correct canonical target.
3. Keep canonical references as ordinary canonical IDs.
4. Move the old numbering map to a historical migration ledger, not the active
   resolver.
5. Remove active aliases whose names collide with canonical records.
6. Make every unresolved, retired, or ambiguous token a hard failure.
7. Finish with **0 failures and 0 warnings**.

This follows the existing rule that legacy DDRs are migration input, not permanent
dependencies of standalone V2 records.

### DOC-02 — Establish one release truth

Use these roles:

- `FINDINGS.md`: open technical findings and verified test inventory;
- `R2_TEST_MAP.md`: test-to-invariant mapping, preferably generated/validated;
- flight-readiness checklist: go/no-go evidence against one RC;
- `ProjectStatus.md`: short human summary that links to the above, not another copied
  issue table.

Update all baseline SHAs and remove closed/stale claims at the same time the release
candidate is cut. Add a CI check that every document declaring a baseline SHA names
the current tagged RC or explicitly says it is historical.

### DOC-03 — Generate volatile inventories

Do not hand-maintain counts such as source-scan sites, suites, warning counts, or binary
sizes in several documents. Generate one machine-readable summary and insert/link it
from the human documents. The current 115-versus-129 scan drift is the proof of need.

---

## 10. Proposed CI structure

Keep the workflow small enough to understand:

### Job 1 — hygiene

- checkout recursively;
- `git diff --check`;
- verify generated build manifest/source inventory;
- DDR manifest: 0 failures, 0 warnings;
- actionlint;
- verify no forbidden test/flight macros in production configuration.

### Job 2 — host GCC

- clean out-of-tree build;
- compile all mandatory suites with strict warnings and `-Werror`;
- run `check` serially with timeouts;
- run characterization separately and publish its explicit open-failure list.

### Job 3 — host Clang

- build/run contracts and linked integration tests;
- enforce the same warning baseline where supported.

### Job 4 — sanitizers

- build every host suite into `build/asan-ubsan`;
- run with abort/halt and leak detection enabled;
- upload logs even on failure.

### Job 5 — static analysis

- Cppcheck debug configuration;
- Cppcheck flight configuration;
- fail on unsuppressed gated findings;
- upload full report.

### Job 6 — ARM matrix

- pinned toolchain;
- clean debug build;
- clean flight build using flags, not source mutation;
- verify mutually exclusive embedded markers;
- verify production source inventory;
- generate manifests, size, symbol, and stack reports;
- upload artifacts.

### Job 7 — release gate

Run on an explicit RC tag or manual release workflow:

- require Jobs 1–6;
- fail if any `EXPECT_UNFIXED` mechanism remains active;
- require approved power-profile and geofence-data IDs;
- require clean checklist release identity;
- package the exact flight artifact without rebuilding it.

Require the relevant job names in branch protection. Pin third-party actions to full
commit SHAs and review updates deliberately.

---

## 11. Implementation sequence for the coding LLM

Every step below ends green. Behavior tests and fixes are separate commits even when
delivered in the same PR.

### Phase 0 — Rebaseline the truth

1. Add this handoff to the working branch.
2. Update stale baseline headers and clearly mark the eight characterization failures.
3. Record exact current counts from scripts, not prose estimates.
4. Add no behavior change.

**Gate:** current host, sanitizer, and ARM CI results reproduce `3da2100`.

### Phase 1 — Build/test hygiene

1. Move host outputs under `tests/host/build/<configuration>`.
2. Fix `.gitignore`.
3. Add honest `check`, `characterization`, `sanitize`, and `release-gate` targets.
4. Add a root wrapper Makefile.
5. Prune unused Debug reference/archive fragments.
6. Add production-source inventory validation.

**Gate:** every build/test target leaves `git status --porcelain` empty.

### Phase 2 — Warning and DDR zero-baseline

1. Fix 19 unique production warning sites.
2. Fix 13 harness sites.
3. Add host `-Werror`.
4. Apply strict warning flags and `-Werror` only to ARM first-party code.
5. Resolve DDR references and retire the overlapping aliases.

**Gate:** compiler warnings 0; DDR failures 0; DDR warnings 0.

### Phase 3 — Complete sanitizers and lint

1. Sanitize every suite separately.
2. Add Cppcheck in non-gating report mode.
3. Triage the complete report.
4. Add narrow documented suppressions only for proven false positives.
5. Turn unsuppressed warning/performance/portability findings into a gate.
6. Add changed-file clang-format and actionlint.

**Gate:** all sanitizer runs clean and lint has no unexplained findings.

### Phase 4 — Correct live behavior

Implement separate red-test/fix pairs in this order:

1. BEH-01 fail-soft admitted wake;
2. BEH-02 strict GNSS authority;
3. BEH-03 post-switch RF authorization;
4. BEH-04 truthful UNKNOWN state;
5. BEH-05 historical recovery disposition;
6. BEH-06 table-driven power profile structure without guessed new data.

Resolve PRETEST-DEC-01 and PRETEST-DEC-02 before this phase is declared complete.

**Gate:** linked behavioral tests prove every decision; no new source-scan-only gate.

### Phase 5 — Close adapter blindness

1. Add the production-compiled adapter module.
2. Move all five high-consequence mappings into it.
3. Add direct linked tests.
4. Reuse FSM predicates and add semantic state queries.
5. Retire only the superseded literal scans.

**Gate:** deliberately reverse each mapping on a temporary branch and prove the linked
test fails. Restore and prove green.

### Phase 6 — Deterministic ARM/flight packaging

1. Replace macro injection with centralized build flags.
2. Pin release runner/toolchain.
3. Make PowerShell and CI call the same underlying targets/packager.
4. Upload complete artifact/evidence bundle.
5. Prove debug and flight markers are mutually exclusive.
6. Prove source tree remains unchanged.

**Gate:** the downloadable CI `.bin` is the only candidate allowed into the flight
flash procedure.

### Phase 7 — Cut the architecture-frozen HIL candidate

Tag, for example, `flight1-hil-rc1`, and record:

- firmware SHA/tag;
- h3lite SHA;
- hardware revision;
- compiler and build environment;
- binary/ELF/map hashes;
- effective configuration;
- power-profile identity and qualification state;
- region-data identity and qualification state;
- decoder/backend version.

Start HIL against that tag. No unrecorded binary is tested.

---

## 12. Change control after HIL starts

There are two freezes, not one:

### Freeze A — architecture/control-flow freeze

Occurs at `flight1-hil-rc1`. After this point:

- no cleanup-only refactors;
- no mass formatting;
- no module renames or file shuffles;
- no test-harness redesign unless the harness itself invalidates evidence;
- only a demonstrated defect, characterized data update, or release blocker may alter
  the candidate.

### Freeze B — qualified flight release

Occurs after cold/power, reset/power-cut, STOP2, sensor-failure, GNSS-loss, RF/network,
and endurance evidence is complete. Any change after Freeze A creates a new RC and
requires an impact assessment:

| Change | Minimum repeated evidence |
|---|---|
| Comment/doc only | hygiene + traceability |
| Build script/toolchain | both ARM builds, marker, hashes, artifact comparison |
| Battery profile data | host power tests + complete cold/load campaign subset |
| Region dataset | generator validation + geo tests + representative HIL lookups |
| Sensor/GNSS logic | host integration + affected fault cases + wake/STOP2 subset |
| Persistence/flash | host fault injection + randomized power cuts + reset soak |
| TX/region policy | host state/integration tests + live network/region HIL subset |
| Wire format | encoder vectors + decoder/backend compatibility + protocol version |

This is how the project avoids casual post-test alteration without pretending that
qualification can never discover a necessary change.

---

## 13. Release-candidate software acceptance checklist

The architecture-frozen HIL RC is acceptable only when all software-only boxes below
are true:

- [ ] BEH-01 through BEH-04 fixed with linked red-first tests.
- [ ] BEH-05 has an explicit Flight-1 disposition.
- [ ] Launch-readiness input is implemented and tested, or pressure-only entry is
      explicitly approved and documented.
- [ ] The Flight-1 region profile is explicit and encoded in the artifact manifest.
- [ ] Power profile is versioned and still clearly marked unqualified until bench data
      exist.
- [ ] Region dataset is versioned and still clearly marked unqualified until the
      restricted set is populated and verified.
- [ ] Host mandatory suite has zero failures without `EXPECT_UNFIXED` masking.
      Characterization failures remain separate until owner data closes them.
- [ ] Every host suite passes ASan+UBSan.
- [ ] GCC and Clang linked tests pass.
- [ ] Production compiler warnings: zero.
- [ ] Test-harness compiler warnings: zero.
- [ ] DDR checker: zero failures, zero warnings.
- [ ] Cppcheck: zero unexplained gated findings.
- [ ] Workflow and changed-file format checks pass.
- [ ] Debug ARM build passes.
- [ ] Flight ARM build passes.
- [ ] Flight marker present; debug marker absent.
- [ ] Build and tests leave the source tree clean.
- [ ] Flight ELF/BIN/map/manifest retained as CI artifacts.
- [ ] Documentation names the exact RC SHA and no stale “fails today” claims remain.

The **flight** release additionally requires all characterization failures closed or a
formal no-go waiver, plus the hardware and backend sections of the existing readiness
checklist.

---

## 14. Work explicitly deferred until after Flight 1

- broad `multiregion_context.c` decomposition;
- full `lora_app.c` rewrite;
- full CMake migration;
- clang-tidy until a trustworthy compilation database exists;
- MISRA compliance programme;
- wholesale test-file renaming merely to remove review dates;
- global automatic formatting of legacy/vendor code;
- Qwiic/camera/best-effort application work;
- OTA firmware update work;
- new protocol fields that are not required for Flight 1.

---

## 15. Source anchors

Reviewed source and evidence:

- baseline commit: <https://github.com/stratosonde/firmware/commit/3da210017f6a62771aaf49c465c52e1201e45002>
- current CI run: <https://github.com/stratosonde/firmware/actions/runs/31917970852>
- first-flight policy: <https://github.com/stratosonde/firmware/blob/3da210017f6a62771aaf49c465c52e1201e45002/Core/Src/first_flight_policy.c>
- application coordinator: <https://github.com/stratosonde/firmware/blob/3da210017f6a62771aaf49c465c52e1201e45002/LoRaWAN/App/lora_app.c>
- region switch implementation: <https://github.com/stratosonde/firmware/blob/3da210017f6a62771aaf49c465c52e1201e45002/Core/Src/multiregion_context.c>
- archive recovery: <https://github.com/stratosonde/firmware/blob/3da210017f6a62771aaf49c465c52e1201e45002/Core/Src/flash_log.c>
- host Makefile: <https://github.com/stratosonde/firmware/blob/3da210017f6a62771aaf49c465c52e1201e45002/tests/host/Makefile>
- CI workflow: <https://github.com/stratosonde/firmware/blob/3da210017f6a62771aaf49c465c52e1201e45002/.github/workflows/ci.yml>
- system invariants: <https://github.com/stratosonde/firmware/blob/3da210017f6a62771aaf49c465c52e1201e45002/docs/SYSTEM-INVARIANTS.md>
- Flight-1 mission: <https://github.com/stratosonde/firmware/blob/3da210017f6a62771aaf49c465c52e1201e45002/docs/requirements/flight1-mission-definition.md>
- readiness checklist: <https://github.com/stratosonde/firmware/blob/3da210017f6a62771aaf49c465c52e1201e45002/docs/requirements/flight1-validation-readiness-checklist.md>

Tool references used for the pipeline recommendation:

- Clang-format documentation: <https://clang.llvm.org/docs/ClangFormat.html>
- Clang-tidy and compilation databases: <https://clang.llvm.org/extra/clang-tidy/>
- actionlint: <https://github.com/rhysd/actionlint>
- GitHub workflow artifacts: <https://docs.github.com/en/actions/tutorials/store-and-share-data>
- GitHub Actions secure-use guidance: <https://docs.github.com/en/actions/reference/security/secure-use>

---

## Final instruction to the coding LLM

Implement this document in the stated phases. Preserve behavior constraints in §2.
Do not combine refactoring, build cleanup, data changes, and functional fixes in one
commit. For every functional correction, first add a linked test that reproduces the
old behavior and fails; then make the smallest root-cause change that turns it green.
At the end of every phase, run all host gates, sanitizers, both ARM configurations, the
tree-cleanliness check, and update the evidence summary against the exact new SHA.
