# 2026-08-13 Intent Interview Merge Ledger

**Date:** 2026-08-13
**Purpose:** Audit trail for the merge of the 2026-08-13 intent-interview package
(three sequential passes, staged under the git-ignored `../temp/`) into the
canonical DDR corpus.

This ledger exists so the three passes are never re-merged, so the renumbered
requirement IDs are traceable back to the interview sheets, and so the
deliberate supersessions are recoverable later.

---

## 1. Source material

| Pass | Source files (under `docs/temp/`) |
|---|---|
| 1 | `README.md`, `MERGE-CHECKLIST.md`, `CONFORMANCE-NOTES.md`, and `*.AMENDMENT.md` for 0002, 0003, 0009, 0010, 0011, 0012, 0013, 0014, 0015, 0016, 0018 |
| 2 | `README(1).md`, `MERGE-CHECKLIST(1).md`, `CONFIRMED-NO-CHANGE.md`, `0010-*.PASS2-AMENDMENT.md`, `0015-*.PASS2-AMENDMENT.md` |
| Final | `README(2).md`, `MERGE-CHECKLIST(2).md`, `SYSTEM-INVARIANTS.md`, `ARCHITECTURE-OVERVIEW.md`, `0022-long-lived-wire-protocol-compatibility.PROPOSED.md` |
| Standalone | `stratosonde_flight_1_mission_definition.md` |

`docs/temp/llm_notes/` is local-only working material and is deliberately **not**
merged or committed.

---

## 2. Intra-package supersessions (later pass wins)

The three passes are not independent. Merging them literally would have injected
retracted intent. Resolved as follows:

### S-1 — Exact next-wake timer is NOT mandatory durable state

Pass 1 (`0010-*.AMENDMENT.md`) listed "Next-wake deadline, wake epoch, or
equivalent scheduling state" as mission-critical persistent state.

The final pass (`README(2).md` §1) **supersedes** this: the *configured cadence*
must survive reset, but the exact transient timer/deadline may be lost when the
consequence is at most one missed or shifted observation.

**Merged as:** DDR-0010 keeps *cadence and scheduling anchor* as critical, and
explicitly classifies the exact transient timer register as tolerable-loss.
DDR-0012 keeps the "no rapid reset/wake loop and no unintended multi-day sleep"
requirement, which was the real intent behind the pass-1 wording.

### S-2 — First-flight energy admission is FULL-or-SLEEP, not a three-tier GNSS shed

Pass 1 (`0016-*.AMENDMENT.md`) proposed three energy tiers:
cadence → GNSS suppression → sleep.

The final pass (`README(2).md` §2) **supersedes** the middle tier: once a
scheduled wake begins, first-flight firmware does not deliberately select an
energy-only partial science mode such as "sensors + radio but no GNSS".

**Merged as:** DDR-0016 records cadence as the long-term lever and FULL/SLEEP as
the per-wake admission decision. Pass-1 "Tier B" is recorded as not-adopted.

Note: the pass-1 Tier B also contradicted the pre-existing `INV-PWR-001`
("GNSS SHALL NOT be shed merely because the energy tier dropped"). The final
pass therefore restores corpus consistency rather than changing it.

---

## 3. Corpus conflict resolved — GNSS-outage RF silence: 6 h → 24 h

**This was the only hard contradiction between the package and shipped,
tested firmware.**

| Side | Position |
|---|---|
| Live corpus + code | DDR-0016 `INV-PWR-009` / `BR-PWR-014`: 6 h grace window, implemented as `GPS_LOSS_SILENCE_S (6U * 3600U)` in `LoRaWAN/App/lora_app.h`, enforced in `lora_app.c`, energy-conservation rationale (maintainer policy 2026-08-12, issue #141) |
| 2026-08-13 pass 2 | DDR-0015: one **24 h** stale-position budget; `BR-STALE-020` forbids any independent time-based RF cutoff |

**Maintainer decision 2026-08-13: 24 h wins, and the code is corrected in the
same change set.**

Consequences recorded in the corpus:

1. **DDR-0015 owns the single budget.** The first-flight maximum
   RF-authoritative position staleness is 24 h, measured from the last accepted
   quality-valid horizontal fix (or, for the never-fixed case, from the flight
   transition).
2. **DDR-0016 `INV-PWR-009` / `BR-PWR-014` are revised to pointers.** The 6 h
   number and the "a stale position is not worth radio energy"
   energy-conservation rationale are marked **superseded 2026-08-13** and
   retained as history.
3. **The silence reason changes.** RF silence beyond the budget exists because of
   *geographic/regulatory uncertainty* (DDR-0015), not because stale-position
   telemetry is a poor use of energy. Energy policy may still independently
   suppress RF work; it no longer owns the staleness cutoff.
4. **No independent RTC-age cutoff.** DDR-0013 records that loss of fresh GNSS
   *time* never by itself silences RF.

Code/test changes made in this change set:

- `LoRaWAN/App/lora_app.h` — `GPS_LOSS_SILENCE_S` `(6U * 3600U)` →
  `(24U * 3600U)`, comment re-cited to DDR-0015 `BR-STALE-017`.
- `LoRaWAN/App/lora_app.c` — silence-policy comment block updated; the
  evaluation itself is macro-driven and unchanged.
- `tests/host/test_review_findings.c` — new regression asserting the 24 h
  binding and the `<= 24h` allowed / `> 24h` silent boundary convention.
- Historical bug narratives that mention the old "6 h dark sawtooth"
  (`Core/Src/transmit_plan.c`, `tests/host/test_main.c`,
  `tests/host/test_stability_review.c`, `tests/host/test_review_20260812.c`,
  `tests/host/test_review_findings.c`) are annotated, not rewritten — they
  describe real historical defects and remain accurate as history.

No host test asserted the 6 h value numerically before this change, so the
existing string-anchor regressions continue to pass unchanged.

---

## 4. New record number reassignment

The package proposed `DDR-0022: Long-Lived Wire Protocol Compatibility`.

`DDR-0022` is **already assigned** to *Mission Purpose, Value Hierarchy, and
Autonomous Continuity* (added 2026-08-12). `manifest.yaml` rule: *an ID, once
assigned, NEVER changes meaning; retire, never renumber.*

**The wire-protocol record is therefore merged as `DDR-0027`.**

All `DDR-0022` references inside the proposal text that meant the wire-protocol
record were remapped to `DDR-0027`.

The retired old-generation alias `DDR-0027 -> DDR-0014` was removed from
`manifest.yaml` at the same time (no live references to the old-generation
meaning existed). This mirrors the 2026-08-12 retirement of aliases 0022-0026.

Note also that this ledger is deliberately **not** named `NNNN-*.md`:
`tools/check_ddr_manifest.py` treats every `docs/decisions/NNNN-*.md` file as a
DDR record that must appear in the manifest.

---

## 5. Requirement-ID reassignment

Every identifier proposed by the amendment sheets collided with a live
identifier of different meaning, or was a `*-NEW-*` placeholder. Mapping:

| Sheet identifier | Collided with (live meaning) | Merged as |
|---|---|---|
| `INV-LIFE-005` (float latch terminal) | `INV-LIFE-005` = "Cadence follows atmospheric change" — the rule being superseded | `INV-LIFE-011` |
| `P-LIFE-FLOAT-001..003` | placeholder | `P-LIFE-012`, `P-LIFE-013`, `P-LIFE-014` |
| `BR-GNSS-021`, `P-GNSS-011`, `P-GNSS-012` | free | unchanged |
| `INV-FAIL-008` | "Unrecoverable internal faults return to a known path" | `INV-FAIL-016` |
| `BR-FAIL-010` | config-corruption safe defaults | `BR-FAIL-016` |
| `P-FAIL-010`, `P-FAIL-011` | `P-FAIL-010` = "Permanent-failure endurance" | `P-FAIL-011`, `P-FAIL-012` |
| `INV-PERSIST-NEW` | placeholder | `INV-PERSIST-010` |
| pass-2 DDR-0010 invariants | unnumbered prose | `INV-PERSIST-011`, `INV-PERSIST-012` |
| `P-PERSIST-010..012` | free | unchanged |
| `INV-STORE-009` | "Reconstructible metadata is not more authoritative than valid science" | `INV-STORE-011` |
| `BR-STORE-NEW-001` | placeholder; `BR-STORE-*` family did not exist | `BR-STORE-001` (plus `BR-STORE-002`, `BR-STORE-003`) |
| `P-STORE-NEW-001..003` | placeholder | `P-STORE-011`, `P-STORE-012`, `P-STORE-013` |
| DDR-0012 refinements | unnumbered prose | `INV-BOOT-010`, `P-BOOT-012..015` |
| `INV-TIME-NEW` | placeholder | `INV-TIME-009` |
| `P-CONFIG-NEW-001..003` | placeholder | `P-CONFIG-009`, `P-CONFIG-010`, `P-CONFIG-011` |
| `BR-STALE-009..012` (pass 1) | four different CONFIRMED requirements | `BR-STALE-013..016` |
| `BR-STALE-013..016` (pass 2) | overlapped the above | `BR-STALE-017..020` |
| `P-STALE-007..009` (pass 1) | three different existing proofs | `P-STALE-011..013` |
| `P-STALE-010..012` (pass 2) | `P-STALE-010` = "No extra provenance machinery" | `P-STALE-014..016` |
| DDR-0015 new bands/invariants | unnumbered prose | `INV-STALE-008`, `INV-STALE-009` |
| `INV-PWR-009` (sparse-complete preference) | "Sustained GNSS outage silences the radio" | `INV-PWR-022` |
| DDR-0016 proofs | unnumbered prose | `P-PWR-012..015` |
| `INV-COMM-NEW` | placeholder | `INV-COMM-009` |
| `P-COMM-008..011` | four different existing proofs | `P-COMM-011..014` |
| final-pass DDR-0001 | unnumbered prose | `INV-WAKE-012`, `BR-WAKE-017`, `P-WAKE-011..013` |
| final-pass DDR-0004 | unnumbered prose | `INV-ARCH-008`, `BR-ARCH-017..019`, `P-ARCH-011..013` |
| final-pass DDR-0005 | unnumbered prose | `INV-TX-010`, `BR-TX-023..025`, `P-TX-011..013` |
| `INV-PROTO-*`, `BR-PROTO-*`, `OD-PROTO-*` | new family | unchanged, in DDR-0027 |

---

## 6. Dispositions

### Amended records

DDR-0001, 0002, 0003, 0004, 0005, 0009, 0010, 0011, 0012, 0013, 0014, 0015,
0016, 0018.

### New records and documents

- `0027-long-lived-wire-protocol-compatibility.md` (DDR-0027).
- `../SYSTEM-INVARIANTS.md` — cross-cutting invariant contract (SI-001..SI-020),
  explanatory, each SI pointing at its owning DDR.
- `../ARCHITECTURE-OVERVIEW.md` — orientation document for new contributors.
- `../requirements/flight1-mission-definition.md` — Flight 1 mission
  hypothesis/objectives/success levels.

### Confirmations only — no normative change

| Record | Reason |
|---|---|
| DDR-0006 | Nearest-region ring search already required. Contrary code is a conformance defect, not a doc gap. |
| DDR-0019 | One SF10 confirmed probe, no ACK ends RF work for the wake, no same-wake retry — already `INV-RADIO-007` / `BR-RADIO-010`. |
| DDR-0020 | No reset-count escalation already normative. |
| DDR-0021 | Runtime recovery defers to DDR-0009; RF legality defers to DDR-0015. |
| DDR-0025 | Non-OTA policy already owned here; DDR-0014 carries only a scope cross-reference. |

Note: pass 2 recorded "DDR-0005 no new normative change" (ACK-gated backlog was
already normative). The **final** pass separately *does* amend DDR-0005, for
explicit-record-request preemption. Both are true; they concern different topics.

### Already covered by the 2026-08-12 merge (folded as confirmations)

- DDR-0002 §17 already descopes `BR-LIFE-013/014` and makes FLOAT a terminal
  one-way latch. The 2026-08-13 amendment promotes that accepted descope to a
  first-class product invariant instead of restating it.
- `INV-CONFIG-014` already resolves `OD-CONFIG-004` (next-wake activation).
- `INV-CONFIG-011/012` already define clamp-vs-reject configuration behavior.

### Open decisions closed or narrowed

| OD | Resolution |
|---|---|
| `OD-FAIL-006` — reset-loop protection | RESOLVED: no escalation, no reset-count policy, no permanent disable (DDR-0020 + 2026-08-13 interview). |
| `OD-BOOT-006` — reset-loop behavior | RESOLVED by the same decision; pointer to DDR-0009/0020. |
| `OD-PERSIST-001` — complete state inventory | NARROWED: the consequence-based classification and the confirmed minimum inventory are now normative; per-object loss budgets remain open. |
| `OD-ARCH-002` — unavailable requested record | NARROWED: the returned record's own ID is authoritative and sufficient; exact substitute selection remains a binding. |
| `OD-STALE-001` — maximum staleness value | RESOLVED for first flight: 24 h (`BR-STALE-017`). Whether it stays per-mission configurable remains a binding. |

---

## 7. Deliberately not invented

The package repeatedly declined to select numbers. These were **not** guessed
during the merge; they are tracked in `open-intent-questions.md`:

1. exact battery go/no-go voltage/model and hysteresis;
2. exact battery-trend window and cadence scaling;
3. exact automatic-launch pressure/rate/persistence thresholds;
4. exact deliberate launch-button gesture;
5. exact local retry counts and peripheral recovery sequence;
6. archive record-ID width and wrap handling;
7. exact replay-checkpoint rollback/duplicate bound (the interview used "about
   5-10" only as an illustration);
8. exact substitute record chosen when a requested ID is gone;
9. exact future downlink byte encoding and authentication;
10. exact protocol-version encoding and backend codec-retention horizon;
11. exact flash erase geometry and bad-area bookkeeping;
12. exact representation of calibration-invalid science when calibration state
    is unrecoverable;
13. exact LED blink/indication pattern for launch-readiness go/no-go.

The 24 h staleness budget is the one number this package *did* bind, and it is
now normative in DDR-0015.

---

## 8. Verification

- `python tools/check_ddr_manifest.py` — DDR identity/manifest gate.
- `make -C tests/host test` and `make -C tests/host flight` — host suites
  (require the build box; `cc` is not available on the authoring machine).
- New proof obligations from this merge are queued in
  `../requirements/firmware-conformance-worklist.md` and
  `../requirements/requirements-traceability-matrix.md`.
