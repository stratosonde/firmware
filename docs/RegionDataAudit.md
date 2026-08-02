# Region Data Audit — hplans vs RP002-1.0.4

**Date:** 2026-08-02
**Status:** Read-only audit. **No region data has been changed.** This document
captures every discrepancy found so that fixes can be reviewed before
anything is modified.

## Sources audited

| Source | Role |
|---|---|
| `C:\working\sonde\hplans\regions.csv` | Per-country channel-plan assignments (primary + 2 alternates), with a `RegionalParameters` column flagging rows verified against RP002-1.0.4 |
| `C:\working\sonde\hplans\<PLAN>.geojson` | 15 per-plan polygon files consumed by `generate_lookup_table.py` |
| `C:\working\sonde\hplans\by_country\*.geojson` | Per-country polygons (OSM-Boundaries), used to measure actual polygon coverage |
| LoRa Alliance **RP002-1.0.4** (Sept 2022), Table 1 "Channel plan per ISO 3166-1 country" and Table 4 "Regional parameter common names" | Authoritative reference |

Audit tool: `C:\working\sonde\hplans\audit_vs_rp002.py` (re-runnable,
read-only; full raw output in `audit_report.txt` next to it).

## Channel plans that actually exist in RP002-1.0.4

EU868, US915, CN779 *(deprecated)*, EU433, AU915, CN470,
AS923-1, AS923-2, AS923-3, AS923-4, KR920, IN865, RU864.

**AS923-1B, AS923-1C, CD900-1A and AU915-SB1 do not exist in RP002-1.0.4.**
The AS923 family has exactly four groups (§2.10.10): AS923-1 (offset 0),
AS923-2 (−1.8 MHz), AS923-3 (−6.6 MHz), AS923-4 (−5.9 MHz).

---

## Finding 1 — Assignments to non-existent plans (BLOCKER)

| Country | CSV assignment | RP002-1.0.4 says | Polygon reality |
|---|---|---|---|
| **Malaysia (MYS)** | **AS923-1B** (+EU433) | **AS923-1** (916–919 & 919–924 MHz), EU433 | `AS923-1B.geojson` covers 100% of Malaysia; `AS923-1.geojson` covers 0% |
| **New Zealand (NZL)** | **AS923-1C** (+AU915, EU433) | **AS923-1** / AU915-928 (footnote 4 recommends AS923-1 for the AU/NZ economic zone), IN865, EU433 | `AS923-1C.geojson` is the NZ polygon (bbox lat −52.8…−29.0) |
| *(no country)* | — | CD900-1A not in RP002 | `CD900-1A.geojson` is a ~1°×0.6° patch of open Pacific (lon 142.1–143.1, lat 11.1–11.7) covering only "Challenger Deep"; **no CSV row uses it** |

Both bogus assignments propagate into the firmware: the H3Lite
`REGION_IDS` map contains `AS923-1B=5`, `AS923-1C=6`, `CD900-1A=15`, and
the on-device table resolves **Wellington/Auckland → AS923-1C** and would
resolve Kuala Lumpur → AS923-1B — region names that do not exist and that
no network server will understand.

Both CSV rows also lack the `1.0.4` verification flag (Finding 3).

## Finding 2 — Confirmed device-visible misassignments

| Location | Device returns | RP002-1.0.4 | Root cause |
|---|---|---|---|
| Wellington, Auckland | AS923-1C | AS923-1 (or AU915) | Finding 1: NZ polygon lives in the bogus `AS923-1C.geojson` |
| Hong Kong | **CN470** | **AS923-1** (920–925 MHz) | **Not a polygon error.** `AS923-1.geojson` covers 100% of HK; `CN470.geojson` contains none of it. HK land area is ~0.24 deg² — far smaller than one res-3 H3 cell (~1 deg²). The cell containing HK has its center over mainland China, so only CN470's polyfill claims it. AS923-1 never contests the cell, so conflict resolution never sees HK. Fix requires an explicit small-territory override (e.g. forced cell claims for HKG/MAC-sized territories), not polygon edits |
| Kuala Lumpur | AS923-1B | AS923-1 | Finding 1 |

(T-CITY currently: 49/52 correct, 0 Unknown; the 3 "wrong" are exactly
Wellington, Auckland, Hong Kong — all three are data issues above, not
lookup-engine bugs.)

## Finding 3 — regions.csv rows never verified against RP002-1.0.4

37 rows have an empty `RegionalParameters` column, including **both bogus
assignments above (MYS, NZL)** and most of Sub-Saharan Africa:
AGO, BFA, CMR, CYM, CAF, TCD, CDA ("Challenger Deep"), COD, COG, DJI,
ERI, SWZ, ETH, GAB, GMB, GHA, GIN, GNB, GUY, IRQ, XXK (Kosovo), LSO,
LBR, LBY, MWI, MLI, MAR, MOZ, ESH, SYC, SLE, SSD, SDN, STP, TGO.

Every unverified African row guesses **EU868** where RP002-1.0.4 lists *no
channel plan at all* for that country. These guesses are baked into
`EU868.geojson` — e.g. **South Sudan is 100% inside the EU868 polygon
while its CSV row says "Unknown"**, and RP002 lists no plan for it.

## Finding 4 — regions.csv vs RP002-1.0.4 Table 1 (verified rows)

Real disagreements among the rows that *are* flagged 1.0.4:

| Country | CSV | RP002-1.0.4 | Severity |
|---|---|---|---|
| **Trinidad & Tobago (TTO)** | US915 | **AU915** | High — wrong plan family |
| **Hungary (HUN)** | EU868, EU433, **AS923-1** | EU868, EU433, **AS923-3** | Medium — wrong AS923 sub-group (frequency offset differs) |
| **China (CHN)** | CN470, CN779, **AS923-1** | CN470, CN779 only | Low (alternate never reachable) |
| **Philippines (PHL)** | AS923-3 only | AS923-3, EU868, EU433 | Low |
| Algeria (DZA) | +EU433 | EU868 only | Low |
| Brunei (BRN) | +EU433 | EU868, AS923-1 | Low |
| Cameroon, Gambia, Ghana, Guinea, Lesotho, Mali, Morocco, Seychelles, Togo | +EU868 | EU433 only or nothing | Low–Medium (all unverified rows too) |
| Cook Is. (COK), Niue (NIU), Tokelau (TKL) | missing IN865/EU433 | RP002 lists them | Low |
| Vatican (VAT), Vietnam (VNM) | missing EU433 | RP002 lists it | Low |

## Finding 5 — Alternate plans have no polygon coverage

The per-plan polygon files encode **primary assignments only**. For
nearly every country whose CSV row lists EU433 or AS923-3 as an
alternate, the corresponding polygon file covers **<2%** of that country
(~180 misses, e.g. all of Europe assigned EU433-alt, all AS923-3-alts).
Consequences:

- The device can **never** select EU433 or AS923-3 as a fallback — those
  plan IDs effectively don't exist on the map outside their primary
  countries.
- AS923-3.geojson covers only its primary countries (PHL, CUB, COM, …);
  Europe/Middle-East AS923-3 alternates are unmapped.
- Decide explicitly: either (a) lookup table encodes primary plans only —
  then Region2/Region3 are documentation only and the firmware comment
  should say so; or (b) polygons must be rebuilt to include alternates.

## Finding 6 — Smaller structural issues

- **CN779**: assigned to China in CSV but has no polygon file (deprecated
  in RP002-1.0.4 — harmless but should be dropped from CSV).
- **AU915-SB1**: referenced in `plans.py` plan list; no polygon file, no
  CSV row, not in RP002 — dead entry.
- **CD900-1A**: see Finding 1 — an ocean-only polygon mapped to a
  non-existent plan; covers "Challenger Deep (CDA)" which is assigned
  `Unknown`.
- **Haiti (HTI)**: 3.5% inside `US915.geojson` (bleed from a US polygon)
  but assigned `Unknown`; RP002 lists no plan for Haiti.
- **XXR.geojson**: a `by_country` polygon with no CSV row, covered by
  EU868 (likely an OSM artifact).
- **Kosovo (XXK)**: non-ISO code, assigned EU868, unverified; RP002
  silent.

## Recommended actions (for review — NOT executed)

1. **regions.csv:** MYS → `AS923-1` (primary), NZL → `AS923-1` (primary,
   keep AU915 as alternate), add the `1.0.4` flag; fix TTO → AU915,
   HUN → AS923-3; drop CN779 from CHN.
2. **Polygons:** rebuild `AS923-1.geojson` to include the MYS and NZL
   country polygons; retire `AS923-1B.geojson`, `AS923-1C.geojson`,
   `CD900-1A.geojson` (or regenerate from corrected CSV).
3. **Firmware `REGION_IDS`:** remove AS923-1B/AS923-1C/CD900-1A (frees 3
   of 15 IDs), then regenerate the H3Lite table.
4. **Hong Kong / Macao-class micro-territories:** add an explicit
   small-territory override in `generate_lookup_table.py` (forced claim of
   the res-3 cells covering HKG/MAC to AS923-1, always winning conflicts)
   — polygons alone cannot fix sub-cell territories at res 3.
5. **Unverified rows (Finding 3):** either verify each against RP002 and
   national regulators, or mark the EU868 guesses as provisional in both
   CSV and documentation.
6. **Alternates policy:** decide and document whether Region2/Region3 are
   in-scope for the lookup table (Finding 5).
