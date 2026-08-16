# Stratosonde Firmware — Project Status

**The one page that answers "what is done and what is not."**
Last updated: 2026-08-15 (second pass, after the 2026-08-14 flight-readiness reviews) — **the FR wave fixed all 8 confirmed code findings (#282, #283, #290, #292–#296; red-first host gates, ARM +368 B flash / +2048 B bss → text 198,992 B); the second review (GEO/PWR/MAC) triaged to trackers #297 (PWR-02, bench-gated P0), #298, #299, h3lite#1, with dispositions recorded on #257/#258/#248; two new EXPECT_UNFIXED-gated host suites (`test_pwr`, `test_geo`) pin the bench/data-dependent findings. CI green.** Prior pass: 2026-08-15 sweep @ `4014d33` (#269–#278 fixed, #279–#289 tracked); before that 2026-08-07 refactor batch + doc sweep (#75/#73/#74/#77, #40).

## Status model

| State | Meaning |
|---|---|
| ✅ DONE | Code complete and build-verified; where hardware matters, also bench-verified with a linked artifact |
| 🟡 CODE-DONE / BENCH-PENDING | Fix is in the code and build-verified; the hardware verification artifact is still missing. **Not** flight sign-off |
| ⬜ OPEN | Not implemented |

Rule: nothing is marked ✅ DONE without a linked verification artifact (bench log, scope capture, test output).

## Where the truth lives

| Question | Authoritative source |
|---|---|
| What should I work on / what is open | **GitHub issues** — the only work tracker (labels `priority:*`, `type:*`, `flight-readiness`, `bench-gate`, `decision`, `project-health`) |
| Done / bench-pending / open summary | This page |
| Why things are the way they are (design rationale) | `docs/decisions/` — DDR-0001…DDR-0027 + `manifest.yaml` (CI-checked identity) |
| Conformance queue (what to prove) | `docs/requirements/firmware-conformance-worklist.md` — per-item statuses, annotated 2026-08-15 |
| Requirement → implementation → proof mapping | `docs/requirements/requirements-traceability-matrix.md` — annotated 2026-08-15 |
| Go/no-go evidence | `docs/requirements/flight1-validation-readiness-checklist.md` — annotated 2026-08-15; no boxes checked (no release candidate yet) |
| Per-finding verification detail | **GitHub issues** — each finding's evidence + fix is reproduced in its issue body (#22–#299) |
| Historical ledgers, reviews, audits | **Git history only** — deleted from the tree 2026-08-04; last commit containing them is `eaaa1db` |
| Review sources for the 2026-08-15 sweep | `docs/temp/` (git-ignored): LT_REVIEW_20260813.md, stratosonde-firmware-stability-review-5930294.md, LT_C01_HANDOFF.md, stratosonde-firmware-flight-readiness-review-2026-08-14.md (FR wave), stratosonde-review-20260814.md (GEO/PWR/MAC triage) |
| Wire formats | `docs/PayloadFormats.md` (heartbeat v2 LE, archive v1-v6 with **v6 current**, golden vectors from CI); `docs/LoRaWANApplicationProtocol.md` §6/§7 normative |
| Expansion / Qwiic architecture | `docs/QwiicApplicationArchitecture.md` (package index) |
| Docs index | `docs/README.md` |

## 1. Done ✅ / code-done 🟡

Everything below is code-complete and CI-verified; where hardware matters it is 🟡 bench-pending (§2). Only artifacts with linked evidence are ✅.

- **2026-08-15 FR wave (flight-readiness review, red-first, host + ARM gated):** FR-15/17/18 multiregion persistence + mask + commissioning latch (`fd5262d`, #293–#295) · FR-12 archive-header checkpoint retry (`25a35a4`, #292) · FR-19 ADC-cal fail-closed, FR-16 CtxRestoreDone clear, FR-02 NVM scratch-validate restore, FR-03 unjoined-region silence (`3b800d7`, #296, #283, #282, #290).
- **2026-08-15 review sweep (LT + stability reviews, all host-verified, ARM-gated):** C-01 provisioning latch / flight door (#270) · LT-01 science-timer underflow (#269) · LT-03 post-acquisition env re-sample (#271) · LT-02/H-04/H-06 region-switch rollback + radio-param checks (#272) · H-02 Config_Init ordering (#273) · H-07 RU864 OTAA capture (#274) · LT-05 STOP2 re-init capability degrade (#275) · LT-04 GNSS sleep-safe teardown (#276) · LT-07 burst-FSM deadline + timer re-arm removal (#277) · LT-08/10/11 hygiene (#278). Plus the LT host-suite wiring (`ce551dd`) and the CI gate fix (`bb5c438`).
- **Pre-sweep stability/feature history (issues #60–#268):** R1-R13 / R2-xx / R3-xx review fixes, FR-01..23, STAB-01..13, RV-01..10, SP-01..15, DR-01..10, F-029+ — row-by-row detail in the issue tracker; highlights: SysCaps capability degrade (#149, #104), NVM MAC store (#109), multiregion session bank + rollback (#218, #246, #247), config module + mission_logic host-tested pure halves, DDR manifest CI gate (#223), warning-clean host builds (#268 tracks the remaining CI warning gate).
- **Gate 1 — all six flight blockers** @ `db4330d`: R01+R02 backup-register ownership map · R24+R23 `$PCAS11,2` deletion + NMEA checksum corrections · F-011 GNSS DMA head-wrap · R30/D6 join timeout + flight-entry gating · R08 RTC clock-source failover · F-001 fatal/recoverable split.
- **Earlier workorder (2026-08-01/02):** T1 two-tier session storage, T2 data honesty, T3 mission state machine, T4 flash ring rewrite, F1–F28 and FW-1…FW-22 — detail in git history (`eaaa1db`).
- **h3lite region engine:** H3-1…H3-7, H3-9 host-verified (H3-8 → #56); bounded ring search on the no-region path (verified 2026-08-15).
- **2026-08-06/07 session:** #22–#25, #28, #30, #32–#38, #42, #44–#55, #57–#59, #29, #31, #47 closed; #56 firmware + generator halves. Highlights: #33 heartbeat v2 / archive v4 + golden vectors · #34 confirmed delivery · #35 flash record v4 · #57 GNSS parser batch · #36 sensor/flash silent failures · #29 STOP2 wake latches · #31 ADC/power batch · #47 SONDE_LOG flight gate.

## 2. Code done, bench pending 🟡

- **Gate 1 bench gates (#26):** GPS-fix backup-register dump · LSE kill · NDTR=0 fault injection · join-timeout soak · airborne-mode persistence · tri-constellation/1 Hz apply.
- **Legacy bench gates:** B1–B8, IWDG option bits, end-to-end energy budget (#12, #20, #21).
- **LT-06 battery compensation bench data (#248):** the −40/−50 °C table values need a measured Li-SOCl₂ curve; the lt0813 host suite already asserts monotonicity, so the data has a gate when it arrives.
- **PWR-02 flight-pack cold characterization (#297, with #261):** the Nichicon LTO bench campaign is the critical path — from it derive both the monotonic comp table (ungates #248/LT-06) and the temperature-scheduled brownout floor(T) (ungates `test_pwr`); today a full pack selects SURVIVAL below −62.3 °C.
- **Everything in the 2026-08-15 sweep** is host-verified only; the target/HIL campaigns are #261/#262.

## 3. Open work ⬜ — one line per item, detail in the issues

| Item | Scope |
|---|---|
| **#27** (bench, first) | R25/D7 GNSS standby TTF measurement — highest-value bench test; gates D7 |
| **#26** (bench) | Gate 1 artifacts + bench checks (confirmed-delivery, restricted-region archive, STOP2 re-entry, W25Q SR1 log, ADC timeout) |
| **#20, #12, #21** (bench) | Current by state · flash durability · 24 h soak |
| **#248** (bench data) | LT-06 compensation-table values; cold chamber feeds it |
| **#257** (owner data, P0) | SP-02/GEO-01: populate the h3lite RESTRICTED dataset (unknown land → restricted), regenerate, bump submodule; land h3lite#1 (GEO-04) guards with it |
| **#297** (bench, P0) | PWR-02: Nichicon cold characterization → floor(T) + monotonic comp table (critical path; feeds #248, ungates `test_pwr`) |
| **#284** (code, high) | H-08: one authoritative GNSS fix-acceptance predicate (weak fix must not reset staleness/authorize switch) |
| **#285** (code, high) | H-09: fresh fix must clear GPS-loss silence on the same wake |
| **#286** (code, high) | H-10: fresh-outage recovery must be newest-first |
| **#287** (code, high) | H-11: commissioned home_region + reset-stable Band-0 age anchor |
| **#288** (code, high, **partial**) | H-12: FULL/SLEEP admission + complete-package science **implemented** (commit A `61647a2`, stage 6 gates); durable energy trend remains **open** (bench evidence, Phase B1) |
| **#279, #280, #281, #289** (code, normal) | LT-09 solar flag (batches with #266) · M-02 config threshold consumption · H-01 durable pre-flight state (design) · M-01 deferred ring reconstruction |
| **#298, #299** (code, post-flight) | MAC-01 clamp network-commanded ChannelsNbTrans + DR floor · GEO-05 BR-RF-007/008 mission-configurable ring-search bound |
| **#261, #262** (infra) | HIL lane + release evidence (M-03) — gates the readiness checklist |
| **#268** (infra) | First-party `-Wall -Wextra` cleanliness + CI warning gate |
| **#266** (protocol) | Next wire-format bump — batches #279's solar bit; relates #78/#79 |
| **#78, #79** (code, deferred) | A-003 unify two-slot persistence · A-005 on-wire version for the heartbeat |
| **#56** (partial) | NK GeoJSON → `RESTRICTED.geojson` regeneration (firmware + generator halves done) |
| **#264** (docs) | M-04 requirements/status consistency — partially addressed by the 2026-08-15 doc passes (this page + requirements/* annotated) |
| **#41** | Post-flight: Qwiic expansion decisions + implementation |
| **#43, #60, #61** | Post-flight: AS923 sub-group runtime-settability · MS5607 + SHT31 characterization |

## Ordering hazards (don't unmask bugs while fixing neighbors)

1. **#284 before #285's predicate half** — the same-wake RF-recovery fix (#285) re-evaluates policy after acquisition; what counts as an "accepted fix" is #284's single predicate. Land the predicate first so #285 doesn't re-derive it.
2. **#288 before #248 tuning** — the FULL/SLEEP admission redesign changes what the power model asks of the compensation table; don't bench-tune mode thresholds against the old policy.
3. ~~**#282 needs its own host tests first**~~ — landed 2026-08-15 (FR-02 `3b800d7`): restore now scratch-validates each slot and selects the newest fully valid; structural scans in the findings suite pin it.
4. **#266 is one wire bump** — heartbeat versioning (#79), the solar staleness bit (#279), and any payload changes land together, versioned, with golden vectors both sides (DDR-0027).
5. **Bench-gate rule** — readiness rows flip to verified only when the fix lands *with* its verification artifact.

## 4. Decisions

- **Decided 2026-08-03:** D9 endianness = **LE-as-truth** with golden-packet release gate · D10 delivery = **confirmed archive, at-least-once, stable record identity, backend dedup** (DDR-0005 — supersedes "document TX-completed").
- **Closed since (verified 2026-08-15):** D1–D4 payload rework (#33) · D5 altitude/history (#35) · D6 finer points (#24) · D8 + D12 (#59) · D11 EraseAll (#52) · A-006 RTT delta (#80).
- **Open maintainer calls (each has an issue):** D7 after TTF measurement (#27) · R46 AS923 runtime-settability (#43, post-flight) · DDR-0027 protocol-version encoding + codec retirement horizon (see `decisions/README.md` Remaining Work) · claim-PIN production decision (OD-ID-001).
- **Open expansion decisions:** addresses `0x42`/`0x50`, rail limits, session durations, producer/schema IDs, FPort 12/13, heartbeat v2 encoding, first-class flash format, delivery evidence, spool policy — #41 (post-flight).

## 5. GitHub tracking notes

- 2026-08-15 (second pass): FR wave closed #282, #283, #290, #292–#296 with red-first evidence comments (`fd5262d`, `25a35a4`, `3b800d7`). 2026-08-14 review (GEO/PWR/MAC) triaged: #297 (PWR-02, P0 bench-gate), #298 (MAC-01), #299 (GEO-05) filed; h3lite#1 (GEO-04) filed in the submodule repo; dispositions corroborated/recorded on #257, #258, #248. New gated suites `test_pwr`/`test_geo` in `all` (EXPECT_UNFIXED). GEO-02 comment honesty fix in `lora_app.c`; DDR-0007 implementation-status note (BR-RF-007/008/009 reconciliation).
- 2026-08-15: the LT/C-01 sweep landed #269–#278 (all closed with evidence comments); wave-2 trackers #279–#289 filed with labels + project board; M-03/M-04 consolidated as comments on #261/#262/#264; LT-06 consolidated onto #248. Requirements + readiness docs annotated with dated statuses (`401c33f`, `fdddcb9`, `e88bbae`, `4014d33`).
- Stale 2025-era issues triaged 2026-08-03: #1–#11 and #13–#19 closed as implemented (with evidence comments); #12, #20, #21 left open (bench/measurement tasks, cross-linked to #26/#27).
- 2026-08-04 consolidation: Gate 4 batch #39 split into #49–#59 and closed; 2026-08-04 review findings filed as R45–R51 = #42–#48; new `project-health` label created.
