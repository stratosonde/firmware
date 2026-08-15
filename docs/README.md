# Stratosonde Docs — Start Here

**One rule:** work tracking lives in [GitHub issues](https://github.com/stratosonde/firmware/issues) (labels `priority:*`, `type:*`, `flight-readiness`, `bench-gate`, `decision`, `project-health`). This directory holds status, design doctrine, and reference specs — not task lists.

| I want to… | Read |
|---|---|
| Know what is done / bench-pending / open | **[ProjectStatus.md](ProjectStatus.md)** — the single status page |
| Get the whole product contract on one page | **[SYSTEM-INVARIANTS.md](SYSTEM-INVARIANTS.md)** — SI-001…SI-020, each naming its owning DDR (explanatory; DDRs are normative) |
| Orient in the system as a new contributor | **[ARCHITECTURE-OVERVIEW.md](ARCHITECTURE-OVERVIEW.md)** — lifecycle, wake cycle, archive, radio, fault philosophy |
| Know *why* things are the way they are | [decisions/](decisions/) — DDR-0001…DDR-0027 (cited at enforcement points in code); start at [decisions/README.md](decisions/README.md) |
| Know what the first flight must demonstrate | [requirements/flight1-mission-definition.md](requirements/flight1-mission-definition.md) (objectives) vs [requirements/flight1-validation-readiness-checklist.md](requirements/flight1-validation-readiness-checklist.md) (pre-launch evidence gate) |
| Know what to prove / trace a requirement to code and test | [requirements/firmware-conformance-worklist.md](requirements/firmware-conformance-worklist.md) (proof queue) · [requirements/requirements-traceability-matrix.md](requirements/requirements-traceability-matrix.md) — both carry dated per-item statuses (last pass 2026-08-15) |
| Work on something | [GitHub issues](https://github.com/stratosonde/firmware/issues) — the only work tracker |
| Understand the system design | [FirmwareArchitecture.md](FirmwareArchitecture.md) + module docs (System, PowerManagement, GNSS, EnvironmentalSensors, FlashLogging, Transmission, RegionLookup, LEDStatus, Configuration, ErrorHandler) |
| Work on wire formats | [LoRaWANApplicationProtocol.md](LoRaWANApplicationProtocol.md) (target spec) · [PayloadFormats.md](PayloadFormats.md) (current: heartbeat v2 LE, archive v1-v4) · [GNSSDetailPacket.md](GNSSDetailPacket.md) |
| Work on the Qwiic expansion | [QwiicApplicationArchitecture.md](QwiicApplicationArchitecture.md) (package index) · [QwiicTransportProtocol.md](QwiicTransportProtocol.md) · [ApplicationServicesProtocol.md](ApplicationServicesProtocol.md) · [ApplicationPayloadDeveloperGuide.md](ApplicationPayloadDeveloperGuide.md) |
| Work on regions / h3lite | [MultiRegionSupport.md](MultiRegionSupport.md) · [H3LiteIntegration.md](H3LiteIntegration.md) |
| Look up an old finding (F-xx, FW-xx, Rxx, N-xx, H3-x, T1–T4) | The [GitHub issue](https://github.com/stratosonde/firmware/issues) carrying it — open findings reproduce their evidence + fix in the issue body. Historical ledgers/reviews/audits were deleted from the tree 2026-08-04; git history retains them (last present @ `eaaa1db`) |
