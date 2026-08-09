# Stratosonde Docs — Start Here

**One rule:** work tracking lives in [GitHub issues](https://github.com/stratosonde/firmware/issues) (labels `gate:1`…`gate:6`, `bench-gate`, `decision`, `project-health`). This directory holds status, design doctrine, and reference specs — not task lists.

| I want to… | Read |
|---|---|
| Know what is done / bench-pending / open | **[ProjectStatus.md](ProjectStatus.md)** — the single status page |
| Know *why* things are the way they are | [decisions/](decisions/) — DDR-0001…DDR-0013 (cited at enforcement points in code) |
| Work on something | [GitHub issues](https://github.com/stratosonde/firmware/issues) — the only work tracker |
| Understand the system design | [FirmwareArchitecture.md](FirmwareArchitecture.md) + module docs (System, PowerManagement, GNSS, EnvironmentalSensors, FlashLogging, Transmission, RegionLookup, LEDStatus, Configuration, ErrorHandler) |
| Work on wire formats | [LoRaWANApplicationProtocol.md](LoRaWANApplicationProtocol.md) (target spec) · [PayloadFormats.md](PayloadFormats.md) ⚠ known-stale until Gate 3 ⑬ · [GNSSDetailPacket.md](GNSSDetailPacket.md) |
| Work on the Qwiic expansion | [QwiicApplicationArchitecture.md](QwiicApplicationArchitecture.md) (package index) · [QwiicTransportProtocol.md](QwiicTransportProtocol.md) · [ApplicationServicesProtocol.md](ApplicationServicesProtocol.md) · [ApplicationPayloadDeveloperGuide.md](ApplicationPayloadDeveloperGuide.md) |
| Work on regions / h3lite | [MultiRegionSupport.md](MultiRegionSupport.md) · [H3LiteIntegration.md](H3LiteIntegration.md) |
| Look up an old finding (F-xx, FW-xx, Rxx, N-xx, H3-x, T1–T4) | The [GitHub issue](https://github.com/stratosonde/firmware/issues) carrying it — open findings reproduce their evidence + fix in the issue body. Historical ledgers/reviews/audits were deleted from the tree 2026-08-04; git history retains them (last present @ `eaaa1db`) |
