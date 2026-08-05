# Stratosonde Docs — Start Here

**One rule:** work tracking lives in [GitHub issues](https://github.com/stratosonde/firmware/issues) (labels `gate:1`…`gate:6`, `bench-gate`, `decision`, `project-health`). This directory holds status, design doctrine, and reference specs — not task lists.

| I want to… | Read |
|---|---|
| Know what is done / bench-pending / open | **[ProjectStatus.md](ProjectStatus.md)** — the single status page |
| Know *why* things are the way they are | [decisions/](decisions/) — DDR-0001…DDR-0012 (cited at enforcement points in code) |
| Work on something | [GitHub issues](https://github.com/stratosonde/firmware/issues) — the only work tracker |
| Understand the system design | [FirmwareArchitecture.md](FirmwareArchitecture.md) + module docs (System, PowerManagement, GNSS, EnvironmentalSensors, FlashLogging, Transmission, RegionLookup, LEDStatus, Configuration, ErrorHandler) |
| Work on wire formats | [LoRaWANApplicationProtocol.md](LoRaWANApplicationProtocol.md) (target spec) · [PayloadFormats.md](PayloadFormats.md) ⚠ known-stale until Gate 3 ⑬ · [GNSSDetailPacket.md](GNSSDetailPacket.md) |
| Work on the Qwiic expansion | [QwiicApplicationArchitecture.md](QwiicApplicationArchitecture.md) (package index) · [QwiicTransportProtocol.md](QwiicTransportProtocol.md) · [ApplicationServicesProtocol.md](ApplicationServicesProtocol.md) · [ApplicationPayloadDeveloperGuide.md](ApplicationPayloadDeveloperGuide.md) |
| Work on regions / h3lite | [MultiRegionSupport.md](MultiRegionSupport.md) · [MultiRegionImplementationGuide.md](MultiRegionImplementationGuide.md) · [RegionDataAudit.md](RegionDataAudit.md) · [H3LiteIntegration.md](H3LiteIntegration.md) |
| Look up an old finding (F-xx, FW-xx, Rxx, N-xx, H3-x, T1–T4) | [archive/](archive/) — point-in-time ledgers and reviews, incl. the per-finding verification detail behind the issue tracker |
| See the legacy readiness matrix (F1–F28, FW-1…22, T1–T4, H3, bench gates B1–B8) | [ProductionReadinessAssessment.md](ProductionReadinessAssessment.md) (legacy ledger — living status is ProjectStatus.md) |
