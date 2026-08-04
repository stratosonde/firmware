# DDR-0011: Heartbeat and Scientific Archive Are Separate Communication Products

Status: Proposed  
Date: 2026-08-03

Context: The longest-range LoRaWAN data rate cannot continuously carry the full-resolution scientific record. The compact 11-byte packet is therefore a compromise: it answers “where am I, am I alive, and roughly what is happening?” It is not the authoritative science product. Coverage can improve abruptly while the balloon passes near populated areas, creating short opportunities to recover the full-resolution archive.

## Decision

Stratosonde treats radio output as two separate communication products:

- **Mission heartbeat:** a compact, long-range packet optimized for presence, coarse state, and detection of a usable network opportunity.
- **Scientific archive:** full-resolution, durably logged records whose recovery is the primary scientific objective.

The default communication posture is long-range heartbeat operation.

A confirmed long-range heartbeat acknowledgement may open an archive opportunity. Stratosonde then switches to the configured high-throughput data rate and transmits one first-class archive packet with a `LinkCheckReq`. If the archive packet receives the required network response and the returned margin and gateway count satisfy configured thresholds, Stratosonde continues an archive burst. If acknowledgement, link-check response, or quality is insufficient, the burst ends immediately and the next cycle returns to long-range heartbeat operation.

Archive delivery uses at-least-once semantics:

- A record is not marked delivered merely because the MAC accepted it or the radio completed transmission.
- A first-class archive packet is committed only after the protocol's declared delivery evidence is received.
- A missing acknowledgement does not prove the server missed the packet; duplicates are expected and the backend deduplicates by stable record identity.

Core heartbeat scheduling may preempt an archive burst so the mission does not disappear while draining history.

The policy optimizes expected recovered scientific value over mission life. Battery remaining after unrecoverable mission loss has no scientific value, although a configured survival floor may still protect the ability to continue heartbeat and future archive attempts.

## Rules

- Heartbeat is a presence and opportunity-detection product, not a substitute for full-resolution science.
- First-class archive traffic has priority over best-effort application traffic.
- Exact spreading factors, thresholds, burst limits, and retry counts are configuration and algorithm details, not fixed by this ADR.
- Radio behavior must remain region legal.
- Detailed state transitions and FPort behavior live in `../LoRaWANApplicationProtocol.md`.

## Consequences

- The system can remain detectable at long range while exploiting brief urban gateway density.
- Full-resolution science remains authoritative even when compact telemetry is lossy.
- Confirmed archive transfer increases downlink dependence and can produce duplicates.
- Backend record identity and deduplication become mandatory.
- The algorithm can evolve after flight data without changing the underlying mission objective.
