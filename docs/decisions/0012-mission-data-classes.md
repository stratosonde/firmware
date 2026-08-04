# DDR-0012: Mission Data Classes — First-Class and Best-Effort

Status: Proposed  
Date: 2026-08-03

Context: Expansion applications can produce data with very different scientific value and storage cost. A calibrated RTD or capacitive humidity measurement belongs in the durable scientific record. A camera image may be valuable but large, opaque, and incomplete. Treating both identically either endangers core science or prevents useful experimentation.

## Decision

All application-produced data is explicitly classified as one of two classes.

### First-class mission data

First-class data is part of the authoritative scientific record.

It is:

- Schema identified and versioned.
- Validated for framing, length, and declared producer identity.
- Assigned a stable Stratosonde archive record identifier.
- Stored transactionally in durable flash.
- Retained until acknowledged under the scientific-archive delivery policy.
- Scheduled with core high-resolution data during archive opportunities.
- Never silently replaced with fabricated defaults.

Examples include calibrated extension temperature, RTD, capacitive humidity, radiation, or chemistry measurements.

### Best-effort application data

Best-effort data is an opaque application object or fragment.

It may be:

- Buffered in a bounded low-priority spool.
- Rejected when time, power, flash, or radio budget is unavailable.
- Evicted before transmission.
- Partially transmitted.
- Transmitted without confirmation.
- Permanently lost without affecting mission success.

Examples include JPEG images, thumbnails, diagnostic captures, or experimental application output.

Acceptance into the best-effort spool means only that Stratosonde copied the accepted bytes under its control. It is not a promise of durable retention or backend delivery.

Classification is based on required guarantees, not on which physical board produced the data. An Application Controller may publish both classes during one session.

## Rules

- First-class storage and acknowledgement cannot be weakened by best-effort load.
- Best-effort data cannot starve heartbeat or first-class archive traffic.
- Every payload carries producer, schema or content type, version, object or record identity, and integrity information appropriate to its class.
- The service contract lives in `../ApplicationServicesProtocol.md`.
- Over-the-air framing and port assignments live in `../LoRaWANApplicationProtocol.md`.

## Consequences

- Experimental payloads can use spare resources without putting scientific integrity at risk.
- Application developers receive a clear contract for what “accepted” means.
- First-class extension records require durable variable-record storage and backend schemas.
- Best-effort consumers must tolerate missing, duplicate, reordered, or partial fragments.
