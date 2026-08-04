# DDR-0005: Worst-Case Region Payload Sizing

**Status:** Accepted  
**Date:** 2026-07-30  
**Context:** LoRaWAN payload size across multiple regions  

## Decision

Payloads are sized to the **worst case across all target regions** — US915 DR0 minimum (11 bytes app payload for compact packet). This ensures packets never exceed the limit regardless of which region the sonde is currently operating in.

### Sizing
- **Compact packet (SF10/DR0):** 10 bytes app payload + 1 byte status = 11 bytes total
- **Bulk packet (SF7/DR3):** 222 bytes (within the ~242B DR3 limit, budgeting ~400ms dwell)
- **LinkCheckReq:** Rides FOpts header (1 byte), separate from app payload budget
- **DeviceTimeReq:** Also rides FOpts; no conflict with ADR OFF

## Verification Required
Confirm the worst-case floor holds for every enabled region (US915, EU868, AS923, AU915). Regional defs may vary by a byte.

## Consequences
- Status byte was incorrectly removed to "make room" for LinkCheck — it should be restored since LinkCheck rides FOpts
- The DR must be pinned and verified committed before each `LmHandlerSend` call