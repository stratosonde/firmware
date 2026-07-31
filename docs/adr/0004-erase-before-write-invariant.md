# ADR-0004: Erase-Before-Write Invariant for NOR Flash

**Status:** Accepted  
**Date:** 2026-07-30  
**Context:** W25Q16JV NOR flash ring buffer integrity  

## Decision

A flash slot must be **erased before it is written — always**. NOR flash can only clear bits (1→0) on write; only a sector erase resets bits (0→1). Writing onto a non-erased slot bit-ANDs old+new → CRC garbage.

### Rules
1. Header A → dedicated sector 0, Header B → dedicated sector 1, with erase-before-write
2. Data region starts at 0x2000 (8 KB offset, after 2 header sectors)
3. In a wrapped ring, erase the sector ahead of the write frontier BEFORE writing into it
4. Advance `oldest_addr` for records lost with erased sectors
5. On boot, validate `write_addr` / `oldest_addr` / `record_count` / sequence for self-consistency
6. Bounded record loss on unclean reboot is acceptable; structural integrity is inviolable

### Frontier Detection
Once the ring has wrapped, there is no blank/erased gap — a "scan until all-ones" detector does NOT work. The frontier is detected by **sequence-number discontinuity** (where next ≠ prev+1).

## Consequences
- Both headers cannot share sector 0 (current bug P0-3)
- Reboot-onto-unsynced-slots is the same problem as wrap — solved uniformly
- `HEADER_UPDATE_INTERVAL` is the tuning knob for worst-case bounded loss