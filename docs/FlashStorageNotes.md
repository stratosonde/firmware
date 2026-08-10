# Flash Storage Implementation Notes

**Status:** Implementation binding  
**Date:** 2026-08-09  
**Intent authority:** DDR-0004 (archive), DDR-0010 (persistence), DDR-0011 (storage mechanics)  
**Origin:** absorbed from the retired legacy erase-before-write record (legacy decisions directory, retired 2026-08-09)

These are the NOR-flash mechanics the V2 records deliberately classify as implementation bindings. They are load-bearing hardware truths, recorded here so the intent corpus stays implementation-independent.

## Erase-before-write invariant (W25Q16JV NOR)

NOR flash can only clear bits (1→0) on program; only a sector erase resets bits (0→1). Writing onto a non-erased slot bit-ANDs old and new data → CRC garbage.

Rules:

1. Header A lives in dedicated sector 0, Header B in dedicated sector 1, each with erase-before-write. The two headers must never share a sector.
2. The data region starts at 0x2000 (8 KB offset, after the two header sectors).
3. In a wrapped ring, erase the sector ahead of the write frontier **before** writing into it.
4. Advance `oldest_addr` for records lost with erased sectors.
5. On boot, validate `write_addr` / `oldest_addr` / `record_count` / sequence for self-consistency.
6. Bounded record loss on an unclean reboot is acceptable; structural integrity is inviolable.

## Frontier detection in a wrapped ring

Once the ring has wrapped there is no blank/erased gap, so a "scan until all-ones" detector does not work. The frontier is detected by **sequence-number discontinuity** (where next ≠ prev + 1).

Reboot-onto-unsynced-slots is the same problem as wrap and is solved uniformly by this rule.

## Tuning knob

`HEADER_UPDATE_INTERVAL` sets the checkpoint cadence and therefore the worst-case bounded record loss on unclean reboot.

## Test coverage

`tests/host/fake_w25q.c` simulates true NOR semantics (program can only clear bits; 4 KB sector erase), making the erase-before-write invariant host-testable (see `test_flightreadiness.c` T-7b header ping-pong tests).
