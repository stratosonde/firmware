# ADR-0003: RTC is Flywheel, GPS is Master Clock

**Status:** Accepted  
**Date:** 2026-07-30  
**Context:** Time source hierarchy for long-duration flights  

## Decision

GPS GPRMC time is the **master clock**. On each valid fix, convert UTC to Unix epoch and set the RTC. The RTC is demoted to a **flywheel** — it only coasts through fixless cycles (cold soak, iced antenna, reacquisition) to provide monotonic timestamps.

### Design
1. GPS is master: on each good fix, GPRMC UTC → Unix epoch → set RTC via `SysTimeSet()`
2. RTC is flywheel: coasts through fixless cycles
3. Re-set RTC from GPS on EVERY cold boot (robust even without backup domain power)
4. Software must not depend on VBAT being backed by coin cell/supercap

## Rejected Alternative
- Using the RTC as an independent time source — rejected because the RTC starts at 0 on power-up and nothing was setting it, making "epoch" actually mean "seconds since boot."

## Consequences
- Timestamps in packets and flash records are real UTC, not boot-relative
- Time field must be wide enough for long-duration flights (uint16 minutes wraps at 45.5 days)
- Ground decoder must handle the time field format change