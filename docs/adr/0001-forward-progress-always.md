# ADR-0001: Forward Progress Always

**Status:** Accepted  
**Date:** 2026-07-30  
**Context:** Governing design principle for all firmware decisions  

## Decision

The sonde cannot be physically reset at 40 km altitude. Every failure path must **degrade and continue** — never halt, hang, wedge, or loop forever.

### Rules
1. Every failure path must degrade-and-continue
2. Prefer skipping a bad unit of work (a record, a sensor read, a region) over getting stuck
3. Every skip MUST advance a counter or watermark so "carry on" always moves forward and can never loop back onto the same bad spot
4. Log the gap/failure so the ground station knows, then keep flying
5. Reserve a **reset** (never a hang) only for genuinely unrecoverable radio/clock failure

## Consequences
- `Error_Handler()` must not use `__disable_irq(); while(1){}` — replaced with logged degradation
- Flash operations must skip corrupt records, not stop
- Join loops must be bounded with region fallback
- Sensor failures transmit last-known-good with stale bits, never poison defaults