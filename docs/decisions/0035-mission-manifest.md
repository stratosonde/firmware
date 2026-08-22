# DDR-0035 — Mission Manifest: durable mission-lifecycle record in internal flash

**Status:** accepted (2026-08-22) · **Issues:** #281 (H-01), #78 (A-003) · **Cross-links:** DDR-0002, DDR-0018, #270 (C-01), #256 (SP-06), #287 (H-11)

## Context

The mission lifecycle state (COMMISSIONING / ASCENT / FLOAT) plus the launch
reference pressure are the two facts that decide the entire post-boot behaviour —
privacy mode, GNSS use, cadence, RF behaviour, energy consumption, and the FLOAT
min-ascent guard. Both lived **only in RTC backup registers** (DR3, DR15), which
survive resets and standby but die on a full power loss whenever the VBAT rail is
dead or unpopulated.

| Fact | Medium | Survives full power loss? |
|---|---|---|
| LoRaWAN keys, DevAddr, PROVISIONED latch | internal flash Tier-1 bank (3 CRC copies) | yes |
| Per-region sessions, frame counters | internal flash Tier-2 bank (2 CRC slots) | yes |
| Config page | internal flash | yes |
| Science archive | external W25Q | yes |
| **Mission state** | **RTC DR3 only** | **no** |
| **Launch reference** | **RTC DR15 only** | **no** |
| Last position, voltage slope, deadman, breadcrumbs | RTC DR4–DR19 | no (diagnostics) |

So the one fact that must never be lost was the one stored in volatile backup RAM.
`MissionState_Init()` resolved "bank commissioned but DR3 wiped" to ASCENT
(DDR-0002: a mid-air reboot must never land in commissioning). That rule is correct
for an airborne brownout, but it also fires for the benign case: a **commissioned
unit power-cycled on the ground** reboots straight into ASCENT, bypassing the
arming gesture, the PROVISIONED gate, and the GNSS gate.

## Decision

A single **Mission Manifest** record in internal flash, page 127 (`0x0803F800`),
holds the durable lifecycle truth: `magic, version, generation,
flags(bit0 = FLIGHT_STARTED), launch_ref_hpa_x10, launch_epoch_s, reserved[4],
crc32`. Two 1 KB slots in the 2 KB page ping-pong by generation (newest valid
wins, wrap-aware) — the Tier-2 / nvm_slot pattern; a torn write can only kill the
slot being written.

**Boot decision order** (was: DR3 → bank anchor):

1. `manifest.FLIGHT_STARTED` set → **ASCENT** (durable, survives VBAT loss).
2. latch clear + DR3 valid → persisted DR3 state (fast reset cache).
3. latch clear + DR3 blank + session bank commissioned → **COMMISSIONING**: the
   manifest's durable "never armed" outranks the volatile-bank ambiguity. The
   airborne-VBAT-loss case is protected because a real flight always committed
   the latch at the door.
4. latch clear + no bank → COMMISSIONING.

RTC backup registers stay as the power-free reset cache (standby/wake path),
cross-checked against the manifest; the manifest is the durable truth.

### Write policy (the one sanctioned post-commissioning flash write)

The manifest is written **once**, at the flight-door open — the arm gesture (PB13)
or autonomous launch detection, both already gated on PROVISIONED + GNSS (#270,
#287). That instant is on the ground, in the operator's hands, on a good power
rail; one 2 KB page erase (~20–40 ms) per mission lifetime is negligible wear
against 100k endurance. The manifest is **never written in flight**; Tier-1's
"no internal-flash writes in FLIGHT" policy holds everywhere else. `FLIGHT_STARTED`
is a **one-way latch**: set once, never cleared by firmware (DDR-0002 one-way door).

### What does not move

Last position, voltage slope, deadman, reset-cause breadcrumb, and the boot-attempt
counter stay in RTC — diagnostics; losing them on full power loss costs
observability, not correctness. The boot-attempt counter (DR6, F-03) deliberately
stays volatile: a full power loss is not a reset loop, so the counter should restart.

### The ST MAC NVM (pages 126/127 legacy) is dead code

`LmHandlerNvmDataStore()` is never called (the sequencer task
`CFG_SEQ_Task_LoRaStoreContextEvent` is registered but never set), so the two-slot
NVM store in pages 126/127 is unreachable. Tier-1/Tier-2 already persist everything
that matters (keys, DevAddr, frame counters with margin). The manifest reclaims
page 127 (previously "legacy multi-region, retired"). The dead NVM path is
documented as such; physical deletion is a post-flight cleanup (#268 tranche).

## Consequences

- A commissioned unit survives full power loss and reboots into the correct state
  on the ground (COMMISSIONING) and in flight (ASCENT) without VBAT.
- The launch reference survives too — the FLOAT min-ascent guard no longer has an
  unavoidable `s_no_launch_ref` fallback after a power cut.
- Three documented persistence schemes, each single-owner with a durability class:
  **Tier-1/Tier-2** (credentials, durable), **manifest** (lifecycle, durable),
  **W25Q** (science, durable). RTC backup registers are a cache, never the truth.
  Closes A-003 (#78) as "unified and documented."
- One new flash page consumed (127, already linker-reserved); no linker change.

## Verification

- `tests/host/test_mission_manifest.c`: pure rules + full I/O path against an
  in-memory NOR-model flash fake — fresh EMPTY, commit round-trip, generation
  race, torn-write survival, both-slot-loss safe default, ground power-cycle → not
  flight, airborne VBAT-dead → flight.
- `tests/host/test_lt_20260813.c` structural scans pin the Init consultation and
  the EnterFlight commit.
- Bench artifact (attached on issue close): power-cycle the commissioned board
  with and without VBAT; boot log shows COMMISSIONING (ground) / ASCENT (post-arm).
