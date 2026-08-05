# Stratosonde Firmware Audit — Confirmation and Remediation Handoff

**Repository:** https://github.com/stratosonde/firmware  
**Prepared for:** A coding LLM or firmware engineer performing a second-pass verification  
**Original static review date:** 2026-08-02  
**Scope:** Project-owned STM32 firmware, LoRaWAN integration, GNSS, sensors, external-flash logging, configuration persistence, payload encoding, regional operation, watchdog behaviour, and low-power transitions.

---

## Instructions to the Reviewing Coding LLM

Perform a fresh audit against the repository's **current HEAD**. Do not assume that any finding below is still present, and do not apply a proposed fix merely because it appears plausible.

Before changing code:

1. Record the branch name and exact commit hash reviewed.
2. Build the firmware using the project's intended toolchain.
3. Identify generated/vendor code versus project-owned code.
4. Confirm, reject, or partially confirm every finding below using direct source evidence.
5. For confirmed findings:
   - Explain the actual failure path.
   - Rate severity and likelihood.
   - Propose the smallest robust fix.
   - Identify all affected callers and persisted/wire formats.
   - Add or describe regression tests.
6. For rejected findings:
   - Reference the code that disproves the finding.
   - Explain whether the concern was impossible, already fixed, or based on an incorrect assumption.
7. Do not change persistent structures, flash layouts, radio payloads, LoRaWAN counters, or protocol fields without documenting backward compatibility and migration behaviour.
8. Prefer explicit status propagation and transactional state machines over hidden side effects.
9. Treat unattended, cold-temperature, low-power, multi-day balloon operation as the operating environment.
10. Produce one final table containing:
    - Finding ID
    - Confirmed / rejected / partially confirmed
    - Severity
    - Files and functions
    - Fix implemented or recommended
    - Tests added or required
    - Compatibility implications
    - Remaining risk

### Required review header

```text
Branch:
Commit:
Toolchain:
Build configuration:
Build result:
Binary size:
Static-analysis tools:
Reviewer/model:
Date:
```

---

# Executive Risk Summary

The original review found a sound overall direction but several possible defects around:

- Fatal-error handling
- Blocking peripheral calls
- GNSS DMA ring-buffer accounting
- Flash-log recovery and acknowledgement
- Configuration persistence and CRC coverage
- LoRaWAN transmit queue semantics
- Compact pressure encoding
- Timestamp width
- Payload validity and field consistency
- Power-loss-safe persistent storage

The central design question is:

> At precisely what point is a stored record considered durably accepted, queued, submitted to the LoRaWAN MAC, transmitted, and safe to mark as delivered?

No read, scan, conversion, or queue-peek operation should silently advance durable delivery state.

---

# P0 Findings — Confirm Before Flight

## F-001 — `Error_Handler()` may return to invalid execution

**Original severity:** Critical  
**Likely files:** `Core/Src/main.c` and initialization callers  
**Likely symbol:** `Error_Handler()`

### Concern

The original review observed an `Error_Handler()` implementation that logs and returns. STM32-generated and application code commonly assumes this function never returns. If initialization fails and execution continues, later code may use invalid HAL handles or falsely report successful initialization.

### Confirm

- [ ] Locate `Error_Handler()`.
- [ ] Determine whether it returns, loops, resets, or transfers control.
- [ ] Find every direct caller.
- [ ] Identify callers that continue initialization after calling it.
- [ ] Determine whether the watchdog eventually resets the unit.
- [ ] Determine whether the same fault will cause a permanent reboot loop.

### Failure scenarios to trace

- Clock initialization failure
- GPIO, DMA, RTC, SPI, I2C, UART, ADC, or radio initialization failure
- Peripheral reinitialization failure after STOP2
- HAL failure followed by a blocking call using an invalid handle

### Recommended repair

Separate fatal and recoverable behaviour:

```c
typedef enum {
    FAULT_CLOCK_INIT,
    FAULT_FLASH_INIT,
    FAULT_RADIO_INIT,
    FAULT_SENSOR_BUS_INIT,
    /* ... */
} FaultReason;

_Noreturn void Fatal_Reset(FaultReason reason);
void Report_Nonfatal(FaultReason reason);
```

`Fatal_Reset()` should:

1. Persist a compact fault breadcrumb when safe.
2. Put outputs and buses in a safe state.
3. Optionally allow a bounded debug-log flush.
4. Reset through `NVIC_SystemReset()`.
5. Never return.

Optional peripherals should enter a deliberate degraded mode instead of invoking a nominally fatal handler.

### Required tests

- [ ] Fault-inject every initialization call.
- [ ] Verify no invalid handle is subsequently used.
- [ ] Verify the persistent reset reason survives reboot.
- [ ] Verify repeated failure produces a diagnosable state rather than an opaque loop.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-002 — First-boot configuration defaults may not be persisted

**Original severity:** High  
**Likely file:** `Core/Src/config.c`  
**Likely symbols:** `Config_Init()`, `Config_Save()`, `g_config_initialized`

### Concern

The initialization path may load defaults and call `Config_Save()` before the global initialized flag is set. If the public save routine refuses to operate before initialization, factory defaults are never written and the recovery path repeats on each boot.

### Confirm

- [ ] Trace blank-flash startup.
- [ ] Trace invalid-CRC startup.
- [ ] Check when `g_config_initialized` becomes true.
- [ ] Check whether `Config_Save()` rejects the call during recovery.
- [ ] Verify the return value of the attempted save is handled.

### Recommended repair

Use a private low-level writer that does not depend on public initialization state:

```c
static ConfigStatus Config_WriteInternal(const SystemConfig_t *config);
```

Set the initialized flag only after either:

- A valid configuration is loaded, or
- Defaults are successfully committed and read back.

### Required tests

- [ ] Entire configuration sector erased.
- [ ] Invalid magic.
- [ ] Valid magic with invalid CRC.
- [ ] Power loss during first default write.
- [ ] Reboot after successful default recovery.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-003 — Configuration CRC may cover the wrong bytes

**Original severity:** High  
**Likely file:** `Core/Src/config.c`  
**Likely structure:** `SystemConfig_t`

### Concern

The code may zero the CRC field but compute over `sizeof(SystemConfig_t) - sizeof(crc32)`. If the CRC member is not physically the last field, this excludes unrelated trailing bytes instead of excluding only the CRC field.

### Confirm

- [ ] Inspect the exact structure layout.
- [ ] Record `offsetof(SystemConfig_t, crc32)`.
- [ ] Check packing, padding, and compiler assumptions.
- [ ] Identify the range passed to the CRC routine.
- [ ] Mutate each byte and verify whether validation fails.

### Recommended repair

```c
SystemConfig_t temp = *config;
temp.crc32 = 0U;
return CRC32(&temp, sizeof(temp));
```

Also consider:

```c
_Static_assert(sizeof(SystemConfig_t) == EXPECTED_SIZE,
               "Persistent configuration layout changed");
```

For long-term maintainability, serialize a defined on-flash representation rather than relying indefinitely on native C structure layout.

### Required tests

- [ ] Bit-flip every byte individually.
- [ ] Confirm every configuration field is protected.
- [ ] Confirm changing only the stored CRC fails.
- [ ] Confirm structure padding is deterministic.
- [ ] Test all compiler optimization levels used in release builds.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-004 — LoRa transmit queue item may be removed before MAC acceptance

**Original severity:** Critical  
**Likely file:** `LoRaWAN/App/lora_app.c`  
**Likely symbol:** queue transmit function calling `LmHandlerSend()`

### Concern

A queued payload may be removed before `LmHandlerSend()` reports success. `BUSY`, duty-cycle restrictions, invalid state, not joined, or other submission failures would then permanently discard it.

### Confirm

- [ ] Trace queue-head selection.
- [ ] Identify when head, tail, and count are updated.
- [ ] Check every `LmHandlerSend()` return code.
- [ ] Determine whether failed submissions are retried.
- [ ] Determine whether a reset between dequeue and submission loses the packet.

### Recommended repair

Use explicit stages:

1. Peek queue head.
2. Attempt MAC submission.
3. Remove from the pending queue only after accepted submission.
4. Track an in-flight item separately.
5. Commit delivery state only according to the chosen semantics:
   - MAC accepted
   - Radio TX completed
   - Confirmed uplink acknowledged
   - Backend receipt, if such acknowledgement exists

Document which semantic is used.

### Required tests

- [ ] MAC busy.
- [ ] Not joined.
- [ ] Duty-cycle restriction.
- [ ] Invalid payload size.
- [ ] Radio error.
- [ ] Reset during submission.
- [ ] Confirmed uplink without ACK.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-005 — Bulk conversion failure may acknowledge unencoded records

**Original severity:** Critical  
**Likely file:** `LoRaWAN/App/lora_app.c`

### Concern

The bulk path may read `record_count` records, encode only `packed_count`, skip failed conversions, then use the original `record_count` when marking records transmitted.

Example:

```text
Read:       sequences 100, 101, 102, 103, 104
Encoded:    100, 101,      103, 104
Committed:  five records
Result:     sequence 102 may be silently lost
```

### Confirm

- [ ] Find the read count.
- [ ] Find the encoded count.
- [ ] Trace conversion-error handling.
- [ ] Identify the exact value later used to advance the watermark.
- [ ] Check whether sequence identities are retained after packing.

### Recommended repair

Commit exact sequence identities, not assumed counts:

```c
typedef struct {
    uint32_t first_sequence;
    uint32_t last_sequence;
    uint16_t encoded_count;
    uint32_t packet_id;
} FlashTxToken;
```

If records in a packet are not contiguous, retain the exact list or refuse to skip internal records.

### Required tests

- [ ] Conversion failure at first record.
- [ ] Conversion failure in middle.
- [ ] Conversion failure at last record.
- [ ] Multiple failures.
- [ ] Packet accepted but not transmitted.
- [ ] Reboot with an in-flight token.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-006 — Flash reads may modify the transmitted watermark

**Original severity:** Critical  
**Likely file:** `Core/Src/flash_log.c`

### Concern

A record-scanning function may update `last_transmitted_sequence` when encountering corruption. That gives a read operation a durable side effect and can skip later valid records when combined with count-based acknowledgement.

### Confirm

- [ ] Search every write to `last_transmitted_sequence`.
- [ ] Classify each write as read, recovery, erase, or transmit commit.
- [ ] Determine what happens when corruption appears between valid records.
- [ ] Determine how the later acknowledgement value is calculated.

### Recommended repair

Make scans side-effect-free. Return:

- Valid records
- Their exact sequence numbers
- Corrupt positions
- End-of-log reason

Only an explicit commit function should advance durable delivery state.

### Required tests

- [ ] Valid, corrupt, valid sequence.
- [ ] Corrupt first record.
- [ ] Corrupt last record.
- [ ] Multiple corrupt regions.
- [ ] Reboot after scan but before transmit.
- [ ] Confirm no scan alone alters the header.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-007 — Flash-header selection may restore stale metadata

**Original severity:** High  
**Likely file:** `Core/Src/flash_log.c`

### Concern

Header freshness may be inferred from `record_count`. Marking records transmitted can alter the watermark without changing that count. Two valid headers can then have equal freshness values but different metadata, and the boot tie-break can choose the stale copy.

### Confirm

- [ ] Identify both header copies.
- [ ] Identify the freshness/generation field.
- [ ] Trace header writes after new record, transmit acknowledgement, corruption handling, and erase.
- [ ] Verify tie handling.
- [ ] Check counter-wrap behaviour.

### Recommended repair

Add an independent monotonic header generation incremented on every committed metadata change. Use wrap-safe comparison.

### Required tests

- [ ] Equal record count, different watermark.
- [ ] Power loss during alternate-header write.
- [ ] Generation-counter wrap.
- [ ] One valid and one invalid header.
- [ ] Both valid but semantically inconsistent.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-008 — Records written before the first header checkpoint may disappear

**Original severity:** High  
**Likely file:** `Core/Src/flash_log.c`

### Concern

If headers are checkpointed every ten records and recovery exits immediately when stored `record_count == 0`, the first one through nine physically written records may not be discovered after early power loss.

### Confirm

- [ ] Identify header checkpoint frequency.
- [ ] Trace reboot with a zero-count header but valid record data following it.
- [ ] Check whether frontier scanning runs from an empty state.
- [ ] Determine how erased versus incomplete records are distinguished.

### Recommended repair

Always perform bounded frontier discovery from the stored write location, including the zero-count case. Continue while records are valid, aligned, and sequence-consistent.

### Required tests

- [ ] Power loss after records 1 through 9 individually.
- [ ] Power loss during the tenth record.
- [ ] Incomplete first record.
- [ ] Valid records followed by erased flash.
- [ ] Valid records followed by corrupt data.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-009 — Erase-all may leave the transmitted watermark unchanged

**Original severity:** Critical  
**Likely file:** `Core/Src/flash_log.c`  
**Likely symbol:** `FlashLog_EraseAll()`

### Concern

Erase may reset addresses, count, and next sequence but leave `last_transmitted_sequence`. New records starting at low sequence values can then appear already delivered.

### Confirm

- [ ] List every state field in the flash-log header.
- [ ] Compare initialization defaults with erase-all assignments.
- [ ] Confirm both header copies are rewritten.
- [ ] Confirm RAM and flash state remain identical after erase.

### Recommended repair

Reset the full logical state:

- Write pointer
- Oldest pointer
- Record count
- Next sequence
- Last delivered/transmitted sequence
- Corruption counters
- In-flight token
- Generation
- Any wrap or mission identifiers

Write and validate two clean empty headers.

### Required tests

- [ ] Add, transmit, erase, add again.
- [ ] Erase after sequence wrap.
- [ ] Power loss during erase.
- [ ] Reboot immediately after erase.
- [ ] Verify first new record is eligible for transmission.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-010 — Failed record writes may consume sequence numbers

**Original severity:** High  
**Likely file:** `Core/Src/flash_log.c`

### Concern

`next_sequence` may be incremented before the flash write is confirmed. A failed write creates a sequence gap and complicates recovery assumptions.

### Confirm

- [ ] Identify when the record sequence is assigned.
- [ ] Identify when `next_sequence` increments.
- [ ] Check write and read-back verification.
- [ ] Determine whether sequence gaps are legal elsewhere.

### Recommended repair

1. Construct the record using the current sequence.
2. Write it.
3. Verify status and, if appropriate, read it back.
4. Commit metadata.
5. Increment the sequence only after successful record commitment.

### Required tests

- [ ] Write failure before any byte.
- [ ] Partial-page write.
- [ ] Verification mismatch.
- [ ] Power loss before header update.
- [ ] Retry after failure.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-011 — GNSS DMA head calculation may permit an infinite loop

**Original severity:** Critical  
**Likely file:** `Core/Src/atgm336h.c`

### Concern

A DMA producer position calculated as:

```c
head = buffer_size - NDTR;
```

can equal `buffer_size` when `NDTR == 0`. If the consumer tail wraps to zero, it can never equal that out-of-range head value, creating a nonterminating processing loop.

### Confirm

- [ ] Inspect the exact DMA mode.
- [ ] Inspect the head calculation.
- [ ] Determine possible values returned by `__HAL_DMA_GET_COUNTER()`.
- [ ] Trace behaviour at transfer-complete boundaries.
- [ ] Confirm whether the watchdog is serviced inside the loop.

### Minimum repair

```c
head = (buffer_size - __HAL_DMA_GET_COUNTER(&hdma_rx)) % buffer_size;
```

### Preferred repair

Maintain an absolute producer count using half-transfer and transfer-complete events, then derive the ring index only when reading. This also enables overrun detection.

### Required tests

- [ ] `NDTR == buffer_size`.
- [ ] `NDTR == 1`.
- [ ] `NDTR == 0`.
- [ ] Half-transfer callback.
- [ ] Transfer-complete callback.
- [ ] Rapid repeated wrap.
- [ ] Parser delayed for more than one buffer duration.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-012 — GNSS DMA overrun may be undetectable

**Original severity:** High  
**Likely file:** `Core/Src/atgm336h.c`

### Concern

A ring represented only by `head == tail` cannot distinguish “no new bytes” from “producer completed one or more full laps.” NMEA data can be lost silently if processing is delayed.

### Confirm

- [ ] Determine whether half/full DMA callbacks increment an epoch or byte count.
- [ ] Determine whether producer-consumer distance is known.
- [ ] Determine whether overruns are counted or logged.
- [ ] Measure worst-case processing latency versus GNSS baud rate and buffer size.

### Recommended repair

Track monotonic producer and consumer totals. If:

```text
producer_total - consumer_total > buffer_size
```

declare an overrun, discard to a known sentence boundary, increment an error counter, and continue.

### Required tests

- [ ] Exactly one full lap.
- [ ] More than one full lap.
- [ ] Overrun in middle of NMEA sentence.
- [ ] Recovery at next `$`.
- [ ] Verify no infinite parser state.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-013 — Compact pressure encoding may collapse most flight pressures to zero

**Original severity:** Critical protocol defect  
**Likely files:** `Core/Src/payload_encode.c`, `Core/Inc/payload_format.h`

### Concern

The original review identified a conversion equivalent to:

```c
(pressure_mbar - 950) / 10
```

with negative values clamped to zero. This would encode 900, 500, 200, 100, and 50 hPa identically.

### Confirm

- [ ] Locate the exact pressure conversion.
- [ ] Generate encoded values for 1050, 1013.25, 950, 900, 500, 200, 100, 50, and 10 hPa.
- [ ] Check backend decoding.
- [ ] Confirm intended float altitude and pressure range.

### Recommended repair options

Preferred scientific option: preserve raw pressure with sufficient range and precision.

Possible compact representations:

- 16-bit pressure in Pa divided by 2
- 16-bit pressure in 0.1 hPa
- Carefully specified logarithmic pressure
- Derived pressure altitude only as an additional field, not a replacement for raw pressure

Version the payload if the wire interpretation changes.

### Required tests

- [ ] Golden encode/decode vectors across the full range.
- [ ] Monotonicity.
- [ ] Saturation behaviour.
- [ ] Backend compatibility.
- [ ] Unit labels and scaling documented.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-014 — ADC acquisition may block indefinitely

**Original severity:** Critical availability risk  
**Likely file:** `Core/Src/adc_if.c`

### Concern

`HAL_ADC_PollForConversion(..., HAL_MAX_DELAY)` can block until the watchdog resets the unit. If the same failure recurs after reboot, the device can enter a reset loop.

### Confirm

- [ ] Find every `HAL_MAX_DELAY`.
- [ ] Determine whether watchdog servicing occurs during those waits.
- [ ] Determine what value is returned after ADC failure.
- [ ] Determine whether zero is indistinguishable from a valid measurement.

### Recommended repair

```c
if (HAL_ADC_PollForConversion(&hadc, ADC_TIMEOUT_MS) != HAL_OK) {
    return ADC_STATUS_TIMEOUT;
}
```

Do not silently encode zero as valid battery or solar voltage.

### Required tests

- [ ] ADC never completes.
- [ ] ADC initialization fails.
- [ ] Calibration fails.
- [ ] Timeout followed by later recovery.
- [ ] Payload validity bit after failure.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-015 — Archived timestamp may wrap every 65,536 minutes

**Original severity:** High protocol defect  
**Likely file:** `Core/Inc/payload_format.h` and encoder/decoder code

### Concern

A 16-bit Unix-minute timestamp wraps every 45.5 days and is ambiguous without an external epoch. Delayed archive uploads can be assigned to the wrong time window.

### Confirm

- [ ] Identify timestamp width and epoch.
- [ ] Inspect backend reconstruction logic.
- [ ] Determine maximum expected offline duration.
- [ ] Determine behaviour around wrap.
- [ ] Determine whether mission/session identity resolves ambiguity.

### Recommended repair

Use one of:

- 32-bit Unix minutes
- 32-bit Unix seconds
- 24-bit minutes from a documented recent epoch
- Mission-relative time plus persistent mission ID and wrap counter

For simplicity and scientific traceability, 32-bit Unix time is preferred unless payload constraints make it impossible.

### Required tests

- [ ] Current date.
- [ ] Wrap boundary.
- [ ] Upload delayed beyond one wrap interval.
- [ ] Reset without valid GNSS time.
- [ ] Backend ordering of mixed live and archived records.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-016 — LoRaWAN session-context flash operations may not be checked

**Original severity:** High  
**Likely file:** `LoRaWAN/App/lora_app.c`

### Concern

The context-store callback may ignore erase/write results, and restore may inadequately propagate read failures. This can cause session loss, unexpected rejoins, or counter rollback without diagnostics.

### Confirm

- [ ] Trace context save callback.
- [ ] Check erase result.
- [ ] Check write result.
- [ ] Check read result.
- [ ] Check context versioning and integrity.
- [ ] Determine power-loss behaviour.
- [ ] Determine LoRaWAN frame-counter implications.

### Recommended repair

Use two transactional copies with:

- Magic
- Schema version
- Payload length
- Generation
- CRC
- Commit marker

Only expose a context to the LoRaWAN stack after full validation.

### Required tests

- [ ] Torn erase.
- [ ] Torn write.
- [ ] Corrupt CRC.
- [ ] Old valid copy plus newer invalid copy.
- [ ] Counter monotonicity after repeated resets.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

## F-017 — Provisioning or session keys may be printed over RTT

**Original severity:** Medium security defect  
**Likely file:** `LoRaWAN/App/lora_app.c`

### Concern

Production firmware should not print root or session keys even through a physical-debug interface.

### Confirm

- [ ] Search logging calls for AppKey, NwkKey, AppSKey, NwkSKey, DevEUI, JoinEUI, and derived session keys.
- [ ] Determine whether logging is compiled into release firmware.
- [ ] Confirm real secret files are excluded from Git.
- [ ] Scan binary strings where practical.

### Recommended repair

```c
#if defined(PROVISIONING_BUILD) && defined(ALLOW_SECRET_LOGGING)
/* Deliberate temporary key output */
#endif
```

Normal flight firmware should not contain the formatting path.

### Required tests

- [ ] Secret-pattern scan of source.
- [ ] `strings` scan of release binary.
- [ ] Verify provisioning build remains explicitly opt-in.

### Reviewer disposition

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
Remaining risk:
```

---

# P1 Findings — Reliability and Maintainability

## F-018 — Configuration validation may be incomplete

Confirm these invariants:

- [ ] `critical_battery < low_battery < bulk_battery`
- [ ] Work intervals have sensible minimum and maximum values
- [ ] GNSS timeout cannot exceed mission/watchdog limits
- [ ] Region, spreading factor, data rate, and TX power combinations are legal
- [ ] Boolean and enum values are in range
- [ ] Bulk count fits actual payload and MAC limits
- [ ] Mission and commissioning modes are valid
- [ ] Reserved bytes are normalized

Recommended repair: create one complete semantic validator and reject CRC-valid but operationally invalid configurations.

```text
Status:
Evidence:
Fix:
Tests:
```

---

## F-019 — Configuration may use only one erase-and-write copy

A brownout during update can destroy the previous valid configuration.

Recommended repair: use two slots with generation, CRC, schema version, and commit marker. Never erase the last known-good copy until the replacement is fully committed.

```text
Status:
Evidence:
Fix:
Tests:
Migration:
```

---

## F-020 — LoRaWAN may initialize before runtime configuration is loaded

### Confirm

- [ ] Record exact startup order.
- [ ] Identify settings consumed only during stack initialization.
- [ ] Determine whether later updates fully reconfigure the stack.

Recommended repair: load and validate configuration before LoRaWAN initialization, or explicitly classify fields as boot-only versus runtime-changeable.

```text
Status:
Evidence:
Fix:
Tests:
```

---

## F-021 — Flash headers may lack semantic validation

Validate:

- [ ] Addresses lie inside the partition
- [ ] Addresses are record-aligned
- [ ] Counts do not exceed capacity
- [ ] Oldest and next-write pointers are consistent
- [ ] Watermark is plausible
- [ ] Generations and sequences are plausible
- [ ] Header does not point into metadata sectors

A correct CRC does not prove that metadata is logically possible.

```text
Status:
Evidence:
Fix:
Tests:
```

---

## F-022 — Full flash erase may exceed the watchdog window

### Confirm

- [ ] Measure worst-case sector erase time at low temperature and low supply voltage.
- [ ] Determine watchdog period.
- [ ] Check refresh behaviour between sectors.
- [ ] Check restart behaviour after power loss.

Recommended repair: use an idempotent erase state machine with watchdog service and persisted progress where needed.

```text
Status:
Evidence:
Fix:
Tests:
```

---

## F-023 — Blocking GNSS API may not service the watchdog

A separate `GNSS_GetPosition()`-style API may block until timeout without servicing the watchdog even if the main flight path uses a safer loop.

Recommended repair: remove unused blocking APIs or give them a hard deadline, watchdog hook, and explicit cancellation/status result.

```text
Status:
Evidence:
Fix:
Tests:
```

---

## F-024 — Negative altitude may wrap into a large unsigned value

Test:

- [ ] -500 m
- [ ] -1 m
- [ ] 0 m
- [ ] Maximum expected balloon altitude
- [ ] Out-of-range positive altitude

Recommended repair: use a signed wire field or a documented offset with explicit saturation.

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
```

---

## F-025 — Archived solar voltage may be hardcoded to zero

If the persistent record does not store solar voltage but the archived packet includes the field, zero falsely appears to be a valid measurement.

Recommended repair:

- Add solar voltage to the durable record and migrate/version it, or
- Remove the field from that payload version, or
- Include a validity bitmap marking it unavailable.

```text
Status:
Evidence:
Fix:
Tests:
Compatibility:
```

---

## F-026 — Region-specific SF/DR mapping may be incomplete

Review every supported region:

- [ ] US915
- [ ] EU868
- [ ] AU915
- [ ] AS923-1
- [ ] AS923-2
- [ ] AS923-3
- [ ] AS923-4
- [ ] Any other compiled region

Check every legal spreading-factor/data-rate combination and payload-size limit.

Recommended interface:

```c
bool Region_MapSfToDr(RegionId region, uint8_t sf, int8_t *dr);
```

Return failure for unsupported combinations. Do not silently substitute a default.

```text
Status:
Evidence:
Fix:
Tests:
```

---

## F-027 — Commissioning join may retry forever

An infinite retry loop that feeds the watchdog is not a crash, but it can leave manufacturing or provisioning indefinitely stuck.

Recommended repair:

- Attempt count
- Elapsed-time limit
- Explicit failure code
- Deliberate user/service recovery path

Do not necessarily apply this limit to autonomous flight rejoin behaviour; commissioning and flight policies may differ.

```text
Status:
Evidence:
Fix:
Tests:
```

---

## F-028 — Ascent timing may reset after reboot

If ascent-to-float transition is based only on RAM uptime, repeated resets can continually restart the timer.

Recommended repair: persist the transition deadline or infer mission phase from pressure/altitude trend plus elapsed absolute time.

```text
Status:
Evidence:
Fix:
Tests:
```

---

## F-029 — STOP2 resume may ignore peripheral reinitialization failures

Inspect wake restoration for:

- [ ] System clock
- [ ] GPIO
- [ ] DMA
- [ ] I2C
- [ ] SPI
- [ ] External flash
- [ ] UART/GNSS
- [ ] RTC
- [ ] Radio
- [ ] ADC

Recommended repair: return a subsystem capability mask and perform bounded recovery. Fatal dependencies should trigger a recorded reset; optional sensors should enter an explicit degraded state.

```text
Status:
Evidence:
Fix:
Tests:
```

---

## F-030 — `EnvSensors_Read()` may always report success

Cached last-known-good values and staleness tracking are useful, but a top-level success return can hide partial or complete acquisition failure.

Recommended result:

```c
typedef enum {
    SENSOR_OK_TEMP     = 1u << 0,
    SENSOR_OK_HUMIDITY = 1u << 1,
    SENSOR_OK_PRESSURE = 1u << 2,
    SENSOR_OK_BATTERY  = 1u << 3,
    SENSOR_OK_SOLAR    = 1u << 4
} SensorStatus;
```

Also distinguish fresh, stale cached, invalid, and never acquired.

```text
Status:
Evidence:
Fix:
Tests:
```

---

# Simplification and Architecture Recommendations

These are not automatically defects. Confirm whether they materially improve safety, power consumption, or clarity before refactoring.

## A-001 — Consolidate ADC acquisition

Acquire battery, VDDA, and solar readings in one bounded operation per work cycle rather than repeatedly initializing and calibrating the ADC.

```c
typedef struct {
    uint16_t raw_vref;
    uint16_t raw_battery;
    uint16_t raw_solar;
    uint16_t vdda_mv;
    uint16_t battery_mv;
    uint16_t solar_mv;
    bool valid;
} AdcSample;
```

Evaluate whether calibration is needed on every read or only at boot/temperature intervals.

---

## A-002 — Separate acquisition, storage, encoding, submission, and commit

Preferred flow:

```text
Acquire immutable sample
        ↓
Persist sample with sequence
        ↓
Select records without modifying durable state
        ↓
Encode packet and create exact TX token
        ↓
Submit packet to MAC
        ↓
Commit exact token at the documented success point
```

Enforce these rules:

- A read function does not change delivery state.
- An encoder does not decide which records are delivered.
- A queue operation does not alter flash metadata.
- A transmit callback does not infer record identity from a count alone.

---

## A-003 — Reuse one transactional persistence layer

Configuration, flash headers, LoRaWAN context, and regional state can share a generic two-slot mechanism:

```c
typedef struct {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t payload_length;
    uint32_t generation;
    uint32_t payload_crc;
    uint32_t commit_marker;
} PersistentRecordHeader;
```

Required properties:

- Last valid copy survives an interrupted update.
- New copy is validated before becoming active.
- Generation comparison is wrap-safe.
- Schema migration is explicit.
- Payload alignment and partition boundaries are checked.

---

## A-004 — Add compile-time layout assertions

```c
_Static_assert(sizeof(CompactPayload) == 11,
               "Compact payload wire format changed");

_Static_assert(sizeof(FlashLogHeader) == EXPECTED_HEADER_SIZE,
               "Flash header layout changed");
```

Also assert field offsets where external compatibility depends on them.

---

## A-005 — Explicitly version every wire format

Each packet should contain or unambiguously imply:

- Protocol version
- Packet type
- Mission/session identifier
- Record sequence
- Timestamp convention
- Validity/staleness information
- Scaling and units

Maintain shared golden vectors for firmware and backend decoders.

---

## A-006 — Reduce production logging

Use compile-time logging levels and compact event IDs. Avoid routine floating-point formatting in the flight build. Never include secrets.

Measure:

- Flash-size difference
- Runtime cost
- Current consumption
- Risk of exposing sensitive values

---

## A-007 — Reduce unnecessary LoRaWAN link checks

Confirm whether a link check is requested on every compact uplink. Consider requesting it periodically, after a material link change, or when cached metrics become stale. Evaluate regional duty-cycle and downlink implications.

---

# Positive Design Elements to Preserve

- [ ] Fault handlers record a breadcrumb and reset rather than hanging.
- [ ] Independent watchdog starts early.
- [ ] Long STOP2 sleeps are segmented within watchdog limits.
- [ ] Sensor subsystem keeps last-known-good values and tracks staleness.
- [ ] I2C bus recovery clocks the bus and generates a STOP.
- [ ] Multi-region credential/state storage uses redundant copies.
- [ ] Persistent multi-region structures have compile-time size checks.
- [ ] Active credential files are ignored by Git.
- [ ] Project documentation and fault IDs are maintained.
- [ ] Release build remains recoverable under peripheral failure.

Do not remove these protections while simplifying the implementation.

---

# Required Test Plan

## Host-side tests

### Configuration

- [ ] Blank flash
- [ ] Invalid magic
- [ ] Invalid CRC
- [ ] Every-byte mutation test
- [ ] Semantic-invalid but CRC-valid values
- [ ] Torn update
- [ ] Schema migration
- [ ] Generation wrap

### Flash log

- [ ] Write one through nine records then reboot
- [ ] Header checkpoint boundary
- [ ] Partial record
- [ ] Partial header
- [ ] Corrupt record between valid records
- [ ] Full partition
- [ ] Circular wrap
- [ ] Erase and reuse
- [ ] Exact-sequence acknowledgement
- [ ] Failed packet conversion
- [ ] Failed MAC submission
- [ ] Reset with in-flight packet
- [ ] Header-generation wrap

### Payloads

Create golden encode/decode vectors for:

- [ ] Pressure: 1050, 1013.25, 950, 900, 500, 200, 100, 50, 10 hPa
- [ ] Temperature minimum and maximum
- [ ] Humidity minimum and maximum
- [ ] Altitude: -500, -1, 0, normal float, maximum
- [ ] Battery and solar bounds
- [ ] Invalid/stale sensor fields
- [ ] Timestamp current date
- [ ] Timestamp wrap boundary
- [ ] Delayed archive records
- [ ] All packet versions

### GNSS DMA and parser

- [ ] `NDTR == buffer_size`
- [ ] `NDTR == 1`
- [ ] `NDTR == 0`
- [ ] Half transfer
- [ ] Full transfer
- [ ] One complete producer lap
- [ ] Multiple producer laps
- [ ] Truncated NMEA sentence
- [ ] Invalid checksum
- [ ] Recovery at next sentence boundary
- [ ] No fix
- [ ] Old-fix expiry
- [ ] GNSS time invalid

### Region and LoRaWAN

- [ ] Every supported region
- [ ] Every legal SF/DR combination
- [ ] Maximum payload by DR
- [ ] MAC busy
- [ ] Duty-cycle delay
- [ ] Not joined
- [ ] Join failure
- [ ] Rejoin after reset
- [ ] Context corruption
- [ ] Counter monotonicity
- [ ] Confirmed uplink without ACK
- [ ] Downlink and callback reentrancy assumptions

---

## Hardware-in-the-loop tests

- [ ] Fault-inject every HAL initialization
- [ ] Interrupt power during record write
- [ ] Interrupt power during header write
- [ ] Interrupt power during configuration write
- [ ] Interrupt power during LoRaWAN-context write
- [ ] 72-hour STOP2/GNSS/LoRa/flash soak
- [ ] Fill and wrap entire external flash
- [ ] Cold-start GNSS repeatedly
- [ ] Low-battery transition
- [ ] Marginal solar input
- [ ] Brownout during radio TX
- [ ] Peripheral-bus lockup
- [ ] I2C recovery at low temperature
- [ ] Repeated STOP2 entry and wake in chamber
- [ ] Watchdog recovery from deliberate hangs
- [ ] Verify reset-reason breadcrumb after every injected fault
- [ ] Measure average and peak current by mission state

Test at representative conditions, including the intended low-temperature range and degraded supply voltage.

---

# CI and Static-Analysis Recommendation

```text
1. Reproducible release and debug builds
2. -Wall
3. -Wextra
4. -Wshadow
5. -Wconversion
6. -Wdouble-promotion
7. cppcheck and/or clang-tidy
8. Host tests for:
   - configuration
   - payloads
   - flash-log model
   - GNSS parser/DMA accounting
   - region mapping
9. Binary flash/RAM size regression limits
10. Secret-pattern scan
11. Golden protocol-vector verification
```

Review warnings individually. Embedded vendor middleware may require narrowly scoped warning suppressions; do not suppress warnings globally merely to obtain a clean build.

---

# Suggested Remediation Order

## Phase 1 — Stop invalid execution and hangs

1. F-001 fatal/recoverable error handling
2. F-014 bounded ADC waits
3. F-011 GNSS DMA boundary bug
4. F-012 GNSS overrun detection
5. F-029 STOP2 restoration status
6. Persistent reset-reason reporting

## Phase 2 — Make telemetry storage transactional

1. F-006 side-effect-free reads
2. F-005 exact sequence identity
3. F-004 queue peek/submit/commit
4. F-007 independent header generation
5. F-008 zero-count frontier recovery
6. F-009 complete erase reset
7. F-010 increment sequence after successful write

## Phase 3 — Correct wire representations

1. F-013 pressure encoding
2. F-015 timestamp width and epoch
3. F-024 signed-altitude behaviour
4. F-025 archived-solar validity
5. Protocol version and golden vectors

## Phase 4 — Harden all persistent state

1. F-002 first-boot configuration
2. F-003 CRC coverage
3. F-019 dual-slot configuration
4. F-016 transactional LoRaWAN context
5. F-021 semantic header validation
6. Generic persistence component

## Phase 5 — Regional, mission, and production hardening

1. F-026 region-specific SF/DR mapping
2. F-027 commissioning termination policy
3. F-028 reset-safe mission timing
4. F-017 remove key logging
5. F-030 structured sensor validity
6. Current and cold-chamber qualification

---

# Required Final Report Template

```markdown
# Verification Report

## Reviewed Revision

- Repository:
- Branch:
- Commit:
- Toolchain:
- Build:
- Reviewer/model:
- Date:

## Summary

- Confirmed:
- Partially confirmed:
- Rejected:
- Already fixed:
- New critical findings:
- Build/test limitations:

## Finding Matrix

| ID | Disposition | Severity | Files/functions | Evidence | Fix | Tests | Compatibility | Remaining risk |
|---|---|---:|---|---|---|---|---|---|
| F-001 | | | | | | | | |
| F-002 | | | | | | | | |
| F-003 | | | | | | | | |
| F-004 | | | | | | | | |
| F-005 | | | | | | | | |
| F-006 | | | | | | | | |
| F-007 | | | | | | | | |
| F-008 | | | | | | | | |
| F-009 | | | | | | | | |
| F-010 | | | | | | | | |
| F-011 | | | | | | | | |
| F-012 | | | | | | | | |
| F-013 | | | | | | | | |
| F-014 | | | | | | | | |
| F-015 | | | | | | | | |
| F-016 | | | | | | | | |
| F-017 | | | | | | | | |
| F-018 | | | | | | | | |
| F-019 | | | | | | | | |
| F-020 | | | | | | | | |
| F-021 | | | | | | | | |
| F-022 | | | | | | | | |
| F-023 | | | | | | | | |
| F-024 | | | | | | | | |
| F-025 | | | | | | | | |
| F-026 | | | | | | | | |
| F-027 | | | | | | | | |
| F-028 | | | | | | | | |
| F-029 | | | | | | | | |
| F-030 | | | | | | | | |

## New Findings

For each new finding include:

- ID
- Severity
- File/function
- Exact failure path
- Trigger
- Consequence
- Recommended fix
- Regression test
- Compatibility implications

## Patch Summary

- Files changed:
- Persistent-layout changes:
- Wire-protocol changes:
- Backend changes required:
- Migration behaviour:
- Rollback behaviour:

## Test Results

### Host tests

### Static analysis

### Hardware-in-the-loop

### Power and cold testing

## Flight Recommendation

Choose one:

- Not suitable for flight
- Bench flight only
- Short recovery flight
- Long-duration experimental flight
- Long-duration scientific flight

Explain the remaining assumptions and risks.
```

---

# Final Constraint

Do not mark a finding resolved solely because the code compiles. A resolution requires:

1. The failure path is removed.
2. The intended invariant is explicit.
3. A regression test exercises the original trigger.
4. Persistent and wire-format compatibility is documented.
5. Power-loss and reboot behaviour remain correct.
