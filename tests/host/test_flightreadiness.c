/**
  ******************************************************************************
  * @file    test_flightreadiness.c
  * @brief   Regression tests pinning the 2026-08-08 flight-readiness findings
  ******************************************************************************
  * Companion to test_main.c. Kept in a separate binary because it needs the
  * simulated NOR backend (fake_w25q.c) and compiles flash_log.c, neither of
  * which the existing runner wants.
  *
  * Each test names the finding it pins (FR-xx) and the work-order test ID
  * (T-x). Tests marked EXPECTED-FAIL-BEFORE-FIX are the regression proof: they
  * MUST fail on the current tree and pass after the corresponding fix. Run
  * with `make -C tests/host flight` (add `EXPECT_UNFIXED=1` to invert the exit
  * code while the fixes are pending, so CI can be made green before or after).
  *
  * Usage:  make -C tests/host flight
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#include "timer_if.h"
#include "stm32_systime.h"
#include "fake_w25q.h"

/* ---- test-controlled stubs (mirror test_main.c) ------------------------- */
static uint32_t g_fake_rtc_seconds = 0;
uint32_t TIMER_IF_GetTime(uint16_t *ms) { if (ms) { *ms = 0; } return g_fake_rtc_seconds; }

uint16_t g_host_dma_cndtr = 0;
uint32_t HAL_GetTick(void) { return g_fake_rtc_seconds * 1000U; }

static uint32_t g_fake_epoch = 1754500000U;
SysTime_t SysTimeGet(void) { SysTime_t t = { g_fake_epoch, 0 }; return t; }

/* Units under test. payload_encode.c is #included to reach static helpers,
 * matching the pattern already used by test_main.c. */
#include "../../Core/Src/payload_encode.c"
#include "../../Core/Src/flash_log.c"

/* Pure header, no HAL dependencies — safe to include for the FR-04 checks. */
#include "config.h"

/* ---- harness ------------------------------------------------------------ */
static int g_failures = 0;
static int g_checks = 0;
static int g_expected_failures = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_EQ_I(actual, expected) do { \
    g_checks++; \
    long _a = (long)(actual), _e = (long)(expected); \
    if (_a != _e) { \
        g_failures++; \
        printf("FAIL %s:%d: %s == %ld, expected %ld\n", __FILE__, __LINE__, #actual, _a, _e); \
    } \
} while (0)

/* Marks a check that is KNOWN to fail on the unfixed tree. Counted separately
 * so `EXPECT_UNFIXED=1` can distinguish "still broken as documented" from
 * "something else broke". */
#define CHECK_REGRESSION(cond, fr_id) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        g_expected_failures++; \
        printf("FAIL [%s] %s:%d: %s\n", fr_id, __FILE__, __LINE__, #cond); \
    } \
} while (0)

/* ========================================================================== */
/* T-2 / FR-04 — flash write length alignment                                 */
/* ========================================================================== */
/* FLASH_IF_INT_Write rejects any length where (len & 7) != 0 and returns
 * FLASH_IF_PARAM_ERROR *after* the caller has already erased the page. Any
 * struct written through FLASH_IF_Write must therefore be a multiple of 8.
 *
 * NOTE ON SCOPE: enum width differs between host (4 B) and arm-none-eabi
 * (1 B, -fshort-enums is the ARM default). Structs containing enums cannot be
 * size-checked here — those need _Static_assert in the firmware headers so the
 * ARM build enforces them. SystemConfig_t contains no enums, so its size is
 * identical on both and IS checkable here. That is the FR-04 bug. */
static void test_flash_write_alignment(void)
{
    printf("-- T-2 / FR-04: flash write length alignment\n");

    /* The bug: sizeof(SystemConfig_t) == 99 on both host and target. */
    CHECK_REGRESSION((sizeof(SystemConfig_t) % 8U) == 0U, "FR-04");

    /* Guard the fix direction too: padding must not push it past its page. */
    CHECK(sizeof(SystemConfig_t) <= CONFIG_FLASH_SIZE);

    /* Host-checkable companions (no enums): the flash log header and record
     * are not written through FLASH_IF_Write, but their sizes are load-bearing
     * elsewhere and cheap to pin. */
    CHECK_EQ_I(sizeof(FlashLog_Record_t), FLASH_LOG_RECORD_SIZE);
    CHECK(sizeof(FlashLog_Header_t) <= W25Q_SECTOR_SIZE);

    printf("   sizeof(SystemConfig_t) = %u (need %% 8 == 0)\n",
           (unsigned)sizeof(SystemConfig_t));
}

/* ========================================================================== */
/* T-4 / FR-07 — bulk packet record identity under skips                      */
/* ========================================================================== */
/* FIXED (FR-07 option A, #87): wire v5 (packet_type 0x05) serializes each
 * record's own sequence, so identity is correct by construction for ANY
 * candidate array — compacted by corrupt-skips, failed conversions, or any
 * future filter. This test encodes batches whose true sequences are
 * non-contiguous and asserts the decoded identity of each record equals its
 * true sequence. */

static HighResTelemetryRecord_t make_highres(uint16_t marker)
{
    HighResTelemetryRecord_t r;
    memset(&r, 0, sizeof(r));
    /* Use battery_mv as a per-record fingerprint so we can identify records
     * after decode without depending on the rest of the field layout. */
    r.battery_voltage = marker;
    return r;
}

/* v5 wire contract: record i's sequence is the u32 LE at 2 + i*36. */
static uint32_t decoded_identity_of(const uint8_t *pkt, uint8_t index)
{
    const uint8_t *p = pkt + 2 + (uint32_t)index * BULK_V5_RECORD_WIRE;
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void test_bulk_identity_with_skips(void)
{
    printf("-- T-4 / FR-07: bulk record identity across skipped records\n");

    /* Contiguous batch: identity must be exact (happy-path guard). */
    {
        HighResTelemetryRecord_t recs[3];
        uint32_t true_seq[3] = { 100, 101, 102 };
        for (int i = 0; i < 3; i++) { recs[i] = make_highres((uint16_t)(5000 + i)); }

        uint8_t buf[BULK_V5_OVERHEAD + BULK_V5_MAX_RECORDS * BULK_V5_RECORD_WIRE];
        uint8_t packed = 0; uint16_t len = 0;
        CHECK(EncodeBulkPacketV5(buf, sizeof(buf), sizeof(buf), recs, true_seq, 3,
                                 &packed, &len));
        CHECK_EQ_I(packed, 3);
        for (uint8_t i = 0; i < packed; i++) {
            CHECK_EQ_I(decoded_identity_of(buf, i), true_seq[i]);
        }
    }

    /* Non-contiguous batch: record 101 was corrupt and skipped by
     * FlashLog_GetUnsentRecordsFIFO, so the array holds sequences
     * {100, 102, 103}. v5 must report exactly {100, 102, 103}. */
    {
        HighResTelemetryRecord_t recs[3];
        uint32_t true_seq[3] = { 100, 102, 103 };   /* 101 skipped */
        for (int i = 0; i < 3; i++) { recs[i] = make_highres((uint16_t)(5000 + i)); }

        uint8_t buf[BULK_V5_OVERHEAD + BULK_V5_MAX_RECORDS * BULK_V5_RECORD_WIRE];
        uint8_t packed = 0; uint16_t len = 0;
        CHECK(EncodeBulkPacketV5(buf, sizeof(buf), sizeof(buf), recs, true_seq, 3,
                                 &packed, &len));
        CHECK_EQ_I(packed, 3);

        for (uint8_t i = 0; i < packed; i++) {
            CHECK_EQ_I(decoded_identity_of(buf, i), true_seq[i]);
        }
        (void)len;
    }
}

/* ========================================================================== */
/* T-7 / FR-10 — erase-ahead slack must not be counted as available           */
/* ========================================================================== */
/* FlashLog_EraseSectorIfNeeded erases a full 4 KB sector (64 records) before
 * writing that sector's first record, but GetAvailableRecords() returns
 * min(record_count, FLASH_LOG_MAX_RECORDS) unconditionally. After the ring
 * wraps, up to 63 counted slots are erased 0xFF. */

static sensor_t make_sensors(uint16_t marker)
{
    sensor_t s;
    memset(&s, 0, sizeof(s));
    s.pressure = 500.0f + (float)marker;
    s.temperature = -40.0f;
    s.humidity = 20.0f;
    s.latitude = 1000;
    s.longitude = 2000;
    s.altitudeGps = 20000;
    s.satellites = 8;
    s.gnss_hdop = 1.5f;
    s.gnss_valid = true;
    s.battery_voltage = 5.0f;
    s.solar_voltage = 1.0f;
    return s;
}

static void test_flashlog_basic_roundtrip(void)
{
    printf("-- T-7a / flash log round-trip (guards the fake NOR backend)\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);
    CHECK(hlog.initialized);
    CHECK_EQ_I(hlog.write_addr, FLASH_LOG_DATA_START);
    CHECK_EQ_I(hlog.record_count, 0);

    for (uint16_t i = 0; i < 5; i++) {
        sensor_t s = make_sensors(i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 1000U + i, 0, 0), FLASH_LOG_OK);
    }
    CHECK_EQ_I(hlog.record_count, 5);
    CHECK_EQ_I(FlashLog_GetAvailableRecords(&hlog), 5);
    CHECK_EQ_I(FlashLog_GetUnsentCount(&hlog), 5);

    /* offset 0 = newest */
    FlashLog_Record_t r;
    CHECK_EQ_I(FlashLog_ReadRecord(&hlog, &r, 0), FLASH_LOG_OK);
    CHECK_EQ_I(r.sequence, 4);
    CHECK_EQ_I(FlashLog_ReadRecord(&hlog, &r, 4), FLASH_LOG_OK);
    CHECK_EQ_I(r.sequence, 0);

    /* FIFO read returns oldest-unsent first */
    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    CHECK_EQ_I(FlashLog_GetUnsentRecordsFIFO(&hlog, batch, 6, &got, &skipped), FLASH_LOG_OK);
    CHECK_EQ_I(got, 5);
    CHECK_EQ_I(skipped, 0);
    CHECK_EQ_I(batch[0].sequence, 0);
    CHECK_EQ_I(batch[4].sequence, 4);

    CHECK_EQ_I(FlashLog_MarkRecordsTransmitted(&hlog, 5), FLASH_LOG_OK);
    CHECK(!FlashLog_HasUnsentData(&hlog));

    fake_w25q_free();
}

static void test_erase_ahead_slack(void)
{
    printf("-- T-7 / FR-10: erase-ahead slack counted as available\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    /* Filling the whole 32,640-record ring writes ~2 MB and is slow but
     * bounded (~0.5 s). Do it once; this is the only way to reach the wrap. */
    const uint32_t total = FLASH_LOG_MAX_RECORDS + 8U;   /* wrap by 8 records */
    for (uint32_t i = 0; i < total; i++) {
        sensor_t s = make_sensors((uint16_t)(i & 0xFFFFu));
        if (FlashLog_WriteRecord(&hlog, &s, 1000U + i, 0, 0) != FLASH_LOG_OK) {
            CHECK(0);   /* write must not fail */
            break;
        }
    }
    CHECK_EQ_I(hlog.record_count, total);
    CHECK(FlashLog_HasWrapped(&hlog));

    uint32_t available = FlashLog_GetAvailableRecords(&hlog);
    printf("   record_count=%u available=%u oldest_addr=0x%06X write_addr=0x%06X\n",
           (unsigned)hlog.record_count, (unsigned)available,
           (unsigned)hlog.oldest_addr, (unsigned)hlog.write_addr);

    /* Walk every slot the accounting claims is available and count how many
     * actually read back as erased (0xFF) rather than a valid record. */
    uint32_t erased_but_counted = 0;
    for (uint32_t off = 0; off < available; off++) {
        FlashLog_Record_t rec;
        uint32_t idx = hlog.next_sequence - 1U - off;
        uint32_t addr = FlashLog_GetRecordAddress(&hlog, idx);
        if (fake_w25q_is_erased(addr, sizeof(FlashLog_Record_t))) {
            erased_but_counted++;
        }
        (void)rec;
    }
    printf("   slots counted as available but physically erased: %u\n",
           (unsigned)erased_but_counted);

    CHECK_REGRESSION(erased_but_counted == 0U, "FR-10");

    fake_w25q_free();
}

/* ========================================================================== */
/* T-5 / T-6 — corrupt-record skip accounting                                 */
/* ========================================================================== */
/* The composition rule that FR-08 and FR-09 are about lives in lora_app.c and
 * is not host-compilable. What IS testable here is the contract
 * GetUnsentRecordsFIFO exports to that caller — if this contract is not what
 * lora_app.c assumes, the caller cannot be correct. These tests pin the
 * contract so the FR-08/FR-09 rework has something to build against. */
static void test_fifo_skip_contract(void)
{
    printf("-- T-5/T-6 / FR-08,FR-09: FIFO skip accounting contract\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    for (uint16_t i = 0; i < 6; i++) {
        sensor_t s = make_sensors(i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 2000U + i, 0, 0), FLASH_LOG_OK);
    }

    /* Corrupt the record holding sequence 2 (index 2 from DATA_START). */
    fake_w25q_corrupt(FLASH_LOG_DATA_START + 2U * FLASH_LOG_RECORD_SIZE,
                      FLASH_LOG_RECORD_SIZE);

    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    CHECK_EQ_I(FlashLog_GetUnsentRecordsFIFO(&hlog, batch, 6, &got, &skipped), FLASH_LOG_OK);

    CHECK_EQ_I(skipped, 1);
    CHECK_EQ_I(got, 5);

    /* Contract the caller depends on: the returned array is NOT contiguous in
     * sequence. lora_app.c's base_seq + i assumption is therefore unsound —
     * this is the machine-checkable statement of FR-07. */
    CHECK_EQ_I(batch[0].sequence, 0);
    CHECK_EQ_I(batch[1].sequence, 1);
    CHECK_EQ_I(batch[2].sequence, 3);   /* NOT 2 — the gap */
    CHECK(batch[2].sequence != batch[1].sequence + 1U);

    /* FR-08: the watermark is count-based, so committing (packed + skipped)
     * is only correct when every skip precedes the packing cut. Here a caller
     * that packed only batch[0..1] (budget-limited) and marked
     * 2 + skipped(1) = 3 would retire sequences 0,1,2 — retiring the corrupt
     * record 2 that sits AFTER the cut is fine, but the same arithmetic with
     * a trailing corrupt record silently retires an untransmitted good one.
     * Pin the entry watermark so the rework has a fixed reference. */
    CHECK_EQ_I(hlog.last_transmitted_sequence, 0);
    CHECK_EQ_I(FlashLog_MarkRecordsTransmitted(&hlog, 3), FLASH_LOG_OK);
    CHECK_EQ_I(hlog.last_transmitted_sequence, 3);
    CHECK_EQ_I(FlashLog_GetUnsentCount(&hlog), 3);   /* 3,4,5 remain */

    fake_w25q_free();
}

/* T-6: a batch where every record is unreadable must not wedge — the
 * watermark has to advance or bulk transfer stalls forever. */
static void test_all_corrupt_does_not_wedge(void)
{
    printf("-- T-6 / FR-09: all-corrupt batch must not wedge the watermark\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    for (uint16_t i = 0; i < 4; i++) {
        sensor_t s = make_sensors(i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 3000U + i, 0, 0), FLASH_LOG_OK);
    }
    for (uint32_t i = 0; i < 4; i++) {
        fake_w25q_corrupt(FLASH_LOG_DATA_START + i * FLASH_LOG_RECORD_SIZE,
                          FLASH_LOG_RECORD_SIZE);
    }

    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    FlashLog_StatusTypeDef st = FlashLog_GetUnsentRecordsFIFO(&hlog, batch, 6, &got, &skipped);
    CHECK_EQ_I(st, FLASH_LOG_OK);
    CHECK_EQ_I(got, 0);
    CHECK_EQ_I(skipped, 4);

    /* The anti-wedge path in lora_app.c retires `skipped` here. Verify that
     * retiring exactly `skipped` clears the backlog — if it does not, the
     * caller's anti-wedge is insufficient (FR-09). */
    CHECK_EQ_I(FlashLog_MarkRecordsTransmitted(&hlog, skipped), FLASH_LOG_OK);
    CHECK(!FlashLog_HasUnsentData(&hlog));

    fake_w25q_free();
}

/* ========================================================================== */
/* T-7b — DDR-0004 erase-before-write invariant on the ping-pong headers      */
/* ========================================================================== */
/* The fake backend enforces real NOR semantics (program can only clear bits),
 * so a header rewrite without an intervening erase produces a CRC failure.
 * This pins T4/DDR-0004: the two headers must live in different sectors and
 * each write must erase first. */
static void test_header_pingpong_survives_rewrites(void)
{
    printf("-- T-7b / DDR-0004: header ping-pong erase-before-write\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    /* Force many header rewrites; on real NOR without erase these corrupt. */
    for (uint32_t i = 0; i < 40; i++) {
        sensor_t s = make_sensors((uint16_t)i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 4000U + i, 0, 0), FLASH_LOG_OK);
    }
    CHECK(hlog.header_generation >= 4U);

    /* Re-init from flash: at least one header must still validate, and the
     * recovered frontier must match. */
    uint32_t expect_count = hlog.record_count;
    uint32_t expect_addr  = hlog.write_addr;

    FlashLog_HandleTypeDef reload;
    CHECK_EQ_I(FlashLog_Init(&reload, &hw), FLASH_LOG_OK);
    CHECK_EQ_I(reload.record_count, expect_count);
    CHECK_EQ_I(reload.write_addr, expect_addr);

    /* Kill the newer header outright; the older must carry the unit. */
    uint32_t victim = (reload.active_header == 0) ? 0U : W25Q_SECTOR_SIZE;
    fake_w25q_corrupt(victim, sizeof(FlashLog_Header_t));
    FlashLog_HandleTypeDef degraded;
    CHECK_EQ_I(FlashLog_Init(&degraded, &hw), FLASH_LOG_OK);
    CHECK(degraded.record_count > 0);

    fake_w25q_free();
}

/* ========================================================================== */
/* T-1 / T-3 / T-8 — require production-side extraction, see notes            */
/* ========================================================================== */
/* T-1 (FR-03 NVM slot length round-trip) and T-3 (FR-05 context CRC restamp)
 * live in lora_app.c and multiregion_context.c respectively, neither of which
 * is host-compilable without stubbing the whole LoRaMac MIB surface.
 *
 * The codebase already has the right precedent for this: R47/R49 moved the
 * pure decision logic out of lora_app.c into transmit_plan.c / power_model.c
 * "so the pure decision logic is host-testable with zero hardware". Do the
 * same here as part of the fixes:
 *
 *   FR-03 -> extract the slot header build/validate into Core/Src/nvm_slot.c:
 *              uint32_t NvmSlot_LogicalLen(void);
 *              void     NvmSlot_BuildHeader(NvmSlotHeader_t *h, const void *p,
 *                                           uint32_t len, uint32_t generation);
 *              bool     NvmSlot_HeaderMatches(const NvmSlotHeader_t *h, uint32_t len);
 *            then T-1 is: build with the store-side length, validate with the
 *            restore-side length, assert match — currently 1496 vs 1492.
 *
 *   FR-05 -> extract the margin application into multiregion_context.c as:
 *              void MultiRegion_ApplyCounterMargin(MinimalRegionContext_t *ctx,
 *                                                  uint32_t margin);
 *            which must both add the margin AND restamp the CRC. T-3 is then:
 *            capture -> apply margin -> ValidateContextCRC() must pass.
 *
 * T-8 (offsetof(MinimalRegionContext_t, crc16)) needs multiregion_context.h,
 * which pulls LoRaMacInterfaces.h. Because arm-none-eabi defaults to
 * -fshort-enums and the host does not, the offsets differ between host and
 * target anyway — so T-8 belongs in the firmware as a _Static_assert, not
 * here. Add to multiregion_context.c:
 *
 *   _Static_assert(offsetof(MinimalRegionContext_t, crc16) == 76,
 *                  "context CRC span assumes crc16 at offset 76");
 *
 * and change UpdateContextCRC/ValidateContextCRC to use
 * offsetof(MinimalRegionContext_t, crc16) instead of sizeof(...) - 2.
 */

/* ========================================================================== */
int main(void)
{
    printf("=== Stratosonde flight-readiness regression tests ===\n\n");

    test_flash_write_alignment();
    test_bulk_identity_with_skips();
    test_flashlog_basic_roundtrip();
    test_fifo_skip_contract();
    test_all_corrupt_does_not_wedge();
    test_header_pingpong_survives_rewrites();
    test_erase_ahead_slack();

    printf("\n%d checks, %d failures", g_checks, g_failures);
    if (g_expected_failures > 0) {
        printf(" (%d are documented FR-xx regressions)", g_expected_failures);
    }
    printf("\n");

    const char *expect_unfixed = getenv("EXPECT_UNFIXED");
    if (expect_unfixed != NULL && expect_unfixed[0] == '1') {
        /* Pre-fix mode: only unexpected failures are errors. */
        int unexpected = g_failures - g_expected_failures;
        if (unexpected == 0) {
            printf("PRE-FIX BASELINE OK (%d documented regressions still open)\n",
                   g_expected_failures);
            return 0;
        }
        printf("UNEXPECTED FAILURES: %d\n", unexpected);
        return 1;
    }

    if (g_failures == 0) {
        printf("ALL FLIGHT-READINESS TESTS PASSED\n");
        return 0;
    }
    printf("FLIGHT-READINESS TESTS FAILED\n");
    return 1;
}
