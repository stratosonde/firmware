/* ARCHIVE — regression record for the flight-readiness campaign (R2/R3 era).
 * Module contracts live in test_<module>.c (see R2_TEST_MAP.md). Do not
 * extend; extend the contract suite for the owning module instead.
 * (Refactor stage 7.) */

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
    const uint8_t *p = pkt + 2 + (uint32_t)index * BULK_V6_RECORD_WIRE;
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

        uint8_t buf[BULK_V6_OVERHEAD + BULK_V6_MAX_RECORDS * BULK_V6_RECORD_WIRE];
        uint8_t packed = 0; uint16_t len = 0;
        CHECK(EncodeBulkPacketV6(buf, sizeof(buf), sizeof(buf), recs, true_seq, 3,
                                 &packed, &len));
        CHECK_EQ_I(packed, 3);
        for (uint8_t i = 0; i < packed; i++) {
            CHECK_EQ_I(decoded_identity_of(buf, i), true_seq[i]);
        }
    }

    /* Non-contiguous batch: record 101 was corrupt and skipped by
     * FlashLog_GetRecoveryRecords, so the array holds sequences
     * {100, 102, 103}. v5 must report exactly {100, 102, 103}. */
    {
        HighResTelemetryRecord_t recs[3];
        uint32_t true_seq[3] = { 100, 102, 103 };   /* 101 skipped */
        for (int i = 0; i < 3; i++) { recs[i] = make_highres((uint16_t)(5000 + i)); }

        uint8_t buf[BULK_V6_OVERHEAD + BULK_V6_MAX_RECORDS * BULK_V6_RECORD_WIRE];
        uint8_t packed = 0; uint16_t len = 0;
        CHECK(EncodeBulkPacketV6(buf, sizeof(buf), sizeof(buf), recs, true_seq, 3,
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
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 1000U + i, 0, 0, 0), FLASH_LOG_OK);
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

    /* R3-04 (#218): recovery read; a fresh log is all pending-live, read
     * ascending from the high water. */
    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    CHECK_EQ_I(FlashLog_GetRecoveryRecords(&hlog, batch, 6, &got, &skipped), FLASH_LOG_OK);
    CHECK_EQ_I(got, 5);
    CHECK_EQ_I(skipped, 0);
    CHECK_EQ_I(batch[0].sequence, 0);
    CHECK_EQ_I(batch[4].sequence, 4);

    /* advance at send time, per record */
    for (uint32_t i = 0; i < got; i++) {
        CHECK_EQ_I(FlashLog_MarkRecoverySent(&hlog, batch[i].sequence), FLASH_LOG_OK);
    }
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
        if (FlashLog_WriteRecord(&hlog, &s, 1000U + i, 0, 0, 0) != FLASH_LOG_OK) {
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
/* SP-19 (#252) — tx_high_water must never lag behind the ring's oldest       */
/* ========================================================================== */
/* A long TX-silent run with continuous logging (GNSS-outage / restricted
 * region silence) wraps the ring past H. Pending-live [H, next) then spans
 * overwritten sequences, and every recovery call burns probes on R2-03
 * identity skips - up to the R13 256-probe bound - per archive opportunity. */
static void test_sp19_high_water_never_behind_oldest(void)
{
    printf("-- SP-19 (#252): ring wrap lifts tx_high_water to oldest\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    /* Wrap by 8 with ZERO sends: sequences 0..7 are overwritten; oldest=8,
     * but tx_high_water still sits at 0. */
    const uint32_t total = FLASH_LOG_MAX_RECORDS + 8U;
    for (uint32_t i = 0; i < total; i++) {
        sensor_t s = make_sensors((uint16_t)(i & 0xFFFFu));
        if (FlashLog_WriteRecord(&hlog, &s, 1000U + i, 0, 0, 0) != FLASH_LOG_OK) {
            CHECK(0);
            break;
        }
    }

    FlashLog_Record_t batch[64];
    uint32_t got = 0, skipped = 0;
    CHECK_EQ_I(FlashLog_GetRecoveryRecords(&hlog, batch, 64, &got, &skipped), FLASH_LOG_OK);

    /* The wrap's oldest resident sequence (erase-ahead slack makes this >8;
     * derive it, don't hardcode). */
    const uint32_t oldest = hlog.next_sequence - FlashLog_GetAvailableRecords(&hlog);
    printf("   got=%u skipped=%u first_seq=%u oldest=%u H=%u\n", got, skipped,
           got ? (unsigned)batch[0].sequence : 0xFFFFFFFFu,
           (unsigned)oldest, (unsigned)hlog.tx_high_water);

    CHECK(oldest > 0U);   /* the ring really wrapped */
    CHECK_REGRESSION(skipped == 0U, "SP-19-no-dead-probes");
    CHECK_REGRESSION(got == 64U, "SP-19-full-batch");
    CHECK_REGRESSION(got == 64U && batch[0].sequence == oldest, "SP-19-first-is-oldest");
    /* The discriminating assertion: H was lifted to oldest (pre-fix it
     * still sits at 0 while the ring starts at 64). */
    CHECK_REGRESSION(hlog.tx_high_water == oldest, "SP-19-h-lifted");

    fake_w25q_free();
}

/* ========================================================================== */
/* T-5 / T-6 — corrupt-record skip accounting                                 */
/* ========================================================================== */
/* The composition rule that FR-08 and FR-09 were about lived in lora_app.c.
 * R3-04 (#218) removed it: marking is per-record by absolute sequence
 * (FlashLog_MarkRecoverySent), so no count arithmetic exists to go wrong.
 * What remains load-bearing is the READ contract: gaps are surfaced, not
 * hidden. */
static void test_fifo_skip_contract(void)
{
    printf("-- T-5/T-6 / FR-08,FR-09 + R3-04: recovery read skip contract\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    for (uint16_t i = 0; i < 6; i++) {
        sensor_t s = make_sensors(i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 2000U + i, 0, 0, 0), FLASH_LOG_OK);
    }

    /* Corrupt the record holding sequence 2 (index 2 from DATA_START). */
    fake_w25q_corrupt(FLASH_LOG_DATA_START + 2U * FLASH_LOG_RECORD_SIZE,
                      FLASH_LOG_RECORD_SIZE);

    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    CHECK_EQ_I(FlashLog_GetRecoveryRecords(&hlog, batch, 6, &got, &skipped), FLASH_LOG_OK);

    CHECK_EQ_I(skipped, 1);
    CHECK_EQ_I(got, 5);

    /* The returned array is NOT contiguous in sequence (the machine-checkable
     * statement of FR-07): the v6 wire format carries per-record explicit
     * sequence identity precisely because of this. */
    CHECK_EQ_I(batch[0].sequence, 0);
    CHECK_EQ_I(batch[1].sequence, 1);
    CHECK_EQ_I(batch[2].sequence, 3);   /* NOT 2 — the gap */
    CHECK(batch[2].sequence != batch[1].sequence + 1U);

    /* R3-04: per-record advance at send. The corrupt record 2 is skipped on
     * every read; marking the good records leaves exactly it behind until a
     * later leading-run retire. */
    CHECK_EQ_I(hlog.tx_high_water, 0);
    for (uint32_t i = 0; i < got; i++) {
        CHECK_EQ_I(FlashLog_MarkRecoverySent(&hlog, batch[i].sequence), FLASH_LOG_OK);
    }
    /* 0,1 marked live-ascending -> H = 2; 3,4,5 raise H to 6. */
    CHECK_EQ_I(hlog.tx_high_water, 6);
    CHECK(!FlashLog_HasUnsentData(&hlog));   /* 2 is corrupt: unrecoverable, excluded */

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
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 3000U + i, 0, 0, 0), FLASH_LOG_OK);
    }
    for (uint32_t i = 0; i < 4; i++) {
        fake_w25q_corrupt(FLASH_LOG_DATA_START + i * FLASH_LOG_RECORD_SIZE,
                          FLASH_LOG_RECORD_SIZE);
    }

    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    FlashLog_StatusTypeDef st = FlashLog_GetRecoveryRecords(&hlog, batch, 6, &got, &skipped);
    CHECK_EQ_I(st, FLASH_LOG_OK);
    CHECK_EQ_I(got, 0);
    CHECK_EQ_I(skipped, 4);

    /* R3-04 (#218): the leading corrupt run is retired INLINE by the read
     * (H advances past the whole contiguous run) - the anti-wedge property
     * no longer depends on any caller-side retire path (FR-09, RV-01). */
    CHECK(!FlashLog_HasUnsentData(&hlog));

    fake_w25q_free();
}

/* ========================================================================== */
/* T-7b — FlashStorageNotes.md erase-before-write invariant on the ping-pong headers      */
/* ========================================================================== */
/* The fake backend enforces real NOR semantics (program can only clear bits),
 * so a header rewrite without an intervening erase produces a CRC failure.
 * This pins T4/FlashStorageNotes.md: the two headers must live in different sectors and
 * each write must erase first. */
static void test_header_pingpong_survives_rewrites(void)
{
    printf("-- T-7b / FlashStorageNotes.md: header ping-pong erase-before-write\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    /* Force many header rewrites; on real NOR without erase these corrupt. */
    for (uint32_t i = 0; i < 40; i++) {
        sensor_t s = make_sensors((uint16_t)i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 4000U + i, 0, 0, 0), FLASH_LOG_OK);
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
/* T-7c — 2026-08-10 review finding #4: ping-pong toggles before write succeeds*/
/* ========================================================================== */
/* FlashLog_SyncHeader() flips hlog->active_header BEFORE the erase+program of
 * the other slot, and both error paths return with the toggle standing. The
 * next call therefore flips back and erases the slot that still held the GOOD
 * header. Two consecutive failed header writes (erase OK, program fails — an
 * intermittent SPI/CS fault or a brownout that outlives the erase) destroy
 * BOTH header copies: record_count resets to 0, the backlog is orphaned
 * (HasUnsentData() == false) and the ring overwrites it.
 *
 * T-7b above never exercises this: it corrupts a header only AFTER successful
 * writes. EXPECTED-FAIL-BEFORE-FIX: passes once active_header is committed
 * only after a successful program (fix verified by the reviewer on a scratch
 * copy: two failed writes then survive). */
static void test_header_pingpong_survives_failed_writes(void)
{
    printf("-- T-7c / finding #4: header ping-pong commit-after-success\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    for (uint32_t i = 0; i < 40; i++) {
        sensor_t s = make_sensors((uint16_t)i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 4000U + i, 0, 0, 0), FLASH_LOG_OK);
    }
    CHECK_EQ_I(FlashLog_SyncHeader(&hlog), FLASH_LOG_OK);
    CHECK_EQ_I(hlog.record_count, 40);

    /* Advance the transmission watermark so the header-loss check below is
     * load-bearing (an untouched watermark is 0 on both sides of the bug).
     * R3-04: per-record advance; marking seq 9 raises H to 10. */
    CHECK_EQ_I(FlashLog_MarkRecoverySent(&hlog, 9U), FLASH_LOG_OK);
    CHECK_EQ_I(hlog.tx_high_water, 10);

    /* Control: ONE failed write must survive even on the unfixed tree (the
     * second header copy carries the unit). Guards the test itself. */
    fake_w25q_fail_next_writes(1);
    CHECK(FlashLog_SyncHeader(&hlog) != FLASH_LOG_OK);
    {
        FlashLog_HandleTypeDef reload1;
        CHECK_EQ_I(FlashLog_Init(&reload1, &hw), FLASH_LOG_OK);
        CHECK_EQ_I(reload1.record_count, 40);
        CHECK_EQ_I(reload1.tx_high_water, 10);
    }

    /* The bug: a SECOND consecutive failed write. On the unfixed tree the
     * in-RAM toggle from the first failure targets the surviving good slot,
     * erases it, and fails the program — both headers are now destroyed. */
    fake_w25q_fail_next_writes(1);
    CHECK(FlashLog_SyncHeader(&hlog) != FLASH_LOG_OK);
    {
        FlashLog_HandleTypeDef reload2;
        CHECK_EQ_I(FlashLog_Init(&reload2, &hw), FLASH_LOG_OK);
        CHECK_REGRESSION(reload2.record_count == 40, "FINDING-4");
        CHECK_REGRESSION(reload2.tx_high_water == 10, "FINDING-4");
        printf("   after 2 failed writes: record_count=%lu (want 40)\n",
               (unsigned long)reload2.record_count);
    }

    fake_w25q_free();
}

/* ========================================================================== */
/* T-8 / finding #8 (2026-08-10): deferred header sync batches a bulk burst    */
/* ========================================================================== */
/* Every watermark advance used to SyncHeader: a 4 KB sector erase per ACKed
 * bulk packet, 20 per burst, all on the same two header sectors. With
 * defer/flush a burst costs ONE erase. Mid-burst reset must replay from the
 * last synced watermark (conservative — backend dedupes by sequence).
 * R3-04 (#218): advances are per-record MarkRecoverySent at send time. */
static void test_deferred_header_sync_batches_burst(void)
{
    printf("-- T-8 / finding #8: deferred header sync batches a bulk burst\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    for (uint32_t i = 0; i < 20; i++) {
        sensor_t s = make_sensors((uint16_t)i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 4000U + i, 0, 0, 0), FLASH_LOG_OK);
    }

    /* Burst of 20 send-time marks, deferred: ZERO header erases, but the
     * RAM watermark is live for the read path. */
    FlashLog_DeferHeaderSync(&hlog);
    uint32_t erases_before = fake_w25q_erase_count;
    for (uint32_t seq = 0; seq < 20; seq++) {
        CHECK_EQ_I(FlashLog_MarkRecoverySent(&hlog, seq), FLASH_LOG_OK);
    }
    CHECK_EQ_I(fake_w25q_erase_count, erases_before);
    CHECK_EQ_I(hlog.tx_high_water, 20);
    CHECK(!FlashLog_HasUnsentData(&hlog));

    /* Reset BEFORE the flush: the watermark replays from the last synced
     * value (0) — conservative direction, records retransmit and dedupe. */
    {
        FlashLog_HandleTypeDef crashed;
        CHECK_EQ_I(FlashLog_Init(&crashed, &hw), FLASH_LOG_OK);
        CHECK_EQ_I(crashed.tx_high_water, 0);
        CHECK(FlashLog_HasUnsentData(&crashed));
    }

    /* Second burst over 20 fresh records, flushed at burst end: exactly ONE
     * additional header erase, and the watermark survives the reboot. */
    for (uint32_t i = 20; i < 40; i++) {
        sensor_t s = make_sensors((uint16_t)i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 4000U + i, 0, 0, 0), FLASH_LOG_OK);
    }
    FlashLog_DeferHeaderSync(&hlog);
    erases_before = fake_w25q_erase_count;
    for (uint32_t seq = 20; seq < 40; seq++) {
        CHECK_EQ_I(FlashLog_MarkRecoverySent(&hlog, seq), FLASH_LOG_OK);
    }
    CHECK_EQ_I(FlashLog_FlushHeaderSync(&hlog), FLASH_LOG_OK);
    CHECK_EQ_I(fake_w25q_erase_count, erases_before + 1);
    {
        FlashLog_HandleTypeDef reload;
        CHECK_EQ_I(FlashLog_Init(&reload, &hw), FLASH_LOG_OK);
        CHECK_EQ_I(reload.tx_high_water, 40);
        CHECK(!FlashLog_HasUnsentData(&reload));
    }

    /* Flush with nothing dirty is a no-op — no erase. */
    erases_before = fake_w25q_erase_count;
    CHECK_EQ_I(FlashLog_FlushHeaderSync(&hlog), FLASH_LOG_OK);
    CHECK_EQ_I(fake_w25q_erase_count, erases_before);

    /* §6a: the veto rides record.flags b5-b7 (DDR-0003: record WHY). */
    {
        sensor_t s = make_sensors(999);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 12345U, 0, 0, 2), FLASH_LOG_OK);
        FlashLog_Record_t rec;
        CHECK_EQ_I(FlashLog_ReadRecord(&hlog, &rec, 0), FLASH_LOG_OK);
        CHECK_EQ_I((rec.flags >> 5) & 0x07, 2);
    }

    fake_w25q_free();
}

/* ========================================================================== */
/* T-9 — 2026-08-11 handoff §6c: frontier scan must not adopt previous-lap     */
/* records when the true frontier sits exactly on a sector boundary            */
/* ========================================================================== */
/* Ring geometry: 64 records/sector, header checkpoint every 10. Fill the ring
 * to a wrap, then stop with the frontier exactly on a sector boundary
 * (320 records = 5 sectors past the wrap). The sector ahead still holds
 * CRC-valid records from the PREVIOUS lap. Without the probe.sequence ==
 * record_count identity check, Init's frontier scan walks up to 10 of them. */
static void test_frontier_scan_rejects_previous_lap(void)
{
    printf("-- T-9 / handoff 6c: frontier scan sector-boundary identity check\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    /* Fill the whole ring, then 320 more so write_addr lands exactly on a
     * sector boundary with the next sector holding previous-lap records.
     * (~2 MB of fake writes, ~1 s — same cost class as T-7.) */
    for (uint32_t i = 0; i < FLASH_LOG_MAX_RECORDS + 320; i++) {
        sensor_t s = make_sensors((uint16_t)i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 4000U + (i & 0xFFFF), 0, 0, 0), FLASH_LOG_OK);
    }
    uint32_t expect_count = hlog.record_count;
    CHECK_EQ_I((hlog.write_addr - FLASH_LOG_DATA_START) % W25Q_SECTOR_SIZE, 0); /* on boundary */
    CHECK_EQ_I(FlashLog_SyncHeader(&hlog), FLASH_LOG_OK);

    /* Reboot: the frontier scan probes the not-yet-erased next sector. */
    FlashLog_HandleTypeDef reload;
    CHECK_EQ_I(FlashLog_Init(&reload, &hw), FLASH_LOG_OK);
    CHECK_EQ_I(reload.record_count, expect_count);
    CHECK_EQ_I(reload.write_addr, hlog.write_addr);

    fake_w25q_free();
}

/* ========================================================================== */
/* T-1 / T-3 — require production-side extraction, see notes (T-8 is above)   */
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

/* ========================================================================== */
/* R2 findings (2026-08-09 pre-flight review, issues #105-#123)               */
/* EXPECTED-FAIL-BEFORE-FIX: each CHECK_REGRESSION proves the bug on the      */
/* current tree and must pass after the root-cause fix lands.                 */
/* ========================================================================== */

static void test_r2_02_watermark_overadvance_on_wrap(void)
{
    printf("-- R2-02 (#106): watermark over-advance on ring wrap\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    /* Fill past capacity: wrap by 8 records. ~0.5 s, same as T-7. */
    const uint32_t total = FLASH_LOG_MAX_RECORDS + 8U;
    for (uint32_t i = 0; i < total; i++) {
        sensor_t s = make_sensors((uint16_t)(i & 0xFFFFu));
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 5000U + i, 0, 0, 0), FLASH_LOG_OK);
    }

    /* Long RF gap: nothing was ever transmitted. */
    CHECK_EQ_I(hlog.tx_high_water, 0);

    /* R3-04 (#218): there is no count-based caller composition left to go
     * wrong (R2-02's hazard class is eliminated, not just fixed) - the read
     * walks by absolute sequence and marking is per-record. What must still
     * hold after a wrap: the phantom low sequences (overwritten slots) are
     * detected as corruption by the identity check and retired without any
     * successful TX, until the real records are reached. */
    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    int reached = 0;
    for (int cycle = 0; cycle < 40 && !reached; cycle++) {
        got = 0; skipped = 0;
        FlashLog_GetRecoveryRecords(&hlog, batch, 6, &got, &skipped);
        if (got > 0) reached = 1;
    }
    CHECK_REGRESSION(reached == 1, "R2-02-reach");

    /* The first good record read is the oldest still retained. */
    CHECK_EQ_I(batch[0].sequence,
               hlog.next_sequence - FlashLog_GetAvailableRecords(&hlog));

    /* Mark exactly what was sent, by absolute sequence; the high water must
     * land exactly past it. */
    uint32_t last_sent = batch[got - 1].sequence;
    for (uint32_t i = 0; i < got; i++) {
        CHECK_EQ_I(FlashLog_MarkRecoverySent(&hlog, batch[i].sequence), FLASH_LOG_OK);
    }
    CHECK_REGRESSION(hlog.tx_high_water == last_sent + 1U, "R2-02");

    /* And the API must refuse to move the watermark backward. */
    CHECK_EQ_I(FlashLog_MarkRecoverySent(&hlog, 1U), FLASH_LOG_OK);
    CHECK_EQ_I(hlog.tx_high_water, last_sent + 1U);

    fake_w25q_free();
}

static void test_r2_03_sequence_identity_crosscheck(void)
{
    printf("-- R2-03 (#107): positional read committed by stored sequence\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    for (uint16_t i = 0; i < 6; i++) {
        sensor_t s = make_sensors(i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 6000U + i, 0, 0, 0), FLASH_LOG_OK);
    }

    /* Plant a CRC-VALID record carrying the WRONG sequence at slot 2 — the
     * sector-boundary/frontier-scan mixup from the finding. Read the real
     * record, swap identity, restamp CRC, poke it back raw. */
    FlashLog_Record_t planted;
    CHECK_EQ_I(W25Q_Read(&hw, FLASH_LOG_DATA_START + 2U * FLASH_LOG_RECORD_SIZE,
                         (uint8_t *)&planted, sizeof(planted)), W25Q_OK);
    planted.sequence = 99;
    planted.crc32 = FlashLog_CRC32((const uint8_t *)&planted,
                                   sizeof(planted) - sizeof(uint32_t));
    fake_w25q_poke(FLASH_LOG_DATA_START + 2U * FLASH_LOG_RECORD_SIZE,
                   &planted, sizeof(planted));

    /* Guards: the plant is genuinely well-formed (passes on every tree). */
    {
        FlashLog_Record_t verify;
        CHECK_EQ_I(W25Q_Read(&hw, FLASH_LOG_DATA_START + 2U * FLASH_LOG_RECORD_SIZE,
                             (uint8_t *)&verify, sizeof(verify)), W25Q_OK);
        CHECK(FlashLog_VerifyRecord(&verify));
        CHECK_EQ_I(verify.sequence, 99);
    }

    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    CHECK_EQ_I(FlashLog_GetRecoveryRecords(&hlog, batch, 6, &got, &skipped),
               FLASH_LOG_OK);

    /* Desired: identity mismatch is treated as corruption — counted skipped,
     * never packed. Today the foreign record sails through and the caller's
     * (seq+1)-entry arithmetic underflows. */
    CHECK_REGRESSION(skipped == 1U, "R2-03");
    bool foreign_packed = false;
    for (uint32_t i = 0; i < got; i++) {
        if (batch[i].sequence == 99) foreign_packed = true;
    }
    CHECK_REGRESSION(!foreign_packed, "R2-03");

    fake_w25q_free();
}

static void test_r2_14_ts_wrap_false_latch(void)
{
    printf("-- R2-14 (#118): TS_WRAP false latch at first GNSS time sync\n");

    /* NOTE: s_ts_wrapped / s_last_ts_min are function-statics in
     * EncodeCompactBinaryPacket; this is the only compact-encode user in
     * this binary, so the statics start fresh here. */
    CompactTelemetryPacket_t pkt;

    sensor_t s = make_sensors(0);
    s.gnss_valid = false;                 /* boot-relative clock, undisciplined */
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 100, 0, MODE_NORMAL));
    CHECK(!(pkt.status & STATUS_TS_WRAP_MASK));   /* guard */

    /* First fix: SysTimeSyncFromGnss jumps the clock to epoch; epoch/60
     * truncated to uint16 lands BELOW the pre-sync value (we pick the ~50%
     * failing case). The wrap bit must NOT latch on the discipline
     * transition. */
    s = make_sensors(0);                  /* gnss_valid = true, not stale */
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 10, 0, MODE_NORMAL));
    CHECK(pkt.status & STATUS_TIME_GNSS_MASK);    /* guard: now disciplined */
    CHECK_REGRESSION(!(pkt.status & STATUS_TS_WRAP_MASK), "R2-14");

    /* Guard the fix direction: a real 45.5-day wrap within the disciplined
     * regime must still set the bit. */
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 60000, 0, MODE_NORMAL));
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 20, 0, MODE_NORMAL));
    CHECK(pkt.status & STATUS_TS_WRAP_MASK);
}

/* RV-01 (#160): all-corrupt probe window (256+ records) wedges bulk transfer.
 * T-6 emulates the caller commit, but the REAL caller's retire path sits
 * inside record_count > 0 — an all-corrupt window (record_count == 0) never
 * reaches it. The read path itself must retire a contiguous corrupt run at
 * the watermark edge (already declared unrecoverable). */
static void test_rv01_all_corrupt_window_wedge(void)
{
    printf("-- RV-01 (#160): all-corrupt probe window must not wedge bulk transfer\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    /* 400 good records, then corrupt the first 300 (a run longer than one
     * 256-probe window). */
    for (uint32_t i = 0; i < 400; i++) {
        sensor_t s = make_sensors(0);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 6000U + i, 0, 0, 0), FLASH_LOG_OK);
    }
    for (uint32_t i = 0; i < 300; i++) {
        fake_w25q_corrupt(FLASH_LOG_DATA_START + i * FLASH_LOG_RECORD_SIZE, 4);
    }

    uint32_t watermark_before = hlog.tx_high_water;
    CHECK_EQ_I(watermark_before, 0);

    FlashLog_Record_t batch[BULK_V6_MAX_RECORDS];
    uint32_t got = 0, skipped = 0;

    /* Ten consecutive bulk cycles, no TX (nothing reads clean). The log must
     * make forward progress across the unrecoverable run WITHOUT needing a
     * successful transmission. R3-04: the leading corrupt run retires inline
     * (live range ascending from the high water). */
    int cycles_with_no_progress = 0;
    for (int cycle = 0; cycle < 10; cycle++) {
        got = 0; skipped = 0;
        uint32_t wm = hlog.tx_high_water;
        FlashLog_GetRecoveryRecords(&hlog, batch, BULK_V6_MAX_RECORDS, &got, &skipped);
        if (got == 0 && hlog.tx_high_water == wm) {
            cycles_with_no_progress++;
        }
    }
    printf("   after 10 no-TX cycles: high water=%lu (was %lu)\n",
           (unsigned long)hlog.tx_high_water, (unsigned long)watermark_before);
    CHECK_REGRESSION(cycles_with_no_progress == 0, "RV-01");
    CHECK_REGRESSION(hlog.tx_high_water > watermark_before, "RV-01-wm");

    /* The 100 good records behind the corrupt run must become reachable. */
    int reached_good = 0;
    for (int cycle = 0; cycle < 400 && !reached_good; cycle++) {
        got = 0; skipped = 0;
        FlashLog_GetRecoveryRecords(&hlog, batch, BULK_V6_MAX_RECORDS, &got, &skipped);
        if (got > 0) reached_good = 1;
    }
    CHECK_REGRESSION(reached_good == 1, "RV-01-reach");

    fake_w25q_free();
}

/* ========================================================================== */
/* R3-04 (#218) — DDR-0005 one-pass recovery: v4->v5 migration, newest-first
 * walker, send-time advance, no autonomous resend, pending-live lead. */
/* ========================================================================== */
static void test_r3_04_one_pass_recovery(void)
{
    printf("-- R3-04 (#218): one-pass newest-first recovery + v4 migration\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    for (uint32_t i = 0; i < 10; i++) {
        sensor_t s = make_sensors((uint16_t)i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 7000U + i, 0, 0, 0), FLASH_LOG_OK);
    }
    CHECK_EQ_I(FlashLog_SyncHeader(&hlog), FLASH_LOG_OK);

    /* Forge a v4 header: legacy FIFO watermark at 3 (seqs 0..2 "sent" under
     * the old protocol). */
    {
        uint32_t addr = (hlog.active_header == 0) ? 0U : (uint32_t)W25Q_SECTOR_SIZE;
        FlashLog_Header_t h4;
        CHECK_EQ_I(W25Q_Read(&hw, addr, (uint8_t *)&h4, sizeof(h4)), W25Q_OK);
        h4.version = FLASH_LOG_HEADER_VERSION_LEGACY_FIFO;
        h4.last_transmitted_seq = 3;
        h4.reserved[0] = 0;
        h4.crc32 = FlashLog_CRC32((const uint8_t *)&h4, sizeof(h4) - sizeof(uint32_t));
        fake_w25q_poke(addr, &h4, sizeof(h4));
    }

    /* Migration: both watermarks restart at the top; the whole retained
     * archive becomes walker-eligible exactly once (backend dedupes). */
    FlashLog_HandleTypeDef mig;
    CHECK_EQ_I(FlashLog_Init(&mig, &hw), FLASH_LOG_OK);
    CHECK_EQ_I(mig.record_count, 10);
    CHECK_REGRESSION(mig.tx_high_water == 10, "R3-04-migrate");
    CHECK_REGRESSION(mig.recovery_frontier == 10, "R3-04-migrate");
    CHECK(FlashLog_HasUnsentData(&mig));

    /* The walker reads NEWEST FIRST (BR-TX-008). */
    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    CHECK_EQ_I(FlashLog_GetRecoveryRecords(&mig, batch, 5, &got, &skipped), FLASH_LOG_OK);
    CHECK_EQ_I(got, 5);
    CHECK_REGRESSION(batch[0].sequence == 9, "R3-04-order");
    CHECK_EQ_I(batch[4].sequence, 5);

    /* Advance at SEND time: the frontier drops to the lowest sent; the next
     * read resumes OLDER - a sent record is never walked again (BR-TX-010). */
    for (uint32_t i = 0; i < got; i++) {
        CHECK_EQ_I(FlashLog_MarkRecoverySent(&mig, batch[i].sequence), FLASH_LOG_OK);
    }
    CHECK_EQ_I(mig.recovery_frontier, 5);
    got = 0; skipped = 0;
    CHECK_EQ_I(FlashLog_GetRecoveryRecords(&mig, batch, 5, &got, &skipped), FLASH_LOG_OK);
    CHECK_EQ_I(got, 5);
    CHECK_REGRESSION(batch[0].sequence == 4, "R3-04-onepass");
    CHECK_EQ_I(batch[4].sequence, 0);
    for (uint32_t i = 0; i < got; i++) {
        FlashLog_MarkRecoverySent(&mig, batch[i].sequence);
    }
    CHECK(!FlashLog_HasUnsentData(&mig));

    /* A NEW write is pending-live and leads the next batch (BR-TX-005). */
    {
        sensor_t s = make_sensors(77);
        CHECK_EQ_I(FlashLog_WriteRecord(&mig, &s, 7100U, 0, 0, 0), FLASH_LOG_OK);
    }
    got = 0; skipped = 0;
    CHECK_EQ_I(FlashLog_GetRecoveryRecords(&mig, batch, 5, &got, &skipped), FLASH_LOG_OK);
    CHECK_EQ_I(got, 1);
    CHECK_REGRESSION(batch[0].sequence == 10, "R3-04-live");
    CHECK_EQ_I(FlashLog_MarkRecoverySent(&mig, 10), FLASH_LOG_OK);
    CHECK(!FlashLog_HasUnsentData(&mig));

    /* Persistence: the watermarks survive a reboot (header synced by the
     * non-deferred marks above). */
    {
        FlashLog_HandleTypeDef reload;
        CHECK_EQ_I(FlashLog_Init(&reload, &hw), FLASH_LOG_OK);
        CHECK_EQ_I(reload.tx_high_water, 11);
        CHECK_EQ_I(reload.recovery_frontier, 0);
        CHECK(!FlashLog_HasUnsentData(&reload));
    }

    fake_w25q_free();
}

/* ========================================================================== */
/* R3-07 (#221) — both headers lost must NOT orphan a recoverable data ring   */
/* ========================================================================== */
static void test_r3_07_double_header_loss_recovers_ring(void)
{
    printf("-- R3-07 (#221): double header loss reconstructs from the data ring\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;

    /* Populate 10 records and sync a valid header pair. */
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);
    for (uint32_t i = 0; i < 10; i++) {
        sensor_t s = make_sensors((uint16_t)i);
        CHECK_EQ_I(FlashLog_WriteRecord(&hlog, &s, 8000U + i, 0, 0, 0), FLASH_LOG_OK);
    }
    CHECK_EQ_I(FlashLog_SyncHeader(&hlog), FLASH_LOG_OK);

    /* Kill BOTH headers (sector 0 and sector 1). */
    fake_w25q_corrupt(0, W25Q_SECTOR_SIZE);
    fake_w25q_corrupt(W25Q_SECTOR_SIZE, W25Q_SECTOR_SIZE);

    /* Init must reconstruct from the ring, not start fresh. */
    FlashLog_HandleTypeDef rec;
    CHECK_EQ_I(FlashLog_Init(&rec, &hw), FLASH_LOG_OK);
    CHECK_REGRESSION(rec.record_count == 10, "R3-07");
    CHECK_REGRESSION(FlashLog_GetAvailableRecords(&rec) == 10, "R3-07");

    /* The archive must be deliverable again (watermarks restart so it
     * re-drains once; the backend dedupes by sequence). */
    CHECK(FlashLog_HasUnsentData(&rec));
    FlashLog_Record_t batch[6];
    uint32_t got = 0, skipped = 0;
    CHECK_EQ_I(FlashLog_GetRecoveryRecords(&rec, batch, 6, &got, &skipped), FLASH_LOG_OK);
    CHECK_EQ_I(got, 6);

    /* New writes must continue ABOVE the recovered maximum sequence - never
     * reusing identity. */
    sensor_t s = make_sensors(555);
    CHECK_EQ_I(FlashLog_WriteRecord(&rec, &s, 9000U, 0, 0, 0), FLASH_LOG_OK);
    FlashLog_Record_t newest;
    CHECK_EQ_I(FlashLog_ReadRecord(&rec, &newest, 0), FLASH_LOG_OK);
    CHECK_REGRESSION(newest.sequence == 10, "R3-07-seq");

    fake_w25q_free();
}

/* R3-07 counterpart: a genuinely BLANK flash must still initialize fresh. */
static void test_r3_07_blank_flash_still_fresh(void)
{
    printf("-- R3-07 (#221): blank flash still initializes fresh\n");

    fake_w25q_init();   /* erased NOR: 0xFF everywhere, no headers, no records */
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);
    CHECK_EQ_I(hlog.record_count, 0);
    CHECK(!FlashLog_HasUnsentData(&hlog));

    fake_w25q_free();
}

/* ========================================================================== */
/* FR-12 (#292) - a failed ordinary header checkpoint must not stay lost     */
/* ========================================================================== */
/* WriteRecord checkpoints the header every HEADER_UPDATE_INTERVAL (10)
 * records. On checkpoint failure it returns the error but sets NO dirty
 * flag, so the sync is never retried until the next scheduled checkpoint.
 * Consecutive failures (worn header sector, sagging rail) keep the durable
 * header stale without bound; once more than 10 records are written past the
 * stale checkpoint, the bounded boot FrontierScan (10 probes) cannot reach
 * the true frontier - the reclaimed write pointer overwrites valid records.
 * Fix contract: a failed checkpoint sets header_dirty (retried at the top of
 * the next WriteRecord) and the record write itself still reports OK. */
static void test_fr12_failed_checkpoint_retried(void)
{
    printf("-- FR-12 (#292): failed header checkpoint sets dirty + is retried\n");

    fake_w25q_init();
    W25Q_HandleTypeDef hw;
    FlashLog_HandleTypeDef hlog;
    CHECK_EQ_I(FlashLog_Init(&hlog, &hw), FLASH_LOG_OK);

    /* Armed AFTER init (a fresh init writes the initial header). Fail the
     * next TWO header-sector writes: pre-fix those are the checkpoints at
     * records 10 and 20; post-fix they are the record-10 checkpoint and its
     * record-11 retry. Data-record writes are unaffected. */
    fake_w25q_fail_writes_below(2U * W25Q_SECTOR_SIZE, 2);

    FlashLog_StatusTypeDef st10 = FLASH_LOG_OK;
    for (uint32_t i = 0; i < 25; i++) {
        sensor_t s = make_sensors((uint16_t)i);
        FlashLog_StatusTypeDef st = FlashLog_WriteRecord(&hlog, &s, 9000U + i, 0, 0, 0);
        if (i == 9) {
            st10 = st;   /* the record-10 call whose checkpoint fails */
        } else if (i == 19) {
            /* record 20's checkpoint also fails pre-fix (2nd armed fault);
             * post-fix the faults were consumed by the dirty retries and
             * this call is clean. Not part of the fix contract. */
        } else {
            CHECK_EQ_I(st, FLASH_LOG_OK);
        }
    }

    /* The failed checkpoint must NOT make the (successfully written) record
     * report an error... */
    CHECK_REGRESSION(st10 == FLASH_LOG_OK, "FR-12-record-reports-ok");

    /* ...and the durable header must have recovered WITHOUT any explicit
     * FlashLog_SyncHeader: a reload sees all 25 records. Pre-fix the durable
     * header is stuck at the boot state and the bounded FrontierScan recovers
     * at most 10 - records 10..24 are invisible and their slots get reused. */
    FlashLog_HandleTypeDef re;
    CHECK_EQ_I(FlashLog_Init(&re, &hw), FLASH_LOG_OK);
    printf("   after failed checkpoints + reload: record_count=%lu (want 25)\n",
           (unsigned long)re.record_count);
    CHECK_REGRESSION(re.record_count == 25, "FR-12-reload-recovers-all");

    fake_w25q_free();
}

int main(void)
{
    printf("=== Stratosonde flight-readiness regression tests ===\n\n");

    test_flash_write_alignment();
    test_bulk_identity_with_skips();
    test_flashlog_basic_roundtrip();
    test_fifo_skip_contract();
    test_all_corrupt_does_not_wedge();
    test_header_pingpong_survives_rewrites();
    test_header_pingpong_survives_failed_writes();
    test_deferred_header_sync_batches_burst();
    test_frontier_scan_rejects_previous_lap();
    test_erase_ahead_slack();
test_sp19_high_water_never_behind_oldest();
    test_r2_02_watermark_overadvance_on_wrap();
    test_r2_03_sequence_identity_crosscheck();
    test_r2_14_ts_wrap_false_latch();
    test_rv01_all_corrupt_window_wedge();
    test_r3_04_one_pass_recovery();
    test_r3_07_double_header_loss_recovers_ring();
    test_r3_07_blank_flash_still_fresh();
    test_fr12_failed_checkpoint_retried();

    printf("\n%d checks, %d failures", g_checks, g_failures);
    if (g_expected_failures > 0) {
        printf(" (%d are documented FR-xx regressions)", g_expected_failures);
    }
    printf("\n");

    const char *expect_unfixed = getenv("EXPECT_UNFIXED");
    if (expect_unfixed != NULL && expect_unfixed[0] == '1') {
        /* Pre-fix mode: only unexpected failures are errors. */
        int unexpected = g_failures - g_expected_failures;
        if (unexpected < 0) {
            /* Fewer failures than documented: fixes are landing. Green,
             * and say how many regressions this tree has retired. */
            printf("BASELINE OK (%d documented regressions still open, %d FIXED)\n",
                   g_expected_failures + unexpected, -unexpected);
            return 0;
        }
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
