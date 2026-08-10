/**
  ******************************************************************************
  * @file    test_main.c
  * @brief   Host test harness for Stratosonde pure logic (R49 / issue #46)
  *
  * Compiles the real firmware sources with the host compiler:
  *   - Core/Src/payload_encode.c  (#included below to reach static helpers)
  *   - Core/Src/power_model.c     (linked separately)
  *
  * Assert-based: any failure prints and exits non-zero. Run via `make test`
  * in tests/host, or in CI (.github/workflows/ci.yml).
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Test-controlled implementations of stubbed dependencies */
#include "timer_if.h"
#include "stm32_systime.h"

static uint32_t g_fake_rtc_seconds = 0;
uint32_t TIMER_IF_GetTime(uint16_t *ms) { if (ms) *ms = 0; return g_fake_rtc_seconds; }

/* HAL stubs needing test-side definitions (#57 GNSS parser inclusion) */
uint16_t g_host_dma_cndtr = 0;
uint32_t HAL_GetTick(void) { return g_fake_rtc_seconds * 1000U; }

static uint32_t g_fake_epoch = 1754500000U;  /* 2025-08-06-ish UTC */
SysTime_t SysTimeGet(void) { SysTime_t t = { g_fake_epoch, 0 }; return t; }

/* The unit under test — #included so static helpers (CRC16/32, converters)
 * are reachable. Include paths put tests/host/stubs first. */
#include "../../Core/Src/payload_encode.c"

/* R31-R34 (#57): the GNSS parser is pure tokenizing + math — include it too.
 * HAL/UART surfaces are stubbed in tests/host/stubs. */
#include "../../Core/Src/atgm336h.c"

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

/* Marks a check KNOWN to fail on the unfixed tree (R2 findings, 2026-08-09).
 * EXPECT_UNFIXED=1 inverts the exit code so CI stays green pre-fix. */
#define CHECK_REGRESSION(cond, fr_id) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        g_expected_failures++; \
        printf("FAIL [%s] %s:%d: %s\n", fr_id, __FILE__, __LINE__, #cond); \
    } \
} while (0)

static void test_crc_vectors(void)
{
    /* CRC-16/MODBUS (poly 0xA001, init 0xFFFF) of "123456789" = 0x4B37 */
    CHECK_EQ_I(CalculateCRC16((const uint8_t *)"123456789", 9), 0x4B37);
    /* CRC-32/ISO-HDLC (poly 0xEDB88320, init/xorout 0xFFFFFFFF) = 0xCBF43926 */
    CHECK_EQ_I(CalculateCRC32((const uint8_t *)"123456789", 9), 0xCBF43926u);
}

static sensor_t make_nominal_sensors(void)
{
    sensor_t s;
    memset(&s, 0, sizeof(s));
    s.pressure = 1013.25f;
    s.temperature = 25.0f;
    s.humidity = 45.0f;
    s.latitude = (int32_t)(45.0 * 8388607.0 / 90.0);   /* 45 deg N */
    s.longitude = (int32_t)(-114.0 * 8388607.0 / 180.0); /* 114 deg W */
    s.altitudeGps = 700;
    /* altitudeBar deleted (D5/#35) */
    s.satellites = 9;
    s.gnss_fix_quality = 1;
    s.gnss_hdop = 1.1f;
    s.gnss_valid = true;
    s.battery_voltage = 5.0f;
    s.regulator_voltage = 3.3f;
    s.solar_voltage = 5.5f;
    return s;
}


static void test_compact_packet(void)
{
    CompactTelemetryPacket_t pkt;
    sensor_t s = make_nominal_sensors();

    CHECK_EQ_I(sizeof(CompactTelemetryPacket_t), 11);
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));

    CHECK_EQ_I(pkt.timestamp_min, 1234);
    CHECK_EQ_I((uint8_t)pkt.temperature_2deg, 76);   /* 25/2 + 64 */

    /* D2 (#33): packed pressure+humidity — 1013.25 hPa -> 1013 units, 45% -> 9 units */
    CHECK_EQ_I(pkt.press_hum & PRESS_HUM_PRESS_MASK, 1013);
    CHECK_EQ_I(pkt.press_hum >> PRESS_HUM_HUM_SHIFT, 9);
    /* stratospheric: 30 hPa must survive (v1 collapsed <950 hPa to 0) */
    s.pressure = 30.0f;
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
    CHECK_EQ_I(pkt.press_hum & PRESS_HUM_PRESS_MASK, 30);
    /* out-of-range -> sentinel 2047 */
    s.pressure = 3000.0f;
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
    CHECK_EQ_I(pkt.press_hum & PRESS_HUM_PRESS_MASK, PRESS_HUM_PRESS_INVALID);
    s.pressure = NAN;
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
    CHECK_EQ_I(pkt.press_hum & PRESS_HUM_PRESS_MASK, PRESS_HUM_PRESS_INVALID);
    s.pressure = 1013.25f;
    s.humidity = NAN;
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
    CHECK_EQ_I(pkt.press_hum >> PRESS_HUM_HUM_SHIFT, PRESS_HUM_HUM_INVALID);
    s.humidity = 45.0f;

    CHECK_EQ_I(pkt.battery_volt_50mv, 100);          /* 5.0V/0.05 */
    CHECK(pkt.latitude_100m > 16300 && pkt.latitude_100m < 16470);
    CHECK(pkt.longitude_100m < -20700 && pkt.longitude_100m > -20810);

    /* v2 status: no stale bits; GNSS-disciplined time bit set (fresh fix) */
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
    CHECK_EQ_I(pkt.status & (STATUS_GPS_STALE_MASK | STATUS_TEMP_STALE_MASK |
                             STATUS_HUM_STALE_MASK | STATUS_PRESS_STALE_MASK), 0);
    CHECK(pkt.status & STATUS_TIME_GNSS_MASK);
    /* stale GPS -> stale bit set, GNSS-time bit clear */
    s.gnss_stale = true;
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
    CHECK(pkt.status & STATUS_GPS_STALE_MASK);
    CHECK(!(pkt.status & STATUS_TIME_GNSS_MASK));
    s.gnss_stale = false;

    s.gnss_valid = false;
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
    CHECK_EQ_I(pkt.latitude_100m, 0);
    CHECK_EQ_I(pkt.longitude_100m, 0);
    CHECK(!(pkt.status & STATUS_TIME_GNSS_MASK));

    /* Timestamp fallback uses SysTime (R45), not boot-relative RTC */
    g_fake_rtc_seconds = 123;
    g_fake_epoch = 1754500000U;
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 0, 0, MODE_NORMAL));
    CHECK_EQ_I(pkt.timestamp_min, (uint16_t)((g_fake_epoch / 60) & 0xFFFF));

    /* Golden vector (D9/§13): dump exact LE bytes for backend cross-check.
     * Must precede the wrap test — the wrap flag is sticky. ts=13000 keeps the
     * minute counter monotone after the 12610 fallback above (wrap detection). */
    {
        sensor_t s3 = make_nominal_sensors();
        CHECK(EncodeCompactBinaryPacket(&pkt, &s3, 13000, 0, MODE_NORMAL));
        printf("GOLDEN heartbeat-v2:");
        for (size_t i = 0; i < sizeof(pkt); i++)
            printf(" %02X", ((const uint8_t *)&pkt)[i]);
        printf("\n");
    }

    /* D4 (#33): wrap detection — a decreasing 16-bit minute count sets the
     * sticky wrap flag */
    {
        sensor_t s2 = make_nominal_sensors();
        CHECK(EncodeCompactBinaryPacket(&pkt, &s2, 65000, 0, MODE_NORMAL));
        CHECK(!(pkt.status & STATUS_TS_WRAP_MASK));
        CHECK(EncodeCompactBinaryPacket(&pkt, &s2, 10, 0, MODE_NORMAL));
        CHECK(pkt.status & STATUS_TS_WRAP_MASK);   /* wrapped 65535 -> 10 */
    }
}

static void test_highres_record(void)
{
    HighResTelemetryRecord_t rec;
    sensor_t s = make_nominal_sensors();

    CHECK_EQ_I(sizeof(HighResTelemetryRecord_t), 32);
    CHECK(EncodeHighResTelemetryRecord(&rec, &s, 1754500123U, -12, MODE_CONSERVATIVE));
    CHECK_EQ_I(rec.timestamp, 1754500123u);
    CHECK_EQ_I(rec.temperature, 250);
    CHECK_EQ_I(rec.humidity, 450);
    CHECK_EQ_I(rec.pressure, 10132);
    CHECK_EQ_I(rec.battery_voltage, 5000);
    CHECK_EQ_I(rec.solar_voltage, 5500);
    CHECK_EQ_I(rec.voltage_slope, -12);
    CHECK_EQ_I(rec.satellites, 9);
    CHECK_EQ_I(rec.power_mode, MODE_CONSERVATIVE);
    CHECK_EQ_I(rec.crc16, CalculateCRC16((const uint8_t *)&rec, sizeof(rec) - 2));

    CHECK(EncodeHighResTelemetryRecord(&rec, &s, 0, 0, MODE_NORMAL));
    CHECK_EQ_I(rec.timestamp, g_fake_epoch);  /* R45 fallback */
}

static void test_gnss_parser(void)
{
    GNSS_HandleTypeDef g;
    memset(&g, 0, sizeof(g));

    /* R33 (#57): a sentence with NO checksum delimiter is rejected (was:
     * accepted-and-parsed). A correct one passes; a corrupt one fails. */
    CHECK(!GNSS_VerifyChecksum("$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"));
    CHECK(GNSS_VerifyChecksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"));
    CHECK(!GNSS_VerifyChecksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48"));

    /* R32 (#57): a REAL fix on the equator/prime meridian (0.0, 0.0) is valid
     * data, not absence. (Parse-level test: GNSS_ParseGGA does not verify the
     * checksum itself — the checksum cases above cover GNSS_VerifyChecksum.) */
    memset(&g, 0, sizeof(g));
    int rc = GNSS_ParseGGA(&g, "$GNGGA,120000,0000.0000,N,0000.0000,E,1,05,1.0,15.0,M,0.0,M,,*6E");
    CHECK_EQ_I(rc, 0);                       /* 0,0 fix parses (was: dropped) */
    CHECK(g.data.latitude == 0.0 && g.data.longitude == 0.0);
    CHECK_EQ_I(g.data.satellites, 5);
    CHECK(g.data.valid);

    /* Empty lat/lon fields (no fix) -> -1, coordinates untouched */
    memset(&g, 0, sizeof(g));
    g.data.latitude = 12.5; g.data.longitude = -45.25;
    rc = GNSS_ParseGGA(&g, "$GNGGA,120000,,,,,0,00,99.0,,M,,M,,*51");
    CHECK_EQ_I(rc, -1);
    CHECK(g.data.latitude == 12.5 && g.data.longitude == -45.25);

    /* R34 (#57): double conversion precision — 4807.0380 N -> 48.1173 exactly */
    {
        double d = GNSS_ConvertToDecimalDegrees(4807.0380);
        CHECK(d > 48.1172999 && d < 48.1173001);
    }
    /* Southern/western hemispheres */
    memset(&g, 0, sizeof(g));
    rc = GNSS_ParseGGA(&g, "$GNGGA,120000,4807.0380,S,01131.0000,W,1,08,0.9,545.4,M,46.9,M,,*4C");
    CHECK(g.data.latitude < -48.11729 && g.data.latitude > -48.11731);
    CHECK(g.data.longitude < -11.5166 && g.data.longitude > -11.5167);
}

static void test_bulk_v3(void)
{
    /* D3 (#33) + DDR-0005 (#34) + FR-07 (#87): variable-length bulk v5,
     * packet_type 0x05, per-record explicit sequence identity, explicit LE (D9) */
    sensor_t s = make_nominal_sensors();
    HighResTelemetryRecord_t recs[3];
    for (int i = 0; i < 3; i++) {
        CHECK(EncodeHighResTelemetryRecord(&recs[i], &s, 1754500123U + (uint32_t)i * 60,
                                           -12, MODE_NORMAL));
    }

    uint8_t buf[198];
    uint8_t packed = 0;
    uint16_t len = 0;
    const uint32_t seqs[3] = { 256, 257, 258 };

    /* Full budget: 3 records -> 2 + 108 + 4 = 114 bytes */
    CHECK(EncodeBulkPacketV5(buf, sizeof(buf), 198, recs, seqs, 3, &packed, &len));
    CHECK_EQ_I(packed, 3);
    CHECK_EQ_I(len, 114);
    CHECK_EQ_I(buf[0], BULK_PACKET_TYPE_V5_EXPLICIT);   /* 0x05 */
    CHECK_EQ_I(buf[1], 3);
    /* record 0 sequence u32 LE at bytes 2-5 (DDR-0005 explicit identity) */
    CHECK_EQ_I((uint32_t)buf[2] | ((uint32_t)buf[3] << 8) |
               ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 24), 256u);
    /* record 1 sequence at bytes 38-41 */
    CHECK_EQ_I((uint32_t)buf[38] | ((uint32_t)buf[39] << 8) |
               ((uint32_t)buf[40] << 16) | ((uint32_t)buf[41] << 24), 257u);
    /* Record 0 payload at buf[6]: timestamp LE */
    CHECK_EQ_I((uint32_t)buf[6] | ((uint32_t)buf[7] << 8) |
               ((uint32_t)buf[8] << 16) | ((uint32_t)buf[9] << 24), 1754500123u);
    /* temperature 0.1°C at record offset 14 -> buf[6+14]: 250 = 0x00FA LE */
    CHECK_EQ_I(buf[20] | (buf[21] << 8), 250);
    /* pressure 0.1hPa at record offset 18 -> buf[24]: 10132.5 -> 10133 or 10132 */
    {
        int p = buf[24] | (buf[25] << 8);
        CHECK(p == 10132 || p == 10133);
    }
    /* per-record crc16 at record offset 30 -> buf[36] over the 30 bytes from buf[6] */
    CHECK_EQ_I(buf[36] | (buf[37] << 8), CalculateCRC16(buf + 6, 30));
    /* packet crc32 LE over [0, len-4) */
    {
        uint32_t wire_crc = (uint32_t)buf[len-4] | ((uint32_t)buf[len-3] << 8) |
                            ((uint32_t)buf[len-2] << 16) | ((uint32_t)buf[len-1] << 24);
        CHECK_EQ_I((long)wire_crc, (long)CalculateCRC32(buf, len - 4));
    }

    /* Budget for exactly 2 records (78 B): packs 2, third stays pending */
    CHECK(EncodeBulkPacketV5(buf, sizeof(buf), 78, recs, seqs, 3, &packed, &len));
    CHECK_EQ_I(packed, 2);
    CHECK_EQ_I(len, 78);

    /* Budget for 1 record (42 B) */
    CHECK(EncodeBulkPacketV5(buf, sizeof(buf), 42, recs, seqs, 3, &packed, &len));
    CHECK_EQ_I(packed, 1);
    CHECK_EQ_I(len, 42);

    /* Budget too small for even one record (41 B): fails, packs nothing */
    CHECK(!EncodeBulkPacketV5(buf, sizeof(buf), 41, recs, seqs, 3, &packed, &len));
    CHECK_EQ_I(packed, 0);

    /* Buffer cap smaller than budget: buf_cap wins */
    CHECK(EncodeBulkPacketV5(buf, 78, 198, recs, seqs, 3, &packed, &len));
    CHECK_EQ_I(packed, 2);
    CHECK_EQ_I(len, 78);

    /* Golden vector (D9/§13): exact LE bytes, 2-record packet */
    CHECK(EncodeBulkPacketV5(buf, sizeof(buf), 78, recs, seqs, 3, &packed, &len));
    printf("GOLDEN bulk-v5 (2 records, seqs=256,257):");
    for (uint16_t i = 0; i < len; i++) printf(" %02X", buf[i]);
    printf("\n");
}


static void test_flashlog_conversion(void)
{
    FlashLog_Record_t fr;
    memset(&fr, 0, sizeof(fr));
    fr.timestamp = 1754500999u;
    fr.latitude = 111111;
    fr.longitude = -222222;
    fr.altitude_gps = 9000;
    fr.temperature = -45.5f;
    fr.humidity = 12.3f;
    fr.pressure = 250.0f;
    fr.battery_mv = 4800;
    fr.solar_mv = 5100;        /* D5/F-025 (#35): archived solar */
    fr.voltage_slope = -7;     /* D5 (#35): archived slope */
    fr.power_mode = MODE_REDUCED;  /* D5 (#35): archived mode */
    fr.satellites = 7;
    fr.gnss_hdop_x10 = 15;
    fr.gnss_valid = 1;

    HighResTelemetryRecord_t hr;
    CHECK(ConvertFlashLogToHighRes(&fr, &hr, -5, MODE_NORMAL));  /* params superseded by record values */
    CHECK_EQ_I(hr.timestamp, 1754500999u);
    CHECK_EQ_I(hr.latitude, 111111);
    CHECK_EQ_I(hr.longitude, -222222);
    CHECK_EQ_I(hr.altitude, 9000);
    CHECK_EQ_I(hr.temperature, -455);
    CHECK_EQ_I(hr.battery_voltage, 4800);
    CHECK_EQ_I(hr.solar_voltage, 5100);       /* from the record, not 0 (F-025) */
    CHECK_EQ_I(hr.voltage_slope, -7);         /* from the record, not the param */
    CHECK_EQ_I(hr.hdop, 15);
    CHECK_EQ_I(hr.power_mode, MODE_REDUCED);  /* from the record, not the param */

    /* D5: int32 flash altitude clamps to the u16 wire field */
    fr.altitude_gps = 40000;   /* float altitude beyond int16 */
    CHECK(ConvertFlashLogToHighRes(&fr, &hr, 0, MODE_NORMAL));
    CHECK_EQ_I(hr.altitude, 40000);
    fr.altitude_gps = -50;
    CHECK(ConvertFlashLogToHighRes(&fr, &hr, 0, MODE_NORMAL));
    CHECK_EQ_I(hr.altitude, 0);
}


/* R23 (#22): pre-commit guard for the PCAS command bodies. Includes the REAL
 * atgm336h.h (HAL stubbed) so an accidental edit of a body string fails here.
 * The XOR is recomputed independently in the test (the review's Appendix-A
 * script as a host test). */
#include "atgm336h.h"

static uint8_t guard_xor(const char *body)
{
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
    return cs;
}

static void test_nmea_checksum_guard(void)
{
    struct { const char *body; uint8_t cs; const char *full; } vectors[] = {
        { GNSS_CMD_BODY_NMEA_CONFIG,       0x03, "$PCAS03,1,0,0,0,1,1,0,0*03\r\n" },
        { GNSS_CMD_BODY_NMEA_CONFIG_DEBUG, 0x02, "$PCAS03,1,0,0,1,1,1,0,0*02\r\n" },
        { GNSS_CMD_BODY_CONSTELLATION,     0x1C, "$PCAS04,5*1C\r\n" },
        { GNSS_CMD_BODY_AIRBORNE_MODE,     0x18, "$PCAS11,5*18\r\n" },
        { GNSS_CMD_BODY_UPDATE_RATE,       0x2E, "$PCAS02,1000*2E\r\n" },
        { GNSS_CMD_BODY_SATELLITE_SYS,     0x1E, "$PCAS04,7*1E\r\n" },
        { GNSS_CMD_BODY_SAVE_CONFIG,       0x01, "$PCAS00*01\r\n" },
        { GNSS_CMD_BODY_STANDBY,           0x1E, "$PCAS12,0*1E\r\n" },
    };
    for (unsigned i = 0; i < sizeof(vectors)/sizeof(vectors[0]); i++) {
        CHECK_EQ_I(guard_xor(vectors[i].body), vectors[i].cs);
        char full[96];
        snprintf(full, sizeof(full), "$%s*%02X\r\n", vectors[i].body, guard_xor(vectors[i].body));
        CHECK(strcmp(full, vectors[i].full) == 0);
    }
}


/* R47 (#44): decide-half tests. Config_Get stubbed to NULL -> defaults path. */
#include "transmit_plan.h"
#include "config.h"
const SystemConfig_t *Config_Get(void) { return NULL; }

static void test_decide_transmit_plan(void)
{
    VoltageSlope_t vs;

    /* Healthy battery, warm, joined, flight -> full go, GPS on */
    memset(&vs, 0, sizeof(vs));
    TransmitPlan_t p = DecideTransmitPlan(&vs, 5200, 22.0f, false, 1000, true, false);
    CHECK_EQ_I(p.veto, VETO_NONE);
    CHECK(p.gps_enabled);
    CHECK_EQ_I(p.gps_timeout_ms, 60000);
    CHECK_EQ_I(p.battery_mv_normalized, 5224);  /* 22C: +24mV interpolation */

    /* Cold below lockout -> GPS off, veto recorded */
    memset(&vs, 0, sizeof(vs));
    p = DecideTransmitPlan(&vs, 5200, -60.0f, false, 1000, true, false);
    CHECK_EQ_I(p.veto, VETO_TEMP_LOCKOUT);
    CHECK(!p.gps_enabled);

    /* Stale temperature -> treated as cold, GPS off (fail-safe, F9/T2) */
    memset(&vs, 0, sizeof(vs));
    p = DecideTransmitPlan(&vs, 5200, 20.0f, true, 1000, true, false);
    CHECK_EQ_I(p.veto, VETO_TEMP_STALE);
    CHECK(!p.gps_enabled);

    /* Brownout floor: normalized floor defeated at -66C is R10's problem;
     * here raw 4200 at 25C -> SURVIVAL, GPS off, 1h interval */
    memset(&vs, 0, sizeof(vs));
    p = DecideTransmitPlan(&vs, 4200, 25.0f, false, 1000, true, false);
    CHECK_EQ_I(p.power_mode, MODE_SURVIVAL);
    CHECK(!p.gps_enabled);
    CHECK_EQ_I(p.tx_interval_ms, 3600000);

    /* FLIGHT + no session -> RF silence veto (DDR-0018); commissioning exempt */
    memset(&vs, 0, sizeof(vs));
    p = DecideTransmitPlan(&vs, 5200, 22.0f, false, 1000, false, false);
    CHECK_EQ_I(p.veto, VETO_RF_SILENCE);
    memset(&vs, 0, sizeof(vs));
    p = DecideTransmitPlan(&vs, 5200, 22.0f, false, 1000, false, true);
    CHECK_EQ_I(p.veto, VETO_NONE);
}


static void test_power_model(void)
{
    /* NormalizeBatteryVoltage */
    CHECK_EQ_I(NormalizeBatteryVoltage(5000, 25.0f), 5000);   /* no comp at 25C */
    CHECK_EQ_I(NormalizeBatteryVoltage(3330, -65.0f), 5500);  /* table point */
    CHECK_EQ_I(NormalizeBatteryVoltage(2800, -66.0f), 5500);  /* table point */
    CHECK_EQ_I(NormalizeBatteryVoltage(2800, -80.0f), 5500);  /* below table: max comp */
    CHECK_EQ_I(NormalizeBatteryVoltage(5000, -5.0f), 5275);   /* halfway 0C/-10C */

    /* CalculateVoltageSlope */
    VoltageSlope_t vs;
    memset(&vs, 0, sizeof(vs));
    CHECK_EQ_I(CalculateVoltageSlope(&vs, 5000, 1000), 0);   /* first sample */
    CHECK_EQ_I(CalculateVoltageSlope(&vs, 5010, 1300), 0);   /* dt<600: last slope */
    CHECK_EQ_I(CalculateVoltageSlope(&vs, 5036, 4600), 36);  /* +36mV over 3600s */
    CHECK_EQ_I(CalculateVoltageSlope(&vs, 5020, 1500), 36);  /* dt<600: repeat last */
    CHECK_EQ_I(CalculateVoltageSlope(&vs, 4900, 4600), -100); /* F-01 (#62): discharging must be negative (-100mV over 3600s from baseline 5000@1000) */

    /* Finding #6 (2026-08-10 review): the battery ADC has no plausibility
     * gate, so one failed conversion (returns 0 mV) can poison the slope
     * baseline. On recovery the true delta exceeds 5462 mV and the int16_t
     * cast wraps: +6400 mV over 600 s = +38400 mV/h comes back as -27136 —
     * a RECOVERING battery reads as catastrophic discharge. The slope must
     * saturate, never wrap. EXPECTED-FAIL-BEFORE-FIX. */
    {
        VoltageSlope_t vs3;
        memset(&vs3, 0, sizeof(vs3));
        CHECK_EQ_I(CalculateVoltageSlope(&vs3, 0, 1000), 0);   /* poisoned baseline latches */
        int16_t wrapped = CalculateVoltageSlope(&vs3, 6400, 1600);  /* dt=600s */
        CHECK_REGRESSION(wrapped >= 0, "FINDING-6");
        printf("   finding #6: slope after 0->6400mV over 600s = %d mV/h (want >= 0)\n",
               (int)wrapped);
    }

    /* PredictTimeToVoltage */
    CHECK_EQ_I(PredictTimeToVoltage(5000, -100, 4500), 5);
    CHECK_EQ_I(PredictTimeToVoltage(5000, 0, 4500), 0xFFFF);
    CHECK_EQ_I(PredictTimeToVoltage(5000, 100, 4500), 0xFFFF);
    CHECK_EQ_I(PredictTimeToVoltage(4500, -100, 4500), 0);

    /* SelectModeFromPredictions — floor MUST win over positive slope (BUG 1.5) */
    CHECK_EQ_I(SelectModeFromPredictions(50, 4200, 0xFFFF, 4200), MODE_SURVIVAL);
    CHECK_EQ_I(SelectModeFromPredictions(25, 5000, 0xFFFF, 5000), MODE_NORMAL);
    CHECK_EQ_I(SelectModeFromPredictions(5, 5000, 0xFFFF, 5000), MODE_CONSERVATIVE);
    CHECK_EQ_I(SelectModeFromPredictions(0, 5000, 0xFFFF, 5000), MODE_CONSERVATIVE);
    CHECK_EQ_I(SelectModeFromPredictions(-10, 5000, 0xFFFF, 5000), MODE_REDUCED);
    CHECK_EQ_I(SelectModeFromPredictions(-20, 5000, 10, 5000), MODE_RECOVERY);
    CHECK_EQ_I(SelectModeFromPredictions(-40, 5000, 3, 5000), MODE_SURVIVAL);

    /* R10 (#37): compensation must NOT defeat the floor. Raw 4200 mV at -66C
     * normalizes to 6900 mV — the old normalized-floor path would NOT pick
     * SURVIVAL. The floor reads raw. */
    CHECK_EQ_I(SelectModeFromPredictions(50, 6900, 0xFFFF, 4200), MODE_SURVIVAL);
    /* R10: stale temperature -> no normalization (decide-level regression) */
    {
        VoltageSlope_t vs2;
        memset(&vs2, 0, sizeof(vs2));
        TransmitPlan_t p2 = DecideTransmitPlan(&vs2, 4200, -66.0f, true, 1000, true, false);
        CHECK_EQ_I(p2.battery_mv_normalized, 4200);   /* raw, uncompensated */
        CHECK_EQ_I(p2.power_mode, MODE_SURVIVAL);
    }

    CHECK(strcmp(GetModeName(MODE_SURVIVAL), "SURVIVAL") == 0);
    CHECK(strcmp(GetModeName((OperatingMode_t)99), "UNKNOWN") == 0);
}


/* ========================================================================== */
/* R2 findings (2026-08-09 pre-flight review) — EXPECTED-FAIL-BEFORE-FIX      */
/* ========================================================================== */

static void test_r2_10_slope_temperature_contamination(void)
{
    /* R2-10 (#114): the slope is computed on the temperature-NORMALIZED
     * voltage. NormalizeBatteryVoltage ADDS compensation as temperature falls
     * (steeply below -55C: +430 mV at -55C, +2170 mV at -65C), so cooling at
     * constant charge state produces a RISING normalized voltage — the slope
     * reads cooling as charging. Sweep temperature at CONSTANT raw voltage:
     * the slope must stay flat. */
    VoltageSlope_t vs;
    memset(&vs, 0, sizeof(vs));
    TransmitPlan_t p1 = DecideTransmitPlan(&vs, 4500, -55.0f, false, 3600, true, false);
    TransmitPlan_t p2 = DecideTransmitPlan(&vs, 4500, -65.0f, false, 7200, true, false);
    (void)p1;
    /* Normalized voltage jumped ~4930 -> ~6670 mV purely from temperature. */
    CHECK_REGRESSION(abs(p2.voltage_slope_mv_per_hour) <= 20, "R2-10");
    /* And the mode must not flip to GPS-on NORMAL because it got COLDER. */
    CHECK_REGRESSION(p2.power_mode != MODE_NORMAL, "R2-10");
}

static void test_r2_11_no_history_default_uses_raw_voltage(void)
{
    /* R2-11 (#115): slope state is RAM-only — after every reset there is no
     * history and SelectModeFromPredictions falls through every branch to
     * MODE_CONSERVATIVE (10-min cadence, GPS ON) even with a marginal
     * battery. The no-history default must derive from RAW voltage, not
     * fall through. (Baseline persistence across resets is the backup-reg
     * half of the fix — bench-verified, DR12-15.) */
    VoltageSlope_t vs;
    memset(&vs, 0, sizeof(vs));
    TransmitPlan_t p = DecideTransmitPlan(&vs, 4400, 25.0f, false, 3600, true, false);
    CHECK_EQ_I(p.voltage_slope_mv_per_hour, 0);   /* guard: truly no history */
    CHECK_REGRESSION(p.power_mode != MODE_CONSERVATIVE, "R2-11");
    CHECK_REGRESSION(!p.gps_enabled, "R2-11");
}

static void test_r2_17_already_critical_never_reports_stable(void)
{
    /* R2-17 (#121): at/below the 4500 mV critical threshold while
     * discharging, both PredictTimeToVoltage calls return 0xFFFF, so
     * DecideTransmitPlan emits time_to_target_h == 0 — which the log path
     * (lora_app.c) and Cayenne ch 12 render as "Stable". 0 must be reserved
     * for genuinely stable; already-critical must be distinguishable
     * (any non-zero encoding is acceptable). */
    VoltageSlope_t vs;
    memset(&vs, 0, sizeof(vs));
    TransmitPlan_t p1 = DecideTransmitPlan(&vs, 4600, 25.0f, false, 3600, true, false);
    (void)p1;
    TransmitPlan_t p2 = DecideTransmitPlan(&vs, 4400, 25.0f, false, 7200, true, false);
    CHECK_EQ_I(p2.voltage_slope_mv_per_hour, -200);  /* guard: discharging */
    CHECK(p2.power_mode != MODE_NORMAL);             /* guard: not charging */
    CHECK_REGRESSION(p2.time_to_target_h != 0, "R2-17");
}

static void test_r2_30_rmc_valid_clears_on_void(void)
{
    /* R2-30 (#130): GNSS_ParseRMC sets data.valid on status 'A' but never
     * clears it on 'V' — a fix lost mid-window still reads as held. */
    GNSS_HandleTypeDef g;
    memset(&g, 0, sizeof(g));
    GNSS_ParseRMC(&g, "$GNRMC,120000,A,4807.038,N,01131.000,E,0.5,180.0,060825,,,A*00");
    CHECK(g.data.valid);   /* guard: 'A' sets it (passes today) */
    GNSS_ParseRMC(&g, "$GNRMC,120100,V,,,,,,,060825,,,N*00");
    CHECK_REGRESSION(!g.data.valid, "R2-30");   /* 'V' must clear it */
}

static void test_r2_19_dma_overrun_blind_spot(void)
{
    /* R2-19 (#123): dma_produced_total advances in 256-byte quanta, so the
     * (produced - consumed) > GNSS_DMA_BUFFER_SIZE check under-reports: the
     * counter can say exactly one buffer-full while up to 255 REAL bytes
     * past that were already produced (and lost) uncounted. A full buffer's
     * worth of unconsmed production must already surface as an overrun. */
    GNSS_HandleTypeDef g;
    memset(&g, 0, sizeof(g));
    g.is_powered = true;
    g_host_dma_cndtr = 0;            /* head = (512-0)%512 = 0 == tail: nothing consumable */
    g.dma_produced_total = 512;      /* two half-callbacks fired; parser never ran */
    GNSS_ProcessDMABuffer(&g);
    CHECK_REGRESSION(g.dma_overrun_count == 1, "R2-19");

    /* Guard (passes on every tree): a clear > SIZE overrun is detected. */
    memset(&g, 0, sizeof(g));
    g.is_powered = true;
    g_host_dma_cndtr = 0;
    g.dma_produced_total = 768;
    GNSS_ProcessDMABuffer(&g);
    CHECK_EQ_I(g.dma_overrun_count, 1);
}

int main(void)
{
    test_crc_vectors();
    test_compact_packet();
    test_highres_record();
    test_flashlog_conversion();
    test_bulk_v3();
    test_power_model();
    test_decide_transmit_plan();
    test_nmea_checksum_guard();
    test_gnss_parser();
    test_r2_10_slope_temperature_contamination();
    test_r2_11_no_history_default_uses_raw_voltage();
    test_r2_17_already_critical_never_reports_stable();
    test_r2_30_rmc_valid_clears_on_void();
    test_r2_19_dma_overrun_blind_spot();

    printf("\n%d checks, %d failures", g_checks, g_failures);
    if (g_expected_failures > 0) {
        printf(" (%d are documented R2 regressions)", g_expected_failures);
    }
    printf("\n");

    const char *expect_unfixed = getenv("EXPECT_UNFIXED");
    if (expect_unfixed != NULL && expect_unfixed[0] == '1') {
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
        printf("ALL HOST TESTS PASSED\n");
        return 0;
    }
    printf("HOST TESTS FAILED\n");
    return 1;
}
