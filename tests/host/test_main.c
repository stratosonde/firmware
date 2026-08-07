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

static uint32_t g_fake_epoch = 1754500000U;  /* 2025-08-06-ish UTC */
SysTime_t SysTimeGet(void) { SysTime_t t = { g_fake_epoch, 0 }; return t; }

/* The unit under test — #included so static helpers (CRC16/32, converters)
 * are reachable. Include paths put tests/host/stubs first. */
#include "../../Core/Src/payload_encode.c"

static int g_failures = 0;
static int g_checks = 0;

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
    s.altitudeBar = 7050;
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
    CHECK_EQ_I(pkt.pressure_10hPa, 6);               /* (1013.25-950)/10 */
    CHECK_EQ_I(pkt.humidity_5pct, 9);                /* 45/5 */
    CHECK_EQ_I(pkt.battery_volt_50mv, 100);          /* 5.0V/0.05 */
    CHECK(pkt.latitude_100m > 16300 && pkt.latitude_100m < 16470);
    CHECK(pkt.longitude_100m < -20700 && pkt.longitude_100m > -20810);
    CHECK_EQ_I(pkt.status & 0x3F, 0);                /* no stale flags */

    s.gnss_valid = false;
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
    CHECK_EQ_I(pkt.latitude_100m, 0);
    CHECK_EQ_I(pkt.longitude_100m, 0);

    /* Timestamp fallback uses SysTime (R45), not boot-relative RTC */
    g_fake_rtc_seconds = 123;
    g_fake_epoch = 1754500000U;
    CHECK(EncodeCompactBinaryPacket(&pkt, &s, 0, 0, MODE_NORMAL));
    CHECK_EQ_I(pkt.timestamp_min, (uint16_t)((g_fake_epoch / 60) & 0xFFFF));
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
    fr.satellites = 7;
    fr.gnss_hdop_x10 = 15;
    fr.gnss_valid = 1;

    HighResTelemetryRecord_t hr;
    CHECK(ConvertFlashLogToHighRes(&fr, &hr, -5, MODE_REDUCED));
    CHECK_EQ_I(hr.timestamp, 1754500999u);
    CHECK_EQ_I(hr.latitude, 111111);
    CHECK_EQ_I(hr.longitude, -222222);
    CHECK_EQ_I(hr.altitude, 9000);
    CHECK_EQ_I(hr.temperature, -455);
    CHECK_EQ_I(hr.battery_voltage, 4800);
    CHECK_EQ_I(hr.hdop, 15);
    CHECK_EQ_I(hr.power_mode, MODE_REDUCED);
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

    /* FLIGHT + no session -> RF silence veto (DDR-0006); commissioning exempt */
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

    /* PredictTimeToVoltage */
    CHECK_EQ_I(PredictTimeToVoltage(5000, -100, 4500), 5);
    CHECK_EQ_I(PredictTimeToVoltage(5000, 0, 4500), 0xFFFF);
    CHECK_EQ_I(PredictTimeToVoltage(5000, 100, 4500), 0xFFFF);
    CHECK_EQ_I(PredictTimeToVoltage(4500, -100, 4500), 0);

    /* SelectModeFromPredictions — floor MUST win over positive slope (BUG 1.5) */
    CHECK_EQ_I(SelectModeFromPredictions(50, 4200, 0xFFFF), MODE_SURVIVAL);
    CHECK_EQ_I(SelectModeFromPredictions(25, 5000, 0xFFFF), MODE_NORMAL);
    CHECK_EQ_I(SelectModeFromPredictions(5, 5000, 0xFFFF), MODE_CONSERVATIVE);
    CHECK_EQ_I(SelectModeFromPredictions(0, 5000, 0xFFFF), MODE_CONSERVATIVE);
    CHECK_EQ_I(SelectModeFromPredictions(-10, 5000, 0xFFFF), MODE_REDUCED);
    CHECK_EQ_I(SelectModeFromPredictions(-20, 5000, 10), MODE_RECOVERY);
    CHECK_EQ_I(SelectModeFromPredictions(-40, 5000, 3), MODE_SURVIVAL);

    CHECK(strcmp(GetModeName(MODE_SURVIVAL), "SURVIVAL") == 0);
    CHECK(strcmp(GetModeName((OperatingMode_t)99), "UNKNOWN") == 0);
}

int main(void)
{
    test_crc_vectors();
    test_compact_packet();
    test_highres_record();
    test_flashlog_conversion();
    test_power_model();
    test_decide_transmit_plan();
    test_nmea_checksum_guard();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) {
        printf("ALL HOST TESTS PASSED\n");
        return 0;
    }
    printf("HOST TESTS FAILED\n");
    return 1;
}
