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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Test-controlled implementations of stubbed dependencies */
#include "stm32_systime.h"
#include "timer_if.h"

static uint32_t g_fake_rtc_seconds = 0;
uint32_t TIMER_IF_GetTime(uint16_t *ms) {
  if (ms)
    *ms = 0;
  return g_fake_rtc_seconds;
}

/* HAL stubs needing test-side definitions (#57 GNSS parser inclusion) */
uint16_t g_host_dma_cndtr = 0;
uint32_t HAL_GetTick(void) { return g_fake_rtc_seconds * 1000U; }

static uint32_t g_fake_epoch = 1754500000U; /* 2025-08-06-ish UTC */
SysTime_t SysTimeGet(void) {
  SysTime_t t = {g_fake_epoch, 0};
  return t;
}

/* The unit under test — #included so static helpers (CRC16/32, converters)
 * are reachable. Include paths put tests/host/stubs first. */
#include "../../Core/Src/payload_encode.c"

/* R31-R34 (#57): the GNSS parser is pure tokenizing + math — include it too.
 * HAL/UART surfaces are stubbed in tests/host/stubs. */
#include "../../Core/Src/atgm336h.c"

static int g_failures = 0;
static int g_checks = 0;
static int g_expected_failures = 0;

#define CHECK(cond)                                          \
  do {                                                       \
    g_checks++;                                              \
    if (!(cond)) {                                           \
      g_failures++;                                          \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                        \
  } while (0)

#define CHECK_EQ_I(actual, expected)                                                        \
  do {                                                                                      \
    g_checks++;                                                                             \
    long _a = (long)(actual), _e = (long)(expected);                                        \
    if (_a != _e) {                                                                         \
      g_failures++;                                                                         \
      printf("FAIL %s:%d: %s == %ld, expected %ld\n", __FILE__, __LINE__, #actual, _a, _e); \
    }                                                                                       \
  } while (0)

/* Marks a check KNOWN to fail on the unfixed tree (R2 findings, 2026-08-09).
 * EXPECT_UNFIXED=1 inverts the exit code so CI stays green pre-fix. */
#define CHECK_REGRESSION(cond, fr_id)                                    \
  do {                                                                   \
    g_checks++;                                                          \
    if (!(cond)) {                                                       \
      g_failures++;                                                      \
      g_expected_failures++;                                             \
      printf("FAIL [%s] %s:%d: %s\n", fr_id, __FILE__, __LINE__, #cond); \
    }                                                                    \
  } while (0)

static void test_crc_vectors(void) {
  /* CRC-16/MODBUS (poly 0xA001, init 0xFFFF) of "123456789" = 0x4B37 */
  CHECK_EQ_I(CalculateCRC16((const uint8_t *)"123456789", 9), 0x4B37);
  /* CRC-32/ISO-HDLC (poly 0xEDB88320, init/xorout 0xFFFFFFFF) = 0xCBF43926 */
  CHECK_EQ_I(CalculateCRC32((const uint8_t *)"123456789", 9), 0xCBF43926u);
}

static sensor_t make_nominal_sensors(void) {
  sensor_t s;
  memset(&s, 0, sizeof(s));
  s.pressure = 1013.25f;
  s.temperature = 25.0f;
  s.humidity = 45.0f;
  s.latitude = (int32_t)(45.0 * 8388607.0 / 90.0);     /* 45 deg N */
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

static void test_compact_packet(void) {
  CompactTelemetryPacket_t pkt;
  sensor_t s = make_nominal_sensors();

  CHECK_EQ_I(sizeof(CompactTelemetryPacket_t), 11);
  CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));

  CHECK_EQ_I(pkt.timestamp_min, 1234);
  CHECK_EQ_I((uint8_t)pkt.temperature_2deg, 76); /* 25/2 + 64 */

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

  /* SP-14 (#251): compact temperature/battery converters must not cast
   * NaN/out-of-range floats to int (UB). No wire sentinel exists for these
   * fields - clamp to range bottom; the status byte carries honesty. */
  CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
  s.temperature = NAN;
  EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL);
  CHECK_REGRESSION((uint8_t)pkt.temperature_2deg == 0, "SP-14-temp-nan");
  s.temperature = -500.0f;
  EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL);
  CHECK_REGRESSION((uint8_t)pkt.temperature_2deg == 0, "SP-14-temp-neg");
  s.temperature = 25.0f;
  s.battery_voltage = NAN;
  EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL);
  CHECK_REGRESSION(pkt.battery_volt_50mv == 0, "SP-14-batt-nan");
  s.battery_voltage = -1.5f;
  EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL);
  CHECK_REGRESSION(pkt.battery_volt_50mv == 0, "SP-14-batt-neg");
  s.battery_voltage = 5.0f;
  EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL); /* re-encode after restore */

  CHECK_EQ_I(pkt.battery_volt_50mv, 100); /* 5.0V/0.05 */
  CHECK(pkt.latitude_100m > 16300 && pkt.latitude_100m < 16470);
  CHECK(pkt.longitude_100m < -20700 && pkt.longitude_100m > -20810);

  /* v2 status: no stale bits; GNSS-disciplined time bit set (fresh fix) */
  CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
  CHECK_EQ_I(pkt.status & (STATUS_GPS_STALE_MASK | STATUS_TEMP_STALE_MASK |
                           STATUS_HUM_STALE_MASK | STATUS_PRESS_STALE_MASK),
             0);
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

  /* SP-11 (#250): 0 is a LEGAL wire value - the 0->fetch-now sentinel is
   * gone. (s is undisciplined here - gnss_valid false - so this never
   * touches the wrap baseline.) */
  g_fake_rtc_seconds = 123;
  g_fake_epoch = 1754500000U;
  CHECK_REGRESSION(EncodeCompactBinaryPacket(&pkt, &s, 0, 0, MODE_NORMAL) &&
                       pkt.timestamp_min == 0,
                   "SP-11");
  /* SP-12 (#250): the wire basis is UTC epoch via ONE public accessor -
   * callers must never stamp TIMER_IF (undisciplined RTC uptime) seconds. */
  CHECK_REGRESSION(Payload_TimestampMinutesNow() ==
                       (uint16_t)((g_fake_epoch / 60) & 0xFFFF),
                   "SP-12");

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
    CHECK(pkt.status & STATUS_TS_WRAP_MASK); /* wrapped 65535 -> 10 */
  }
}

static void test_highres_record(void) {
  HighResTelemetryRecord_t rec;
  sensor_t s = make_nominal_sensors();

  CHECK_EQ_I(sizeof(HighResTelemetryRecord_t), 34); /* STAB-04 (#151): v6 */
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
  CHECK_EQ_I(rec.timestamp, g_fake_epoch); /* R45 fallback */
}

static void test_gnss_parser(void) {
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
  CHECK_EQ_I(rc, 0); /* 0,0 fix parses (was: dropped) */
  CHECK(g.data.latitude == 0.0 && g.data.longitude == 0.0);
  CHECK_EQ_I(g.data.satellites, 5);
  CHECK(g.data.valid);

  /* Empty lat/lon fields (no fix) -> -1, coordinates untouched */
  memset(&g, 0, sizeof(g));
  g.data.latitude = 12.5;
  g.data.longitude = -45.25;
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

  /* R2-16 (#120): Null-Island poison. A PARTIAL GGA carries fix_quality/sats/
   * hdop but leaves lat/lon empty; a following RMC 'A' then sets data.valid.
   * The combination passed GNSS_IsFixValid with (0,0) coordinates — and
   * AcquireGnssFix stored (0,0) into the backup regs as last-known-good,
   * persisting across resets. GNSS_HasPosition() requires the parser's
   * field-presence flag, which only a GGA with lat/lon tokens sets. */
  memset(&g, 0, sizeof(g));
  rc = GNSS_ParseGGA(&g, "$GNGGA,120000,,,,,1,06,1.0,,M,,M,,*51");
  CHECK_EQ_I(rc, -1); /* no lat/lon -> parse fails */
  /* DR-01 (#236): a rejected sentence commits NOTHING - the fix fields stay
   * at their pre-parse values (the old commit-before-validate behaviour
   * this asserted was the bug). */
  CHECK_EQ_I(g.data.fix_quality, 0);
  CHECK_EQ_I(g.data.satellites, 0);
  GNSS_ParseRMC(&g, "$GNRMC,120000,A,,,,,,,060825,,,N*00");
  CHECK(g.data.valid);                              /* RMC 'A' latches valid */
  CHECK_REGRESSION(!GNSS_HasPosition(&g), "R2-16"); /* but no position fields */

  /* Guard the fix direction: a REAL (0,0) Gulf of Guinea fix (fields
   * present) must still count as a position (R32 semantics preserved). */
  memset(&g, 0, sizeof(g));
  rc = GNSS_ParseGGA(&g, "$GNGGA,120000,0000.0000,N,0000.0000,E,1,05,1.0,15.0,M,0.0,M,,*6E");
  CHECK_EQ_I(rc, 0);
  GNSS_ParseRMC(&g, "$GNRMC,120000,A,,,,,,,060825,,,N*00");
  CHECK(GNSS_HasPosition(&g));

  /* FR-08 (#291): semantic gate BEFORE the enum/int cast. Negative
   * satellites used to wrap to 255 via uint8_t; negative finite HDOP
   * passes hdop<=5.0; any nonzero fix_quality was accepted. */
  memset(&g, 0, sizeof(g));
  g.data.latitude = 12.5;
  g.data.longitude = -45.25;
  rc = GNSS_ParseGGA(&g, "$GNGGA,120000,4807.0380,N,01131.0000,E,1,-1,1.0,15.0,M,0.0,M,,*32");
  CHECK_EQ_I(rc, -1);
  CHECK(g.data.latitude == 12.5 && g.data.longitude == -45.25);

  memset(&g, 0, sizeof(g));
  rc = GNSS_ParseGGA(&g, "$GNGGA,120000,4807.0380,N,01131.0000,E,1,08,-1.0,15.0,M,0.0,M,,*24");
  CHECK_EQ_I(rc, -1);

  memset(&g, 0, sizeof(g));
  rc = GNSS_ParseGGA(&g, "$GNGGA,120000,4807.0380,N,01131.0000,E,9,08,1.0,15.0,M,0.0,M,,*28");
  CHECK_EQ_I(rc, -1);
}

static void test_bulk_v3(void) {
  /* D3 (#33) + DDR-0005 (#34) + FR-07 (#87) + STAB-04 (#151): variable-length
   * bulk v6, packet_type 0x06, per-record explicit sequence identity,
   * explicit LE (D9), 34-byte records with sensor_quality + veto_reason. */
  sensor_t s = make_nominal_sensors();
  HighResTelemetryRecord_t recs[3];
  for (int i = 0; i < 3; i++) {
    CHECK(EncodeHighResTelemetryRecord(&recs[i], &s, 1754500123U + (uint32_t)i * 60,
                                       -12, MODE_NORMAL));
  }
  /* STAB-04 (#151): give record 1 historical provenance (stale pressure,
   * veto reason 2) — it must appear ON THE WIRE at the new record offsets. */
  recs[1].sensor_quality = 0x01;
  recs[1].veto_reason = 2;
  recs[1].crc16 = CalculateCRC16((const uint8_t *)&recs[1], sizeof(recs[1]) - 2);

  uint8_t buf[198];
  uint8_t packed = 0;
  uint16_t len = 0;
  const uint32_t seqs[3] = {256, 257, 258};

  /* Full budget: 3 records -> 2 + 3*38 + 4 = 120 bytes */
  CHECK(EncodeBulkPacketV6(buf, sizeof(buf), 198, recs, seqs, 3, &packed, &len));
  CHECK_EQ_I(packed, 3);
  CHECK_EQ_I(len, 120);
  CHECK_EQ_I(buf[0], BULK_PACKET_TYPE_V6_PROVENANCE); /* 0x06 */
  CHECK_EQ_I(buf[1], 3);
  /* record 0 sequence u32 LE at bytes 2-5 (DDR-0005 explicit identity) */
  CHECK_EQ_I((uint32_t)buf[2] | ((uint32_t)buf[3] << 8) |
                 ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 24),
             256u);
  /* record 1 sequence at bytes 40-43 (2 + 38) */
  CHECK_EQ_I((uint32_t)buf[40] | ((uint32_t)buf[41] << 8) |
                 ((uint32_t)buf[42] << 16) | ((uint32_t)buf[43] << 24),
             257u);
  /* Record 0 payload at buf[6]: timestamp LE */
  CHECK_EQ_I((uint32_t)buf[6] | ((uint32_t)buf[7] << 8) |
                 ((uint32_t)buf[8] << 16) | ((uint32_t)buf[9] << 24),
             1754500123u);
  /* temperature 0.1°C at record offset 14 -> buf[6+14]: 250 = 0x00FA LE */
  CHECK_EQ_I(buf[20] | (buf[21] << 8), 250);
  /* pressure 0.1hPa at record offset 18 -> buf[24]: 10132.5 -> 10133 or 10132 */
  {
    int p = buf[24] | (buf[25] << 8);
    CHECK(p == 10132 || p == 10133);
  }
  /* STAB-04: record 1 quality/veto at record offsets 30/31 -> buf[40+4+30..] */
  CHECK_REGRESSION(buf[74] == 0x01, "STAB-04"); /* sensor_quality */
  CHECK_REGRESSION(buf[75] == 0x02, "STAB-04"); /* veto_reason */
  /* per-record crc16 at record offset 32 -> buf[6+32] over 32 bytes from buf[6] */
  CHECK_EQ_I(buf[38] | (buf[39] << 8), CalculateCRC16(buf + 6, 32));
  /* packet crc32 LE over [0, len-4) */
  {
    uint32_t wire_crc = (uint32_t)buf[len - 4] | ((uint32_t)buf[len - 3] << 8) |
                        ((uint32_t)buf[len - 2] << 16) | ((uint32_t)buf[len - 1] << 24);
    CHECK_EQ_I((long)wire_crc, (long)CalculateCRC32(buf, (uint32_t)(len - 4)));
  }

  /* Budget for exactly 2 records (82 B): packs 2, third stays pending */
  CHECK(EncodeBulkPacketV6(buf, sizeof(buf), 82, recs, seqs, 3, &packed, &len));
  CHECK_EQ_I(packed, 2);
  CHECK_EQ_I(len, 82);

  /* Budget for 1 record (44 B) */
  CHECK(EncodeBulkPacketV6(buf, sizeof(buf), 44, recs, seqs, 3, &packed, &len));
  CHECK_EQ_I(packed, 1);
  CHECK_EQ_I(len, 44);

  /* Budget too small for even one record (43 B): fails, packs nothing */
  CHECK(!EncodeBulkPacketV6(buf, sizeof(buf), 43, recs, seqs, 3, &packed, &len));
  CHECK_EQ_I(packed, 0);

  /* STAB-P3#8 (#243): budget BELOW the overhead - the unsigned subtraction
   * underflowed and "packed" records past the buffer. Must fail cleanly
   * with zeroed outputs. */
  packed = 99;
  len = 999;
  CHECK_REGRESSION(!EncodeBulkPacketV6(buf, sizeof(buf), 4, recs, seqs, 3, &packed, &len), "STAB-P3-8");
  CHECK_REGRESSION(packed == 0 && len == 0, "STAB-P3-8-zeroed");
  packed = 99;
  len = 999;
  CHECK_REGRESSION(!EncodeBulkPacketV6(buf, 2, 198, recs, seqs, 3, &packed, &len), "STAB-P3-8-bufcap");

  /* Buffer cap smaller than budget: buf_cap wins */
  CHECK(EncodeBulkPacketV6(buf, 82, 198, recs, seqs, 3, &packed, &len));
  CHECK_EQ_I(packed, 2);
  CHECK_EQ_I(len, 82);

  /* Golden vector (D9/§13): exact LE bytes, 2-record packet */
  CHECK(EncodeBulkPacketV6(buf, sizeof(buf), 82, recs, seqs, 3, &packed, &len));
  printf("GOLDEN bulk-v6 (2 records, seqs=256,257):");
  for (uint16_t i = 0; i < len; i++)
    printf(" %02X", buf[i]);
  printf("\n");
}

/* COMM-TX (2026-08-18, DDR-0002 §7 / BR-LIFE-004): the commissioning live
 * record withholds the horizontal position while keeping every other field
 * honest, and re-seals the CRC over the zeroed record. */
static void test_commissioning_live_record(void) {
  printf("-- COMM-TX: commissioning live record withholds X/Y, CRC re-sealed\n");
  sensor_t s = make_nominal_sensors();
  HighResTelemetryRecord_t full, live;
  CHECK(EncodeHighResTelemetryRecord(&full, &s, 1754500000U, 0, MODE_NORMAL));
  CHECK(EncodeCommissioningLiveRecord(&live, &s, 1754500000U, 0, MODE_NORMAL));
  CHECK(full.latitude != 0 && full.longitude != 0); /* guard: input HAS a position */
  CHECK_EQ_I(live.latitude, 0);
  CHECK_EQ_I(live.longitude, 0);
  /* honesty: fix validity/sats, environment, and timestamp survive */
  CHECK_EQ_I(live.flags, full.flags);
  CHECK_EQ_I(live.sensor_quality, full.sensor_quality);
  CHECK_EQ_I(live.temperature, full.temperature);
  CHECK_EQ_I(live.pressure, full.pressure);
  CHECK_EQ_I(live.timestamp, full.timestamp);
  CHECK_EQ_I(live.satellites, full.satellites);
  /* the CRC covers the zeroed record, not the un-zeroed one */
  CHECK(live.crc16 != full.crc16);
  CHECK_EQ_I(live.crc16, CalculateCRC16((const uint8_t *)&live, sizeof(live) - 2));
}
static void test_flashlog_conversion(void) {
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
  fr.solar_mv = 5100;           /* D5/F-025 (#35): archived solar */
  fr.voltage_slope = -7;        /* D5 (#35): archived slope */
  fr.power_mode = MODE_REDUCED; /* D5 (#35): archived mode */
  fr.satellites = 7;
  fr.gnss_hdop_x10 = 15;
  fr.gnss_valid = 1;

  HighResTelemetryRecord_t hr;
  /* STAB-05 (#152): the voltage_slope/power_mode params are gone — every
   * historical field must come from the record itself. */
  CHECK(ConvertFlashLogToHighRes(&fr, &hr));
  CHECK_EQ_I(hr.timestamp, 1754500999u);
  CHECK_EQ_I(hr.latitude, 111111);
  CHECK_EQ_I(hr.longitude, -222222);
  CHECK_EQ_I(hr.altitude, 9000);
  CHECK_EQ_I(hr.temperature, -455);
  CHECK_EQ_I(hr.battery_voltage, 4800);
  CHECK_EQ_I(hr.solar_voltage, 5100); /* from the record, not 0 (F-025) */
  CHECK_EQ_I(hr.voltage_slope, -7);   /* from the record */
  CHECK_EQ_I(hr.hdop, 15);
  CHECK_EQ_I(hr.power_mode, MODE_REDUCED); /* from the record */
  /* STAB-05 (#152): the flags byte's mode field is the HISTORICAL mode too
   * (was: live retransmission-time mode packed into a historical record) */
  CHECK_REGRESSION(((hr.flags >> 5) & 0x07) == MODE_REDUCED, "STAB-05");

  /* STAB-04 (#151): the flash record's data-honesty provenance survives
   * conversion. b0 press_stale + veto reason 3 (b5-b7) in flash flags. */
  fr.flags = 0x01 | (3u << 5);
  CHECK(ConvertFlashLogToHighRes(&fr, &hr));
  CHECK_REGRESSION(hr.sensor_quality == 0x01, "STAB-04"); /* press_stale */
  CHECK_REGRESSION(hr.veto_reason == 3, "STAB-04");
  /* ...and the stale bits do NOT leak into the legacy flags byte */
  CHECK((hr.flags & 0x1F) == (1u | (7u << 1))); /* gps_valid + 7 sats only */
  /* all five stale bits round-trip */
  fr.flags = 0x1F | (7u << 5);
  CHECK(ConvertFlashLogToHighRes(&fr, &hr));
  CHECK_REGRESSION(hr.sensor_quality == 0x1F, "STAB-04");
  CHECK_REGRESSION(hr.veto_reason == 7, "STAB-04");
  /* fresh record -> zero provenance */
  fr.flags = 0;
  CHECK(ConvertFlashLogToHighRes(&fr, &hr));
  CHECK(hr.sensor_quality == 0 && hr.veto_reason == 0);

  /* D5: int32 flash altitude clamps to the u16 wire field */
  fr.altitude_gps = 40000; /* float altitude beyond int16 */
  CHECK(ConvertFlashLogToHighRes(&fr, &hr));
  CHECK_EQ_I(hr.altitude, 40000);
  fr.altitude_gps = -50;
  CHECK(ConvertFlashLogToHighRes(&fr, &hr));
  CHECK_EQ_I(hr.altitude, 0);
}

/* R23 (#22): pre-commit guard for the PCAS command bodies. Includes the REAL
 * atgm336h.h (HAL stubbed) so an accidental edit of a body string fails here.
 * The XOR is recomputed independently in the test (the review's Appendix-A
 * script as a host test). */
#include "atgm336h.h"

static uint8_t guard_xor(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; p++)
    cs ^= (uint8_t)*p;
  return cs;
}

static void test_nmea_checksum_guard(void) {
  struct {
    const char *body;
    uint8_t cs;
    const char *full;
  } vectors[] = {
      {GNSS_CMD_BODY_NMEA_CONFIG, 0x03, "$PCAS03,1,0,0,0,1,1,0,0*03\r\n"},
      {GNSS_CMD_BODY_NMEA_CONFIG_DEBUG, 0x02, "$PCAS03,1,0,0,1,1,1,0,0*02\r\n"},
      {GNSS_CMD_BODY_CONSTELLATION, 0x1C, "$PCAS04,5*1C\r\n"},
      {GNSS_CMD_BODY_AIRBORNE_MODE, 0x18, "$PCAS11,5*18\r\n"},
      {GNSS_CMD_BODY_UPDATE_RATE, 0x2E, "$PCAS02,1000*2E\r\n"},
      {GNSS_CMD_BODY_SATELLITE_SYS, 0x1E, "$PCAS04,7*1E\r\n"},
      {GNSS_CMD_BODY_SAVE_CONFIG, 0x01, "$PCAS00*01\r\n"},
      {GNSS_CMD_BODY_STANDBY, 0x1E, "$PCAS12,0*1E\r\n"},
  };
  for (unsigned i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++) {
    CHECK_EQ_I(guard_xor(vectors[i].body), vectors[i].cs);
    char full[96];
    snprintf(full, sizeof(full), "$%s*%02X\r\n", vectors[i].body, guard_xor(vectors[i].body));
    CHECK(strcmp(full, vectors[i].full) == 0);
  }
}

/* R47 (#44): decide-half tests. Config_Get stubbed to NULL -> defaults path. */
#include "config.h"
#include "transmit_plan.h"
const SystemConfig_t *Config_Get(void) { return NULL; }

/* STAB-06/07 (#153/#154): pure launch/float detectors (compiled via Makefile) */
#include "mission_logic.h"

static void test_decide_transmit_plan(void) {
  VoltageSlope_t vs;

  /* Healthy battery, warm, joined, flight -> full go, GPS on */
  memset(&vs, 0, sizeof(vs));
  TransmitPlan_t p = DecideTransmitPlan(&vs, 5200, 22.0f, false, 1000, true, false, false);
  CHECK_EQ_I(p.veto, VETO_NONE);
  CHECK(p.gps_enabled);
  CHECK_EQ_I(p.gps_timeout_ms, 60000);
  CHECK_EQ_I(p.battery_mv_normalized, 5224); /* 22C: +24mV interpolation */

  /* RV-08 (#164, DDR-0021 conformance): temperature no longer gates GPS.
   * -60C is normal float ambient for this airframe; the cold lockout x
   * GPS-loss silence composed into a 6 h dark sawtooth for the whole float.
   * (Historical window; the silence budget is 24 h since 2026-08-13 per
   *  DDR-0015 BR-STALE-017. The lockout removal is unaffected.) */
  memset(&vs, 0, sizeof(vs));
  p = DecideTransmitPlan(&vs, 5200, -60.0f, false, 1000, true, false, false);
  CHECK_REGRESSION(p.veto == VETO_NONE, "RV-08");
  CHECK_REGRESSION(p.gps_enabled, "RV-08");

  /* Stale temperature still skips normalization (R2-10) but does not veto */
  memset(&vs, 0, sizeof(vs));
  p = DecideTransmitPlan(&vs, 5200, 20.0f, true, 1000, true, false, false);
  CHECK_REGRESSION(p.veto == VETO_NONE, "RV-08-stale");
  CHECK_EQ_I(p.battery_mv_normalized, 5200); /* raw, uncompensated */

  /* RV-02 (#161): a rejected (0 mV) battery read must not become the slope
   * baseline - the next real reading must not report runaway charging. */
  {
    VoltageSlope_t vs3;
    memset(&vs3, 0, sizeof(vs3));
    TransmitPlan_t a = DecideTransmitPlan(&vs3, 0, 25.0f, false, 1000, true, false, true);
    CHECK_EQ_I(a.power_mode, MODE_SURVIVAL); /* raw floor still fires (R10) */
    TransmitPlan_t b = DecideTransmitPlan(&vs3, 5200, 25.0f, false, 1700, true, false, false);
    CHECK_REGRESSION(abs(b.voltage_slope_mv_per_hour) < 3000, "RV-02");
    CHECK_REGRESSION(b.power_mode != MODE_NORMAL, "RV-02");
  }

  /* RV-03 (#161): 6 h of frozen (cached) battery reads must not read as a
   * healthy stable battery - stale measurement caps the mode at REDUCED. */
  {
    VoltageSlope_t vs4;
    memset(&vs4, 0, sizeof(vs4));
    TransmitPlan_t q = DecideTransmitPlan(&vs4, 4800, 25.0f, false, 1000, true, false, false);
    CHECK_EQ_I(q.voltage_slope_mv_per_hour, 0);
    for (uint32_t t = 1700; t <= 1000 + 6 * 3600; t += 700) {
      q = DecideTransmitPlan(&vs4, 4800, 25.0f, false, t, true, false, true);
    }
    CHECK_REGRESSION(q.power_mode != MODE_CONSERVATIVE &&
                         q.power_mode != MODE_NORMAL,
                     "RV-03");
  }

  /* F8 (#172): power-mode hysteresis - one noisy sample (10 mV over the
   * ~600 s slope window = 60 mV/h, larger than every mode threshold) must
   * not flap the mode toward higher power. Downgrades stay immediate. */
  {
    VoltageSlope_t vs5;
    memset(&vs5, 0, sizeof(vs5));
    TransmitPlan_t r;
    r = DecideTransmitPlan(&vs5, 5200, 25.0f, false, 1000, true, false, false);
    CHECK(r.power_mode == MODE_CONSERVATIVE); /* baseline, slope 0 */
    /* +20 mV over 700 s: slope ~+103 mV/h -> proposes NORMAL */
    r = DecideTransmitPlan(&vs5, 5220, 25.0f, false, 1700, true, false, false);
    CHECK_REGRESSION(r.power_mode == MODE_CONSERVATIVE, "F8-noise1");
    /* still +51 mV/h at 1400 s: second proposal, still not committed */
    r = DecideTransmitPlan(&vs5, 5220, 25.0f, false, 2400, true, false, false);
    CHECK_REGRESSION(r.power_mode == MODE_CONSERVATIVE, "F8-noise2");
    /* third consecutive upgrade proposal commits */
    r = DecideTransmitPlan(&vs5, 5220, 25.0f, false, 3100, true, false, false);
    CHECK(r.power_mode == MODE_NORMAL);
    /* downgrade applies immediately (energy protection never waits) */
    r = DecideTransmitPlan(&vs5, 4800, 25.0f, false, 3800, true, false, false);
    CHECK_REGRESSION(r.power_mode != MODE_NORMAL, "F8-down-immediate");
  }

  /* Planner retains the survival cadence at 4200 mV. The first-flight
   * admission gate rejects this voltage before the planner is called; the
   * plan itself must not encode a reusable GNSS-less degradation mode. */
  memset(&vs, 0, sizeof(vs));
  p = DecideTransmitPlan(&vs, 4200, 25.0f, false, 1000, true, false, false);
  CHECK_EQ_I(p.power_mode, MODE_SURVIVAL);
  CHECK(p.gps_enabled);
  CHECK(p.gps_timeout_ms > 0U);
  CHECK_EQ_I(p.tx_interval_ms, 3600000);

  /* FLIGHT + no session -> RF silence veto (DDR-0018); commissioning exempt */
  memset(&vs, 0, sizeof(vs));
  p = DecideTransmitPlan(&vs, 5200, 22.0f, false, 1000, false, false, false);
  CHECK_EQ_I(p.veto, VETO_RF_SILENCE);
  memset(&vs, 0, sizeof(vs));
  p = DecideTransmitPlan(&vs, 5200, 22.0f, false, 1000, false, true, false);
  CHECK_EQ_I(p.veto, VETO_NONE);
}

static void test_power_model(void) {
  /* NormalizeBatteryVoltage */
  CHECK_EQ_I(NormalizeBatteryVoltage(5000, 25.0f), 5000);  /* no comp at 25C */
  CHECK_EQ_I(NormalizeBatteryVoltage(3330, -65.0f), 5500); /* table point */
  CHECK_EQ_I(NormalizeBatteryVoltage(2800, -66.0f), 5500); /* table point */
  CHECK_EQ_I(NormalizeBatteryVoltage(2800, -80.0f), 5500); /* below table: max comp */
  CHECK_EQ_I(NormalizeBatteryVoltage(5000, -5.0f), 5275);  /* halfway 0C/-10C */

  /* CalculateVoltageSlope */
  VoltageSlope_t vs;
  memset(&vs, 0, sizeof(vs));
  CHECK_EQ_I(CalculateVoltageSlope(&vs, 5000, 1000), 0);    /* first sample */
  CHECK_EQ_I(CalculateVoltageSlope(&vs, 5010, 1300), 0);    /* dt<600: last slope */
  CHECK_EQ_I(CalculateVoltageSlope(&vs, 5036, 4600), 36);   /* +36mV over 3600s */
  CHECK_EQ_I(CalculateVoltageSlope(&vs, 5020, 1500), 36);   /* dt<600: repeat last */
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
    CHECK_EQ_I(CalculateVoltageSlope(&vs3, 0, 1000), 0);       /* poisoned baseline latches */
    int16_t wrapped = CalculateVoltageSlope(&vs3, 6400, 1600); /* dt=600s */
    CHECK_REGRESSION(wrapped >= 0, "FINDING-6");
    printf("   finding #6: slope after 0->6400mV over 600s = %d mV/h (want >= 0)\n",
           (int)wrapped);
  }

  /* STAB-08 (#155): direction-aware prediction API. The old
   * PredictTimeToVoltage had logically dead boundary tests (delta>0 &&
   * current>=target can never hold) and folded "already past" into the same
   * 0xFFFF as "stable/never". Exhaustive table: current {<,=,>} target x
   * slope {<,0,>}, both directions. */
  {
    uint16_t hrs = 0xAAAA;
    /* LOWER threshold (e.g. 4500 mV critical) */
    CHECK_EQ_I(PredictTimeToLowerThreshold(4400, -100, 4500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToLowerThreshold(4500, -100, 4500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToLowerThreshold(4400, 0, 4500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToLowerThreshold(4400, 100, 4500, &hrs), PRED_AT_OR_PAST); /* rising but still below */
    CHECK_EQ_I(PredictTimeToLowerThreshold(5000, -100, 4500, &hrs), PRED_REACHABLE);
    CHECK_EQ_I(hrs, 5);
    CHECK_EQ_I(PredictTimeToLowerThreshold(5000, 0, 4500, &hrs), PRED_STABLE);
    CHECK_EQ_I(PredictTimeToLowerThreshold(5000, 100, 4500, &hrs), PRED_MOVING_AWAY);
    CHECK_EQ_I(PredictTimeToLowerThreshold(4500, 0, 4500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToLowerThreshold(4500, 100, 4500, &hrs), PRED_AT_OR_PAST);
    /* saturation: clamps to 9999, never wraps */
    CHECK_EQ_I(PredictTimeToLowerThreshold(65535, -1, 0, &hrs), PRED_REACHABLE);
    CHECK_EQ_I(hrs, 9999);
    /* UPPER threshold (e.g. 5500 mV full) */
    CHECK_EQ_I(PredictTimeToUpperThreshold(5600, 100, 5500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToUpperThreshold(5500, 100, 5500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToUpperThreshold(5600, 0, 5500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToUpperThreshold(5600, -100, 5500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToUpperThreshold(5000, 100, 5500, &hrs), PRED_REACHABLE);
    CHECK_EQ_I(hrs, 5);
    CHECK_EQ_I(PredictTimeToUpperThreshold(5000, 0, 5500, &hrs), PRED_STABLE);
    CHECK_EQ_I(PredictTimeToUpperThreshold(5000, -100, 5500, &hrs), PRED_MOVING_AWAY);
    CHECK_EQ_I(PredictTimeToUpperThreshold(5500, 0, 5500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToUpperThreshold(5500, -100, 5500, &hrs), PRED_AT_OR_PAST);
    CHECK_EQ_I(PredictTimeToUpperThreshold(0, 1, 65535, &hrs), PRED_REACHABLE);
    CHECK_EQ_I(hrs, 9999); /* saturation, upper direction */
  }

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
    TransmitPlan_t p2 = DecideTransmitPlan(&vs2, 4200, -66.0f, true, 1000, true, false, false);
    CHECK_EQ_I(p2.battery_mv_normalized, 4200); /* raw, uncompensated */
    CHECK_EQ_I(p2.power_mode, MODE_SURVIVAL);
  }

  CHECK(strcmp(GetModeName(MODE_SURVIVAL), "SURVIVAL") == 0);
  CHECK(strcmp(GetModeName((OperatingMode_t)99), "UNKNOWN") == 0);
}

/* ========================================================================== */
/* R2 findings (2026-08-09 pre-flight review) — EXPECTED-FAIL-BEFORE-FIX      */
/* ========================================================================== */

static void test_r2_10_slope_temperature_contamination(void) {
  /* R2-10 (#114): the slope is computed on the temperature-NORMALIZED
   * voltage. NormalizeBatteryVoltage ADDS compensation as temperature falls
   * (steeply below -55C: +430 mV at -55C, +2170 mV at -65C), so cooling at
   * constant charge state produces a RISING normalized voltage — the slope
   * reads cooling as charging. Sweep temperature at CONSTANT raw voltage:
   * the slope must stay flat. */
  VoltageSlope_t vs;
  memset(&vs, 0, sizeof(vs));
  TransmitPlan_t p1 = DecideTransmitPlan(&vs, 4500, -55.0f, false, 3600, true, false, false);
  TransmitPlan_t p2 = DecideTransmitPlan(&vs, 4500, -65.0f, false, 7200, true, false, false);
  (void)p1;
  /* Normalized voltage jumped ~4930 -> ~6670 mV purely from temperature. */
  CHECK_REGRESSION(abs(p2.voltage_slope_mv_per_hour) <= 20, "R2-10");
  /* And the mode must not flip to GPS-on NORMAL because it got COLDER. */
  CHECK_REGRESSION(p2.power_mode != MODE_NORMAL, "R2-10");
}

static void test_r2_11_no_history_default_uses_raw_voltage(void) {
  /* R2-11 (#115): slope state is RAM-only — after every reset there is no
   * history and SelectModeFromPredictions falls through every branch to
   * MODE_CONSERVATIVE (10-min cadence) even with a marginal
   * battery. The no-history default must derive from RAW voltage, not
   * fall through. (The slope baseline is RAM-only — DR12-15 persistence was
   * never implemented; finding #9 corrected the overstated comment.) */
  VoltageSlope_t vs;
  memset(&vs, 0, sizeof(vs));
  TransmitPlan_t p = DecideTransmitPlan(&vs, 4400, 25.0f, false, 3600, true, false, false);
  CHECK_EQ_I(p.voltage_slope_mv_per_hour, 0); /* guard: truly no history */
  CHECK_REGRESSION(p.power_mode != MODE_CONSERVATIVE, "R2-11");
  CHECK_REGRESSION(p.gps_enabled && p.gps_timeout_ms > 0U, "R2-11");
}

static void test_r2_17_already_critical_never_reports_stable(void) {
  /* R2-17 (#121): at/below the 4500 mV critical threshold while
   * discharging, both PredictTimeToVoltage calls return 0xFFFF, so
   * DecideTransmitPlan emits time_to_target_h == 0 — which the log path
   * (lora_app.c) and Cayenne ch 12 render as "Stable". 0 must be reserved
   * for genuinely stable; already-critical must be distinguishable
   * (any non-zero encoding is acceptable). */
  VoltageSlope_t vs;
  memset(&vs, 0, sizeof(vs));
  TransmitPlan_t p1 = DecideTransmitPlan(&vs, 4600, 25.0f, false, 3600, true, false, false);
  (void)p1;
  TransmitPlan_t p2 = DecideTransmitPlan(&vs, 4400, 25.0f, false, 7200, true, false, false);
  CHECK_EQ_I(p2.voltage_slope_mv_per_hour, -200); /* guard: discharging */
  CHECK(p2.power_mode != MODE_NORMAL);            /* guard: not charging */
  CHECK_REGRESSION(p2.time_to_target_h != 0, "R2-17");

  /* STAB-08 (#155): RISING but still below critical. Old code: delta=50,
   * slope=+150 -> hours = 50/150 = 0 -> time_to_target_h = -0 = 0 -> the
   * log path and Cayenne ch 12 render a below-critical battery "Stable".
   * PRED_AT_OR_PAST must map to -1 (critical now), never 0. */
  {
    VoltageSlope_t vs2;
    memset(&vs2, 0, sizeof(vs2));
    TransmitPlan_t q1 = DecideTransmitPlan(&vs2, 4300, 25.0f, false, 3600, true, false, false);
    (void)q1;
    TransmitPlan_t q2 = DecideTransmitPlan(&vs2, 4450, 25.0f, false, 7200, true, false, false);
    CHECK(q2.voltage_slope_mv_per_hour > 0); /* guard: charging */
    CHECK(q2.battery_mv_normalized < 4500);  /* guard: below critical */
    CHECK_REGRESSION(q2.time_to_target_h == -1, "STAB-08");
  }
}

/* STAB-06 (#153) + STAB-07 (#154): pure launch/float detectors (mission_logic.c) */
static void test_mission_logic(void) {
  /* --- STAB-06: launch reference is a BOUNDED window, not all-time max --- */
  LaunchDetector_t ld;
  LaunchDetector_Reset(&ld);
  /* high-pressure system, then ordinary lower pressure 3 h later: the aged
   * reference must NOT serve as a launch baseline (was: false launch) */
  CHECK(!LaunchDetector_Update(&ld, 1000.0f, true, 0));
  CHECK_REGRESSION(!LaunchDetector_Update(&ld, 994.0f, true, 3 * 3600), "STAB-06");
  CHECK(LaunchDetector_HasRef(&ld));
  CHECK(LaunchDetector_RefHpa(&ld) == 994.0f); /* reseeded to current */

  /* a REAL launch inside the window still trips (R3-10: once the drop is
   * SUSTAINED for the confirm window) */
  LaunchDetector_Reset(&ld);
  CHECK(!LaunchDetector_Update(&ld, 1000.0f, true, 100));
  CHECK(!LaunchDetector_Update(&ld, 993.9f, true, 200)); /* -6.1 hPa: candidate arms */
  CHECK(LaunchDetector_Update(&ld, 993.9f, true, 200 + MISSION_LAUNCH_CONFIRM_S));

  /* boundary: exactly at the window edge the reference is still valid
   * (cross early enough that the confirm window closes before expiry) */
  LaunchDetector_Reset(&ld);
  LaunchDetector_Update(&ld, 1000.0f, true, 0);
  CHECK(!LaunchDetector_Update(&ld, 993.0f, true,
                               MISSION_LAUNCH_REF_WINDOW_S - MISSION_LAUNCH_CONFIRM_S));
  CHECK(LaunchDetector_Update(&ld, 993.0f, true, MISSION_LAUNCH_REF_WINDOW_S));
  /* ...and one second past it, the same drop is weather, not launch */
  LaunchDetector_Reset(&ld);
  LaunchDetector_Update(&ld, 1000.0f, true, 0);
  CHECK_REGRESSION(!LaunchDetector_Update(&ld, 993.0f, true,
                                          MISSION_LAUNCH_REF_WINDOW_S + 1),
                   "STAB-06");

  /* invalid/stale pressure neither reseeds nor triggers */
  LaunchDetector_Reset(&ld);
  LaunchDetector_Update(&ld, 1000.0f, true, 0);
  CHECK(!LaunchDetector_Update(&ld, 0.0f, false, 60));
  CHECK(LaunchDetector_RefHpa(&ld) == 1000.0f);
  CHECK(!LaunchDetector_Update(&ld, 993.0f, true, 120)); /* ref intact, candidate arms */
  CHECK(LaunchDetector_Update(&ld, 993.0f, true, 120 + MISSION_LAUNCH_CONFIRM_S));

  /* R3-10 (#222): a momentary downward spike arms a candidate that any
   * recovery cancels - it must NEVER latch launch. */
  LaunchDetector_Reset(&ld);
  LaunchDetector_Update(&ld, 1000.0f, true, 0);
  CHECK(!LaunchDetector_Update(&ld, 993.0f, true, 60));   /* spike: candidate */
  CHECK(!LaunchDetector_Update(&ld, 1000.0f, true, 120)); /* recovered: cancel */
  CHECK_REGRESSION(!LaunchDetector_Update(&ld, 1000.0f, true, 400), "R3-10-spike");

  /* R3-10: a sustained drop fires only once the confirm window closes */
  LaunchDetector_Reset(&ld);
  LaunchDetector_Update(&ld, 1000.0f, true, 0);
  CHECK(!LaunchDetector_Update(&ld, 993.0f, true, 100));
  CHECK_REGRESSION(!LaunchDetector_Update(&ld, 993.5f, true,
                                          100 + MISSION_LAUNCH_CONFIRM_S - 1),
                   "R3-10-early");
  CHECK(LaunchDetector_Update(&ld, 993.5f, true, 100 + MISSION_LAUNCH_CONFIRM_S));

  /* 14-day stationary weather replay: a SLOW triangle, +-6 hPa over a 24 h
   * period (1 hPa/h — realistic frontal drift, no fast cliff). Inside any
   * 2 h window the swing is <= 2 hPa, far below the 6 hPa launch threshold;
   * and no aged maximum may accumulate across days. Never launches. */
  LaunchDetector_Reset(&ld);
  {
    bool launched = false;
    for (uint32_t h = 0; h < 14 * 24; h++) {
      int phase = (int)(h % 24);
      int tri = (phase < 12) ? (phase - 6) : (18 - phase); /* -6..+6, +-1/h */
      float p = 1000.0f + (float)tri;
      if (LaunchDetector_Update(&ld, p, true, h * 3600))
        launched = true;
    }
    CHECK_REGRESSION(!launched, "STAB-06-14day");
  }
  /* documented accepted tradeoff (mission_state.c): a fast storm FRONT that
   * drops 6 hPa inside the window still arms ASCENT cadence (once sustained
   * through the R3-10 confirm window) */
  LaunchDetector_Reset(&ld);
  LaunchDetector_Update(&ld, 1000.0f, true, 0);
  CHECK(!LaunchDetector_Update(&ld, 993.9f, true, 3600));
  CHECK(LaunchDetector_Update(&ld, 993.9f, true, 3600 + MISSION_LAUNCH_CONFIRM_S));

  /* --- STAB-07: FLOAT latch guards --- */
  FloatDetector_t fd;

  /* bench/armed-but-grounded: flat pressure, no ascent -> NEVER latch */
  FloatDetector_Reset(&fd);
  {
    bool latched = false;
    for (uint32_t t = 0; t <= 7200; t += 10)
      latched |= FloatDetector_Update(&fd, 1000.0f, true, t, false);
    CHECK_REGRESSION(!latched, "STAB-07-bench");
  }

  /* genuine float at altitude (ascent guard met, gentle wobble): latches */
  FloatDetector_Reset(&fd);
  {
    bool latched = false;
    uint32_t latch_t = 0;
    for (uint32_t t = 0; t <= 2400 && !latched; t += 10) {
      float p = 300.0f + ((t / 10) % 2 ? 0.3f : -0.3f);
      if (FloatDetector_Update(&fd, p, true, t, true)) {
        latched = true;
        latch_t = t;
      }
    }
    CHECK(latched);
    CHECK(latch_t >= MISSION_FLOAT_WINDOW_S);
  }

  /* slow climbs never latch (net-displacement guard): 0.5/1/1.5/3/5 m/s at ~9 km */
  {
    const float dp_per_10s[5] = {0.215f, 0.43f, 0.645f, 1.29f, 2.15f};
    for (int k = 0; k < 5; k++) {
      FloatDetector_Reset(&fd);
      bool latched = false;
      float p = 300.0f;
      for (uint32_t t = 0; t <= 3600 && !latched; t += 10) {
        if (FloatDetector_Update(&fd, p, true, t, true))
          latched = true;
        p -= dp_per_10s[k];
      }
      CHECK_REGRESSION(!latched, "STAB-07-climb");
    }
  }

  /* ascent stalls 1/3/5/10 min mid-climb (window is 15 min): never latch */
  {
    const uint32_t stall_s[4] = {60, 180, 300, 600};
    for (int k = 0; k < 4; k++) {
      FloatDetector_Reset(&fd);
      bool latched = false;
      float p = 500.0f;
      for (uint32_t t = 0; t <= 2400 && !latched; t += 10) {
        if (t < 600 || t >= 600 + stall_s[k])
          p -= 1.29f; /* 3 m/s, else flat */
        if (FloatDetector_Update(&fd, p, true, t, true))
          latched = true;
      }
      CHECK_REGRESSION(!latched, "STAB-07-stall");
    }
  }

  /* descent (pressure rising steadily): window restarts, never latch */
  FloatDetector_Reset(&fd);
  {
    bool latched = false;
    float p = 300.0f;
    for (uint32_t t = 0; t <= 1800 && !latched; t += 10) {
      if (FloatDetector_Update(&fd, p, true, t, true))
        latched = true;
      p += 1.0f;
    }
    CHECK(!latched);
  }

  /* stale pressure kills the window: a stale gap mid-flat restarts timing */
  FloatDetector_Reset(&fd);
  {
    bool latched = false;
    for (uint32_t t = 0; t <= 2400 && !latched; t += 10) {
      bool valid = !(t >= 800 && t < 900); /* 100 s stale gap */
      if (FloatDetector_Update(&fd, 300.0f, valid, t, true))
        latched = true;
    }
    CHECK(latched); /* still latches eventually... */
  }
  FloatDetector_Reset(&fd);
  {
    /* ...but NOT at 800+window: the stale gap must have reset the clock */
    bool early = false;
    for (uint32_t t = 0; t < 800 + MISSION_FLOAT_WINDOW_S; t += 10) {
      bool valid = !(t >= 800 && t < 900);
      if (FloatDetector_Update(&fd, 300.0f, valid, t, true))
        early = true;
    }
    CHECK(!early);
  }

  /* RV-06 (#162): a BACKWARD time step (LSE->LSI failover restarts the RTC
   * counter near zero) must not wrap (now - start) into an instant latch.
   * Deadman_Check pattern: re-seed, never evaluate a wrapped delta. */
  FloatDetector_Reset(&fd);
  {
    bool latched = false;
    float p = 900.0f;
    for (int i = 0; i < 5; i++) { /* 50 s of 10 s-cadence ascent */
      latched |= FloatDetector_Update(&fd, p, true, 4000 + (uint32_t)i * 10, true);
      p -= 0.6f;
    }
    CHECK(!latched);
    /* clock restarts: now jumps 4050 -> 3 */
    latched |= FloatDetector_Update(&fd, p, true, 3, true);
    CHECK_REGRESSION(!latched, "RV-06");
    /* keep climbing after the step: window re-seeded, still no latch */
    for (int i = 0; i < 10; i++) {
      p -= 5.0f;
      latched |= FloatDetector_Update(&fd, p, true, 13 + (uint32_t)i * 10, true);
    }
    CHECK_REGRESSION(!latched, "RV-06-post");
    /* and a genuine float AFTER the step still latches (guard not wedged) */
    FloatDetector_Reset(&fd);
    bool late = false;
    for (uint32_t t = 100; t <= 100 + 2 * MISSION_FLOAT_WINDOW_S && !late; t += 10)
      late |= FloatDetector_Update(&fd, 300.0f, true, t, true);
    CHECK(late);
  }
  /* launch detector: same wrapped-delta guard on the reference window */
  LaunchDetector_Reset(&ld);
  CHECK(!LaunchDetector_Update(&ld, 1000.0f, true, 50000));
  CHECK(!LaunchDetector_Update(&ld, 999.0f, true, 50060));
  /* clock restarts below ref_set_s: re-seed from current, no false launch */
  CHECK_REGRESSION(!LaunchDetector_Update(&ld, 994.0f, true, 10), "RV-06-launch");
  CHECK(LaunchDetector_RefHpa(&ld) == 994.0f);

  /* F1 (#167): a launch reference restored from the backup domain after a
   * mid-ascent reset must NOT age out like a weather maximum - it is THE
   * launch altitude, permanently valid. LaunchDetector_SetRef pins it. */
  LaunchDetector_Reset(&ld);
  LaunchDetector_SetRef(&ld, 1000.0f, 5000);
  CHECK(LaunchDetector_HasRef(&ld));
  CHECK(LaunchDetector_RefHpa(&ld) == 1000.0f);
  /* hours later, a nearby sample must not reseed/age out the pinned ref
   * (without pinning, the 2 h aging window would reseed to 995) */
  CHECK(!LaunchDetector_Update(&ld, 995.0f, true, 5000 + 4 * 3600));
  CHECK_REGRESSION(LaunchDetector_RefHpa(&ld) == 1000.0f, "F1-pinned");
  /* and the restored ref still gates min-ascent correctly for FLOAT */
  CHECK_REGRESSION((LaunchDetector_RefHpa(&ld) - 300.0f) >=
                       MISSION_FLOAT_MIN_ASCENT_DP_HPA,
                   "F1-minascent");

  /* RV-07 (#163): a degenerate two-sample window (SURVIVAL 1-h cadence) must
   * not satisfy the latch - minimum sample count in the window. */
  FloatDetector_Reset(&fd);
  {
    bool latched = false;
    latched |= FloatDetector_Update(&fd, 60.0f, true, 10000, true);
    latched |= FloatDetector_Update(&fd, 60.0f, true, 13600, true); /* flat, +1 h */
    CHECK_REGRESSION(!latched, "RV-07");
    /* 4+ measured samples over hours at 1-h cadence: legitimate latch */
    latched |= FloatDetector_Update(&fd, 60.0f, true, 17200, true);
    latched |= FloatDetector_Update(&fd, 60.0f, true, 20800, true);
    CHECK(latched);
  }
}

/* STAB-12 (#159): the timestamp-wrap latch must be restorable across reset */
static void test_ts_wrap_restore(void) {
  /* statics are shared with earlier compact-encode tests in this binary —
   * start from a known-clear latch (the restore path itself is under test) */
  Payload_SetTimestampWrapped(false);
  CHECK(!Payload_IsTimestampWrapped());
  Payload_SetTimestampWrapped(true);
  CHECK_REGRESSION(Payload_IsTimestampWrapped(), "STAB-12");
  /* and the restored latch must reach the wire (status b5) */
  sensor_t s = make_nominal_sensors();
  CompactTelemetryPacket_t pkt;
  CHECK(EncodeCompactBinaryPacket(&pkt, &s, 1234, 0, MODE_NORMAL));
  CHECK_REGRESSION((pkt.status & STATUS_TS_WRAP_MASK) != 0, "STAB-12-wire");
  Payload_SetTimestampWrapped(false); /* leave no trace */
  CHECK(!Payload_IsTimestampWrapped());
}

static void test_r2_30_rmc_valid_clears_on_void(void) {
  /* R2-30 (#130): GNSS_ParseRMC sets data.valid on status 'A' but never
   * clears it on 'V' — a fix lost mid-window still reads as held. */
  GNSS_HandleTypeDef g;
  memset(&g, 0, sizeof(g));
  GNSS_ParseRMC(&g, "$GNRMC,120000,A,4807.038,N,01131.000,E,0.5,180.0,060825,,,A*00");
  CHECK(g.data.valid); /* guard: 'A' sets it (passes today) */
  GNSS_ParseRMC(&g, "$GNRMC,120100,V,,,,,,,060825,,,N*00");
  CHECK_REGRESSION(!g.data.valid, "R2-30"); /* 'V' must clear it */
}

static void test_r2_19_dma_overrun_blind_spot(void) {
  /* R2-19 (#123): dma_produced_total advances in 256-byte quanta, so the
   * (produced - consumed) > GNSS_DMA_BUFFER_SIZE check under-reports: the
   * counter can say exactly one buffer-full while up to 255 REAL bytes
   * past that were already produced (and lost) uncounted. A full buffer's
   * worth of unconsmed production must already surface as an overrun. */
  GNSS_HandleTypeDef g;
  memset(&g, 0, sizeof(g));
  g.is_powered = true;
  g_host_dma_cndtr = 0;       /* head = (512-0)%512 = 0 == tail: nothing consumable */
  g.dma_produced_total = 512; /* two half-callbacks fired; parser never ran */
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

int main(void) {
  test_crc_vectors();
  test_compact_packet();
  test_highres_record();
  test_commissioning_live_record();
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
  test_mission_logic();
  test_ts_wrap_restore();

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
