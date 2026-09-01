/* GENERATED FILE -- DO NOT HAND-EDIT.
 * Source of truth: wire/wire_schema.json. Regenerate: python wire/tools/gen_c_wire.py
 * Constants asserted against the real firmware headers by test_main.c (wire-schema drift gate). */
#ifndef WIRE_FORMAT_GEN_H
#define WIRE_FORMAT_GEN_H

/* heartbeat (port 10) */
#define WIRE_HEARTBEAT_LEN 11U
#define WIRE_HEARTBEAT_VERSION 2U
#define WIRE_HB_OFF_TIMESTAMP 0U
#define WIRE_HB_OFF_LAT 2U
#define WIRE_HB_OFF_LON 4U
#define WIRE_HB_OFF_TEMP 6U
#define WIRE_HB_OFF_PRESSHUM 7U
#define WIRE_HB_OFF_BATT 9U
#define WIRE_HB_OFF_STATUS 10U
#define WIRE_HB_PRESS_LSB 0U
#define WIRE_HB_PRESS_BITS 11U
#define WIRE_HB_PRESS_INVALID 2047U
#define WIRE_HB_HUM_LSB 11U
#define WIRE_HB_HUM_BITS 5U
#define WIRE_HB_HUM_INVALID 31U

/* archive record v7 (port 11) */
#define WIRE_ARCHIVE_RECORD_LEN 36U
#define WIRE_REC_OFF_SQ 32U
#define WIRE_REC_OFF_VETO 33U
#define WIRE_REC_OFF_CRC16 34U
#define WIRE_REC_CRC_COVER 34U /* bytes covered by record crc16 */
#define WIRE_REC_CRC_ALGO 0U /* 0=modbus */

/* bulk envelope v7 (port 11) */
#define WIRE_BULK_TYPE 7U
#define WIRE_BULK_OVERHEAD 6U /* type+count+crc32 */
#define WIRE_BULK_RECORD_STRIDE 40U /* seq u32 + 36B record */
#define WIRE_BULK_CRC_ALGO 2U /* 2=crc32-iso-hdlc */

/* version report (port 20) */
#define WIRE_VERSION_LEN 12U
#define WIRE_VERSION_MAGIC 86U
#define WIRE_VR_OFF_CRC16 10U
#define WIRE_VR_CRC_COVER 10U
#define WIRE_VR_CRC_ALGO 1U /* 1=ccitt-false */

#endif /* WIRE_FORMAT_GEN_H */
