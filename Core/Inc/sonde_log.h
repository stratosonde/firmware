/**
  ******************************************************************************
  * @file    sonde_log.h
  * @brief   Compile-time logging gate (R50, issue #47)
  *
  * Every debug/log line goes through SONDE_LOG / SONDE_LOG_STR / SONDE_LOG_WRITE.
  * Bench builds (default): full RTT output. Flight build (-DSONDE_FLIGHT_BUILD):
  * everything compiles to nothing — 500+ format strings and call overhead leave
  * FLASH/RAM for the expansion work. Control-path RTT (Init, LOCK, down-buffer
  * console input) is intentionally NOT gated.
  ******************************************************************************
  */
#ifndef SONDE_LOG_H
#define SONDE_LOG_H

#ifdef SONDE_FLIGHT_BUILD
  #define SONDE_LOG(...)              ((void)0)
  #define SONDE_LOG_STR(s)            ((void)0)
  #define SONDE_LOG_WRITE(d, n)       ((void)0)
#else
  #include "SEGGER_RTT.h"
  #define SONDE_LOG(...)              SEGGER_RTT_printf(0, __VA_ARGS__)
  #define SONDE_LOG_STR(s)            SEGGER_RTT_WriteString(0, s)
  #define SONDE_LOG_WRITE(d, n)       SEGGER_RTT_Write(0, d, n)
#endif

#endif /* SONDE_LOG_H */
