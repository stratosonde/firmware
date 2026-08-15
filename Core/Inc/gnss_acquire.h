/**
  ******************************************************************************
  * @file    gnss_acquire.h
  * @brief   Pure GNSS acquisition policy (refactor stage 4)
  ******************************************************************************
  * Calendar math and the acquisition decision rules extracted from
  * lora_app.c's AcquireGnssFix / SysTimeSyncFromGnss. The mechanism (UART/DMA
  * driving, GNSS_* calls, power control, watchdog, WFI sleep) stays in
  * lora_app.c. No hardware, no globals, no time reads: time is an input.
  *
  * Context preserved from the code:
  * - S-A (#211): the acquisition loop must run a POSITIVE number of
  *   iterations for every admitted plan; zero only for an explicit
  *   non-acquisition path. The budget is therefore a named, testable
  *   function, not an inline multiply.
  * - DR-01/DR-02: validation runs BEFORE the caller commits anything; the
  *   acceptance predicate is a pure conjunction the caller evaluates before
  *   any state update.
  ******************************************************************************
  */

#ifndef GNSS_ACQUIRE_H
#define GNSS_ACQUIRE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Days since 1970-01-01 for a proleptic-Gregorian civil date. Pure. */
uint32_t GnssAcquire_DaysFromCivil(int y, unsigned m, unsigned d);

/**
  * @brief  UTC epoch seconds from NMEA DDMMYY / HHMMSS fields.
  *         Input validity is the caller's job
  *         (FirstFlightPolicy_GnssDateTimeValid runs first); this composes
  *         the epoch from fields already accepted.
  */
uint32_t GnssAcquire_EpochFromUtc(uint32_t date_ddmmyy, uint32_t time_hhmmss);

/**
  * @brief  Clock-independent iteration backstop for the acquisition loop
  *         (F-2/#177): 32 iterations per millisecond mirrors
  *         W25Q_MAX_BUSY_POLLS_PER_MS - far above any real iteration rate
  *         (each pass sleeps in WFI until an interrupt). The tick bound
  *         stays the normal exit; this cap is the backstop that does not
  *         depend on the clock under suspicion.
  * @retval 0 only for a zero timeout (the explicit non-acquisition path);
  *         strictly positive for every admitted plan (S-A/#211).
  */
uint32_t GnssAcquire_IterationBudget(uint32_t timeout_ms);

/**
  * @brief  First-flight acceptance: a same-wake GNSS result is complete only
  *         when the accepted-quality position AND the valid RMC date/time
  *         are both present. Do not accept GGA early merely because RMC
  *         normally follows milliseconds later.
  */
bool GnssAcquire_PackageComplete(bool fix_good_quality, bool gnss_datetime_valid);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_ACQUIRE_H */
