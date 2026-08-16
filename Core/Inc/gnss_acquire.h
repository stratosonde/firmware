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

/**
  * @brief  The ONE authoritative, configured mission fix-acceptance rule
  *         (2026-08-15 handoff A5; tracker #284/H-08). Pure: no HAL, no
  *         configuration storage, no globals - the adapter snapshots the
  *         configured limits once per acquisition/cycle and builds the
  *         candidate from the current fix fields with designated
  *         initializers.
  *
  *         Every location-freshness decision must use this one result. Only
  *         an accepted fix may return GNSS_ACQUIRE_FRESH_GOOD_FIX, clear the
  *         EnvSensors GNSS stale state, update the fresh-fix epoch, persist
  *         the trusted last position, or authorize a region switch. A
  *         rejected candidate may still discipline the RTC via valid RMC
  *         time; its position stays rejected and stale.
  */
typedef struct {
    bool    valid;               /* fix latch (RMC 'A' / data.valid) */
    bool    position_present;    /* real lat/lon tokens (STAB-P1#1/#237) */
    bool    fix_quality_valid;   /* fix_quality != GNSS_FIX_INVALID */
    bool    coordinates_valid;   /* range check; (0,0) is genuine (R32/#57) */
    uint8_t satellites;
    float   hdop;
} GnssFixCandidate_t;

typedef struct {
    uint8_t minimum_satellites;  /* config gps_min_satellites (default 4) */
    uint8_t maximum_hdop_x10;    /* config gps_max_hdop_x10 (default 25 = 2.5) */
} GnssFixLimits_t;

/**
  * @brief  Configured acceptance: all candidate invariants hold, satellites
  *         >= the configured minimum, and HDOP <= the configured maximum.
  *         Equality at both configured boundaries is accepted. Non-finite or
  *         negative HDOP is rejected before any conversion/comparison; the
  *         comparison stays in the float domain (the x10 limit is exact), so
  *         no unsafe float-to-integer conversion ever runs. NULL -> false.
  */
bool GnssAcquire_FixAccepted(const GnssFixCandidate_t *candidate,
                             const GnssFixLimits_t *limits);

/**
 * @brief  Disposition of a non-package-complete acquisition outcome
 *         (BEH-02 / #284): what a weak/basic fix may touch. Only an
 *         ACCEPTED fix earns trusted position (last-known-good RAM copy,
 *         persistence, the fresh-fix epoch, the stale-flag clearing); a
 *         rejected candidate keeps its position in the current diagnostic
 *         sample with stale/weak provenance. RTC discipline is a separate
 *         time-validity decision: valid date/time may discipline the
 *         clock, but successful discipline never proves position quality.
 */
typedef struct {
  bool update_trusted_position; /* last-known-good + persistence + epoch */
  bool mark_gnss_stale;         /* rejected/absent position = stale provenance */
  bool discipline_time;         /* valid date/time may discipline the RTC */
} GnssFixDisposition_t;

GnssFixDisposition_t GnssAcquire_Disposition(bool fix_accepted,
                                             bool position_present,
                                             bool gnss_datetime_valid);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_ACQUIRE_H */
