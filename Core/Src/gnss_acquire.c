/**
  ******************************************************************************
  * @file    gnss_acquire.c
  * @brief   Pure GNSS acquisition policy (refactor stage 4)
  ******************************************************************************
  * DaysFromCivil and the epoch composition moved verbatim from lora_app.c;
  * the iteration budget and package-completeness rule name the decisions the
  * acquisition loop already encoded. Same inputs -> same outputs; no I/O.
  ******************************************************************************
  */

#include "gnss_acquire.h"

#include <math.h>
#include <stddef.h>

uint32_t GnssAcquire_DaysFromCivil(int y, unsigned m, unsigned d)
{
  y -= (m <= 2);
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153U * (m + (m > 2 ? (unsigned)-3 : 9U)) + 2U) / 5U + d - 1U;
  unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  return (uint32_t)(era * 146097 + (int)doe - 719468);
}

uint32_t GnssAcquire_EpochFromUtc(uint32_t date_ddmmyy, uint32_t time_hhmmss)
{
  uint32_t d = date_ddmmyy;             /* DDMMYY */
  uint32_t t = time_hhmmss;             /* HHMMSS */
  int day = (int)(d / 10000U);
  int mon = (int)((d / 100U) % 100U);
  int yr  = 2000 + (int)(d % 100U);
  return GnssAcquire_DaysFromCivil(yr, (unsigned)mon, (unsigned)day) * 86400U
         + (t / 10000U) * 3600U + ((t / 100U) % 100U) * 60U + (t % 100U);
}

uint32_t GnssAcquire_IterationBudget(uint32_t timeout_ms)
{
  return timeout_ms * 32u;
}

bool GnssAcquire_PackageComplete(bool fix_good_quality, bool gnss_datetime_valid)
{
  return fix_good_quality && gnss_datetime_valid;
}

bool GnssAcquire_FixAccepted(const GnssFixCandidate_t *candidate,
                             const GnssFixLimits_t *limits)
{
  if (candidate == NULL || limits == NULL) {
    return false;
  }
  if (!candidate->valid || !candidate->position_present ||
      !candidate->fix_quality_valid || !candidate->coordinates_valid) {
    return false;
  }
  /* Non-finite or negative HDOP is rejected BEFORE any conversion or
   * comparison. The comparison itself stays in the float domain with the
   * x10 limit exact, so no unsafe float-to-integer conversion ever runs;
   * hdop * 10 overflow yields +Inf, which fails the comparison, so
   * unrepresentable values reject safely. Boundary equality accepts. */
  if (!isfinite(candidate->hdop) || candidate->hdop < 0.0f) {
    return false;
  }
  if (candidate->satellites < limits->minimum_satellites) {
    return false;
  }
  return (candidate->hdop * 10.0f) <= (float)limits->maximum_hdop_x10;
}
