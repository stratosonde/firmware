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
