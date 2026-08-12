/* Host-test stub: STM32WL LL RTC (F-002/#201 timer_if.c suite). Only the
 * one inline the unit under test uses. */
#ifndef STM32WLXX_LL_RTC_H_STUB
#define STM32WLXX_LL_RTC_H_STUB
#include "stm32wlxx_hal.h"
static inline uint32_t LL_RTC_TIME_GetSubSecond(RTC_TypeDef *RTCx) { return RTCx->SSR; }
#endif
