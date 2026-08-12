/* Host-test stub: SysTime (R49). Test provides the implementation.
 * F-002 (#201): suites compiling the REAL timer_if.c define
 * HOST_USE_REAL_TIMER_IF and get the real header instead. */
#ifdef HOST_USE_REAL_TIMER_IF
#include "../../Utilities/misc/stm32_systime.h"
#else
#ifndef STM32_SYSTIME_H_STUB
#define STM32_SYSTIME_H_STUB
#include <stdint.h>
typedef struct SysTime_s { uint32_t Seconds; int16_t SubSeconds; } SysTime_t;
SysTime_t SysTimeGet(void);
#endif
#endif
