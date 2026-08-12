/* Host-test stub: timer interface (R49). Test provides the implementation.
 * F-002 (#201): suites that compile the REAL timer_if.c define
 * HOST_USE_REAL_TIMER_IF and get the real header instead. */
#ifdef HOST_USE_REAL_TIMER_IF
#include "../../Core/Inc/timer_if.h"
#else
#ifndef TIMER_IF_H_STUB
#define TIMER_IF_H_STUB
#include <stdint.h>
uint32_t TIMER_IF_GetTime(uint16_t *ms);
#endif
#endif
