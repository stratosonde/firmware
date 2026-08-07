/* Host-test stub (#57): low-power manager surface used by atgm336h.c. */
#ifndef STM32_LPM_H_STUB
#define STM32_LPM_H_STUB
#include <stdint.h>
#define UTIL_LPM_ENABLE   1
#define UTIL_LPM_DISABLE  0
static inline void UTIL_LPM_SetStopMode(uint32_t mask, int state) { (void)mask; (void)state; }
#endif
