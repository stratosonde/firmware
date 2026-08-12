/* Host-test stub (#57): low-power manager surface used by atgm336h.c. */
#ifndef STM32_LPM_H_STUB
#define STM32_LPM_H_STUB
#include <stdint.h>
#define UTIL_LPM_ENABLE   1
#define UTIL_LPM_DISABLE  0
/* R9 (#194) observability: last STOP-mode state requested (per-TU static). */
static int g_host_lpm_stop_state __attribute__((unused)) = -1;
static inline void UTIL_LPM_SetStopMode(uint32_t mask, int state) { (void)mask; g_host_lpm_stop_state = state; }
#endif
