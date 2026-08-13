/* Host-test stub: reset cause (R49). */
#ifndef RESET_CAUSE_H_STUB
#define RESET_CAUSE_H_STUB
#include <stdint.h>
static inline uint8_t ResetCause_Get(void) { return 0; }
/* S-03 (#227): test-controlled consecutive-boot counter (defined per-TU). */
extern uint32_t g_host_boot_attempts;
static inline uint32_t ResetCause_GetBootAttempts(void) { return g_host_boot_attempts; }
#endif
