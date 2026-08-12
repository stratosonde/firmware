/* Host-test stub: mission state (R49). */
#ifndef MISSION_STATE_H_STUB
#define MISSION_STATE_H_STUB
#include <stdint.h>
#include <stdbool.h>
static inline uint8_t MissionState_GetStatusBits(void) { return 0; }
static inline bool MissionState_IsCommissioning(void) { return true; }
/* multiregion_context.c calls this on join attempts (R15 harness) */
static inline void MissionState_Update(float pressure_hpa, bool press_fresh, uint32_t now_s) { (void)pressure_hpa; (void)press_fresh; (void)now_s; }
#endif
