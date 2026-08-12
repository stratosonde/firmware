/* sys_caps.h - F-014 (#207): slimmed system-capability tracking.
 *
 * The degrade-and-carry-on policy is correct for the mission, but a generic
 * nonfatal Error_Handler can let execution continue past a PARTIAL
 * peripheral init, leaving a subsystem "half initialized" rather than
 * clearly unavailable. This module is the deliberately small version of the
 * review's capability model: a bitmask, marked at the concrete init-failure
 * sites, printable at boot, consumable anywhere a guard is needed.
 *
 * NOT the full system_capabilities_t architecture - existing per-handle
 * guards (FlashLog_* on hlog->initialized, GNSS on hgnss.initialized) remain
 * the primary mechanism; this is the unified, telemetry-visible view.
 */
#ifndef SYS_CAPS_H
#define SYS_CAPS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SYS_CAP_FLASH   = 0x01,   /* W25Q16JV + FlashLog archive */
    SYS_CAP_GNSS    = 0x02,   /* GNSS UART/DMA + driver */
    SYS_CAP_SENSORS = 0x04,   /* I2C2 environmental sensors */
    SYS_CAP_RADIO   = 0x08    /* LoRaWAN MAC */
} sys_cap_t;

/* All capabilities start ASSUMED OK at boot; failure sites mark them down.
 * (Optimistic default matches the existing guards: a capability is only
 * revoked on an observed failure, never forgotten.) */
void SysCaps_MarkFailed(sys_cap_t cap);
bool SysCaps_Available(sys_cap_t cap);
uint8_t SysCaps_Raw(void);

#endif /* SYS_CAPS_H */
