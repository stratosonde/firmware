/* Host-test stub: lora_app surface used by multiregion_context.c (R15
 * harness). Implementations live in test_multiregion.c. */
#ifndef LORA_APP_H_STUB
#define LORA_APP_H_STUB
#include <stdbool.h>
#include "LoRaMacInterfaces.h"

/* F-01 (#245): real signature returns LmHandlerErrorStatus_t - callers check. */
LmHandlerErrorStatus_t LoRaApp_ReInitStack(LoRaMacRegion_t region);
bool LoRaApp_EraseNvmSlots(void);

#endif