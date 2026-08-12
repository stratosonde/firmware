/* Host-test stub: LoRaMac interfaces (R15 — real multiregion_context.c
 * against a fake LoRaMac). Region enum values MUST match the real
 * Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacInterfaces.h: flash-persisted
 * contexts carry these ordinals. */
#ifndef LORAMAC_INTERFACES_H_STUB
#define LORAMAC_INTERFACES_H_STUB
#include <stdint.h>

typedef enum eLoRaMacRegion
{
    LORAMAC_REGION_AS923 = 0,
    LORAMAC_REGION_AU915,
    LORAMAC_REGION_CN470,
    LORAMAC_REGION_CN779,
    LORAMAC_REGION_EU433,
    LORAMAC_REGION_EU868,
    LORAMAC_REGION_KR920,
    LORAMAC_REGION_IN865,
    LORAMAC_REGION_US915,
    LORAMAC_REGION_RU864,
} LoRaMacRegion_t;

typedef enum eActivationType
{
    ACTIVATION_TYPE_NONE = 0,
    ACTIVATION_TYPE_ABP,
    ACTIVATION_TYPE_OTAA,
} ActivationType_t;

#endif