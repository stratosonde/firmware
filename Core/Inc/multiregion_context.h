/**
  ******************************************************************************
  * @file    multiregion_context.h
  * @brief   Multi-region LoRaWAN ABP configuration
  ******************************************************************************
  * @attention
  *
  * Simple ABP-based multi-region support for seamless region switching
  * without OTAA join overhead or session key management complexity.
  *
  ******************************************************************************
  */

#ifndef __MULTIREGION_CONTEXT_H__
#define __MULTIREGION_CONTEXT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "LoRaMacInterfaces.h"
#include "LmHandler.h"

/* Exported defines ----------------------------------------------------------*/
#define MAX_ABP_REGIONS  2  // US915, EU868

/* Enable/disable auto region switching based on GPS */
#ifndef MULTIREGION_AUTO_SWITCH_ENABLED
#define MULTIREGION_AUTO_SWITCH_ENABLED  1
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ABP configuration for a single region
 */
typedef struct {
    LoRaMacRegion_t region;
    uint8_t dev_eui[8];
    uint32_t dev_addr;
    uint8_t app_s_key[16];
    uint8_t nwk_s_key[16];
} ABP_RegionConfig_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize multi-region ABP support
 * @note Loads ABP credentials for all regions from se-identity.h
 */
void MultiRegion_Init(void);

/**
 * @brief Get current active region
 * @retval LoRaMacRegion_t Currently active region
 */
LoRaMacRegion_t MultiRegion_GetActiveRegion(void);

/**
 * @brief Switch to a different region
 * @param region Target region to switch to
 * @retval LmHandlerErrorStatus_t Success or error code
 * @note Loads ABP credentials and reconfigures radio for target region
 */
LmHandlerErrorStatus_t MultiRegion_SwitchToRegion(LoRaMacRegion_t region);

/**
 * @brief Auto-switch region based on GPS coordinates
 * @param lat Latitude in decimal degrees
 * @param lon Longitude in decimal degrees
 * @retval LmHandlerErrorStatus_t Success or error code
 * @note Uses H3Lite to detect region, then switches if different from current
 */
LmHandlerErrorStatus_t MultiRegion_AutoSwitchForLocation(float lat, float lon);

#ifdef __cplusplus
}
#endif

#endif /* __MULTIREGION_CONTEXT_H__ */
