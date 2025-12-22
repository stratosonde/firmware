/**
  ******************************************************************************
  * @file    multiregion_h3.h
  * @brief   H3Lite-based multi-region detection for LoRaWAN
  ******************************************************************************
  * @attention
  *
  * This module provides precise LoRaWAN region detection using H3Lite
  * geospatial hexagon indexing system.
  *
  ******************************************************************************
  */

#ifndef MULTIREGION_H3_H
#define MULTIREGION_H3_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "h3lite.h"
#include "LoRaMacInterfaces.h"

/* Exported types ------------------------------------------------------------*/
// LoRaMacRegion_t is defined in LoRaMacInterfaces.h

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Detect LoRaWAN region from GPS coordinates using H3Lite
 * @param  lat: Latitude in decimal degrees
 * @param  lon: Longitude in decimal degrees
 * @retval LoRaMacRegion_t: Detected LoRaWAN region
 * @note   Uses H3 geospatial indexing for precise region detection
 *         Falls back to nearest neighbor search if outside known regions
 */
LoRaMacRegion_t MultiRegion_DetectFromGPS_H3(float lat, float lon);

/**
 * @brief  Map H3Lite RegionId to LoRaMacRegion_t enum
 * @param  h3Region: Region ID from h3lite library
 * @retval LoRaMacRegion_t: Corresponding LoRaWAN region enum
 * @note   Used internally but exposed for testing
 */
LoRaMacRegion_t H3Region_ToLoRaMacRegion(RegionId h3Region);

/**
 * @brief  Get current active LoRaWAN region
 * @retval LoRaMacRegion_t: Currently active region
 * @note   Placeholder until multiregion context manager is implemented
 */
LoRaMacRegion_t MultiRegion_GetActiveRegion(void);

#ifdef __cplusplus
}
#endif

#endif /* MULTIREGION_H3_H */
