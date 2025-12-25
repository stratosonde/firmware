/**
  ******************************************************************************
  * @file    multiregion_h3.c
  * @brief   H3Lite-based multi-region detection implementation
  ******************************************************************************
  * @attention
  *
  * This module provides precise LoRaWAN region detection using H3Lite
  * geospatial hexagon indexing system.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "multiregion_h3.h"
#include "multiregion_context.h"  // For MultiRegion_GetActiveRegion()
#include "sys_app.h"  // For debug logging
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define H3_MAX_DISTANCE_KM  500.0f  // Maximum distance to consider for nearest neighbor

/* Private variables ---------------------------------------------------------*/
// Removed static currentRegion - now using MultiRegion_GetActiveRegion() from multiregion_context.c

/* Private function prototypes -----------------------------------------------*/
static void LogRegionDetection(const char* h3RegionName, float lat, float lon, LoRaMacRegion_t loraRegion);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Map H3Lite RegionId to LoRaMacRegion_t enum
 */
LoRaMacRegion_t H3Region_ToLoRaMacRegion(RegionId h3Region)
{
    const char* name = getRegionName(h3Region);
    
    // Map h3lite region names to LoRaMac regions
    if (strcmp(name, "US915") == 0) return LORAMAC_REGION_US915;
    if (strcmp(name, "EU868") == 0) return LORAMAC_REGION_EU868;
    if (strcmp(name, "AS923-1") == 0) return LORAMAC_REGION_AS923;
    if (strcmp(name, "AS923-1B") == 0) return LORAMAC_REGION_AS923;
    if (strcmp(name, "AS923-1C") == 0) return LORAMAC_REGION_AS923;
    if (strcmp(name, "AS923-2") == 0) return LORAMAC_REGION_AS923;
    if (strcmp(name, "AS923-3") == 0) return LORAMAC_REGION_AS923;
    if (strcmp(name, "AS923-4") == 0) return LORAMAC_REGION_AS923;
    if (strcmp(name, "AU915") == 0) return LORAMAC_REGION_AU915;
    if (strcmp(name, "CN470") == 0) return LORAMAC_REGION_CN470;
    if (strcmp(name, "KR920") == 0) return LORAMAC_REGION_KR920;
    if (strcmp(name, "IN865") == 0) return LORAMAC_REGION_IN865;
    if (strcmp(name, "RU864") == 0) return LORAMAC_REGION_RU864;
    if (strcmp(name, "EU433") == 0) return LORAMAC_REGION_EU433;
    // CD900-1A not currently mapped to standard LoRaMac region
    
    // Unknown region - keep current
    APP_LOG(TS_ON, VLEVEL_M, "H3: Unknown region '%s', keeping current\r\n", name);
    return MultiRegion_GetActiveRegion();
}

/**
 * @brief  Detect LoRaWAN region from GPS coordinates using H3Lite
 */
LoRaMacRegion_t MultiRegion_DetectFromGPS_H3(float lat, float lon)
{
    // Use h3lite for precise region detection
    RegionId h3Region = latLngToRegion((double)lat, (double)lon);
    
    if (h3Region == 0) {
        // Not in any region - try nearest neighbor search
        NearestRegionsInfo nearest = findNearestRegions((double)lat, (double)lon, 3);
        
        if (nearest.numRegions > 0 && nearest.regions[0].distanceKm < H3_MAX_DISTANCE_KM) {
            APP_LOG(TS_ON, VLEVEL_M, 
                    "H3: Outside regions, nearest: %s (%.1f km)\r\n",
                    nearest.regions[0].regionName,
                    nearest.regions[0].distanceKm);
            
            // Use nearest region
            h3Region = nearest.regions[0].regionId;
        } else {
            // No nearby regions found - keep current
            APP_LOG(TS_ON, VLEVEL_M, 
                    "H3: No nearby regions found (%.4f, %.4f)\r\n", 
                    lat, lon);
            return MultiRegion_GetActiveRegion();
        }
    }
    
    // Convert h3lite region to LoRaMac region
    LoRaMacRegion_t loRaRegion = H3Region_ToLoRaMacRegion(h3Region);
    
    // Log the detection
    LogRegionDetection(getRegionName(h3Region), lat, lon, loRaRegion);
    
    return loRaRegion;
}

// MultiRegion_GetActiveRegion() now implemented in multiregion_context.c

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Log region detection result
 */
static void LogRegionDetection(const char* h3RegionName, float lat, float lon, LoRaMacRegion_t loraRegion)
{
    char logMsg[128];
    snprintf(logMsg, sizeof(logMsg), 
             "H3: Detected %s at (%.4f, %.4f) -> LoRa region %d\r\n",
             h3RegionName, lat, lon, loraRegion);
    APP_LOG(TS_ON, VLEVEL_M, logMsg);
}
