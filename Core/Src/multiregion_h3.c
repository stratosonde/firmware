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
#include "SEGGER_RTT.h"
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */  // For profiling output
#include "stm32wlxx_hal.h"  // For HAL_GetTick()
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
/* Maximum distance to consider for nearest neighbor. findNearestRegions()
 * reports ~120 km per ring at res 3 (corrected from a ~45%-low figure of
 * 65 km/ring), and this module searches at most 3 rings, so the largest
 * reported distance is ~360 km — comfortably under this gate. */
#define H3_MAX_DISTANCE_KM  500.0f

/* Private variables ---------------------------------------------------------*/
// Removed static currentRegion - now using MultiRegion_GetActiveRegion() from multiregion_context.c

/* Private function prototypes -----------------------------------------------*/
static void LogRegionDetection(const char* h3RegionName, float lat, float lon, LoRaMacRegion_t loraRegion);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Map H3Lite RegionId to LoRaMacRegion_t enum
 */
/* R51: lookup table instead of a strcmp ladder. Pairs with #43's
 * sub-group-as-data work (deferred post-flight). The _Static_assert pins the
 * entry count so an accidental edit/merge deletion fails at compile time;
 * the #46 host harness should additionally enumerate getRegionName() across
 * all RegionIds so an upstream h3lite rename fails loudly in tests instead
 * of silently falling through to "keep current region" in flight. */
typedef struct {
    const char *name;
    LoRaMacRegion_t region;
} H3RegionMapEntry_t;

static const H3RegionMapEntry_t h3_region_map[] = {
    { "US915",   LORAMAC_REGION_US915 },
    { "EU868",   LORAMAC_REGION_EU868 },
    { "AS923-1",  LORAMAC_REGION_AS923 },
    { "AS923-1B", LORAMAC_REGION_AS923 },
    { "AS923-1C", LORAMAC_REGION_AS923 },
    { "AS923-2",  LORAMAC_REGION_AS923 },
    { "AS923-3",  LORAMAC_REGION_AS923 },
    { "AS923-4",  LORAMAC_REGION_AS923 },
    { "AU915",   LORAMAC_REGION_AU915 },
    { "CN470",   LORAMAC_REGION_CN470 },
    { "KR920",   LORAMAC_REGION_KR920 },
    { "IN865",   LORAMAC_REGION_IN865 },
    { "RU864",   LORAMAC_REGION_RU864 },
    { "EU433",   LORAMAC_REGION_EU433 },
};

#define H3_REGION_MAP_COUNT  (sizeof(h3_region_map) / sizeof(h3_region_map[0]))
_Static_assert(H3_REGION_MAP_COUNT == 14,
               "h3_region_map drift: expected 14 name->region entries");

LoRaMacRegion_t H3Region_ToLoRaMacRegion(RegionId h3Region)
{
    const char* name = getRegionName(h3Region);

    for (uint32_t i = 0; i < H3_REGION_MAP_COUNT; i++) {
        if (strcmp(name, h3_region_map[i].name) == 0) {
            return h3_region_map[i].region;
        }
    }
    /* RESTRICTED (ID 15, repurposed from the CD900-1A test plan slot) is
     * handled upstream in lora_app.c, which blocks transmission before this
     * mapping is consulted; it intentionally has no LoRaMac region. */

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
    return MultiRegion_DetectFromH3Region(latLngToRegion((double)lat, (double)lon), lat, lon);
}

/**
 * @brief  F-R4 (#77): detection with a caller-resolved H3 region. The geofence
 *         resolution (latLngToRegion) happens ONCE per cycle in the caller;
 *         this applies the nearest-neighbour fallback, conversion, and logging.
 *         lat/lon are used only for the fallback search and log messages.
 */
LoRaMacRegion_t MultiRegion_DetectFromH3Region(RegionId h3Region, float lat, float lon)
{
    if (h3Region == 0) {
        // Not in any region - try nearest neighbor search
        NearestRegionsInfo nearest = findNearestRegions((double)lat, (double)lon, 3);
        
        if (nearest.numRegions > 0 && nearest.regions[0].distanceKm < H3_MAX_DISTANCE_KM) {
            /* FW-16: integer-only print (float printf support is not linked) */
            int32_t dist_d = (int32_t)(nearest.regions[0].distanceKm * 10.0f);
            APP_LOG(TS_ON, VLEVEL_M,
                    "H3: Outside regions, nearest: %s (%d.%d km)\r\n",
                    nearest.regions[0].regionName,
                    (int)(dist_d / 10), (int)((dist_d < 0 ? -dist_d : dist_d) % 10));
            
            // Use nearest region
            h3Region = nearest.regions[0].regionId;
        } else {
            // No nearby regions found - keep current
            /* FW-16: integer-only print (float printf support is not linked) */
            int32_t lat_m = (int32_t)(lat * 10000.0f);
            int32_t lon_m = (int32_t)(lon * 10000.0f);
            APP_LOG(TS_ON, VLEVEL_M,
                    "H3: No nearby regions found (%d.%04d, %d.%04d)\r\n",
                    (int)(lat_m / 10000), (int)((lat_m < 0 ? -lat_m : lat_m) % 10000),
                    (int)(lon_m / 10000), (int)((lon_m < 0 ? -lon_m : lon_m) % 10000));
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
    /* FW-16: integer-only print (float printf support is not linked) */
    int32_t lat_m = (int32_t)(lat * 10000.0f);
    int32_t lon_m = (int32_t)(lon * 10000.0f);
    snprintf(logMsg, sizeof(logMsg),
             "H3: Detected %s at (%d.%04d, %d.%04d) -> LoRa region %d\r\n",
             h3RegionName,
             (int)(lat_m / 10000), (int)((lat_m < 0 ? -lat_m : lat_m) % 10000),
             (int)(lon_m / 10000), (int)((lon_m < 0 ? -lon_m : lon_m) % 10000),
             loraRegion);
    APP_LOG(TS_ON, VLEVEL_M, logMsg);
}


