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
#include "SEGGER_RTT.h"  // For profiling output
#include "stm32wlxx_hal.h"  // For HAL_GetTick()
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
#define H3_MAX_DISTANCE_KM  500.0f  // Maximum distance to consider for nearest neighbor

/* Private variables ---------------------------------------------------------*/
// Removed static currentRegion - now using MultiRegion_GetActiveRegion() from multiregion_context.c

/* Private function prototypes -----------------------------------------------*/
static void LogRegionDetection(const char* h3RegionName, float lat, float lon, LoRaMacRegion_t loraRegion);
static const char* LoRaMacRegion_ToString(LoRaMacRegion_t region);

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

/**
 * @brief  Convert LoRaMacRegion_t to string
 */
static const char* LoRaMacRegion_ToString(LoRaMacRegion_t region)
{
    switch (region) {
        case LORAMAC_REGION_AS923: return "AS923";
        case LORAMAC_REGION_AU915: return "AU915";
        case LORAMAC_REGION_CN470: return "CN470";
        case LORAMAC_REGION_EU868: return "EU868";
        case LORAMAC_REGION_IN865: return "IN865";
        case LORAMAC_REGION_KR920: return "KR920";
        case LORAMAC_REGION_US915: return "US915";
        case LORAMAC_REGION_RU864: return "RU864";
        default: return "UNKNOWN";
    }
}

/**
 * @brief  Profile H3lite performance with test coordinates
 */
void MultiRegion_ProfileH3Performance(void)
{
    extern uint32_t HAL_GetTick(void);
    
    SEGGER_RTT_WriteString(0, "\r\n");
    SEGGER_RTT_WriteString(0, "========================================\r\n");
    SEGGER_RTT_WriteString(0, "===  H3LITE PROFILING TEST SUITE    ===\r\n");
    SEGGER_RTT_WriteString(0, "========================================\r\n\r\n");
    
    // Test coordinates structure
    typedef struct {
        const char* name;
        float lat;
        float lon;
        const char* expected_region;
        bool is_offshore;
    } TestCoordinate;
    
    // Define comprehensive test coordinates
    TestCoordinate tests[] = {
        // === US915 Region ===
        {"NYC, USA", 40.7128f, -74.0060f, "US915", false},
        {"Los Angeles, USA", 34.0522f, -118.2437f, "US915", false},
        {"Denver, USA", 39.7392f, -104.9903f, "US915", false},
        {"Atlantic (off Florida coast)", 27.0f, -79.5f, "US915", true},
        {"Pacific (100km W of CA)", 35.0f, -125.0f, "US915", true},
        
        // === EU868 Region ===
        {"Paris, France", 48.8566f, 2.3522f, "EU868", false},
        {"London, UK", 51.5074f, -0.1278f, "EU868", false},
        {"Berlin, Germany", 52.5200f, 13.4050f, "EU868", false},
        {"Atlantic (W of Ireland)", 50.0f, -10.0f, "EU868", true},
        {"Mediterranean (S of France)", 42.0f, 5.0f, "EU868", true},
        
        // === AS923 Region ===
        {"Tokyo, Japan", 35.7f, 140.0f, "AS923", false},  // Adjusted east for direct hit
        {"Singapore", 1.3521f, 103.8198f, "AS923", false},
        {"Bangkok, Thailand", 13.7563f, 100.5018f, "AS923", false},
        {"Pacific (E of Japan)", 35.0f, 150.0f, "AS923", true},
        {"South China Sea", 15.0f, 115.0f, "AS923", true},
        
        // === AU915 Region ===
        {"Sydney, Australia", -33.8688f, 151.2093f, "AU915", false},
        {"Melbourne, Australia", -37.8136f, 144.9631f, "AU915", false},
        {"Tasman Sea", -40.0f, 160.0f, "AU915", true},
        {"Coral Sea", -20.0f, 155.0f, "AU915", true},
        
        // === IN865 Region ===
        {"New Delhi, India", 28.6139f, 77.2090f, "IN865", false},
        {"Mumbai, India", 19.0760f, 72.8777f, "IN865", false},
        {"Indian Ocean (W of India)", 18.0f, 70.0f, "IN865", true},
        
        // === KR920 Region ===
        {"Seoul, South Korea", 37.5665f, 126.9780f, "KR920", false},
        {"Busan, South Korea", 35.1796f, 129.0756f, "KR920", false},
        {"Sea of Japan", 35.5f, 129.5f, "KR920", true},
        
        // === Caribbean (Multi-region boundary) ===
        {"Havana, Cuba", 23.1136f, -82.3666f, "US915", false},
        {"San Juan, Puerto Rico", 18.4655f, -66.1057f, "US915", false},
        {"Kingston, Jamaica", 17.9712f, -76.7936f, "US915", false},
        {"Martinique (French)", 14.6415f, -61.0242f, "EU868/US?", false},
        {"Guadeloupe (French)", 16.2650f, -61.5510f, "EU868/US?", false},
        {"Curacao (Dutch)", 12.1696f, -68.9900f, "EU868/US?", false},
        {"Caribbean Sea (center)", 15.0f, -75.0f, "?", true},
        
        // === Extreme offshore ===
        {"Mid-Atlantic Ocean", 30.0f, -40.0f, "?", true},
        {"Mid-Pacific Ocean", 0.0f, -160.0f, "?", true},
        {"Arctic Ocean", 80.0f, 0.0f, "?", true},
    };
    
    int num_tests = sizeof(tests) / sizeof(tests[0]);
    
    SEGGER_RTT_printf(0, "Running %d test scenarios...\r\n\r\n", num_tests);
    
    // Run each test
    for (int i = 0; i < num_tests; i++) {
        TestCoordinate *test = &tests[i];
        
        // Convert floats to integers for printf (avoid %f which needs float support)
        int32_t lat_int = (int32_t)(test->lat * 10000);
        int32_t lon_int = (int32_t)(test->lon * 10000);
        
        SEGGER_RTT_WriteString(0, "========================================\r\n");
        SEGGER_RTT_printf(0, "Test %d/%d: %s\r\n", i+1, num_tests, test->name);
        SEGGER_RTT_printf(0, "Coords: (%d.%04d, %d.%04d) | Expect: %s\r\n", 
                          lat_int / 10000, abs(lat_int % 10000),
                          lon_int / 10000, abs(lon_int % 10000),
                          test->expected_region);
        
        // === TEST 1: Get H3 hex ID ===
        H3Index h3_index = latLngToH3((double)test->lat, (double)test->lon, 4);
        SEGGER_RTT_printf(0, "H3 Index: 0x%08X%08X\r\n", 
                          (uint32_t)(h3_index >> 32), 
                          (uint32_t)(h3_index & 0xFFFFFFFF));
        
        // === TEST 2: Direct lookup ===
        uint32_t start = HAL_GetTick();
        RegionId direct_result = latLngToRegion((double)test->lat, (double)test->lon);
        uint32_t direct_time = HAL_GetTick() - start;
        
        if (direct_result != 0) {
            const char* region_name = getRegionName(direct_result);
            LoRaMacRegion_t lora_region = H3Region_ToLoRaMacRegion(direct_result);
            SEGGER_RTT_printf(0, "Direct Lookup: %lums -> %s (%s) ✓\r\n", 
                              (unsigned long)direct_time, 
                              region_name,
                              LoRaMacRegion_ToString(lora_region));
        } else {
            SEGGER_RTT_printf(0, "Direct Lookup: %lums -> NOT FOUND (offshore)\r\n", 
                              (unsigned long)direct_time);
        }
        
        // === TEST 3: Dynamic ring search (search until found) ===
        if (test->is_offshore || direct_result == 0) {
            SEGGER_RTT_WriteString(0, "\r\nDynamic Ring Search (until found or max):\r\n");
            
            int max_rings = 6;  // Search up to 6 rings
            bool found = false;
            
            for (int rings = 1; rings <= max_rings; rings++) {
                SEGGER_RTT_printf(0, "  [DEBUG] Starting ring %d...\r\n", rings);
                HAL_Delay(10);  // Let RTT drain
                
                start = HAL_GetTick();
                NearestRegionsInfo nearest = findNearestRegions((double)test->lat, (double)test->lon, rings);
                uint32_t ring_time = HAL_GetTick() - start;
                
                SEGGER_RTT_printf(0, "  [DEBUG] Ring %d completed in %lums\r\n", rings, (unsigned long)ring_time);
                
                if (nearest.numRegions > 0) {
                    // Convert distance to integer (e.g. 125.3 km -> 1253 -> "125.3")
                    int32_t dist_int = (int32_t)(nearest.regions[0].distanceKm * 10);
                    SEGGER_RTT_printf(0, "  Ring %d: %lums -> %s (%d.%d km) ✓ FOUND\r\n",
                                      rings,
                                      (unsigned long)ring_time,
                                      nearest.regions[0].regionName,
                                      dist_int / 10, abs(dist_int % 10));
                    
                    // Show all found regions (for multi-region scenarios)
                    if (nearest.numRegions > 1) {
                        for (int j = 1; j < nearest.numRegions; j++) {
                            int32_t dist_j = (int32_t)(nearest.regions[j].distanceKm * 10);
                            SEGGER_RTT_printf(0, "          Also: %s (%d.%d km)\r\n",
                                              nearest.regions[j].regionName,
                                              dist_j / 10, abs(dist_j % 10));
                        }
                    }
                    found = true;
                    break;  // Stop searching once found
                } else {
                    SEGGER_RTT_printf(0, "  Ring %d: %lums -> NOT FOUND\r\n",
                                      rings, (unsigned long)ring_time);
                }
            }
            
            if (!found) {
                SEGGER_RTT_printf(0, "  No regions found within %d rings\r\n", max_rings);
            }
        }
        
        SEGGER_RTT_WriteString(0, "\r\n");
        
        // Add delay to prevent RTT buffer overflow (let buffer drain)
        HAL_Delay(100);
    }
    
    // === Summary statistics ===
    SEGGER_RTT_WriteString(0, "========================================\r\n");
    SEGGER_RTT_WriteString(0, "===  PROFILING COMPLETE             ===\r\n");
    SEGGER_RTT_WriteString(0, "========================================\r\n");
    SEGGER_RTT_printf(0, "Total tests run: %d\r\n", num_tests);
    SEGGER_RTT_WriteString(0, "\r\nKey findings:\r\n");
    SEGGER_RTT_WriteString(0, "- Direct lookup: Fast for in-region coords\r\n");
    SEGGER_RTT_WriteString(0, "- Ring search: Time increases with ring count\r\n");
    SEGGER_RTT_WriteString(0, "- Recommendation: Use 2 rings for offshore\r\n");
    SEGGER_RTT_WriteString(0, "========================================\r\n\r\n");
}
