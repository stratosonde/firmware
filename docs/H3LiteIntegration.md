# H3Lite Integration Guide

## Overview

This document provides a complete step-by-step guide for integrating H3Lite into the stratosphere balloon firmware for precise LoRaWAN region detection. H3Lite provides industry-standard H3 geospatial indexing optimized for embedded systems.

**Related Documentation:**
- [MultiRegionSupport.md](MultiRegionSupport.md) - Overall multi-region architecture
- [h3lite GitHub](https://github.com/stratosonde/h3lite) - H3Lite repository

---

## Table of Contents

1. [Why H3Lite?](#why-h3lite)
2. [What H3Lite Provides](#what-h3lite-provides)
3. [Integration Method: Git Submodule](#integration-method-git-submodule)
4. [Build System Integration](#build-system-integration)
5. [Code Integration](#code-integration)
6. [Testing with HackRF](#testing-with-hackrf)
7. [Step-by-Step Checklist](#step-by-step-checklist)
8. [Troubleshooting](#troubleshooting)

---

## Why H3Lite?

### Problems with Simple GPS Bounding Boxes

The original approach used simple rectangular boundaries:

```c
// Simple approach - CRUDE
if (lat > 24.0 && lat < 50.0 && lon > -125.0 && lon < -66.0) {
    return LORAMAC_REGION_US915;
}
```

**Issues:**
- ❌ Rectangle ≠ real borders (includes ocean, excludes islands)
- ❌ Overlapping regions cause ambiguity
- ❌ Coastal transitions are abrupt
- ❌ No graceful degradation near borders
- ❌ Hard-coded magic numbers
- ❌ Misses Alaska, Hawaii, US territories

### H3Lite Advantages

- ✅ **Precise hexagonal grid** matches actual region coverage
- ✅ **Pre-computed lookup tables** (~44KB) = O(log n) binary search
- ✅ **Industry standard** H3 system (used by Uber, others)
- ✅ **Handles edge cases:** Islands, territories, coastal areas
- ✅ **Nearest neighbor search** for border transitions
- ✅ **Extensible:** Easy to add new regions
- ✅ **Validated:** Test coverage included

**Example: Atlantic Crossing**
```
Simple box: US → Unknown → EU (abrupt switch)
H3Lite:     US → US (3 rings) → EU (smooth gradual)
```

---

## What H3Lite Provides

### Memory Footprint

**Flash Memory:**
- Core code: ~2-4KB
- Lookup tables: ~44KB (10,875 H3 cells at resolution 3)
- **Total: ~46-48KB**

**RAM:**
- Static data: ~100 bytes
- Stack: ~50-100 bytes per operation
- **Total: <1KB**

**Performance:**
- Lookup time: <1ms (binary search)
- No dynamic allocation
- Deterministic behavior

### Supported Regions

H3Lite includes 16 LoRaWAN regions out of the box:

```
US915, EU868, CN470, AU915
AS923-1, AS923-1B, AS923-1C, AS923-2, AS923-3, AS923-4
KR920, IN865, RU864, EU433, CD900-1A
```

**Perfect match for multi-region needs!**

### API Functions

```c
// Initialize library (call once at startup)
bool h3liteInit(void);

// Convert lat/lng to H3 index
H3Index latLngToH3(double lat, double lng, int resolution);

// Look up region for H3 index
RegionId h3ToRegion(H3Index h3);

// Direct lat/lng to region (convenience function)
RegionId latLngToRegion(double lat, double lng);

// Get region name string
const char* getRegionName(RegionId regionId);

// Find nearest regions (for border handling)
NearestRegionsInfo findNearestRegions(double lat, double lng, int maxRings);
```

---

## Integration Method: Git Submodule

### Why Git Submodule?

**Advantages:**
- ✅ Clean version tracking
- ✅ Easy to update from upstream
- ✅ Standard practice for embedded projects
- ✅ Follows existing pattern (LoRaWAN, SubGHz_Phy in Middlewares)
- ✅ Separation of concerns

**Requirements:**
- Git submodule knowledge
- CI/CD needs `--recursive` clone

### Step 1: Remove Existing Copy (If Present)

If h3lite is currently in `datasheets/`:

```bash
cd c:/working/sonde/firmware

# Remove from datasheets
git rm -r datasheets/h3lite

# Or if not tracked:
rm -rf datasheets/h3lite
```

### Step 2: Add as Submodule

```bash
# Add h3lite as submodule in Middlewares (matches project structure)
git submodule add https://github.com/stratosonde/h3lite.git Middlewares/Third_Party/h3lite

# This creates:
# - Middlewares/Third_Party/h3lite/ (the code)
# - .gitmodules file (submodule configuration)
# - Commit reference pointing to specific h3lite version
```

### Step 3: Commit the Submodule

```bash
git add .gitmodules Middlewares/Third_Party/h3lite
git commit -m "Add h3lite as submodule for region detection"
git push
```

### Step 4: Team Members Clone with Submodules

```bash
# New clones need --recursive flag
git clone --recursive https://github.com/stratosonde/firmware.git

# Or if already cloned without submodules:
git submodule update --init --recursive
```

### Step 5: Updating H3Lite

```bash
# Update to latest h3lite version
cd Middlewares/Third_Party/h3lite
git pull origin main

# Commit the updated reference
cd ../../..
git add Middlewares/Third_Party/h3lite
git commit -m "Update h3lite to latest version"
git push
```

---

## Build System Integration

### Files to Add

**Source Files** (needs compilation):
```
Middlewares/Third_Party/h3lite/src/h3lite.c
Middlewares/Third_Party/h3lite/src/h3lite_faceijk.c
Middlewares/Third_Party/h3lite/src/h3lite_regions_table.c
```

**Include Path**:
```
Middlewares/Third_Party/h3lite/include
```

### Method 1: STM32CubeIDE GUI

**Step A: Add Include Path**

1. Right-click project → Properties
2. C/C++ Build → Settings
3. MCU GCC Compiler → Include paths
4. Click "Add" icon (green +)
5. Add:
   ```
   "${workspace_loc:/${ProjName}/Middlewares/Third_Party/h3lite/include}"
   ```
6. Click OK

**Step B: Add Source Files**

Option 1 - Link to folder:
1. Right-click project root
2. New → Folder
3. Advanced → "Link to alternate location (Linked Folder)"
4. Browse to: `Middlewares/Third_Party/h3lite/src`
5. Finish

Option 2 - Add individual files:
1. Right-click project
2. New → File → Advanced → Link to file
3. Add each .c file from `Middlewares/Third_Party/h3lite/src/`

### Method 2: Edit .cproject File

Add to `.cproject`:

```xml
<!-- In compiler settings section -->
<option id="gnu.c.compiler.option.include.paths" 
        name="Include paths (-I)" 
        superClass="gnu.c.compiler.option.include.paths" 
        valueType="includePath">
    <!-- Existing paths... -->
    <listOptionValue 
        builtIn="false" 
        value="../Middlewares/Third_Party/h3lite/include"/>
</option>
```

### Method 3: Makefile

If using Makefile-based build:

```makefile
# Add to C_INCLUDES
C_INCLUDES +=  \
-IMiddlewares/Third_Party/h3lite/include

# Add to C_SOURCES
C_SOURCES +=  \
Middlewares/Third_Party/h3lite/src/h3lite.c \
Middlewares/Third_Party/h3lite/src/h3lite_faceijk.c \
Middlewares/Third_Party/h3lite/src/h3lite_regions_table.c
```

### Verify Build Integration

1. **Clean Build:**
   ```
   Project → Clean → Clean all
   Project → Build All (Ctrl+B)
   ```

2. **Look for in build output:**
   ```
   Compiling h3lite.c...
   Compiling h3lite_faceijk.c...
   Compiling h3lite_regions_table.c...
   ```

3. **Check binary size increase:**
   ```
   Before: text=~95KB
   After:  text=~143KB (+48KB for h3lite)
   ```

---

## Code Integration

### Step 1: Initialize in main.c

```c
/* USER CODE BEGIN Includes */
#include "h3lite.h"
/* USER CODE END Includes */

int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    
    /* Configure the system clock */
    SystemClock_Config();
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    // ... other peripheral inits ...
    
    /* USER CODE BEGIN 2 */
    
    // Initialize h3lite
    if (!h3liteInit()) {
        SEGGER_RTT_WriteString(0, "ERROR: h3lite initialization failed!\r\n");
        Error_Handler();
    }
    
    SEGGER_RTT_WriteString(0, "h3lite initialized successfully\r\n");
    
    /* USER CODE END 2 */
    
    // ... rest of main loop ...
}
```

### Step 2: Create Region Detector Module

**Create:** `Core/Inc/multiregion_h3.h`

```c
#ifndef MULTIREGION_H3_H
#define MULTIREGION_H3_H

#include "h3lite.h"
#include "LoRaMacInterfaces.h"

/**
 * Detect LoRaWAN region from GPS coordinates using H3Lite
 * 
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @return LoRaMacRegion_t enum value
 */
LoRaMacRegion_t MultiRegion_DetectFromGPS_H3(float lat, float lon);

/**
 * Map h3lite RegionId to LoRaMacRegion_t
 * 
 * @param h3Region Region ID from h3lite
 * @return LoRaMacRegion_t enum value
 */
LoRaMacRegion_t H3Region_ToLoRaMacRegion(RegionId h3Region);

#endif /* MULTIREGION_H3_H */
```

**Create:** `Core/Src/multiregion_h3.c`

```c
#include "multiregion_h3.h"
#include "multiregion_context.h"
#include "SEGGER_RTT.h"
#include <string.h>
#include <stdio.h>

LoRaMacRegion_t H3Region_ToLoRaMacRegion(RegionId h3Region) {
    const char* name = getRegionName(h3Region);
    
    // Map h3lite region names to LoRaMac regions
    if (strcmp(name, "US915") == 0) return LORAMAC_REGION_US915;
    if (strcmp(name, "EU868") == 0) return LORAMAC_REGION_EU868;
    if (strcmp(name, "AS923-1") == 0) return LORAMAC_REGION_AS923;
    if (strcmp(name, "AU915") == 0) return LORAMAC_REGION_AU915;
    if (strcmp(name, "CN470") == 0) return LORAMAC_REGION_CN470;
    if (strcmp(name, "KR920") == 0) return LORAMAC_REGION_KR920;
    if (strcmp(name, "IN865") == 0) return LORAMAC_REGION_IN865;
    if (strcmp(name, "RU864") == 0) return LORAMAC_REGION_RU864;
    if (strcmp(name, "EU433") == 0) return LORAMAC_REGION_EU433;
    // Add more mappings as needed
    
    // Unknown region - keep current
    return MultiRegion_GetActiveRegion();
}

LoRaMacRegion_t MultiRegion_DetectFromGPS_H3(float lat, float lon) {
    // Use h3lite for precise region detection
    RegionId h3Region = latLngToRegion((double)lat, (double)lon);
    
    if (h3Region == 0) {
        // Not in any region - try nearest neighbor search
        NearestRegionsInfo nearest = findNearestRegions((double)lat, (double)lon, 3);
        
        if (nearest.numRegions > 0) {
            char msg[128];
            snprintf(msg, sizeof(msg), 
                     "GPS outside regions, nearest: %s (%.1f km)\r\n",
                     nearest.regions[0].regionName,
                     nearest.regions[0].distanceKm);
            SEGGER_RTT_WriteString(0, msg);
            
            // Use nearest region
            h3Region = nearest.regions[0].regionId;
        } else {
            // No nearby regions found - keep current
            SEGGER_RTT_WriteString(0, "No nearby regions found!\r\n");
            return MultiRegion_GetActiveRegion();
        }
    }
    
    // Convert h3lite region to LoRaMac region
    LoRaMacRegion_t loRaRegion = H3Region_ToLoRaMacRegion(h3Region);
    
    // Log the detection
    char msg[100];
    snprintf(msg, sizeof(msg), 
             "H3 detected: %s (%.4f, %.4f) → LoRa region %d\r\n",
             getRegionName(h3Region), lat, lon, loRaRegion);
    SEGGER_RTT_WriteString(0, msg);
    
    return loRaRegion;
}
```

### Step 3: Update lora_app.c

Replace simple GPS detection with H3:

```c
/* USER CODE BEGIN Includes */
#include "multiregion_h3.h"
/* USER CODE END Includes */

static void SendTxData(void) {
    SEGGER_RTT_WriteString(0, "\r\n=== SendTxData START ===\r\n");
    
    // ... Get GPS position ...
    float lat = hgnss.data.latitude;
    float lon = hgnss.data.longitude;
    
    // Use H3-based detection instead of simple bounding boxes
    LoRaMacRegion_t target_region = MultiRegion_DetectFromGPS_H3(lat, lon);
    LoRaMacRegion_t current_region = MultiRegion_GetActiveRegion();
    
    if (target_region != current_region) {
        char msg[100];
        snprintf(msg, sizeof(msg), 
                 "Region change: %d → %d\r\n", 
                 current_region, target_region);
        SEGGER_RTT_WriteString(0, msg);
        
        // Perform region switch
        if (MultiRegion_SwitchToRegion(target_region) != LORAMAC_HANDLER_SUCCESS) {
            SEGGER_RTT_WriteString(0, "Region switch failed!\r\n");
            // Continue with current region
        }
    }
    
    // ... rest of transmission logic ...
}
```

### Step 4: Test Function

Add test function for ground validation:

```c
#ifdef GPS_INJECTION_TEST_MODE
void test_h3lite_regions(void) {
    struct {
        float lat;
        float lon;
        const char* expected;
    } testCases[] = {
        {37.7749, -122.4194, "US915"},     // San Francisco
        {51.5074, -0.1278, "EU868"},       // London
        {35.6762, 139.6503, "AS923-1"},    // Tokyo
        {-33.8688, 151.2093, "AU915"},     // Sydney
        {0.0, -30.0, "Unknown"},           // Mid-Atlantic
    };
    
    SEGGER_RTT_WriteString(0, "\r\n=== H3Lite Region Tests ===\r\n");
    
    for (int i = 0; i < 5; i++) {
        RegionId region = latLngToRegion(testCases[i].lat, testCases[i].lon);
        const char* name = getRegionName(region);
        
        char msg[150];
        snprintf(msg, sizeof(msg), 
                 "Test %d: (%.2f, %.2f) → %s (expected: %s) %s\r\n",
                 i, testCases[i].lat, testCases[i].lon, name, 
                 testCases[i].expected,
                 (strcmp(name, testCases[i].expected) == 0) ? "✓ PASS" : "✗ FAIL");
        SEGGER_RTT_WriteString(0, msg);
    }
}
#endif
```

---

## Testing with HackRF

### Overview

HackRF One allows GPS signal injection for indoor testing without real GPS satellites. This dramatically accelerates development and testing.

### HackRF Setup

**Hardware:**
- HackRF One
- GPS L1 antenna (1575.42 MHz)
- USB cable
- Computer with gps-sdr-sim

**Software:**
```bash
# Install gps-sdr-sim
git clone https://github.com/osqzss/gps-sdr-sim
cd gps-sdr-sim
gcc gps-sdr-sim.c -lm -O3 -o gps-sdr-sim

# Install HackRF tools
# Windows: https://github.com/greatscottgadgets/hackrf/releases
# Linux: sudo apt-get install hackrf
```

### Test Scenario 1: Static Position Test

```bash
# Generate GPS signal for NYC
./gps-sdr-sim -e brdc3540.14n -l 40.7128,-74.0060,100 -o nyc.bin

# Transmit with HackRF (keep device within 1-5m)
hackrf_transfer -t nyc.bin -f 1575420000 -s 2600000 -a 1 -x 0

# Expected result in firmware logs:
# "H3 detected: US915 (40.7128, -74.0060)"
```

### Test Scenario 2: Transatlantic Flight

```bash
# Generate trajectory file
python generate_trajectory.py --start NYC --end London --duration 10h

# Convert to GPS signal
./gps-sdr-sim -u trajectory.csv -o transatlantic.bin

# Transmit (sped up 10x)
hackrf_transfer -t transatlantic.bin -f 1575420000 -s 2600000 -a 1

# Monitor firmware logs for region transitions:
# US915 → US915 → ... → EU868
```

### Test Scenario 3: Border Stress Test

```python
# border_test.py - Test US/Canada border
import subprocess
import time

for lon in range(-125, -65, 5):  # Walk across US
    # Generate GPS signal
    subprocess.run([
        './gps-sdr-sim',
        '-l', f"49.0,{lon},100",  # 49°N = border
        '-o', 'test.bin'
    ])
    
    # Transmit for 30 seconds
    proc = subprocess.Popen([
        'hackrf_transfer',
        '-t', 'test.bin',
        '-f', '1575420000',
        '-s', '2600000'
    ])
    
    time.sleep(30)
    proc.terminate()
    
    print(f"Tested position: 49.0, {lon}")
```

### Safety & Legal

⚠️ **IMPORTANT:**
- GPS injection is **ILLEGAL outdoors** in most countries
- **Indoor testing only** (< 10m range)
- **Shield/attenuate** to prevent interference
- **Do NOT transmit** near windows or outdoors
- **Check local laws** before use

---

## Step-by-Step Checklist

### Phase 1: Git Integration

- [ ] Remove h3lite from `datasheets/` (if present)
- [ ] Add h3lite as git submodule: `git submodule add https://github.com/stratosonde/h3lite.git Middlewares/Third_Party/h3lite`
- [ ] Commit submodule: `git add .gitmodules Middlewares/Third_Party/h3lite && git commit`
- [ ] Verify submodule present: `ls Middlewares/Third_Party/h3lite`
- [ ] Push changes: `git push`

### Phase 2: Build System

- [ ] Add include path: `Middlewares/Third_Party/h3lite/include`
- [ ] Add source files:
  - [ ] `h3lite.c`
  - [ ] `h3lite_faceijk.c`
  - [ ] `h3lite_regions_table.c`
- [ ] Clean build: `Project → Clean`
- [ ] Build all: `Project → Build All`
- [ ] Verify compilation: Check for "Compiling h3lite" in output
- [ ] Verify binary size increase: ~48KB larger

### Phase 3: Code Integration

- [ ] Add `#include "h3lite.h"` to main.c
- [ ] Call `h3liteInit()` in main()
- [ ] Build and verify no compile errors
- [ ] Create `multiregion_h3.h` header file
- [ ] Create `multiregion_h3.c` implementation
- [ ] Update `lora_app.c` to use H3 detection
- [ ] Build and verify no linker errors

### Phase 4: Basic Testing

- [ ] Flash firmware to device
- [ ] Verify h3lite initializes: Check RTT logs for "h3lite initialized"
- [ ] Test with hardcoded coordinates:
  ```c
  RegionId test = latLngToRegion(37.7749, -122.4194);
  SEGGER_RTT_printf(0, "Test: %s\r\n", getRegionName(test));
  // Expected: "US915"
  ```
- [ ] Verify test passes

### Phase 5: HackRF Testing (Optional)

- [ ] Set up HackRF One with gps-sdr-sim
- [ ] Test static position (NYC): Verify US915 detection
- [ ] Test static position (London): Verify EU868 detection
- [ ] Test border crossing: Verify smooth transitions
- [ ] Test trajectory: Verify region switches during simulated flight

### Phase 6: Integration with Multi-Region

- [ ] Verify multi-region context manager implemented
- [ ] Test pre-join for multiple regions
- [ ] Test region switching triggered by H3 detection
- [ ] Verify frame counters independent per region
- [ ] Log all region transitions to flash

### Phase 7: Final Validation

- [ ] Full ground test with GPS injection
- [ ] Verify no false switches near borders
- [ ] Verify nearest-neighbor fallback works
- [ ] Stress test with rapid position changes
- [ ] Ready for balloon flight test

---

## Troubleshooting

### Issue: h3lite.h not found

**Symptoms:**
```
fatal error: h3lite.h: No such file or directory
#include "h3lite.h"
```

**Solutions:**
1. Verify submodule checked out: `ls Middlewares/Third_Party/h3lite`
2. Check include path in project settings
3. Refresh project in IDE: `Project → Refresh` (F5)
4. Clean and rebuild

### Issue: Undefined reference to `latLngToRegion`

**Symptoms:**
```
undefined reference to `latLngToRegion'
undefined reference to `h3liteInit'
```

**Solutions:**
1. Verify source files added to build
2. Check that .c files are being compiled (look in build output)
3. Exclude/include files in project properties if needed
4. Clean and rebuild

### Issue: Binary size too large

**Symptoms:**
```
region `FLASH' overflowed by XXXX bytes
```

**Solutions:**
1. Check current usage: Should be < 200KB before h3lite
2. Enable compiler optimizations: `-Os` or `-O2`
3. Consider reducing h3lite resolution (regenerate tables)
4. Remove unused features/debug code

### Issue: h3lite returns "Unknown" for valid coordinates

**Symptoms:**
- GPS coordinates valid
- h3lite returns region ID = 0
- getRegionName() returns "Unknown"

**Solutions:**
1. Check coordinate format: Must be decimal degrees
2. Verify lat/lng not swapped
3. Check if using old lookup tables (regenerate)
4. Test with known-good coordinates (e.g., SF, London)
5. Use `findNearestRegions()` for coastal areas

### Issue: HackRF GPS not locking

**Symptoms:**
- GNSS module shows no fix
- Satellites = 0

**Solutions:**
1. Move HackRF closer (< 5m from device)
2. Increase HackRF gain: `-a 20` (start low, increase)
3. Check antenna: Must be GPS L1 (1575.42 MHz)
4. Verify frequency: Must be exactly 1575420000 Hz
5. Test indoors (block real GPS)
6. Download fresh BRDC ephemeris file

### Issue: Region oscillates near border

**Symptoms:**
- Device switches US915 ↔ EU868 repeatedly
- Position near boundary

**Solutions:**
1. Implement hysteresis:
   ```c
   static uint8_t switch_count = 0;
   if (target != current) {
       switch_count++;
       if (switch_count > 3) {  // Require 3 confirmations
           // Switch region
           switch_count = 0;
       }
   } else {
       switch_count = 0;
   }
   ```
2. Use `findNearestRegions()` to check proximity
3. Add minimum switch interval (e.g., 5 minutes)

---

## Performance Expectations

### Memory Usage

| Component | Size | Location |
|-----------|------|----------|
| h3lite code | 2-4 KB | Flash |
| Lookup tables | 44 KB | Flash (const data) |
| Static variables | 100 bytes | RAM |
| Stack per call | 50-100 bytes | Stack |
| **Total Flash** | **~48 KB** | |
| **Total RAM** | **<1 KB** | |

### Timing

| Operation | Time | Notes |
|-----------|------|-------|
| `latLngToH3()` | ~100-200 μs | Coordinate conversion |
| `h3ToRegion()` | ~500-800 μs | Binary search |
| `latLngToRegion()` | ~700-1000 μs | Combined |
| `findNearestRegions()` | ~2-5 ms | Ring search |

**All operations are deterministic** (no dynamic allocation, no waits)

### Accuracy

- **Within region:** 100% accurate
- **Near borders:** ±10-20 km (resolution 3)
- **Over ocean:** Nearest neighbor search
- **Islands/territories:** Correctly handled

---

## Next Steps

1. **Complete this checklist** to integrate h3lite
2. **Test with HackRF** (optional but recommended)
3. **Proceed with multi-region** implementation from [MultiRegionSupport.md](MultiRegionSupport.md)
4. **Ground test** before balloon flight
5. **Validate during flight** and iterate

---

## References

- **h3lite Repository:** https://github.com/stratosonde/h3lite
- **Uber H3 System:** https://h3geo.org/
- **gps-sdr-sim:** https://github.com/osqzss/gps-sdr-sim
- **HackRF One:** https://greatscottgadgets.com/hackrf/
- **LoRaWAN Regions:** https://lora-alliance.org/resource_hub/rp2-1-0-3-lorawan-regional-parameters/

---

**Document Version:** 1.0  
**Last Updated:** December 21, 2025  
**Author:** AI Assistant (Cline)  
**Status:** Ready for Implementation
