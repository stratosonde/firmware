# Multi-Region LoRaWAN Support

## Overview

This document describes the design and implementation of multi-region LoRaWAN support for the stratosphere balloon project. The system enables pre-joining multiple LoRaWAN regions (US915, EU868, etc.) on the ground and seamlessly switching between them during flight based on GPS coordinates.

**Key Benefits:**
- ✅ No in-flight joins required (critical for balloon operations)
- ✅ Seamless region switching based on GPS location
- ✅ Minimal flash footprint (~300 bytes for 4 regions)
- ✅ Works with ChirpStack multi-region support
- ✅ Session keys and frame counters preserved per region

## Architecture

### Minimal Context Structure

Instead of storing the full `LoRaMacNvmData_t` (2-4KB per region), we use a minimal 75-byte session context:

```c
typedef struct {
    LoRaMacRegion_t region;              // 1 byte - US915, EU868, etc.
    ActivationType_t activation;          // 1 byte - OTAA/ABP
    uint32_t dev_addr;                    // 4 bytes - Device address
    
    // Session keys (32 bytes total)
    uint8_t app_s_key[16];               // 16 bytes - Application session key
    uint8_t nwk_s_key[16];               // 16 bytes - Network session key
    
    // Frame counters (critical for security)
    uint32_t uplink_counter;             // 4 bytes
    uint32_t downlink_counter;           // 4 bytes
    
    // Minimal state
    uint32_t last_rx_mic;                // 4 bytes - Replay protection
    uint8_t datarate;                    // 1 byte
    int8_t tx_power;                     // 1 byte
    bool adr_enabled;                    // 1 byte
    
    // RX2 window params
    uint32_t rx2_frequency;              // 4 bytes
    uint8_t rx2_datarate;                // 1 byte
    
    // Metadata
    uint32_t last_used;                  // 4 bytes - LRU timestamp
    uint16_t crc16;                      // 2 bytes - Validation
    
} __attribute__((packed)) MinimalRegionContext_t;
```

**Total per region: 75 bytes**

### Multi-Region Storage Manager

```c
#define MAX_REGION_CONTEXTS 4  // US915, EU868, AS923, future/satellite

typedef struct {
    uint32_t magic;                      // 0xDEADBEEF - Validity check
    uint8_t active_slot;                 // Currently active context index
    uint8_t num_valid;                   // How many contexts are joined
    uint16_t version;                    // For future compatibility
    
    MinimalRegionContext_t contexts[MAX_REGION_CONTEXTS];
    
    uint32_t crc32;                      // Whole structure validation
} MultiRegionStorage_t;
```

**Total storage: 312 bytes** (fits in single 2KB flash page)

### Flash Memory Layout

```
STM32WLE5 Flash Organization:
┌────────────────────────────────────┐
│  App Code/Data (0x0800_0000)      │
├────────────────────────────────────┤
│  ...                                │
├────────────────────────────────────┤
│  Multi-Region Contexts             │
│  (0x0803_F800 - 2KB page)          │
│  ┌──────────────────────────────┐  │
│  │ Magic + Metadata (12 bytes)  │  │
│  ├──────────────────────────────┤  │
│  │ Slot 0: US915 (75 bytes)     │  │
│  ├──────────────────────────────┤  │
│  │ Slot 1: EU868 (75 bytes)     │  │
│  ├──────────────────────────────┤  │
│  │ Slot 2: AS923 (75 bytes)     │  │
│  ├──────────────────────────────┤  │
│  │ Slot 3: Future (75 bytes)    │  │
│  ├──────────────────────────────┤  │
│  │ Unused (1.7KB)               │  │
│  └──────────────────────────────┘  │
└────────────────────────────────────┘
```

## API Design

### Core Functions

```c
/* Initialize the multi-region manager */
void MultiRegion_Init(void);

/* Join a specific region and store context */
LmHandlerErrorStatus_t MultiRegion_JoinRegion(LoRaMacRegion_t region);

/* Switch to a different region context */
LmHandlerErrorStatus_t MultiRegion_SwitchToRegion(LoRaMacRegion_t region);

/* Get current active region */
LoRaMacRegion_t MultiRegion_GetActiveRegion(void);

/* Determine best region based on GPS coordinates */
LoRaMacRegion_t MultiRegion_DetectRegionFromGPS(float latitude, float longitude);

/* Save all contexts to flash */
void MultiRegion_SaveAllContexts(void);

/* Restore contexts from flash on boot */
bool MultiRegion_RestoreContexts(void);

/* Check if a region has a valid joined context */
bool MultiRegion_IsRegionJoined(LoRaMacRegion_t region);

/* Auto-switch based on GPS location */
void MultiRegion_AutoSwitchForLocation(float lat, float lon);
```

## Implementation Details

### Region Detection Logic

```c
LoRaMacRegion_t MultiRegion_DetectRegionFromGPS(float lat, float lon) {
    // North America: US915
    // Rough bounding box for US/Canada/Mexico
    if (lat > 24.0 && lat < 50.0 && lon > -125.0 && lon < -66.0) {
        return LORAMAC_REGION_US915;
    }
    
    // Europe: EU868
    // Rough bounding box for Europe
    if (lat > 36.0 && lat < 71.0 && lon > -10.0 && lon < 40.0) {
        return LORAMAC_REGION_EU868;
    }
    
    // Over Atlantic or unknown: keep current region
    // Don't switch mid-ocean to avoid unnecessary transitions
    return MultiRegion_GetActiveRegion();
}
```

### Context Switching Flow

```
1. GPS detects new region from coordinates
2. Check if target region has joined context
   ├─ No context → Stay on current region
   └─ Has context → Proceed to switch
3. Verify MAC layer is idle (not mid-transmission)
4. Halt current MAC layer
5. Save current context to its flash slot
6. Load target region context from flash
7. Reconfigure MAC layer for new region
   ├─ Load region defaults (channels, params)
   ├─ Inject saved session keys via MIB
   └─ Restore frame counters to crypto module
8. Start MAC layer with new configuration
9. Transmission proceeds on new region
```

### Critical: Frame Counter Restoration

Frame counters are security-critical and buried in internal structures. Must be restored correctly:

```c
LmHandlerErrorStatus_t MultiRegion_SwitchToRegion(LoRaMacRegion_t target) {
    MinimalRegionContext_t *ctx = MultiRegion_GetContext(target);
    
    if (!ctx || ctx->region != target) {
        return LORAMAC_HANDLER_ERROR;  // Not joined
    }
    
    // CRITICAL: Only switch when MAC is idle
    if (LoRaMacIsBusy()) {
        return LORAMAC_HANDLER_BUSY_ERROR;  // Retry later
    }
    
    // 1. Halt current MAC
    LmHandlerHalt();
    HAL_Delay(100);  // Let radio settle
    
    // 2. Save current context
    MultiRegion_SaveActiveContext();
    
    // 3. Reconfigure for new region with defaults
    LmHandlerParams.ActiveRegion = target;
    LmHandlerConfigure(&LmHandlerParams);  // Loads region defaults
    
    // 4. Inject saved session data via MIB
    MibRequestConfirm_t mib;
    
    // Access internal NVM context
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&mib);
    LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t*)mib.Param.Contexts;
    
    // Restore session keys
    memcpy(nvm->SecureElement.KeyList[APP_S_KEY], ctx->app_s_key, 16);
    memcpy(nvm->SecureElement.KeyList[NWK_S_KEY], ctx->nwk_s_key, 16);
    
    // Restore DevAddr
    nvm->MacGroup2.DevAddr = ctx->dev_addr;
    
    // CRITICAL: Restore frame counters
    nvm->Crypto.FCntList.FCntUp = ctx->uplink_counter;
    nvm->Crypto.FCntList.NFCntDown = ctx->downlink_counter;
    nvm->MacGroup1.LastRxMic = ctx->last_rx_mic;
    
    // Mark as activated
    nvm->MacGroup2.NetworkActivation = ACTIVATION_TYPE_ABP;  // Treat as ABP
    
    // Write back modified context
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibSetRequestConfirm(&mib);
    
    // 5. Start MAC
    LoRaMacStart();
    
    return LORAMAC_HANDLER_SUCCESS;
}
```

### Pre-Join Sequence (Ground Operations)

Before flight, join all required regions:

```c
void LoRaWAN_PreJoinAllRegions(void) {
    SEGGER_RTT_WriteString(0, "Starting multi-region pre-join sequence...\r\n");
    
    // Join US915
    SEGGER_RTT_WriteString(0, "Joining US915...\r\n");
    if (MultiRegion_JoinRegion(LORAMAC_REGION_US915) == LORAMAC_HANDLER_SUCCESS) {
        SEGGER_RTT_WriteString(0, "US915 joined successfully!\r\n");
    } else {
        SEGGER_RTT_WriteString(0, "US915 join failed!\r\n");
        return;
    }
    HAL_Delay(5000);  // Wait between joins
    
    // Join EU868
    SEGGER_RTT_WriteString(0, "Joining EU868...\r\n");
    if (MultiRegion_JoinRegion(LORAMAC_REGION_EU868) == LORAMAC_HANDLER_SUCCESS) {
        SEGGER_RTT_WriteString(0, "EU868 joined successfully!\r\n");
    } else {
        SEGGER_RTT_WriteString(0, "EU868 join failed!\r\n");
    }
    HAL_Delay(5000);
    
    // Save all contexts to flash
    MultiRegion_SaveAllContexts();
    SEGGER_RTT_WriteString(0, "All regions pre-joined and saved!\r\n");
    
    // Switch back to starting region (US915 for US launches)
    MultiRegion_SwitchToRegion(LORAMAC_REGION_US915);
}
```

### Integration with Transmission Logic

Modify `SendTxData()` in `lora_app.c`:

```c
static void SendTxData(void) {
    SEGGER_RTT_WriteString(0, "\r\n=== SendTxData START ===\r\n");
    
    // Get current GPS position
    float lat = hgnss.data.latitude;
    float lon = hgnss.data.longitude;
    
    // Auto-detect and switch region if needed
    LoRaMacRegion_t target_region = MultiRegion_DetectRegionFromGPS(lat, lon);
    LoRaMacRegion_t current_region = MultiRegion_GetActiveRegion();
    
    if (target_region != current_region) {
        char msg[100];
        snprintf(msg, sizeof(msg), "Switching from region %d to %d\r\n", 
                 current_region, target_region);
        SEGGER_RTT_WriteString(0, msg);
        
        if (MultiRegion_SwitchToRegion(target_region) != LORAMAC_HANDLER_SUCCESS) {
            SEGGER_RTT_WriteString(0, "Region switch failed!\r\n");
            // Continue with current region
        }
    }
    
    // Proceed with normal transmission
    // ... existing sensor reading and transmission code ...
}
```

## File Structure

### New Files to Create

```
Core/Inc/multiregion_context.h    - API declarations
Core/Src/multiregion_context.c    - Core implementation (~500 lines)
Core/Src/multiregion_flash.c      - Flash I/O operations (~200 lines)
```

### Files to Modify

```
LoRaWAN/App/lora_app.c            - Add pre-join + auto-switch logic
LoRaWAN/App/lora_app.h            - Expose pre-join function
LoRaWAN/Target/lorawan_conf.h     - Enable CONTEXT_MANAGEMENT_ENABLED
Core/Src/flash_if.c               - Extend for multi-region storage area
```

## ChirpStack Configuration

### Device Registration

1. **Register device once** with single DevEUI
2. **Enable multi-region** in ChirpStack device settings
3. **Configure gateways** in both US915 and EU868 regions
4. **Frame counters**: ChirpStack tracks independently per region

### Network Server Setup

```yaml
Device Settings (ChirpStack):
  DevEUI: 70B3D57ED005XXXX
  Device Profile: 
    - Supports Class A
    - OTAA activation
    - Multi-region: Enabled ✓
  Regions:
    - US915 (primary)
    - EU868 (secondary)
  Frame Counter Mode: Per-region tracking
```

## Implementation Phases

### Phase 1: Basic Context Storage (1 week)

**Goal:** Prove single-region save/restore works

- [ ] Create `multiregion_context.h/c` files
- [ ] Implement flash read/write for 312-byte structure
- [ ] Add CRC validation
- [ ] Test save/restore of US915 context
- [ ] Verify frame counters survive restore
- [ ] **Gate:** Can we restore a session without rejoining?

### Phase 2: Multi-Region Logic (1 week)

**Goal:** Implement switching between regions

- [ ] Implement region detection from GPS
- [ ] Add MAC state checking before switch
- [ ] Implement context switch with frame counter restoration
- [ ] Test US915 → EU868 switch on ground
- [ ] Verify transmissions work on both regions
- [ ] **Gate:** Does switching work reliably?

### Phase 3: Network Integration (1 week)

**Goal:** Validate ChirpStack multi-region handling

- [ ] Register device on ChirpStack with multi-region
- [ ] Test joins from both regions
- [ ] Verify frame counter independence
- [ ] Test switching with live gateways
- [ ] Monitor for packet loss or rejection
- [ ] **Gate:** Does network server handle both regions?

### Phase 4: Flight Validation

**Goal:** Real-world balloon test

- [ ] Pre-join all regions on ground
- [ ] Launch balloon
- [ ] Monitor region switches during flight
- [ ] Collect telemetry data
- [ ] Analyze switch timing and reliability
- [ ] **Gate:** Does it work in production?

## Edge Cases & Gotchas

### 1. MAC State During Switch

**Problem:** Switching mid-transmission corrupts radio state

**Solution:**
```c
if (LoRaMacIsBusy()) {
    return LORAMAC_HANDLER_BUSY_ERROR;  // Retry on next Tx cycle
}
```

### 2. Frame Counter Synchronization

**Problem:** Network rejects packets if counters are out of sync

**Solution:**
- Save counters immediately after every confirmed transmission
- Never decrement counters (security requirement)
- Test: Verify counters increment correctly after switch

### 3. Duty Cycle Violations (EU868)

**Problem:** EU868 has strict 1% duty cycle limits

**Solution:**
```c
if (target == LORAMAC_REGION_EU868) {
    HAL_Delay(60000);  // 1-minute safety margin after switch
}
```

### 4. Channel Configuration

**Problem:** US915 has 64 channels, EU868 has ~10

**Solution:**
- `LmHandlerConfigure()` resets channel masks per region
- Verify: Check that illegal frequencies aren't used after switch

### 5. DevAddr Collision (Theoretical)

**Problem:** Random DevAddr assignment could collide across regions

**Probability:** ~1 in 4 billion (extremely low but catastrophic)

**Mitigation:**
- Use different JoinEUI per region to get different DevAddr pools
- Monitor for unexpected packet loss patterns

## Testing Procedures

### Ground Testing

```bash
# Test 1: Single region save/restore
1. Join US915
2. Transmit packets (verify counters increment)
3. Power off device
4. Power on device
5. Verify context restored (no rejoin)
6. Transmit packets (counters should continue)

# Test 2: Region switching
1. Pre-join US915 and EU868
2. Set GPS to US coordinates (mock)
3. Transmit on US915
4. Change GPS to EU coordinates (mock)
5. Verify switch to EU868
6. Transmit on EU868
7. Verify frame counters independent
```

### Network Server Validation

```bash
# ChirpStack Dashboard Checks
1. Device Events → Verify both US915 and EU868 joins
2. Frame Logs → Check uplink counters per region
3. Gateway Coverage → Ensure gateways in both regions
4. Packet Loss → Should be <1% per region
```

## Performance Metrics

### Flash Write Cycles

- **Frequency:** Once per transmission (~every 30s)
- **Data:** 75 bytes per update
- **Flash Endurance:** 10,000 cycles typical
- **Lifetime:** 10,000 × 30s = ~83 hours of continuous operation
- **Mitigation:** Wear leveling (future optimization)

### Switching Time

- Halt MAC: ~50ms
- Flash read: ~5ms
- Reconfigure MAC: ~100ms
- Restore context: ~50ms
- **Total: ~200ms typical**

### Memory Footprint

- Code size: ~2KB
- RAM usage: ~400 bytes (working buffers)
- Flash storage: 312 bytes (persistent)

## Future Enhancements

### Satellite Region Support

Add slot for satellite LoRaWAN (proposed future ISM band):

```c
#define LORAMAC_REGION_SATELLITE  0xFF  // Future definition

// Detection: If altitude > 50km, use satellite region
if (altitude > 50000) {
    MultiRegion_SwitchToRegion(LORAMAC_REGION_SATELLITE);
}
```

### Wear Leveling

Implement circular buffer to distribute flash writes:

```
Instead of: Single 312-byte block
Use: 4× 312-byte blocks (rotating)
Benefit: 4× flash lifetime
```

### Predictive Switching

Switch regions before crossing boundary (based on trajectory):

```c
// If moving east at 50 m/s, switch 5 minutes before boundary
if (distance_to_boundary < velocity * 300) {
    MultiRegion_PrepareSwitch(next_region);
}
```

## Troubleshooting

### Symptom: Packets Rejected After Switch

**Causes:**
1. Frame counters not restored correctly
2. Session keys corrupted
3. DevAddr mismatch

**Debug:**
```c
SEGGER_RTT_printf(0, "FCntUp: %lu, FCntDown: %lu\r\n", 
                  uplink_counter, downlink_counter);
SEGGER_RTT_printf(0, "DevAddr: 0x%08lX\r\n", dev_addr);
```

### Symptom: Switch Takes Too Long

**Causes:**
1. Flash erase delay
2. MAC not idle

**Debug:**
```c
uint32_t start = HAL_GetTick();
MultiRegion_SwitchToRegion(target);
SEGGER_RTT_printf(0, "Switch time: %lu ms\r\n", HAL_GetTick() - start);
```

### Symptom: Context Lost After Power Cycle

**Causes:**
1. CRC mismatch
2. Flash corruption
3. Magic number wrong

**Debug:**
```c
if (storage.magic != 0xDEADBEEF) {
    SEGGER_RTT_WriteString(0, "ERROR: Invalid magic number!\r\n");
}
```

## References

- **LoRaWAN 1.0.4 Specification**: Frame counters, session management
- **STM32WLE5 Reference Manual**: Flash memory organization
- **ChirpStack Documentation**: Multi-region device setup
- **Regional Parameters**: US915, EU868 channel plans

## Conclusion

This multi-region system enables seamless LoRaWAN operation across continents without requiring in-flight joins. The minimal 75-byte context approach balances storage efficiency with functional requirements. ChirpStack's multi-region support eliminates network-side complexity, making this a viable solution for trans-Atlantic balloon flights.

**Key Success Factors:**
- Accurate frame counter management
- Robust flash storage with CRC
- GPS-based region detection
- Thorough ground testing before flight

**Estimated Development Time:** 3-4 weeks (3 phases + testing)

---

**Document Version:** 1.0  
**Last Updated:** December 21, 2025  
**Author:** AI Assistant (Cline)  
**Status:** Design Complete - Ready for Implementation
