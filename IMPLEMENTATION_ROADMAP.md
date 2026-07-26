# Stratosonde Firmware Implementation Roadmap
## Comprehensive Action Plan Based on Documentation Analysis

**Date:** January 19, 2026  
**Based on:** DOCUMENTATION_VS_IMPLEMENTATION_ANALYSIS.md feedback  
**Author:** AI Assistant (Cline)  
**Status:** Roadmap for completing firmware features

---

## 📋 Executive Summary

This roadmap addresses the gaps identified in the documentation vs. implementation analysis and incorporates user feedback to prioritize and complete the Stratosonde firmware.

### Key Decisions Made
- ✅ **Flash logging**: Complete integration (Option B) - implement 11-byte + high-res packets
- ✅ **Multi-region**: Keep all code, document DevEUI requirement for Chirpstack
- ✅ **H3Lite geo-fencing**: Add restricted region check
- ✅ **Custom binary payload**: Implement alongside CayenneLPP for debug/development
- ✅ **Configuration module**: Centralize scattered configs
- ✅ **Watchdog**: Must implement (safety-critical)
- ✅ **Flash write frequency**: Fix excessive wear issue

---

## 🎯 Implementation Phases

### Phase 1: Critical Safety & Reliability (1-2 weeks)

#### 1.1 Watchdog Implementation ⚠️ **HIGH PRIORITY**
**Status:** Not implemented (safety-critical)  
**Location:** `main.c` system initialization  

**Requirements:**
- Independent watchdog (IWDG) configuration
- Watchdog refresh in main loop
- Timeout calculation based on longest sleep period
- Integration with power management modes

**Implementation Plan:**
```c
// In main.c initialization
RCC_OscInitStruct.LSIState = RCC_LSI_ON;  // Enable LSI for IWDG
IWDG_Config();  // ~4 second timeout

// In main loop
HAL_IWDG_Refresh(&hiwdg);  // Before each sleep
```

#### 1.2 Frame Counter Flash Write Optimization ⚠️ **HIGH PRIORITY** 
**Status:** Keep existing per-region counters, add batched saves  
**Current Issue:** Frame counters saved to flash on every successful TX  
**Impact:** High flash write frequency could cause premature wear  

**Selected Solution: Batched Saves of Per-Region Counters**
```c
// Per-region counter tracking (KEEP existing implementation)
// Add batch save mechanism to reduce flash writes

static uint8_t unsaved_tx_count = 0;

// In OnTxData callback
if (params->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
    // Frame counter incremented automatically by LoRaWAN stack
    unsaved_tx_count++;
    
    if (unsaved_tx_count >= 10) {  // Save every 10 TXs
        MultiRegion_SaveCurrentContext();  // Save to flash
        unsaved_tx_count = 0;
    }
}
```

**Benefits:**
- ✅ Maintains LoRaWAN security (per-session frame counters)
- ✅ Reduces flash writes by 10x (every 10 TX instead of every TX)
- ✅ Preserves existing multi-region architecture
- ✅ Compatible with Chirpstack ABP configuration
- ✅ No breaking changes to working code

**Note:** Per-region frame counters are required for LoRaWAN security and multi-region operation.

### Phase 2: Data Architecture & Payloads (2-3 weeks)

#### 2.1 Custom Binary Packet Format Design
**Status:** Ready to implement (data stable)  
**Requirements:** Efficient 11-byte packet + high-resolution logging  
**Clarification:** MAC layer DevStatusAns is on-demand only → Must include battery voltage in application payload

**11-Byte Compact Packet Structure (SF10 Probe):**
```c
typedef struct __attribute__((packed)) {
    uint16_t timestamp_min;     // Minutes since epoch (2 bytes)
    int16_t  latitude_100m;     // Latitude in 100m resolution (2 bytes)
    int16_t  longitude_100m;    // Longitude in 100m resolution (2 bytes)
    int8_t   temperature_2deg;  // Temperature / 2°C offset +64 (1 byte)
    uint8_t  pressure_10hPa;    // Pressure / 10hPa from 950hPa base (1 byte)
    uint8_t  battery_volt_50mv; // Battery voltage / 50mV (1 byte) - REQUIRED in payload
    uint8_t  humidity_5pct;     // Humidity / 5% resolution (1 byte, 0-100% = 0-20)
    uint8_t  status_flags;      // GPS fix + power mode + satellites (1 byte)
} CompactTelemetryPacket_t;  // Total: 11 bytes (maximum SF10 US915 payload)
```

**Key Features:**
- 🎯 **GPS precision**: 100m resolution (adequate for balloon tracking at 30km altitude)
- 🔋 **Battery voltage**: Included in payload (DevStatusAns is on-demand only, not automatic)
- 📏 **Altitude**: Calculated on ground station from pressure + temperature (more accurate)
- 💧 **Humidity**: Atmospheric humidity data (5% resolution = 0-100% in 20 steps)
- 📊 **Status flags**: Packed GPS fix quality, power mode, satellite count
- 📦 **Full utilization**: Uses all 11 bytes available at SF10 on US915

**Status Flags Bit Packing (1 byte):**
```c
// Bit allocation for status_flags byte
#define GPS_FIX_VALID_MASK    0x01  // Bit 0: GPS fix valid
#define GPS_SATS_MASK         0x1E  // Bits 1-4: Satellite count (0-15)
#define POWER_MODE_MASK       0xE0  // Bits 5-7: Power mode (0-7)

// Extract values
bool gps_valid = (status_flags & GPS_FIX_VALID_MASK) != 0;
uint8_t satellites = (status_flags & GPS_SATS_MASK) >> 1;
uint8_t power_mode = (status_flags & POWER_MODE_MASK) >> 5;
```

**Ground Station Altitude Calculation:**
```python
def calculate_altitude(pressure_hPa, temperature_C):
    """Calculate altitude from pressure using hypsometric formula"""
    P0 = 1013.25  # Sea level standard pressure
    T = temperature_C + 273.15  # Convert to Kelvin
    altitude = ((P0 / pressure_hPa) ** (1/5.257) - 1) * T / 0.0065
    return altitude
```

**High-Resolution Flash Record:**
```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp;         // Full precision timestamp
    int32_t  latitude;          // Full GPS precision
    int32_t  longitude;         // Full GPS precision  
    uint16_t altitude;          // Meters
    int16_t  temperature;       // 0.1°C resolution
    uint16_t humidity;          // 0.1% resolution
    uint16_t pressure;          // 0.1hPa resolution
    uint16_t battery_voltage;   // mV
    uint16_t solar_voltage;     // mV
    int16_t  voltage_slope;     // mV/hour
    uint8_t  satellites;        // GPS satellite count
    uint8_t  hdop;             // GPS HDOP * 10
    uint8_t  power_mode;       // Operating mode enum
    uint8_t  flags;            // Status flags
    uint16_t crc16;            // Data integrity
} HighResTelemetryRecord_t;   // Total: 32 bytes
```

#### 2.2 Flash Logging Integration
**Status:** Module exists but not integrated  
**Files:** `flash_log.c` (~500 lines of "dead code")  
**Action:** Complete integration

**Integration Points:**
1. **Initialization:** Call `FlashLog_Init()` in `main.c`
2. **Data Storage:** Store high-res records continuously
3. **Transmission:** Convert to 11-byte packets for LoRaWAN
4. **Recovery:** Read back unsent data after power cycles

### Phase 3: System Architecture Improvements (3-4 weeks)

#### 3.1 Centralized Configuration Module
**Status:** Not implemented (configs scattered everywhere)  
**Current Problem:** Hardcoded #defines across multiple files
- `APP_TX_DUTYCYCLE` in lora_app.h
- `LORAWAN_DEFAULT_DATA_RATE` in lorawan_conf.h  
- `GPS_TEMPERATURE_LOCKOUT` in lora_app.h
- Battery thresholds in power management functions

**Proposed Structure:**
```c
// Core/Inc/config.h
typedef struct {
    uint32_t magic;                    // Validation
    uint16_t version;                  // Config version
    
    // Transmission settings
    uint32_t tx_interval_normal;       // milliseconds
    uint32_t tx_interval_lowpower;     // milliseconds
    uint8_t  lorawan_datarate;         // DR0-DR5
    uint8_t  lorawan_txpower;          // dBm
    
    // Power management
    uint16_t battery_low_threshold;    // mV
    uint16_t battery_critical_threshold; // mV
    int8_t   gps_temperature_lockout;  // °C
    
    // GPS settings
    uint8_t  gps_timeout_seconds;
    uint8_t  gps_min_satellites;
    
    // Flash logging
    uint16_t flash_save_interval;      // TXs between saves
    
    uint32_t crc32;                    // Structure validation
} SystemConfig_t;
```

#### 3.2 Error Handler Module
**Status:** Only basic `Error_Handler()` exists (infinite loop)  
**Current Issue:** No error recovery, logging, or systematic handling

**Proposed Implementation:**
```c
// Core/Inc/error_mgr.h
typedef enum {
    ERR_GPS_TIMEOUT = 0x01,
    ERR_SENSOR_I2C = 0x02,
    ERR_FLASH_SPI = 0x03,
    ERR_LORAWAN_JOIN = 0x04,
    ERR_POWER_CRITICAL = 0x05,
    ERR_WATCHDOG_RESET = 0x06
} ErrorCode_t;

typedef enum {
    ERR_RECOVERY_RETRY = 0,
    ERR_RECOVERY_DEGRADE = 1,
    ERR_RECOVERY_FATAL = 2
} ErrorRecovery_t;

ErrorRecovery_t ErrorMgr_HandleError(ErrorCode_t error, const char* context);
void ErrorMgr_LogError(ErrorCode_t error, uint32_t data);
uint32_t ErrorMgr_GetErrorCount(ErrorCode_t error);
```

#### 3.3 Enhanced LED Status Module
**Status:** Simplified implementation (boot blink only)  
**User Preference:** "LED on when wake, off when sleep" + error patterns

**Proposed Pattern:**
- **Wake:** LED ON
- **GPS fix:** Brief double-blink
- **Sensor acquire:** Brief single blink  
- **LoRaWAN TX:** LED ON during transmission
- **RX windows:** LED ON during RX1/RX2
- **Sleep:** LED OFF
- **Errors:** Distinctive flash patterns (3-flash for GPS timeout, 5-flash for critical)

---

## 🔧 Technical Solutions & Clarifications

### HAL_GetTick() vs TIMER_IF_GetTime() - RESOLVED ✅

**User Question:** "does HAL_GetTick halt with STOP2, what is wrong with that?"

**Answer:** `HAL_GetTick()` **STOPS** completely in STOP2 because:
- Uses SysTick timer (part of CPU core)
- STOP2 halts CPU core → SysTick stops
- Time appears frozen during sleep

**Your Implementation is CORRECT:**
- Voltage slope calculation uses `TIMER_IF_GetTime()` (RTC-based)
- RTC continues running in STOP2 (LSE crystal powered)
- Provides accurate time across sleep cycles
- `HAL_GetTick()` only used for non-critical timeouts (acceptable)

### GPS Configuration Every Boot - ACCEPTABLE ✅

**Current Behavior:** `GNSS_Configure()` runs on every boot  
**User Decision:** "Every boot for GNSS is okay as it doesn't happen very often"

**Technical Detail:**
```c
GNSS_Configure(&hgnss);  // Sends PCAS04,7 + PCAS00 (save)
HAL_Delay(500);          // Wait for GPS flash write
GNSS_PowerOff(&hgnss);
```

**Potential Optimization (Future):**
- Add STM32 flash flag "GPS_configured_v1"
- Skip configuration if flag present
- **Current approach acceptable for deployment**

### H3Lite Performance Data - UPDATE DOCS ✅

**User Data:** "~2ms per region per ring"  
**Documentation Fix:** Update H3LiteIntegration.md with actual performance  
**Current Doc Claims:** ~700-1000μs (needs correction)

### Multi-Region DevEUI Requirement - DOCUMENT ✅

**User Clarification:** "DevEUI per region is required for Chirpstack"  
**Current Status:** Implemented but undocumented  
**Action:** Update MultiRegionSupport.md to document this requirement

---

## 📡 Adaptive Transmission Strategy ⚡ **NEW REQUIREMENT**

### User Requirements
**Strategy:** LinkCheckReq on every SF10 transmission, followed by opportunistic bulk transfer at SF7

**Flow:**
1. **PROBE**: Send 11-byte packet at SF10 + LinkCheckReq (every transmission)
2. **EVALUATE**: Check margin ≥ threshold AND gateways ≥ threshold  
3. **BULK**: If conditions met + battery good + cached data → Send all cached at SF7/222-bytes (LIFO)

### Transmission State Machine
```c
typedef enum {
    TX_STATE_PROBE_SF10 = 0,      // Send 11-byte at SF10
    TX_STATE_WAIT_PROBE_ACK,      // Waiting for LinkCheckAns
    TX_STATE_BULK_TRANSFER,       // Sending cached packets at SF7
    TX_STATE_COMPLETE             // Done with cycle
} TxState_t;

static void SendTxData(void) {
    static TxState_t tx_state = TX_STATE_PROBE_SF10;
    
    switch(tx_state) {
    case TX_STATE_PROBE_SF10:
        // Send 11-byte critical telemetry at SF10
        LmHandlerSetTxDatarate(DR_0);  // SF10
        LmHandlerLinkCheckReq();       // Request link quality
        LmHandlerSend(&compact_packet, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0);
        tx_state = TX_STATE_WAIT_PROBE_ACK;
        break;
        
    case TX_STATE_BULK_TRANSFER:
        // Send 222-byte packets from flash (LIFO order)
        if (FlashLog_HasUnsentData() && bulk_packets_sent < max_bulk) {
            LmHandlerSetTxDatarate(DR_3);  // SF7
            SendBulkPacketFromFlash();
            bulk_packets_sent++;
        } else {
            tx_state = TX_STATE_COMPLETE;
            bulk_packets_sent = 0;
        }
        break;
    }
}

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params) {
    if (tx_state == TX_STATE_WAIT_PROBE_ACK) {
        // Get LinkCheckAns result
        MibRequestConfirm_t mib;
        mib.Type = MIB_LINK_CHECK;
        LoRaMacMibGetRequestConfirm(&mib);
        
        uint8_t margin = mib.Param.LinkCheck.DemodMargin;
        uint8_t gw_count = mib.Param.LinkCheck.NbGateways;
        uint16_t battery_mv = SYS_GetBatteryVoltage();
        
        // Check all conditions
        bool link_good = (margin >= config.link_margin_threshold && 
                          gw_count >= config.gateway_count_threshold);
        bool battery_good = (battery_mv >= config.bulk_battery_min_mv);
        bool has_cache = FlashLog_HasUnsentData();
        
        if (link_good && battery_good && has_cache) {
            SEGGER_RTT_printf(0, "Conditions met → Bulk transfer (margin=%ddB, GW=%d)\r\n", 
                             margin, gw_count);
            tx_state = TX_STATE_BULK_TRANSFER;
            bulk_packets_sent = 0;
            // Trigger immediate bulk transfer
            UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
        } else {
            SEGGER_RTT_WriteString(0, "Conditions not met → Done (conservative)\r\n");
            tx_state = TX_STATE_COMPLETE;
        }
    }
}
```

### Maximum Packet Sizes (US915)
| DR | SF | Bandwidth | Max Payload |
|----|----|-----------| ------------|
| DR0 | SF10 | 125 kHz | **11 bytes** (probe) |
| DR3 | SF7 | 125 kHz | **222 bytes** (bulk) |

### 222-Byte Bulk Packet Structure
```c
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;           // 1 byte - Format version
    uint8_t  record_count;          // 1 byte - Records in this packet
    uint32_t flash_page_addr;       // 4 bytes - Source page (for LIFO tracking)
    
    // High-resolution records (32 bytes each)
    // 222 - 6 header = 216 bytes available
    // 216 ÷ 32 = 6 complete records per packet
    HighResTelemetryRecord_t records[6];  // 6 × 32 = 192 bytes
    
    // Metadata/padding (222 - 198 = 24 bytes remaining)
    uint8_t  voltage_trend[10];       // 10 bytes - recent voltage samples
    uint8_t  mode_changes[10];        // 10 bytes - power mode history  
    uint32_t crc32;                   // 4 bytes - Packet integrity
    
} BulkTelemetryPacket_t;  // Total: 222 bytes (SF7/US915 maximum)
```

### LIFO Flash Read Strategy
```c
// Read from flash in LIFO order (newest first)
FlashLog_StatusTypeDef FlashLog_GetNextUnsentLIFO(HighResTelemetryRecord_t *records, uint8_t max_records, uint8_t *actual_count) {
    *actual_count = 0;
    
    // Start from most recent written record
    uint32_t read_index = flash_log.write_pointer;
    
    for (uint8_t i = 0; i < max_records; i++) {
        // Move backwards (LIFO)
        if (read_index > 0) {
            read_index--;
        } else if (flash_log.wrapped) {
            read_index = flash_log.max_records - 1;  // Wrap to end
        } else {
            break;  // No more data
        }
        
        // Skip if already transmitted
        if (read_index <= flash_log.last_transmitted_index) {
            break;
        }
        
        // Read record
        if (FlashLog_ReadRecord(read_index, &records[i]) == FLASHLOG_OK) {
            (*actual_count)++;
        }
    }
    
    return (*actual_count > 0) ? FLASHLOG_OK : FLASHLOG_NO_DATA;
}
```

### Power Budget Considerations
**Note:** Power consumption analysis to be completed with accurate measurements during implementation and testing phase.

**Key Points:**
- SF10 provides maximum range but longer airtime
- SF7 is 6x faster than SF10 but requires better signal
- Bulk transfer only occurs when battery and link conditions are excellent
- Strategy provides graceful degradation (always sends critical 11-byte data)

### Configuration Parameters
```c
typedef struct {
    // Link quality thresholds
    uint8_t  link_margin_sf10_threshold;   // e.g., 15 dB
    uint8_t  link_gateway_count_threshold; // e.g., 2 gateways
    
    // Bulk transfer criteria  
    uint16_t bulk_battery_min_mv;          // e.g., 5000 mV (good charge)
    uint8_t  max_bulk_packets;             // e.g., 20 packets
    uint32_t bulk_timeout_ms;              // e.g., 60000 ms (1 minute)
    
} AdaptiveTxConfig_t;
```

---

## 🔄 Development to Production Migration - Dual-Mode Port Strategy

### Overview
**Strategy:** Run both CayenneLPP and binary formats simultaneously on different ports during development, with compile-time flags to disable debug formats for production.

### Port Assignments
```c
#define LORAWAN_LPP_PORT          2   // CayenneLPP (development/debug)
#define LORAWAN_GNSS_DETAIL_PORT  3   // GNSS satellite detail (development/debug) 
#define LORAWAN_COMPACT_PORT      10  // 11-byte compact binary (SF10 probe) - PRODUCTION
#define LORAWAN_BULK_PORT         11  // 222-byte bulk binary (SF7 bulk) - PRODUCTION
```

### Compile-Time Control Flags
```c
// In lora_app.h or config.h
#define ENABLE_DEBUG_LPP           1  // 1 = Enable CayenneLPP on port 2, 0 = Disable
#define ENABLE_GNSS_DETAIL_PACKET  1  // 1 = Enable GNSS detail on port 3, 0 = Disable
#define DEBUG_LPP_TX_INTERVAL      5  // Send LPP every 5th transmission (reduce airtime)
```

### Implementation in SendTxData()
```c
static void SendTxData(void) {
    static uint32_t tx_count = 0;
    tx_count++;
    
    // ========== PRIMARY: Compact binary at SF10 (ALWAYS SENT) ==========
    CompactTelemetryPacket_t compact_packet;
    EncodeCompactBinaryPacket(&compact_packet, &sensor_data);
    
    LmHandlerSetTxDatarate(DR_0);  // SF10 for maximum range
    LmHandlerLinkCheckReq();       // Request link quality info
    
    LmHandlerAppData_t appData;
    appData.Port = LORAWAN_COMPACT_PORT;
    appData.Buffer = (uint8_t*)&compact_packet;
    appData.BufferSize = sizeof(CompactTelemetryPacket_t);
    
    LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0);
    
    // ========== SECONDARY: Debug formats (OPTIONAL) ==========
    
    #if ENABLE_DEBUG_LPP
    if ((tx_count % DEBUG_LPP_TX_INTERVAL) == 0) {
        // Queue CayenneLPP for transmission after RX windows complete
        CayenneLppReset();
        // ... build LPP packet with all sensors ...
        
        PacketQueue_Push(&g_packet_queue, 
                        CayenneLppGetBuffer(), 
                        CayenneLppGetSize(), 
                        LORAWAN_LPP_PORT);
        SEGGER_RTT_printf(0, "Queued debug LPP packet (%d bytes)\r\n", CayenneLppGetSize());
    }
    #endif
    
    #if ENABLE_GNSS_DETAIL_PACKET
    if (sensor_data.gnss_valid && (tx_count % DEBUG_LPP_TX_INTERVAL) == 0) {
        // Queue GNSS detail packet
        static uint8_t gnss_detail_buffer[150];
        uint16_t gnss_packet_size = EncodeGNSSDetailPacket(gnss_detail_buffer, sizeof(gnss_detail_buffer));
        
        if (gnss_packet_size > 0) {
            PacketQueue_Push(&g_packet_queue, gnss_detail_buffer, gnss_packet_size, LORAWAN_GNSS_DETAIL_PORT);
            SEGGER_RTT_printf(0, "Queued debug GNSS packet (%d bytes)\r\n", gnss_packet_size);
        }
    }
    #endif
    
    // ========== ADAPTIVE BULK: SF7 high-speed transfer (CONDITIONAL) ==========
    // Handled in OnRxData callback based on LinkCheckAns evaluation
}
```

### Migration Timeline
| Phase | Duration | LPP Setting | GNSS Setting | Usage |
|-------|----------|-------------|--------------|--------|
| **Development** | 2-4 weeks | `ENABLE_DEBUG_LPP = 1`<br/>Interval = 5 | `ENABLE_GNSS_DETAIL = 1`<br/>Interval = 5 | Compare formats, validate encoding |
| **Field Testing** | 2-4 weeks | `ENABLE_DEBUG_LPP = 1`<br/>Interval = 10 | `ENABLE_GNSS_DETAIL = 1`<br/>Interval = 10 | Mostly binary, spot check with LPP |
| **Pre-Production** | 2-4 weeks | `ENABLE_DEBUG_LPP = 1`<br/>Interval = 50 | `ENABLE_GNSS_DETAIL = 0` | Rare LPP for anomaly checks |
| **Production** | Flight ready | `ENABLE_DEBUG_LPP = 0` | `ENABLE_GNSS_DETAIL = 0` | Binary only, maximum efficiency |

### Benefits
- ✅ **Gradual transition**: Side-by-side validation before switching
- ✅ **Flexible debugging**: Re-enable LPP anytime during development
- ✅ **Efficient production**: Pure binary when ready
- ✅ **Zero waste**: Debug formats completely removed from production builds
- ✅ **Port organization**: Clear separation by function
- ✅ **Airtime management**: Configurable debug packet frequency

### Ground Station Decoder
```javascript
// Chirpstack payload decoder
function decodeUplink(input) {
    switch(input.fPort) {
        case 2:  return decodeCayenneLPP(input.bytes);        // Development
        case 3:  return decodeGNSSDetail(input.bytes);        // Development  
        case 10: return decodeCompactBinary(input.bytes);     // Production
        case 11: return decodeBulkBinary(input.bytes);        // Production
        default: return {errors: ["Unknown port " + input.fPort]};
    }
}
```

---

## 🔒 H3Lite Geo-Fencing Requirements

### Restricted Regions Implementation
**User Requirement:** "Geofencing should be part of h3lite. There should be a restricted region returned we'll need to respect."

**Implementation Plan:**
1. **H3Lite Library Enhancement:**
   - Add `REGION_RESTRICTED` return value
   - Implement geo-fencing data for restricted areas
   - North Korea, disputed territories, etc.

2. **Firmware Integration:**
```c
LoRaMacRegion_t detected = MultiRegion_DetectFromGPS_H3(lat, lon);
if (detected == REGION_RESTRICTED) {
    // Skip transmission entirely
    SEGGER_RTT_WriteString(0, "RESTRICTED: No transmission allowed\r\n");
    return;
}
```

---

## 📂 File Structure for New Components

### Configuration Module
```
Core/Inc/config.h           - Configuration structure definitions
Core/Src/config.c           - Load/save/validate configuration
```

### Custom Binary Payloads  
```
Core/Inc/payload_format.h   - Packet format structures
Core/Src/payload_encode.c   - Encode functions (11-byte + high-res)
Core/Src/payload_decode.c   - Decode functions (ground station)
```

### Enhanced Error Manager
```
Core/Inc/error_mgr.h        - Error codes and recovery strategies
Core/Src/error_mgr.c        - Error handling implementation
```

### Enhanced LED Module
```
Core/Inc/led_status.h       - LED pattern definitions
Core/Src/led_status.c       - Pattern generation and control
```

---

## 📊 Testing & Validation Plan

### Phase 1 Testing: Critical Functions
1. **Watchdog Testing:**
   - Verify timeout calculation
   - Test watchdog reset recovery
   - Confirm refresh timing in all power modes

2. **Flash Write Frequency:**
   - Measure flash endurance improvement (10x reduction)
   - Test frame counter accuracy after batch saves
   - Verify no data loss scenarios

### Phase 2 Testing: Data Integrity
1. **Packet Format Validation:**
   - Verify 11-byte encoding/decoding accuracy
   - Test high-res to compact conversion
   - Compare against CayenneLPP for accuracy

2. **Flash Logging Integration:**
   - Test circular buffer overflow handling
   - Verify data persistence across power cycles
   - Stress test with rapid data generation

### Phase 3 Testing: System Integration
1. **Multi-Region Real-Time Testing:**
   - Test H3 lookup + automatic switching
   - Verify frame counter independence
   - Test geo-fencing restrictions

2. **Power Management:**
   - Analyze 5 vs 3 operating mode efficiency
   - Test voltage slope accuracy across temperature
   - Validate sleep/wake timing

---

## 📋 Implementation Priority Matrix

| Task | Priority | Complexity | Time | Dependencies |
|------|----------|------------|------|--------------|
| **Watchdog Implementation** | Critical | Low | 1 day | None |
| **Flash Write Frequency Fix** | Critical | Low | 1 day | None |
| **Custom Packet Format** | High | Medium | 3-5 days | None |
| **Flash Logging Integration** | High | Medium | 5-7 days | Packet format |
| **Configuration Module** | Medium | Medium | 5-7 days | None |
| **Error Handler Module** | Medium | Medium | 3-5 days | None |
| **H3Lite Geo-fencing** | Medium | High | 7-10 days | H3Lite library |
| **LED Status Enhancement** | Low | Low | 2-3 days | Error handler |

---

## 🚀 Immediate Next Steps (Week 1-2)

### Day 1-2: Watchdog + Flash Write Fix
1. **Morning:** Implement IWDG watchdog configuration
2. **Afternoon:** Add watchdog refresh to main loop
3. **Next day:** Implement periodic flash save logic
4. **Test:** Verify both features work correctly

### Day 3-5: Packet Format Design
1. **Design:** Finalize 11-byte packet structure
2. **Implement:** Encoding/decoding functions  
3. **Test:** Accuracy vs CayenneLPP
4. **Integrate:** Add to transmission options

### Day 6-10: Flash Logging Integration
1. **Initialize:** Add FlashLog_Init() to startup
2. **Storage:** Implement high-res data logging
3. **Retrieval:** Add conversion to transmission packets
4. **Test:** Full store/retrieve cycle

### Week 2: Configuration Module Foundation
1. **Structure:** Define SystemConfig_t
2. **Storage:** Flash-based config persistence
3. **API:** Load/save/validate functions
4. **Migration:** Convert hardcoded values

---

## 📝 Documentation Updates Required

### High Priority Updates
1. **MultiRegionSupport.md** - Add DevEUI per region requirement
2. **H3LiteIntegration.md** - Update performance data (~2ms per region per ring)
3. **FirmwareArchitecture.md** - Remove state machine or mark as future
4. **TransmissionModule.md** - Simplify to match actual implementation

### New Documentation Needed
1. **CustomPacketFormat.md** - Document 11-byte + high-res formats
2. **ConfigurationModule.md** - Centralized configuration approach
3. **ErrorHandling.md** - Error codes and recovery strategies
4. **TestingProcedures.md** - Validation and testing protocols

---

## ❓ Open Questions for Clarification

1. **Flash save interval:** Confirm 10 TXs is acceptable data loss risk?
2. **LED patterns:** Simple (wake/sleep) or enhanced (GPS/TX/RX indicators)?
3. **Power mode simplification:** Should we test 3 vs 5 modes before other work?
4. **Geo-fencing integration:** H3Lite library modification or firmware wrapper?
5. **Configuration priority:** Before or after flash logging integration?

---

## 📈 Success Metrics

### Technical Metrics
- ⚡ Flash endurance: >800 hours (10x improvement from current 83 hours)
- 🛡️ System reliability: Watchdog prevents lockups
- 📦 Data efficiency: 11-byte packets vs ~60-byte CayenneLPP  
- 🔧 Configuration: All settings centralized and persistent
- 🌍 Multi-region: Real-time H3 switching with geo-fencing

### Operational Metrics  
- 🧪 Testing coverage: All critical paths validated
- 📚 Documentation: Implementation matches documentation
- 🚀 Deployment readiness: Production-quality error handling
- 🔋 Power efficiency: Optimized for balloon flight duration

---

## 📅 Timeline Summary

| Phase | Duration | Key Deliverables |
|-------|----------|------------------|
| **Phase 1** | 1-2 weeks | Watchdog + Flash write fix |
| **Phase 2** | 2-3 weeks | Custom packets + Flash logging |
| **Phase 3** | 3-4 weeks | Config module + Error handling |
| **Testing** | 1-2 weeks | Integration + Validation |
| **Total** | **7-11 weeks** | **Production-ready firmware** |

---

**END OF ROADMAP**

*This roadmap will be updated as implementation progresses and priorities shift based on testing results and operational requirements.*

---

**Document Version:** 1.0  
**Created:** 2026-01-19  
**Next Review:** After Phase 1 completion  
**Status:** Ready for implementation
