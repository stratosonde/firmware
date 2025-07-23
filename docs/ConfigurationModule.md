# Configuration Module

## Overview

The Configuration Module provides a centralized repository for all configurable parameters in the Stratosonde firmware. It manages parameter storage in flash memory, provides a unified interface for parameter access, and supports default configurations for first boot or recovery scenarios.

## Responsibilities

- Store all configurable parameters in flash memory
- Provide a unified interface for parameter access
- Support default configurations for first boot
- Validate configuration integrity
- Handle parameter updates during operation

## Configuration Parameters

The Configuration Module manages parameters for all aspects of the Stratosonde's operation:

### System Parameters

- **System mode**: Initial operating mode
- **Watchdog timeout**: Timeout period for independent watchdog
- **Error handling policy**: How to handle different error types

### Power Management Parameters

- **Battery thresholds**: Voltage levels for state transitions
  - Normal operation threshold
  - Low power threshold
  - Critical power threshold
- **Duty cycle parameters**: Active and sleep durations for different power states

### GNSS Parameters

- **Update intervals**: How often to acquire position in different states
- **Acquisition timeout**: Maximum time to wait for a valid fix
- **Quality thresholds**: Minimum satellites and maximum HDOP for valid fix
- **High altitude mode**: Enable/disable high altitude mode

### Sensor Parameters

- **Sampling intervals**: How often to read sensors in different states
- **Sensor precision**: Resolution settings for different power states
- **Calibration values**: Sensor calibration coefficients

### Flash Logging Parameters

- **Record format**: Structure of telemetry records
- **Buffer size**: Size of circular buffer for telemetry
- **Metadata location**: Where to store flash metadata

### Transmission Parameters

- **LoRaWAN parameters**: Region, data rate, TX power, etc.
- **Transmission strategy**: Confirmed/unconfirmed, retry count
- **Adaptive parameters**: Rules for adapting spreading factor
- **Batch size**: Number of records to send after confirmation

## Data Structure

```c
typedef struct {
    // Magic number and version
    uint32_t magic;               // Magic number for validation
    uint16_t version;             // Configuration version
    
    // System parameters
    uint8_t system_mode;          // Initial system mode
    uint16_t watchdog_timeout;    // Watchdog timeout in ms
    uint8_t error_policy;         // Error handling policy
    
    // Power management parameters
    uint16_t battery_normal;      // Normal battery threshold in mV
    uint16_t battery_low;         // Low battery threshold in mV
    uint16_t battery_critical;    // Critical battery threshold in mV
    uint16_t sleep_normal;        // Sleep duration in normal mode (seconds)
    uint16_t sleep_low_power;     // Sleep duration in low power mode (seconds)
    uint16_t sleep_critical;      // Sleep duration in critical mode (seconds)
    
    // GNSS parameters
    uint16_t gnss_interval_normal;    // GNSS update interval in normal mode (seconds)
    uint16_t gnss_interval_low;       // GNSS update interval in low power mode (seconds)
    uint16_t gnss_timeout;            // GNSS acquisition timeout (seconds)
    uint8_t gnss_min_satellites;      // Minimum satellites for valid fix
    uint8_t gnss_max_hdop;            // Maximum HDOP for valid fix (x10)
    uint8_t gnss_high_altitude;       // High altitude mode (0=disabled, 1=enabled)
    
    // Sensor parameters
    uint16_t sensor_interval_normal;  // Sensor sampling interval in normal mode (seconds)
    uint16_t sensor_interval_low;     // Sensor sampling interval in low power mode (seconds)
    uint8_t pressure_osr;             // Pressure oversampling ratio
    uint8_t temperature_osr;          // Temperature oversampling ratio
    uint8_t humidity_precision;       // Humidity measurement precision
    int16_t temp_calibration;         // Temperature calibration offset (x10)
    int16_t pressure_calibration;     // Pressure calibration offset (x10)
    int16_t humidity_calibration;     // Humidity calibration offset (x10)
    
    // Flash logging parameters
    uint32_t flash_buffer_size;       // Size of circular buffer in bytes
    uint32_t flash_metadata_addr;     // Address of flash metadata
    uint8_t flash_record_version;     // Version of telemetry record format
    
    // Transmission parameters
    uint8_t lora_region;              // LoRaWAN region code
    uint8_t lora_datarate;            // LoRaWAN data rate
    uint8_t lora_tx_power;            // LoRaWAN TX power
    uint8_t lora_confirmed;           // Use confirmed messages (0=no, 1=yes)
    uint8_t lora_retry_count;         // Number of retries for confirmed messages
    uint8_t lora_adaptive_sf;         // Enable adaptive spreading factor (0=no, 1=yes)
    uint8_t lora_batch_size;          // Number of records to send after confirmation
    
    // CRC for validation
    uint32_t crc;                     // CRC for configuration validation
} Configuration_t;
```

## Key Functions

### Initialization

```c
ConfigStatus_t Config_Init(void);
```
- Initializes the Configuration Module
- Reads configuration from flash
- Validates configuration integrity
- Loads default values if necessary

### Parameter Access

```c
ConfigStatus_t Config_GetParameter(ConfigParam_t param, void *value, size_t size);
ConfigStatus_t Config_SetParameter(ConfigParam_t param, const void *value, size_t size);
```
- Provides unified interface for parameter access
- Handles type conversion and validation
- Supports different parameter types

### Configuration Management

```c
ConfigStatus_t Config_SaveToFlash(void);
ConfigStatus_t Config_LoadFromFlash(void);
ConfigStatus_t Config_ResetToDefaults(void);
```
- Manages configuration storage in flash
- Handles loading and saving operations
- Provides factory reset capability

### Validation

```c
bool Config_IsValid(void);
uint32_t Config_CalculateCRC(void);
```
- Validates configuration integrity
- Calculates CRC for configuration data
- Detects corrupted configurations

## Flash Storage

The Configuration Module stores configuration data in a dedicated flash sector:

```
+---------------------------+
|      Magic Number         |
+---------------------------+
|      Configuration        |
|          Data             |
|                           |
+---------------------------+
|          CRC              |
+---------------------------+
```

- **Sector Selection**: Uses a dedicated flash sector for configuration
- **Wear Leveling**: Simple alternating sector approach for wear leveling
- **Validation**: Magic number and CRC for configuration validation
- **Recovery**: Default values if configuration is invalid

## Default Configuration

The Configuration Module provides default values for all parameters:

```c
static const Configuration_t DEFAULT_CONFIG = {
    .magic = CONFIG_MAGIC,
    .version = CONFIG_VERSION,
    
    // System defaults
    .system_mode = SYSTEM_MODE_NORMAL,
    .watchdog_timeout = 10000,  // 10 seconds
    .error_policy = ERROR_POLICY_RETRY,
    
    // Power management defaults
    .battery_normal = 3600,     // 3.6V
    .battery_low = 3300,        // 3.3V
    .battery_critical = 3000,   // 3.0V
    .sleep_normal = 300,        // 5 minutes
    .sleep_low_power = 900,     // 15 minutes
    .sleep_critical = 3600,     // 60 minutes
    
    // GNSS defaults
    .gnss_interval_normal = 60,  // 1 minute
    .gnss_interval_low = 900,    // 15 minutes
    .gnss_timeout = 60,          // 60 seconds
    .gnss_min_satellites = 4,
    .gnss_max_hdop = 50,         // 5.0
    .gnss_high_altitude = 1,     // Enabled
    
    // Sensor defaults
    .sensor_interval_normal = 60,  // 1 minute
    .sensor_interval_low = 900,    // 15 minutes
    .pressure_osr = 4,             // OSR 4096
    .temperature_osr = 4,          // OSR 4096
    .humidity_precision = 1,       // Medium precision
    .temp_calibration = 0,
    .pressure_calibration = 0,
    .humidity_calibration = 0,
    
    // Flash logging defaults
    .flash_buffer_size = 0x1F0000,  // ~2MB
    .flash_metadata_addr = 0x1000,
    .flash_record_version = 1,
    
    // Transmission defaults
    .lora_region = REGION_EU868,
    .lora_datarate = 5,            // SF7/125kHz
    .lora_tx_power = 14,           // 14 dBm
    .lora_confirmed = 1,           // Use confirmed messages
    .lora_retry_count = 3,         // 3 retries
    .lora_adaptive_sf = 1,         // Enable adaptive SF
    .lora_batch_size = 5,          // Send 5 records after confirmation
    
    // CRC will be calculated during initialization
    .crc = 0
};
```

## Runtime Updates

The Configuration Module supports runtime updates to parameters:

- **Immediate Updates**: Some parameters take effect immediately
- **Deferred Updates**: Some parameters require system restart
- **Persistent Updates**: Changes can be saved to flash for persistence
- **Temporary Updates**: Changes can be kept in RAM only

## Error Handling

1. **Flash Errors**:
   - Retry mechanism for flash operations
   - Fallback to default values if flash is corrupted
   - Error reporting to System Module

2. **Parameter Validation**:
   - Range checking for parameter values
   - Type checking for parameter access
   - Error reporting for invalid parameters

3. **CRC Validation**:
   - Detection of corrupted configuration
   - Automatic recovery with default values
   - Logging of configuration corruption events

## Implementation Notes

- Parameters stored in dedicated flash sector
- CRC validation for configuration integrity
- Default values for factory reset or recovery
- Simple interface for parameter access
- Support for different parameter types
- Efficient storage format to minimize flash usage
