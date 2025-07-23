# Error Handler

## Overview

The Error Handler provides centralized error management and recovery strategies for the Stratosonde firmware. It implements error logging, recovery procedures, and diagnostic information collection to ensure the system can operate reliably even when components fail.

## Responsibilities

- Centralized error logging
- Recovery strategy implementation
- Watchdog management
- Diagnostic information collection
- Error reporting via telemetry
- Graceful degradation of functionality

## Error Categories

| Category | Description | Examples |
|----------|-------------|----------|
| **Hardware Failures** | Physical component issues | Sensor failure, communication bus error |
| **Software Exceptions** | Unexpected software behavior | Stack overflow, invalid state transition |
| **Resource Exhaustion** | Running out of system resources | Memory allocation failure, flash full |
| **Communication Failures** | Failed external communication | LoRaWAN join failure, I2C timeout |
| **Power Issues** | Power-related problems | Low battery, unexpected power drop |
| **Configuration Errors** | Invalid configuration | CRC mismatch, out-of-range values |

## Error Severity Levels

| Level | Description | System Response |
|-------|-------------|-----------------|
| **Fatal** | System cannot continue operation | System reset or safe mode |
| **Critical** | Major functionality compromised | Attempt recovery, disable affected component |
| **Error** | Significant issue but system can function | Retry operation, fallback to alternative |
| **Warning** | Minor issue that may affect performance | Log and continue, adjust parameters |
| **Info** | Informational event | Log only |

## Data Structures

### Error Record

```c
typedef struct {
    uint32_t timestamp;           // When the error occurred
    uint8_t module;               // Module that reported the error
    uint8_t error_code;           // Specific error code
    uint8_t severity;             // Error severity level
    uint32_t additional_data;     // Error-specific additional data
} ErrorRecord_t;
```

### Error Log

```c
typedef struct {
    uint16_t count;               // Number of errors in log
    uint16_t next_index;          // Index for next error
    ErrorRecord_t records[ERROR_LOG_SIZE];  // Circular buffer of error records
} ErrorLog_t;
```

### Module Error Codes

```c
// System Module Error Codes
#define SYS_ERR_NONE              0x00
#define SYS_ERR_INIT_FAILED       0x01
#define SYS_ERR_INVALID_STATE     0x02
#define SYS_ERR_WATCHDOG_RESET    0x03

// Power Management Error Codes
#define PWR_ERR_NONE              0x00
#define PWR_ERR_LOW_BATTERY       0x01
#define PWR_ERR_CRITICAL_BATTERY  0x02
#define PWR_ERR_SLEEP_FAILED      0x03

// GNSS Error Codes
#define GNSS_ERR_NONE             0x00
#define GNSS_ERR_TIMEOUT          0x01
#define GNSS_ERR_NO_FIX           0x02
#define GNSS_ERR_COMM_FAILED      0x03

// Sensor Error Codes
#define SENSOR_ERR_NONE           0x00
#define SENSOR_ERR_INIT_FAILED    0x01
#define SENSOR_ERR_READ_FAILED    0x02
#define SENSOR_ERR_OUT_OF_RANGE   0x03

// Flash Error Codes
#define FLASH_ERR_NONE            0x00
#define FLASH_ERR_WRITE_FAILED    0x01
#define FLASH_ERR_READ_FAILED     0x02
#define FLASH_ERR_ERASE_FAILED    0x03

// Transmission Error Codes
#define TX_ERR_NONE               0x00
#define TX_ERR_JOIN_FAILED        0x01
#define TX_ERR_SEND_FAILED        0x02
#define TX_ERR_NO_ACK             0x03
```

## Key Functions

### Initialization

```c
ErrorStatus_t Error_Init(void);
```
- Initializes the Error Handler
- Sets up error log in RAM
- Checks for previous errors in flash
- Determines if system was reset due to watchdog

### Error Reporting

```c
ErrorStatus_t Error_Report(uint8_t module, uint8_t error_code, uint8_t severity, uint32_t data);
ErrorStatus_t Error_ReportFatal(uint8_t module, uint8_t error_code, uint32_t data);
```
- Records an error in the error log
- Triggers appropriate recovery action based on severity
- Updates error statistics

### Error Handling

```c
ErrorStatus_t Error_Handle(ErrorRecord_t *error);
ErrorStatus_t Error_ExecuteRecovery(ErrorRecord_t *error);
```
- Implements recovery strategies for different errors
- Coordinates with other modules for recovery actions
- Tracks recovery attempts and success/failure

### Error Log Management

```c
ErrorStatus_t Error_GetLog(ErrorLog_t *log);
ErrorStatus_t Error_ClearLog(void);
ErrorStatus_t Error_SaveLogToFlash(void);
```
- Provides access to the error log
- Manages error log storage
- Implements circular buffer for error records

### Error Status

```c
bool Error_IsActive(uint8_t module, uint8_t error_code);
uint16_t Error_GetCount(uint8_t module, uint8_t error_code);
ErrorRecord_t *Error_GetLatest(uint8_t module);
```
- Checks if specific errors are active
- Retrieves error statistics
- Gets latest error for a module

### Diagnostic Information

```c
ErrorStatus_t Error_GetDiagnostics(ErrorDiagnostics_t *diagnostics);
ErrorStatus_t Error_AppendTelemetry(uint8_t *buffer, uint8_t *length);
```
- Collects diagnostic information for troubleshooting
- Prepares error data for inclusion in telemetry

## Recovery Strategies

### Hardware Failures

1. **Sensor Failures**:
   - Reset sensor via power cycle
   - Try alternative configuration
   - Disable sensor if persistent failure
   - Use data from other sensors when possible

2. **Communication Bus Failures**:
   - Reset bus (I2C/SPI/UART)
   - Reduce bus speed
   - Try alternative communication method if available

### Software Exceptions

1. **Invalid State Transitions**:
   - Reset to known good state
   - Log state machine history
   - Implement state validation

2. **Stack/Memory Issues**:
   - Perform memory cleanup
   - Reset affected module
   - Trigger system reset if critical

### Resource Exhaustion

1. **Memory Allocation Failures**:
   - Free non-critical resources
   - Implement memory pools for critical functions
   - Simplify operations in low-memory conditions

2. **Flash Storage Full**:
   - Implement circular buffer overflow handling
   - Prioritize critical data
   - Compress data if possible

### Communication Failures

1. **LoRaWAN Join Failures**:
   - Implement exponential backoff retry
   - Try alternative regions
   - Fall back to ABP if OTAA fails repeatedly

2. **Transmission Failures**:
   - Retry with different parameters
   - Adjust data rate or power
   - Queue data for later transmission

### Power Issues

1. **Low Battery**:
   - Enter low power mode
   - Reduce functionality
   - Prioritize critical operations

2. **Power Transients**:
   - Implement brownout detection
   - Save state before power loss
   - Recover gracefully after power returns

## Error Logging

The Error Handler maintains an in-memory circular buffer of recent errors:

- **Size**: Configurable number of error records
- **Storage**: RAM with periodic backup to flash
- **Persistence**: Critical errors saved to flash for post-reset analysis
- **Telemetry**: Summary included in telemetry packets

## Watchdog Integration

The Error Handler works with the System Module to manage the watchdog:

- **Watchdog Reset Detection**: Identifies if system was reset by watchdog
- **Recovery After Watchdog**: Special handling after watchdog reset
- **Safe Refresh Strategy**: Ensures watchdog is only refreshed when system is healthy

## Graceful Degradation

The Error Handler implements a graceful degradation strategy:

1. **Component Isolation**: Disable failing components while maintaining core functionality
2. **Feature Reduction**: Reduce functionality based on available resources
3. **Mission Prioritization**: Ensure critical mission functions continue even with failures
4. **Limp Mode**: Define minimal operational mode for critical failures

## Implementation Notes

- Centralized error handling for consistent approach across modules
- Error codes with severity levels for appropriate response
- Flash-based error log for post-reset analysis
- Recovery procedures based on error type
- Optional inclusion of error data in telemetry
- Integration with watchdog for system recovery
- Support for graceful degradation of functionality
