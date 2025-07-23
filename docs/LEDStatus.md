# LED Status Module

## Overview

The LED Status Module provides visual feedback during initialization and early operation phases of the Stratosonde. It is primarily designed for pre-launch configuration verification and basic status indication, with minimal power usage during flight.

## Hardware Interface

- **LED**: One red LED
- **GPIO Pin**: PA0
- **Control Method**: Direct GPIO manipulation

## Status Indications

| State | LED Pattern | Description |
|-------|-------------|-------------|
| Boot/Commissioning | 1 Hz blink | System is initializing or in commissioning mode |
| GNSS acquisition | 2 Hz blink | System is acquiring GPS fix |
| Ascent transmission | Solid ON | System is in ascent mode and actively transmitting |
| Telemetry cycle | Brief flash | System is transmitting telemetry data |
| Sleep/Float mode | OFF | System is in sleep or float mode to conserve power |
| Error conditions | Various patterns | Different patterns indicate different error types |

## Error Indication Patterns

| Error Type | LED Pattern | Description |
|------------|-------------|-------------|
| Hardware failure | Triple blink, pause | Critical hardware component failure |
| Communication error | Double blink, pause | LoRaWAN or other communication failure |
| Sensor error | Single long, single short | Sensor reading or calibration error |
| Flash error | Single short, single long | Flash memory read/write error |
| Low battery | Rapid blinks (5 Hz) | Battery voltage below critical threshold |
| Configuration error | Alternating long-short | Invalid configuration parameters |

## Key Functions

### Initialization

```c
void LED_Init(void);
```
- Initializes the GPIO pin for the LED
- Configures it as output
- Sets initial state (OFF)

### Basic Control

```c
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
```
- Simple control functions for direct LED manipulation
- Used by pattern functions and for manual control

### Pattern Generation

```c
void LED_SetPattern(LED_Pattern_t pattern);
void LED_StopPattern(void);
```
- Sets the LED to display a specific pattern
- Stops any active pattern and turns LED off

### Error Indication

```c
void LED_IndicateError(ErrorType_t errorType);
```
- Displays the appropriate pattern for a specific error type
- Can be called from any module to indicate errors

### Power Management

```c
void LED_Disable(void);
void LED_Enable(void);
bool LED_IsEnabled(void);
```
- Completely disables LED functionality to save power
- Re-enables LED if needed for debugging
- Checks if LED functionality is currently enabled

## Implementation Approach

The LED Status Module uses a simple, direct approach to LED control:

```c
// Example implementation of LED_Toggle
void LED_Toggle(void)
{
    if (LED_IsEnabled())
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}

// Example implementation of error indication
void LED_IndicateError(ErrorType_t errorType)
{
    if (!LED_IsEnabled())
    {
        return;
    }
    
    switch (errorType)
    {
        case ERROR_HARDWARE:
            // Triple blink pattern
            LED_On();
            HAL_Delay(100);
            LED_Off();
            HAL_Delay(100);
            LED_On();
            HAL_Delay(100);
            LED_Off();
            HAL_Delay(100);
            LED_On();
            HAL_Delay(100);
            LED_Off();
            HAL_Delay(500);
            break;
            
        // Other error patterns...
    }
}
```

## Power Considerations

- LED is completely disabled during float mode to save power
- No timer-based approach to minimize overhead
- LED is only used during pre-launch and for critical errors
- Direct GPIO control for minimal power consumption

## Usage Guidelines

1. **Pre-Launch Phase**:
   - LED provides visual feedback for configuration and testing
   - Different patterns confirm successful initialization of components
   - Error patterns help identify issues before launch

2. **Ascent Phase**:
   - Limited LED usage during initial ascent
   - Solid ON during active transmission periods
   - Brief flashes during telemetry cycles

3. **Float Phase**:
   - LED completely disabled to conserve power
   - No visual indication during long-duration flight

4. **Error Handling**:
   - Different flash patterns for different error types
   - Helps identify issues during pre-launch testing
   - Can be used for post-mortem analysis if recovered

## Implementation Notes

- Simple GPIO control for the red LED on PA0
- Direct LED toggling in functions (no timer-based approach)
- LED completely disabled during float mode
- Focus on pre-launch functionality for configuration verification
- Different flash patterns for different error types
- Minimal power consumption approach
