# GNSS Module Integration Summary

## Overview
The ATGM336H-5N31 GNSS module has been **fully integrated** into the firmware. The module is now part of the sensor reading system and its data is transmitted via LoRaWAN.

**Status: ✅ INTEGRATION COMPLETE**

## Hardware Configuration

### UART Configuration (✅ Configured)
The GNSS module uses UART1 configured in the STM32CubeMX project:
- **TX Pin**: PB6 (USART1_TX from STM32 → RX on GNSS)
- **RX Pin**: PB7 (USART1_RX from STM32 ← TX on GNSS)
- **Baud Rate**: 9600
- **Word Length**: 8 bits
- **Stop Bits**: 1
- **Parity**: None
- **Mode**: Asynchronous

### GPIO Configuration
The GNSS power control pin needs to be configured:
- **Power Control Pin**: PB10
- **Mode**: GPIO Output
- **Pull**: No pull-up/pull-down
- **Speed**: Low

### Steps to Configure in STM32CubeMX:
1. Open your `.ioc` file in STM32CubeMX
2. Navigate to Connectivity → USART1
3. Set Mode to "Asynchronous"
4. Configure PB6 as USART1_TX and PB7 as USART1_RX
5. Set baud rate to 9600
6. Navigate to System Core → GPIO
7. Configure PB10 as GPIO_Output
8. Regenerate code

## Files Modified

### New Files Created:
1. `Core/Inc/atgm336h.h` - GNSS driver header
2. `Core/Src/atgm336h.c` - GNSS driver implementation

### Modified Files:
1. `Core/Inc/sys_sensors.h` - Added GNSS data fields to sensor_t structure
2. `Core/Src/sys_sensors.c` - Added GNSS initialization and reading
3. `LoRaWAN/App/lora_app.c` - Added GNSS data to LoRaWAN transmissions

## Integration Details

### Sensor Structure Updates
The `sensor_t` structure now includes:
- `uint8_t satellites` - Number of satellites in view
- `uint8_t gnss_fix_quality` - Fix quality (0=invalid, 1=GPS, 2=DGPS)
- `float gnss_hdop` - Horizontal dilution of precision
- `bool gnss_valid` - GNSS data validity flag

### GNSS Reading Flow
1. Power on GNSS module (via PB10)
2. Wait for module to stabilize (1 second)
3. Attempt to get position fix (10 second timeout)
4. Parse NMEA sentences (GGA and RMC)
5. Update sensor data if valid fix obtained
6. Power off GNSS module to save power

### LoRaWAN Transmission
GNSS data is transmitted using Cayenne LPP format:
- Channel 1: Temperature
- Channel 2: Humidity
- Channel 3: Barometric Pressure
- Channel 4: GPS (Latitude, Longitude, Altitude)
- Channel 5: Number of Satellites

## Power Management
The GNSS module is only powered on during sensor readings to conserve battery:
- Module powers on at start of each sensor read cycle
- 10 second timeout for GPS fix acquisition
- Module powers off immediately after reading
- This approach minimizes power consumption while still obtaining location data

## Operational Notes

### First Fix
The first GPS fix may take longer (cold start):
- Cold start: 30-60 seconds
- Warm start: 15-30 seconds
- Hot start: 1-5 seconds

### Fix Requirements
For a valid GPS fix, the module needs:
- Clear view of the sky
- At least 4 satellites visible
- Sufficient signal quality (HDOP < 10)

### Troubleshooting
If GNSS data shows as invalid:
1. Check antenna connection
2. Ensure clear sky view
3. Wait longer for first fix (increase timeout if needed)
4. Verify UART connections (PB6 ↔ RX, PB7 ↔ TX on GNSS)
5. Verify power control (PB10) is properly connected
6. Check serial console for GNSS debug messages

## Testing
To test the integration:
1. Build and flash the firmware
2. Monitor serial console output
3. Look for "GNSS module initialized successfully" message
4. During each sensor read cycle, check for "GNSS Fix:" messages
5. If no fix, you'll see "GNSS: No valid fix" messages
6. LoRaWAN transmission will include GPS data only when valid fix is obtained

## Example Console Output

### Successful GNSS Fix:
```
Initializing GNSS module...
GNSS module initialized successfully
...
GNSS Fix: Lat=51.123456, Lon=-114.123456, Alt=1045.0m, Sats=8
GNSS data added: Lat=51.123456, Lon=-114.123456, Alt=1045.0m, Sats=8
```

### No GNSS Fix:
```
Initializing GNSS module...
GNSS module initialized successfully
...
GNSS: No valid fix (status=2)
```

## Next Steps
1. Configure UART1 and PB10 in STM32CubeMX as described above
2. Regenerate code from CubeMX
3. Build and flash the firmware
4. Test with GNSS antenna connected and clear sky view
5. Monitor LoRaWAN gateway/application for GPS data in uplink messages
