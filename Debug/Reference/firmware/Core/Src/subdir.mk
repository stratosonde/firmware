################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Reference/firmware/Core/Src/SEGGER_RTT.c \
../Reference/firmware/Core/Src/adc_if.c \
../Reference/firmware/Core/Src/atgm336h.c \
../Reference/firmware/Core/Src/flash_if.c \
../Reference/firmware/Core/Src/main.c \
../Reference/firmware/Core/Src/ms5607.c \
../Reference/firmware/Core/Src/sht31.c \
../Reference/firmware/Core/Src/stm32_lpm_if.c \
../Reference/firmware/Core/Src/stm32wlxx_hal_msp.c \
../Reference/firmware/Core/Src/stm32wlxx_it.c \
../Reference/firmware/Core/Src/sys_app.c \
../Reference/firmware/Core/Src/sys_debug.c \
../Reference/firmware/Core/Src/sys_sensors.c \
../Reference/firmware/Core/Src/syscalls.c \
../Reference/firmware/Core/Src/sysmem.c \
../Reference/firmware/Core/Src/system_stm32wlxx.c \
../Reference/firmware/Core/Src/timer_if.c \
../Reference/firmware/Core/Src/usart_if.c 

OBJS += \
./Reference/firmware/Core/Src/SEGGER_RTT.o \
./Reference/firmware/Core/Src/adc_if.o \
./Reference/firmware/Core/Src/atgm336h.o \
./Reference/firmware/Core/Src/flash_if.o \
./Reference/firmware/Core/Src/main.o \
./Reference/firmware/Core/Src/ms5607.o \
./Reference/firmware/Core/Src/sht31.o \
./Reference/firmware/Core/Src/stm32_lpm_if.o \
./Reference/firmware/Core/Src/stm32wlxx_hal_msp.o \
./Reference/firmware/Core/Src/stm32wlxx_it.o \
./Reference/firmware/Core/Src/sys_app.o \
./Reference/firmware/Core/Src/sys_debug.o \
./Reference/firmware/Core/Src/sys_sensors.o \
./Reference/firmware/Core/Src/syscalls.o \
./Reference/firmware/Core/Src/sysmem.o \
./Reference/firmware/Core/Src/system_stm32wlxx.o \
./Reference/firmware/Core/Src/timer_if.o \
./Reference/firmware/Core/Src/usart_if.o 

C_DEPS += \
./Reference/firmware/Core/Src/SEGGER_RTT.d \
./Reference/firmware/Core/Src/adc_if.d \
./Reference/firmware/Core/Src/atgm336h.d \
./Reference/firmware/Core/Src/flash_if.d \
./Reference/firmware/Core/Src/main.d \
./Reference/firmware/Core/Src/ms5607.d \
./Reference/firmware/Core/Src/sht31.d \
./Reference/firmware/Core/Src/stm32_lpm_if.d \
./Reference/firmware/Core/Src/stm32wlxx_hal_msp.d \
./Reference/firmware/Core/Src/stm32wlxx_it.d \
./Reference/firmware/Core/Src/sys_app.d \
./Reference/firmware/Core/Src/sys_debug.d \
./Reference/firmware/Core/Src/sys_sensors.d \
./Reference/firmware/Core/Src/syscalls.d \
./Reference/firmware/Core/Src/sysmem.d \
./Reference/firmware/Core/Src/system_stm32wlxx.d \
./Reference/firmware/Core/Src/timer_if.d \
./Reference/firmware/Core/Src/usart_if.d 


# Each subdirectory must supply rules for building sources it contributes
Reference/firmware/Core/Src/%.o Reference/firmware/Core/Src/%.su Reference/firmware/Core/Src/%.cyclo: ../Reference/firmware/Core/Src/%.c Reference/firmware/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Reference-2f-firmware-2f-Core-2f-Src

clean-Reference-2f-firmware-2f-Core-2f-Src:
	-$(RM) ./Reference/firmware/Core/Src/SEGGER_RTT.cyclo ./Reference/firmware/Core/Src/SEGGER_RTT.d ./Reference/firmware/Core/Src/SEGGER_RTT.o ./Reference/firmware/Core/Src/SEGGER_RTT.su ./Reference/firmware/Core/Src/adc_if.cyclo ./Reference/firmware/Core/Src/adc_if.d ./Reference/firmware/Core/Src/adc_if.o ./Reference/firmware/Core/Src/adc_if.su ./Reference/firmware/Core/Src/atgm336h.cyclo ./Reference/firmware/Core/Src/atgm336h.d ./Reference/firmware/Core/Src/atgm336h.o ./Reference/firmware/Core/Src/atgm336h.su ./Reference/firmware/Core/Src/flash_if.cyclo ./Reference/firmware/Core/Src/flash_if.d ./Reference/firmware/Core/Src/flash_if.o ./Reference/firmware/Core/Src/flash_if.su ./Reference/firmware/Core/Src/main.cyclo ./Reference/firmware/Core/Src/main.d ./Reference/firmware/Core/Src/main.o ./Reference/firmware/Core/Src/main.su ./Reference/firmware/Core/Src/ms5607.cyclo ./Reference/firmware/Core/Src/ms5607.d ./Reference/firmware/Core/Src/ms5607.o ./Reference/firmware/Core/Src/ms5607.su ./Reference/firmware/Core/Src/sht31.cyclo ./Reference/firmware/Core/Src/sht31.d ./Reference/firmware/Core/Src/sht31.o ./Reference/firmware/Core/Src/sht31.su ./Reference/firmware/Core/Src/stm32_lpm_if.cyclo ./Reference/firmware/Core/Src/stm32_lpm_if.d ./Reference/firmware/Core/Src/stm32_lpm_if.o ./Reference/firmware/Core/Src/stm32_lpm_if.su ./Reference/firmware/Core/Src/stm32wlxx_hal_msp.cyclo ./Reference/firmware/Core/Src/stm32wlxx_hal_msp.d ./Reference/firmware/Core/Src/stm32wlxx_hal_msp.o ./Reference/firmware/Core/Src/stm32wlxx_hal_msp.su ./Reference/firmware/Core/Src/stm32wlxx_it.cyclo ./Reference/firmware/Core/Src/stm32wlxx_it.d ./Reference/firmware/Core/Src/stm32wlxx_it.o ./Reference/firmware/Core/Src/stm32wlxx_it.su ./Reference/firmware/Core/Src/sys_app.cyclo ./Reference/firmware/Core/Src/sys_app.d ./Reference/firmware/Core/Src/sys_app.o ./Reference/firmware/Core/Src/sys_app.su ./Reference/firmware/Core/Src/sys_debug.cyclo ./Reference/firmware/Core/Src/sys_debug.d ./Reference/firmware/Core/Src/sys_debug.o ./Reference/firmware/Core/Src/sys_debug.su ./Reference/firmware/Core/Src/sys_sensors.cyclo ./Reference/firmware/Core/Src/sys_sensors.d ./Reference/firmware/Core/Src/sys_sensors.o ./Reference/firmware/Core/Src/sys_sensors.su ./Reference/firmware/Core/Src/syscalls.cyclo ./Reference/firmware/Core/Src/syscalls.d ./Reference/firmware/Core/Src/syscalls.o ./Reference/firmware/Core/Src/syscalls.su ./Reference/firmware/Core/Src/sysmem.cyclo ./Reference/firmware/Core/Src/sysmem.d ./Reference/firmware/Core/Src/sysmem.o ./Reference/firmware/Core/Src/sysmem.su ./Reference/firmware/Core/Src/system_stm32wlxx.cyclo ./Reference/firmware/Core/Src/system_stm32wlxx.d ./Reference/firmware/Core/Src/system_stm32wlxx.o ./Reference/firmware/Core/Src/system_stm32wlxx.su ./Reference/firmware/Core/Src/timer_if.cyclo ./Reference/firmware/Core/Src/timer_if.d ./Reference/firmware/Core/Src/timer_if.o ./Reference/firmware/Core/Src/timer_if.su ./Reference/firmware/Core/Src/usart_if.cyclo ./Reference/firmware/Core/Src/usart_if.d ./Reference/firmware/Core/Src/usart_if.o ./Reference/firmware/Core/Src/usart_if.su

.PHONY: clean-Reference-2f-firmware-2f-Core-2f-Src

