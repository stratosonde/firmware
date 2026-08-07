################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/firmware/Core/Src/SEGGER_RTT.c \
../archive/Reference/firmware/Core/Src/adc_if.c \
../archive/Reference/firmware/Core/Src/atgm336h.c \
../archive/Reference/firmware/Core/Src/flash_if.c \
../archive/Reference/firmware/Core/Src/main.c \
../archive/Reference/firmware/Core/Src/ms5607.c \
../archive/Reference/firmware/Core/Src/sht31.c \
../archive/Reference/firmware/Core/Src/stm32_lpm_if.c \
../archive/Reference/firmware/Core/Src/stm32wlxx_hal_msp.c \
../archive/Reference/firmware/Core/Src/stm32wlxx_it.c \
../archive/Reference/firmware/Core/Src/sys_app.c \
../archive/Reference/firmware/Core/Src/sys_debug.c \
../archive/Reference/firmware/Core/Src/sys_sensors.c \
../archive/Reference/firmware/Core/Src/syscalls.c \
../archive/Reference/firmware/Core/Src/sysmem.c \
../archive/Reference/firmware/Core/Src/system_stm32wlxx.c \
../archive/Reference/firmware/Core/Src/timer_if.c \
../archive/Reference/firmware/Core/Src/usart_if.c 

OBJS += \
./archive/Reference/firmware/Core/Src/SEGGER_RTT.o \
./archive/Reference/firmware/Core/Src/adc_if.o \
./archive/Reference/firmware/Core/Src/atgm336h.o \
./archive/Reference/firmware/Core/Src/flash_if.o \
./archive/Reference/firmware/Core/Src/main.o \
./archive/Reference/firmware/Core/Src/ms5607.o \
./archive/Reference/firmware/Core/Src/sht31.o \
./archive/Reference/firmware/Core/Src/stm32_lpm_if.o \
./archive/Reference/firmware/Core/Src/stm32wlxx_hal_msp.o \
./archive/Reference/firmware/Core/Src/stm32wlxx_it.o \
./archive/Reference/firmware/Core/Src/sys_app.o \
./archive/Reference/firmware/Core/Src/sys_debug.o \
./archive/Reference/firmware/Core/Src/sys_sensors.o \
./archive/Reference/firmware/Core/Src/syscalls.o \
./archive/Reference/firmware/Core/Src/sysmem.o \
./archive/Reference/firmware/Core/Src/system_stm32wlxx.o \
./archive/Reference/firmware/Core/Src/timer_if.o \
./archive/Reference/firmware/Core/Src/usart_if.o 

C_DEPS += \
./archive/Reference/firmware/Core/Src/SEGGER_RTT.d \
./archive/Reference/firmware/Core/Src/adc_if.d \
./archive/Reference/firmware/Core/Src/atgm336h.d \
./archive/Reference/firmware/Core/Src/flash_if.d \
./archive/Reference/firmware/Core/Src/main.d \
./archive/Reference/firmware/Core/Src/ms5607.d \
./archive/Reference/firmware/Core/Src/sht31.d \
./archive/Reference/firmware/Core/Src/stm32_lpm_if.d \
./archive/Reference/firmware/Core/Src/stm32wlxx_hal_msp.d \
./archive/Reference/firmware/Core/Src/stm32wlxx_it.d \
./archive/Reference/firmware/Core/Src/sys_app.d \
./archive/Reference/firmware/Core/Src/sys_debug.d \
./archive/Reference/firmware/Core/Src/sys_sensors.d \
./archive/Reference/firmware/Core/Src/syscalls.d \
./archive/Reference/firmware/Core/Src/sysmem.d \
./archive/Reference/firmware/Core/Src/system_stm32wlxx.d \
./archive/Reference/firmware/Core/Src/timer_if.d \
./archive/Reference/firmware/Core/Src/usart_if.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/Core/Src/%.o archive/Reference/firmware/Core/Src/%.su archive/Reference/firmware/Core/Src/%.cyclo: ../archive/Reference/firmware/Core/Src/%.c archive/Reference/firmware/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-firmware-2f-Core-2f-Src

clean-archive-2f-Reference-2f-firmware-2f-Core-2f-Src:
	-$(RM) ./archive/Reference/firmware/Core/Src/SEGGER_RTT.cyclo ./archive/Reference/firmware/Core/Src/SEGGER_RTT.d ./archive/Reference/firmware/Core/Src/SEGGER_RTT.o ./archive/Reference/firmware/Core/Src/SEGGER_RTT.su ./archive/Reference/firmware/Core/Src/adc_if.cyclo ./archive/Reference/firmware/Core/Src/adc_if.d ./archive/Reference/firmware/Core/Src/adc_if.o ./archive/Reference/firmware/Core/Src/adc_if.su ./archive/Reference/firmware/Core/Src/atgm336h.cyclo ./archive/Reference/firmware/Core/Src/atgm336h.d ./archive/Reference/firmware/Core/Src/atgm336h.o ./archive/Reference/firmware/Core/Src/atgm336h.su ./archive/Reference/firmware/Core/Src/flash_if.cyclo ./archive/Reference/firmware/Core/Src/flash_if.d ./archive/Reference/firmware/Core/Src/flash_if.o ./archive/Reference/firmware/Core/Src/flash_if.su ./archive/Reference/firmware/Core/Src/main.cyclo ./archive/Reference/firmware/Core/Src/main.d ./archive/Reference/firmware/Core/Src/main.o ./archive/Reference/firmware/Core/Src/main.su ./archive/Reference/firmware/Core/Src/ms5607.cyclo ./archive/Reference/firmware/Core/Src/ms5607.d ./archive/Reference/firmware/Core/Src/ms5607.o ./archive/Reference/firmware/Core/Src/ms5607.su ./archive/Reference/firmware/Core/Src/sht31.cyclo ./archive/Reference/firmware/Core/Src/sht31.d ./archive/Reference/firmware/Core/Src/sht31.o ./archive/Reference/firmware/Core/Src/sht31.su ./archive/Reference/firmware/Core/Src/stm32_lpm_if.cyclo ./archive/Reference/firmware/Core/Src/stm32_lpm_if.d ./archive/Reference/firmware/Core/Src/stm32_lpm_if.o ./archive/Reference/firmware/Core/Src/stm32_lpm_if.su ./archive/Reference/firmware/Core/Src/stm32wlxx_hal_msp.cyclo ./archive/Reference/firmware/Core/Src/stm32wlxx_hal_msp.d ./archive/Reference/firmware/Core/Src/stm32wlxx_hal_msp.o ./archive/Reference/firmware/Core/Src/stm32wlxx_hal_msp.su ./archive/Reference/firmware/Core/Src/stm32wlxx_it.cyclo ./archive/Reference/firmware/Core/Src/stm32wlxx_it.d ./archive/Reference/firmware/Core/Src/stm32wlxx_it.o ./archive/Reference/firmware/Core/Src/stm32wlxx_it.su ./archive/Reference/firmware/Core/Src/sys_app.cyclo ./archive/Reference/firmware/Core/Src/sys_app.d ./archive/Reference/firmware/Core/Src/sys_app.o ./archive/Reference/firmware/Core/Src/sys_app.su ./archive/Reference/firmware/Core/Src/sys_debug.cyclo ./archive/Reference/firmware/Core/Src/sys_debug.d ./archive/Reference/firmware/Core/Src/sys_debug.o ./archive/Reference/firmware/Core/Src/sys_debug.su ./archive/Reference/firmware/Core/Src/sys_sensors.cyclo ./archive/Reference/firmware/Core/Src/sys_sensors.d ./archive/Reference/firmware/Core/Src/sys_sensors.o ./archive/Reference/firmware/Core/Src/sys_sensors.su ./archive/Reference/firmware/Core/Src/syscalls.cyclo ./archive/Reference/firmware/Core/Src/syscalls.d ./archive/Reference/firmware/Core/Src/syscalls.o ./archive/Reference/firmware/Core/Src/syscalls.su ./archive/Reference/firmware/Core/Src/sysmem.cyclo ./archive/Reference/firmware/Core/Src/sysmem.d ./archive/Reference/firmware/Core/Src/sysmem.o ./archive/Reference/firmware/Core/Src/sysmem.su ./archive/Reference/firmware/Core/Src/system_stm32wlxx.cyclo ./archive/Reference/firmware/Core/Src/system_stm32wlxx.d ./archive/Reference/firmware/Core/Src/system_stm32wlxx.o ./archive/Reference/firmware/Core/Src/system_stm32wlxx.su ./archive/Reference/firmware/Core/Src/timer_if.cyclo ./archive/Reference/firmware/Core/Src/timer_if.d ./archive/Reference/firmware/Core/Src/timer_if.o ./archive/Reference/firmware/Core/Src/timer_if.su ./archive/Reference/firmware/Core/Src/usart_if.cyclo ./archive/Reference/firmware/Core/Src/usart_if.d ./archive/Reference/firmware/Core/Src/usart_if.o ./archive/Reference/firmware/Core/Src/usart_if.su

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-Core-2f-Src

