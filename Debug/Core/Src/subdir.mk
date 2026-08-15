################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/SEGGER_RTT.c \
../Core/Src/adc_if.c \
../Core/Src/atgm336h.c \
../Core/Src/config.c \
../Core/Src/flash_if.c \
../Core/Src/flash_log.c \
../Core/Src/first_flight_policy.c \
../Core/Src/main.c \
../Core/Src/mission_state.c \
../Core/Src/mission_logic.c \
../Core/Src/ms5607.c \
../Core/Src/reset_cause.c \
../Core/Src/multiregion_context.c \
../Core/Src/multiregion_h3.c \
../Core/Src/nvm_slot.c \
../Core/Src/payload_encode.c \
../Core/Src/packet_queue.c \
../Core/Src/power_model.c \
../Core/Src/region_policy.c \
../Core/Src/transmit_plan.c \
../Core/Src/sht31.c \
../Core/Src/stm32_lpm_if.c \
../Core/Src/stm32wlxx_hal_msp.c \
../Core/Src/stm32wlxx_it.c \
../Core/Src/sys_app.c \
../Core/Src/sys_caps.c \
../Core/Src/sys_debug.c \
../Core/Src/sys_sensors.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32wlxx.c \
../Core/Src/timer_if.c \
../Core/Src/usart_if.c \
../Core/Src/w25q16jv.c 

OBJS += \
./Core/Src/SEGGER_RTT.o \
./Core/Src/adc_if.o \
./Core/Src/atgm336h.o \
./Core/Src/config.o \
./Core/Src/flash_if.o \
./Core/Src/flash_log.o \
./Core/Src/first_flight_policy.o \
./Core/Src/main.o \
./Core/Src/mission_state.o \
./Core/Src/mission_logic.o \
./Core/Src/ms5607.o \
./Core/Src/reset_cause.o \
./Core/Src/multiregion_context.o \
./Core/Src/multiregion_h3.o \
./Core/Src/nvm_slot.o \
./Core/Src/payload_encode.o \
./Core/Src/packet_queue.o \
./Core/Src/power_model.o \
./Core/Src/region_policy.o \
./Core/Src/transmit_plan.o \
./Core/Src/sht31.o \
./Core/Src/stm32_lpm_if.o \
./Core/Src/stm32wlxx_hal_msp.o \
./Core/Src/stm32wlxx_it.o \
./Core/Src/sys_app.o \
./Core/Src/sys_caps.o \
./Core/Src/sys_debug.o \
./Core/Src/sys_sensors.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32wlxx.o \
./Core/Src/timer_if.o \
./Core/Src/usart_if.o \
./Core/Src/w25q16jv.o 

C_DEPS += \
./Core/Src/SEGGER_RTT.d \
./Core/Src/adc_if.d \
./Core/Src/atgm336h.d \
./Core/Src/config.d \
./Core/Src/flash_if.d \
./Core/Src/flash_log.d \
./Core/Src/first_flight_policy.d \
./Core/Src/main.d \
./Core/Src/mission_state.d \
./Core/Src/mission_logic.d \
./Core/Src/ms5607.d \
./Core/Src/reset_cause.d \
./Core/Src/multiregion_context.d \
./Core/Src/multiregion_h3.d \
./Core/Src/nvm_slot.d \
./Core/Src/payload_encode.d \
./Core/Src/packet_queue.d \
./Core/Src/power_model.d \
./Core/Src/region_policy.d \
./Core/Src/transmit_plan.d \
./Core/Src/sht31.d \
./Core/Src/stm32_lpm_if.d \
./Core/Src/stm32wlxx_hal_msp.d \
./Core/Src/stm32wlxx_it.d \
./Core/Src/sys_app.d \
./Core/Src/sys_caps.d \
./Core/Src/sys_debug.d \
./Core/Src/sys_sensors.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32wlxx.d \
./Core/Src/timer_if.d \
./Core/Src/usart_if.d \
./Core/Src/w25q16jv.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/SEGGER_RTT.cyclo ./Core/Src/SEGGER_RTT.d ./Core/Src/SEGGER_RTT.o ./Core/Src/SEGGER_RTT.su ./Core/Src/adc_if.cyclo ./Core/Src/adc_if.d ./Core/Src/adc_if.o ./Core/Src/adc_if.su ./Core/Src/atgm336h.cyclo ./Core/Src/atgm336h.d ./Core/Src/atgm336h.o ./Core/Src/atgm336h.su ./Core/Src/config.cyclo ./Core/Src/config.d ./Core/Src/config.o ./Core/Src/config.su ./Core/Src/flash_if.cyclo ./Core/Src/flash_if.d ./Core/Src/flash_if.o ./Core/Src/flash_if.su ./Core/Src/flash_log.cyclo ./Core/Src/flash_log.d ./Core/Src/flash_log.o ./Core/Src/flash_log.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ms5607.cyclo ./Core/Src/ms5607.d ./Core/Src/ms5607.o ./Core/Src/ms5607.su ./Core/Src/multiregion_context.cyclo ./Core/Src/multiregion_context.d ./Core/Src/multiregion_context.o ./Core/Src/multiregion_context.su ./Core/Src/multiregion_h3.cyclo ./Core/Src/multiregion_h3.d ./Core/Src/multiregion_h3.o ./Core/Src/multiregion_h3.su ./Core/Src/payload_encode.cyclo ./Core/Src/payload_encode.d ./Core/Src/payload_encode.o ./Core/Src/payload_encode.su ./Core/Src/power_model.cyclo ./Core/Src/power_model.d ./Core/Src/power_model.o ./Core/Src/power_model.su ./Core/Src/transmit_plan.cyclo ./Core/Src/transmit_plan.d ./Core/Src/transmit_plan.o ./Core/Src/transmit_plan.su ./Core/Src/sht31.cyclo ./Core/Src/sht31.d ./Core/Src/sht31.o ./Core/Src/sht31.su ./Core/Src/stm32_lpm_if.cyclo ./Core/Src/stm32_lpm_if.d ./Core/Src/stm32_lpm_if.o ./Core/Src/stm32_lpm_if.su ./Core/Src/stm32wlxx_hal_msp.cyclo ./Core/Src/stm32wlxx_hal_msp.d ./Core/Src/stm32wlxx_hal_msp.o ./Core/Src/stm32wlxx_hal_msp.su ./Core/Src/stm32wlxx_it.cyclo ./Core/Src/stm32wlxx_it.d ./Core/Src/stm32wlxx_it.o ./Core/Src/stm32wlxx_it.su ./Core/Src/sys_app.cyclo ./Core/Src/sys_app.d ./Core/Src/sys_app.o ./Core/Src/sys_app.su ./Core/Src/sys_caps.cyclo ./Core/Src/sys_caps.d ./Core/Src/sys_caps.o ./Core/Src/sys_caps.su ./Core/Src/sys_debug.cyclo ./Core/Src/sys_debug.d ./Core/Src/sys_debug.o ./Core/Src/sys_debug.su ./Core/Src/sys_sensors.cyclo ./Core/Src/sys_sensors.d ./Core/Src/sys_sensors.o ./Core/Src/sys_sensors.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32wlxx.cyclo ./Core/Src/system_stm32wlxx.d ./Core/Src/system_stm32wlxx.o ./Core/Src/system_stm32wlxx.su ./Core/Src/timer_if.cyclo ./Core/Src/timer_if.d ./Core/Src/timer_if.o ./Core/Src/timer_if.su ./Core/Src/usart_if.cyclo ./Core/Src/usart_if.d ./Core/Src/usart_if.o ./Core/Src/usart_if.su ./Core/Src/w25q16jv.cyclo ./Core/Src/w25q16jv.d ./Core/Src/w25q16jv.o ./Core/Src/w25q16jv.su

	-$(RM) ./Core/Src/first_flight_policy.cyclo ./Core/Src/first_flight_policy.d ./Core/Src/first_flight_policy.o ./Core/Src/first_flight_policy.su ./Core/Src/packet_queue.cyclo ./Core/Src/packet_queue.d ./Core/Src/packet_queue.o ./Core/Src/packet_queue.su ./Core/Src/nvm_slot.cyclo ./Core/Src/nvm_slot.d ./Core/Src/nvm_slot.o ./Core/Src/nvm_slot.su ./Core/Src/region_policy.cyclo ./Core/Src/region_policy.d ./Core/Src/region_policy.o ./Core/Src/region_policy.su

.PHONY: clean-Core-2f-Src
