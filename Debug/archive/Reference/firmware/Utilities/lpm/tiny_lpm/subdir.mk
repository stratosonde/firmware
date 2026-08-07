################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/firmware/Utilities/lpm/tiny_lpm/stm32_lpm.c 

OBJS += \
./archive/Reference/firmware/Utilities/lpm/tiny_lpm/stm32_lpm.o 

C_DEPS += \
./archive/Reference/firmware/Utilities/lpm/tiny_lpm/stm32_lpm.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/Utilities/lpm/tiny_lpm/%.o archive/Reference/firmware/Utilities/lpm/tiny_lpm/%.su archive/Reference/firmware/Utilities/lpm/tiny_lpm/%.cyclo: ../archive/Reference/firmware/Utilities/lpm/tiny_lpm/%.c archive/Reference/firmware/Utilities/lpm/tiny_lpm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-firmware-2f-Utilities-2f-lpm-2f-tiny_lpm

clean-archive-2f-Reference-2f-firmware-2f-Utilities-2f-lpm-2f-tiny_lpm:
	-$(RM) ./archive/Reference/firmware/Utilities/lpm/tiny_lpm/stm32_lpm.cyclo ./archive/Reference/firmware/Utilities/lpm/tiny_lpm/stm32_lpm.d ./archive/Reference/firmware/Utilities/lpm/tiny_lpm/stm32_lpm.o ./archive/Reference/firmware/Utilities/lpm/tiny_lpm/stm32_lpm.su

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-Utilities-2f-lpm-2f-tiny_lpm

