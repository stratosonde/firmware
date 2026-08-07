################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/firmware/Utilities/misc/stm32_mem.c \
../archive/Reference/firmware/Utilities/misc/stm32_systime.c \
../archive/Reference/firmware/Utilities/misc/stm32_tiny_sscanf.c \
../archive/Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.c 

OBJS += \
./archive/Reference/firmware/Utilities/misc/stm32_mem.o \
./archive/Reference/firmware/Utilities/misc/stm32_systime.o \
./archive/Reference/firmware/Utilities/misc/stm32_tiny_sscanf.o \
./archive/Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.o 

C_DEPS += \
./archive/Reference/firmware/Utilities/misc/stm32_mem.d \
./archive/Reference/firmware/Utilities/misc/stm32_systime.d \
./archive/Reference/firmware/Utilities/misc/stm32_tiny_sscanf.d \
./archive/Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/Utilities/misc/%.o archive/Reference/firmware/Utilities/misc/%.su archive/Reference/firmware/Utilities/misc/%.cyclo: ../archive/Reference/firmware/Utilities/misc/%.c archive/Reference/firmware/Utilities/misc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-firmware-2f-Utilities-2f-misc

clean-archive-2f-Reference-2f-firmware-2f-Utilities-2f-misc:
	-$(RM) ./archive/Reference/firmware/Utilities/misc/stm32_mem.cyclo ./archive/Reference/firmware/Utilities/misc/stm32_mem.d ./archive/Reference/firmware/Utilities/misc/stm32_mem.o ./archive/Reference/firmware/Utilities/misc/stm32_mem.su ./archive/Reference/firmware/Utilities/misc/stm32_systime.cyclo ./archive/Reference/firmware/Utilities/misc/stm32_systime.d ./archive/Reference/firmware/Utilities/misc/stm32_systime.o ./archive/Reference/firmware/Utilities/misc/stm32_systime.su ./archive/Reference/firmware/Utilities/misc/stm32_tiny_sscanf.cyclo ./archive/Reference/firmware/Utilities/misc/stm32_tiny_sscanf.d ./archive/Reference/firmware/Utilities/misc/stm32_tiny_sscanf.o ./archive/Reference/firmware/Utilities/misc/stm32_tiny_sscanf.su ./archive/Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.cyclo ./archive/Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.d ./archive/Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.o ./archive/Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.su

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-Utilities-2f-misc

