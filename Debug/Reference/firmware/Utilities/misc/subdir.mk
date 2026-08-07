################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Reference/firmware/Utilities/misc/stm32_mem.c \
../Reference/firmware/Utilities/misc/stm32_systime.c \
../Reference/firmware/Utilities/misc/stm32_tiny_sscanf.c \
../Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.c 

OBJS += \
./Reference/firmware/Utilities/misc/stm32_mem.o \
./Reference/firmware/Utilities/misc/stm32_systime.o \
./Reference/firmware/Utilities/misc/stm32_tiny_sscanf.o \
./Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.o 

C_DEPS += \
./Reference/firmware/Utilities/misc/stm32_mem.d \
./Reference/firmware/Utilities/misc/stm32_systime.d \
./Reference/firmware/Utilities/misc/stm32_tiny_sscanf.d \
./Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.d 


# Each subdirectory must supply rules for building sources it contributes
Reference/firmware/Utilities/misc/%.o Reference/firmware/Utilities/misc/%.su Reference/firmware/Utilities/misc/%.cyclo: ../Reference/firmware/Utilities/misc/%.c Reference/firmware/Utilities/misc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Reference-2f-firmware-2f-Utilities-2f-misc

clean-Reference-2f-firmware-2f-Utilities-2f-misc:
	-$(RM) ./Reference/firmware/Utilities/misc/stm32_mem.cyclo ./Reference/firmware/Utilities/misc/stm32_mem.d ./Reference/firmware/Utilities/misc/stm32_mem.o ./Reference/firmware/Utilities/misc/stm32_mem.su ./Reference/firmware/Utilities/misc/stm32_systime.cyclo ./Reference/firmware/Utilities/misc/stm32_systime.d ./Reference/firmware/Utilities/misc/stm32_systime.o ./Reference/firmware/Utilities/misc/stm32_systime.su ./Reference/firmware/Utilities/misc/stm32_tiny_sscanf.cyclo ./Reference/firmware/Utilities/misc/stm32_tiny_sscanf.d ./Reference/firmware/Utilities/misc/stm32_tiny_sscanf.o ./Reference/firmware/Utilities/misc/stm32_tiny_sscanf.su ./Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.cyclo ./Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.d ./Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.o ./Reference/firmware/Utilities/misc/stm32_tiny_vsnprintf.su

.PHONY: clean-Reference-2f-firmware-2f-Utilities-2f-misc

