################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Reference/RTT/RTT/SEGGER_RTT.c \
../Reference/RTT/RTT/SEGGER_RTT_printf.c 

S_UPPER_SRCS += \
../Reference/RTT/RTT/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./Reference/RTT/RTT/SEGGER_RTT.o \
./Reference/RTT/RTT/SEGGER_RTT_ASM_ARMv7M.o \
./Reference/RTT/RTT/SEGGER_RTT_printf.o 

S_UPPER_DEPS += \
./Reference/RTT/RTT/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./Reference/RTT/RTT/SEGGER_RTT.d \
./Reference/RTT/RTT/SEGGER_RTT_printf.d 


# Each subdirectory must supply rules for building sources it contributes
Reference/RTT/RTT/%.o Reference/RTT/RTT/%.su Reference/RTT/RTT/%.cyclo: ../Reference/RTT/RTT/%.c Reference/RTT/RTT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Reference/RTT/RTT/%.o: ../Reference/RTT/RTT/%.S Reference/RTT/RTT/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Reference-2f-RTT-2f-RTT

clean-Reference-2f-RTT-2f-RTT:
	-$(RM) ./Reference/RTT/RTT/SEGGER_RTT.cyclo ./Reference/RTT/RTT/SEGGER_RTT.d ./Reference/RTT/RTT/SEGGER_RTT.o ./Reference/RTT/RTT/SEGGER_RTT.su ./Reference/RTT/RTT/SEGGER_RTT_ASM_ARMv7M.d ./Reference/RTT/RTT/SEGGER_RTT_ASM_ARMv7M.o ./Reference/RTT/RTT/SEGGER_RTT_printf.cyclo ./Reference/RTT/RTT/SEGGER_RTT_printf.d ./Reference/RTT/RTT/SEGGER_RTT_printf.o ./Reference/RTT/RTT/SEGGER_RTT_printf.su

.PHONY: clean-Reference-2f-RTT-2f-RTT

