################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.c \
../archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_IAR.c \
../archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_KEIL.c \
../archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_SES.c 

OBJS += \
./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.o \
./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_IAR.o \
./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_KEIL.o \
./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_SES.o 

C_DEPS += \
./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.d \
./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_IAR.d \
./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_KEIL.d \
./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_SES.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/RTT/Syscalls/%.o archive/Reference/RTT/Syscalls/%.su archive/Reference/RTT/Syscalls/%.cyclo: ../archive/Reference/RTT/Syscalls/%.c archive/Reference/RTT/Syscalls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-RTT-2f-Syscalls

clean-archive-2f-Reference-2f-RTT-2f-Syscalls:
	-$(RM) ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.cyclo ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.d ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.o ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.su ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_IAR.cyclo ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_IAR.d ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_IAR.o ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_IAR.su ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_KEIL.cyclo ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_KEIL.d ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_KEIL.o ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_KEIL.su ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_SES.cyclo ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_SES.d ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_SES.o ./archive/Reference/RTT/Syscalls/SEGGER_RTT_Syscalls_SES.su

.PHONY: clean-archive-2f-Reference-2f-RTT-2f-Syscalls

