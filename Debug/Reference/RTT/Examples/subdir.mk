################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Reference/RTT/Examples/Main_RTT_InputEchoApp.c \
../Reference/RTT/Examples/Main_RTT_MenuApp.c \
../Reference/RTT/Examples/Main_RTT_PrintfTest.c \
../Reference/RTT/Examples/Main_RTT_SpeedTestApp.c 

OBJS += \
./Reference/RTT/Examples/Main_RTT_InputEchoApp.o \
./Reference/RTT/Examples/Main_RTT_MenuApp.o \
./Reference/RTT/Examples/Main_RTT_PrintfTest.o \
./Reference/RTT/Examples/Main_RTT_SpeedTestApp.o 

C_DEPS += \
./Reference/RTT/Examples/Main_RTT_InputEchoApp.d \
./Reference/RTT/Examples/Main_RTT_MenuApp.d \
./Reference/RTT/Examples/Main_RTT_PrintfTest.d \
./Reference/RTT/Examples/Main_RTT_SpeedTestApp.d 


# Each subdirectory must supply rules for building sources it contributes
Reference/RTT/Examples/%.o Reference/RTT/Examples/%.su Reference/RTT/Examples/%.cyclo: ../Reference/RTT/Examples/%.c Reference/RTT/Examples/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Reference-2f-RTT-2f-Examples

clean-Reference-2f-RTT-2f-Examples:
	-$(RM) ./Reference/RTT/Examples/Main_RTT_InputEchoApp.cyclo ./Reference/RTT/Examples/Main_RTT_InputEchoApp.d ./Reference/RTT/Examples/Main_RTT_InputEchoApp.o ./Reference/RTT/Examples/Main_RTT_InputEchoApp.su ./Reference/RTT/Examples/Main_RTT_MenuApp.cyclo ./Reference/RTT/Examples/Main_RTT_MenuApp.d ./Reference/RTT/Examples/Main_RTT_MenuApp.o ./Reference/RTT/Examples/Main_RTT_MenuApp.su ./Reference/RTT/Examples/Main_RTT_PrintfTest.cyclo ./Reference/RTT/Examples/Main_RTT_PrintfTest.d ./Reference/RTT/Examples/Main_RTT_PrintfTest.o ./Reference/RTT/Examples/Main_RTT_PrintfTest.su ./Reference/RTT/Examples/Main_RTT_SpeedTestApp.cyclo ./Reference/RTT/Examples/Main_RTT_SpeedTestApp.d ./Reference/RTT/Examples/Main_RTT_SpeedTestApp.o ./Reference/RTT/Examples/Main_RTT_SpeedTestApp.su

.PHONY: clean-Reference-2f-RTT-2f-Examples

