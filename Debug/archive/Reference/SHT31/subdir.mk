################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/SHT31/sht3x.c \
../archive/Reference/SHT31/sht3x_example_usage.c 

OBJS += \
./archive/Reference/SHT31/sht3x.o \
./archive/Reference/SHT31/sht3x_example_usage.o 

C_DEPS += \
./archive/Reference/SHT31/sht3x.d \
./archive/Reference/SHT31/sht3x_example_usage.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/SHT31/%.o archive/Reference/SHT31/%.su archive/Reference/SHT31/%.cyclo: ../archive/Reference/SHT31/%.c archive/Reference/SHT31/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-SHT31

clean-archive-2f-Reference-2f-SHT31:
	-$(RM) ./archive/Reference/SHT31/sht3x.cyclo ./archive/Reference/SHT31/sht3x.d ./archive/Reference/SHT31/sht3x.o ./archive/Reference/SHT31/sht3x.su ./archive/Reference/SHT31/sht3x_example_usage.cyclo ./archive/Reference/SHT31/sht3x_example_usage.d ./archive/Reference/SHT31/sht3x_example_usage.o ./archive/Reference/SHT31/sht3x_example_usage.su

.PHONY: clean-archive-2f-Reference-2f-SHT31

