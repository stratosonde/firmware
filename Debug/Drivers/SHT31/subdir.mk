################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SHT31/sht3x.c \
../Drivers/SHT31/sht3x_example_usage.c 

OBJS += \
./Drivers/SHT31/sht3x.o \
./Drivers/SHT31/sht3x_example_usage.o 

C_DEPS += \
./Drivers/SHT31/sht3x.d \
./Drivers/SHT31/sht3x_example_usage.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SHT31/%.o Drivers/SHT31/%.su Drivers/SHT31/%.cyclo: ../Drivers/SHT31/%.c Drivers/SHT31/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-SHT31

clean-Drivers-2f-SHT31:
	-$(RM) ./Drivers/SHT31/sht3x.cyclo ./Drivers/SHT31/sht3x.d ./Drivers/SHT31/sht3x.o ./Drivers/SHT31/sht3x.su ./Drivers/SHT31/sht3x_example_usage.cyclo ./Drivers/SHT31/sht3x_example_usage.d ./Drivers/SHT31/sht3x_example_usage.o ./Drivers/SHT31/sht3x_example_usage.su

.PHONY: clean-Drivers-2f-SHT31

