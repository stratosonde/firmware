################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/MS5607-STM32-SPI/MS5607SPI.c 

OBJS += \
./archive/Reference/MS5607-STM32-SPI/MS5607SPI.o 

C_DEPS += \
./archive/Reference/MS5607-STM32-SPI/MS5607SPI.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/MS5607-STM32-SPI/%.o archive/Reference/MS5607-STM32-SPI/%.su archive/Reference/MS5607-STM32-SPI/%.cyclo: ../archive/Reference/MS5607-STM32-SPI/%.c archive/Reference/MS5607-STM32-SPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-MS5607-2d-STM32-2d-SPI

clean-archive-2f-Reference-2f-MS5607-2d-STM32-2d-SPI:
	-$(RM) ./archive/Reference/MS5607-STM32-SPI/MS5607SPI.cyclo ./archive/Reference/MS5607-STM32-SPI/MS5607SPI.d ./archive/Reference/MS5607-STM32-SPI/MS5607SPI.o ./archive/Reference/MS5607-STM32-SPI/MS5607SPI.su

.PHONY: clean-archive-2f-Reference-2f-MS5607-2d-STM32-2d-SPI

