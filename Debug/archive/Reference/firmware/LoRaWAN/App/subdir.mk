################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/firmware/LoRaWAN/App/CayenneLpp.c \
../archive/Reference/firmware/LoRaWAN/App/app_lorawan.c \
../archive/Reference/firmware/LoRaWAN/App/lora_app.c \
../archive/Reference/firmware/LoRaWAN/App/lora_info.c 

OBJS += \
./archive/Reference/firmware/LoRaWAN/App/CayenneLpp.o \
./archive/Reference/firmware/LoRaWAN/App/app_lorawan.o \
./archive/Reference/firmware/LoRaWAN/App/lora_app.o \
./archive/Reference/firmware/LoRaWAN/App/lora_info.o 

C_DEPS += \
./archive/Reference/firmware/LoRaWAN/App/CayenneLpp.d \
./archive/Reference/firmware/LoRaWAN/App/app_lorawan.d \
./archive/Reference/firmware/LoRaWAN/App/lora_app.d \
./archive/Reference/firmware/LoRaWAN/App/lora_info.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/LoRaWAN/App/%.o archive/Reference/firmware/LoRaWAN/App/%.su archive/Reference/firmware/LoRaWAN/App/%.cyclo: ../archive/Reference/firmware/LoRaWAN/App/%.c archive/Reference/firmware/LoRaWAN/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-firmware-2f-LoRaWAN-2f-App

clean-archive-2f-Reference-2f-firmware-2f-LoRaWAN-2f-App:
	-$(RM) ./archive/Reference/firmware/LoRaWAN/App/CayenneLpp.cyclo ./archive/Reference/firmware/LoRaWAN/App/CayenneLpp.d ./archive/Reference/firmware/LoRaWAN/App/CayenneLpp.o ./archive/Reference/firmware/LoRaWAN/App/CayenneLpp.su ./archive/Reference/firmware/LoRaWAN/App/app_lorawan.cyclo ./archive/Reference/firmware/LoRaWAN/App/app_lorawan.d ./archive/Reference/firmware/LoRaWAN/App/app_lorawan.o ./archive/Reference/firmware/LoRaWAN/App/app_lorawan.su ./archive/Reference/firmware/LoRaWAN/App/lora_app.cyclo ./archive/Reference/firmware/LoRaWAN/App/lora_app.d ./archive/Reference/firmware/LoRaWAN/App/lora_app.o ./archive/Reference/firmware/LoRaWAN/App/lora_app.su ./archive/Reference/firmware/LoRaWAN/App/lora_info.cyclo ./archive/Reference/firmware/LoRaWAN/App/lora_info.d ./archive/Reference/firmware/LoRaWAN/App/lora_info.o ./archive/Reference/firmware/LoRaWAN/App/lora_info.su

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-LoRaWAN-2f-App

