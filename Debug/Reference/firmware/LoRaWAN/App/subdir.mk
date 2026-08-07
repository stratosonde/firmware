################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Reference/firmware/LoRaWAN/App/CayenneLpp.c \
../Reference/firmware/LoRaWAN/App/app_lorawan.c \
../Reference/firmware/LoRaWAN/App/lora_app.c \
../Reference/firmware/LoRaWAN/App/lora_info.c 

OBJS += \
./Reference/firmware/LoRaWAN/App/CayenneLpp.o \
./Reference/firmware/LoRaWAN/App/app_lorawan.o \
./Reference/firmware/LoRaWAN/App/lora_app.o \
./Reference/firmware/LoRaWAN/App/lora_info.o 

C_DEPS += \
./Reference/firmware/LoRaWAN/App/CayenneLpp.d \
./Reference/firmware/LoRaWAN/App/app_lorawan.d \
./Reference/firmware/LoRaWAN/App/lora_app.d \
./Reference/firmware/LoRaWAN/App/lora_info.d 


# Each subdirectory must supply rules for building sources it contributes
Reference/firmware/LoRaWAN/App/%.o Reference/firmware/LoRaWAN/App/%.su Reference/firmware/LoRaWAN/App/%.cyclo: ../Reference/firmware/LoRaWAN/App/%.c Reference/firmware/LoRaWAN/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Reference-2f-firmware-2f-LoRaWAN-2f-App

clean-Reference-2f-firmware-2f-LoRaWAN-2f-App:
	-$(RM) ./Reference/firmware/LoRaWAN/App/CayenneLpp.cyclo ./Reference/firmware/LoRaWAN/App/CayenneLpp.d ./Reference/firmware/LoRaWAN/App/CayenneLpp.o ./Reference/firmware/LoRaWAN/App/CayenneLpp.su ./Reference/firmware/LoRaWAN/App/app_lorawan.cyclo ./Reference/firmware/LoRaWAN/App/app_lorawan.d ./Reference/firmware/LoRaWAN/App/app_lorawan.o ./Reference/firmware/LoRaWAN/App/app_lorawan.su ./Reference/firmware/LoRaWAN/App/lora_app.cyclo ./Reference/firmware/LoRaWAN/App/lora_app.d ./Reference/firmware/LoRaWAN/App/lora_app.o ./Reference/firmware/LoRaWAN/App/lora_app.su ./Reference/firmware/LoRaWAN/App/lora_info.cyclo ./Reference/firmware/LoRaWAN/App/lora_info.d ./Reference/firmware/LoRaWAN/App/lora_info.o ./Reference/firmware/LoRaWAN/App/lora_info.su

.PHONY: clean-Reference-2f-firmware-2f-LoRaWAN-2f-App

