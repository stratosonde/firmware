################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.c 

OBJS += \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.o 

C_DEPS += \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.d 


# Each subdirectory must supply rules for building sources it contributes
Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/%.o Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/%.su Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/%.cyclo: ../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/%.c Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac

clean-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac:
	-$(RM) ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.su

.PHONY: clean-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac

