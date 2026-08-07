################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.c 

OBJS += \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.o 

C_DEPS += \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/%.o archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/%.su archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/%.cyclo: ../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/%.c archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac

clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac:
	-$(RM) ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacAdr.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacClassB.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCommands.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacParser.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.su

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac

