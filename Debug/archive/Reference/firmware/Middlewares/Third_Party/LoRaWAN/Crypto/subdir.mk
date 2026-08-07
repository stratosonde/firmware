################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/cmac.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/lorawan_aes.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/soft-se.c 

OBJS += \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/cmac.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/lorawan_aes.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/soft-se.o 

C_DEPS += \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/cmac.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/lorawan_aes.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/soft-se.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/%.o archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/%.su archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/%.cyclo: ../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/%.c archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Crypto

clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Crypto:
	-$(RM) ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/cmac.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/cmac.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/cmac.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/cmac.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/lorawan_aes.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/lorawan_aes.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/lorawan_aes.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/lorawan_aes.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/soft-se.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/soft-se.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/soft-se.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Crypto/soft-se.su

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Crypto

