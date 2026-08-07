################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/LmHandler.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/NvmDataMgmt.c 

OBJS += \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/LmHandler.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/NvmDataMgmt.o 

C_DEPS += \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/LmHandler.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/NvmDataMgmt.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/%.o archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/%.su archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/%.cyclo: ../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/%.c archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-LmHandler

clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-LmHandler:
	-$(RM) ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/LmHandler.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/LmHandler.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/LmHandler.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/LmHandler.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/NvmDataMgmt.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/NvmDataMgmt.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/NvmDataMgmt.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/LmHandler/NvmDataMgmt.su

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-LmHandler

