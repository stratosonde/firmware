################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio.c \
../archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.c \
../archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_fw.c 

OBJS += \
./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio.o \
./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.o \
./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_fw.o 

C_DEPS += \
./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio.d \
./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.d \
./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_fw.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/%.o archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/%.su archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/%.cyclo: ../archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/%.c archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-SubGHz_Phy-2f-stm32_radio_driver

clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-SubGHz_Phy-2f-stm32_radio_driver:
	-$(RM) ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio.d ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio.o ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio.su ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.d ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.o ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.su ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_fw.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_fw.d ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_fw.o ./archive/Reference/firmware/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_fw.su

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-SubGHz_Phy-2f-stm32_radio_driver

