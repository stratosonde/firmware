################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.c \
../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.c 

OBJS += \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.o \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.o 

C_DEPS += \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.d \
./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/%.o archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/%.su archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/%.cyclo: ../archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/%.c archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac-2f-Region

clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac-2f-Region:
	-$(RM) ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.su ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.cyclo ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.d ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.o ./archive/Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.su

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac-2f-Region

