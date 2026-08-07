################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.c \
../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.c 

OBJS += \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.o \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.o 

C_DEPS += \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.d \
./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.d 


# Each subdirectory must supply rules for building sources it contributes
Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/%.o Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/%.su Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/%.cyclo: ../Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/%.c Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac-2f-Region

clean-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac-2f-Region:
	-$(RM) ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/Region.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAS923.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionAU915.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionBaseUS.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A20.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470A26.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B20.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN470B26.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCN779.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionCommon.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU433.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionEU868.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionIN865.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionKR920.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionRU864.su ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.cyclo ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.d ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.o ./Reference/firmware/Middlewares/Third_Party/LoRaWAN/Mac/Region/RegionUS915.su

.PHONY: clean-Reference-2f-firmware-2f-Middlewares-2f-Third_Party-2f-LoRaWAN-2f-Mac-2f-Region

