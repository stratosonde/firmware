################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/h3lite/src/h3lite.c \
../Middlewares/Third_Party/h3lite/src/h3lite_basecells.c \
../Middlewares/Third_Party/h3lite/src/h3lite_faceijk.c \
../Middlewares/Third_Party/h3lite/src/h3lite_neighbor.c \
../Middlewares/Third_Party/h3lite/src/h3lite_regions_table.c 

OBJS += \
./Middlewares/Third_Party/h3lite/src/h3lite.o \
./Middlewares/Third_Party/h3lite/src/h3lite_basecells.o \
./Middlewares/Third_Party/h3lite/src/h3lite_faceijk.o \
./Middlewares/Third_Party/h3lite/src/h3lite_neighbor.o \
./Middlewares/Third_Party/h3lite/src/h3lite_regions_table.o 

C_DEPS += \
./Middlewares/Third_Party/h3lite/src/h3lite.d \
./Middlewares/Third_Party/h3lite/src/h3lite_basecells.d \
./Middlewares/Third_Party/h3lite/src/h3lite_faceijk.d \
./Middlewares/Third_Party/h3lite/src/h3lite_neighbor.d \
./Middlewares/Third_Party/h3lite/src/h3lite_regions_table.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/h3lite/src/%.o Middlewares/Third_Party/h3lite/src/%.su Middlewares/Third_Party/h3lite/src/%.cyclo: ../Middlewares/Third_Party/h3lite/src/%.c Middlewares/Third_Party/h3lite/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-h3lite-2f-src

clean-Middlewares-2f-Third_Party-2f-h3lite-2f-src:
	-$(RM) ./Middlewares/Third_Party/h3lite/src/h3lite.cyclo ./Middlewares/Third_Party/h3lite/src/h3lite.d ./Middlewares/Third_Party/h3lite/src/h3lite.o ./Middlewares/Third_Party/h3lite/src/h3lite.su ./Middlewares/Third_Party/h3lite/src/h3lite_basecells.cyclo ./Middlewares/Third_Party/h3lite/src/h3lite_basecells.d ./Middlewares/Third_Party/h3lite/src/h3lite_basecells.o ./Middlewares/Third_Party/h3lite/src/h3lite_basecells.su ./Middlewares/Third_Party/h3lite/src/h3lite_faceijk.cyclo ./Middlewares/Third_Party/h3lite/src/h3lite_faceijk.d ./Middlewares/Third_Party/h3lite/src/h3lite_faceijk.o ./Middlewares/Third_Party/h3lite/src/h3lite_faceijk.su ./Middlewares/Third_Party/h3lite/src/h3lite_neighbor.cyclo ./Middlewares/Third_Party/h3lite/src/h3lite_neighbor.d ./Middlewares/Third_Party/h3lite/src/h3lite_neighbor.o ./Middlewares/Third_Party/h3lite/src/h3lite_neighbor.su ./Middlewares/Third_Party/h3lite/src/h3lite_regions_table.cyclo ./Middlewares/Third_Party/h3lite/src/h3lite_regions_table.d ./Middlewares/Third_Party/h3lite/src/h3lite_regions_table.o ./Middlewares/Third_Party/h3lite/src/h3lite_regions_table.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-h3lite-2f-src

