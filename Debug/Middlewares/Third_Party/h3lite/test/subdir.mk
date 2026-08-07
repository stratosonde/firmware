################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/h3lite/test/h3lite_grid_test.c \
../Middlewares/Third_Party/h3lite/test/h3lite_nearest_test.c 

OBJS += \
./Middlewares/Third_Party/h3lite/test/h3lite_grid_test.o \
./Middlewares/Third_Party/h3lite/test/h3lite_nearest_test.o 

C_DEPS += \
./Middlewares/Third_Party/h3lite/test/h3lite_grid_test.d \
./Middlewares/Third_Party/h3lite/test/h3lite_nearest_test.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/h3lite/test/%.o Middlewares/Third_Party/h3lite/test/%.su Middlewares/Third_Party/h3lite/test/%.cyclo: ../Middlewares/Third_Party/h3lite/test/%.c Middlewares/Third_Party/h3lite/test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../LoRaWAN/App -I../LoRaWAN/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Middlewares/Third_Party/LoRaWAN/Mac -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/h3lite/include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-h3lite-2f-test

clean-Middlewares-2f-Third_Party-2f-h3lite-2f-test:
	-$(RM) ./Middlewares/Third_Party/h3lite/test/h3lite_grid_test.cyclo ./Middlewares/Third_Party/h3lite/test/h3lite_grid_test.d ./Middlewares/Third_Party/h3lite/test/h3lite_grid_test.o ./Middlewares/Third_Party/h3lite/test/h3lite_grid_test.su ./Middlewares/Third_Party/h3lite/test/h3lite_nearest_test.cyclo ./Middlewares/Third_Party/h3lite/test/h3lite_nearest_test.d ./Middlewares/Third_Party/h3lite/test/h3lite_nearest_test.o ./Middlewares/Third_Party/h3lite/test/h3lite_nearest_test.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-h3lite-2f-test

