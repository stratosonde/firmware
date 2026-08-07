################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../archive/Reference/firmware/Core/Startup/startup_stm32wle5jcix.s 

OBJS += \
./archive/Reference/firmware/Core/Startup/startup_stm32wle5jcix.o 

S_DEPS += \
./archive/Reference/firmware/Core/Startup/startup_stm32wle5jcix.d 


# Each subdirectory must supply rules for building sources it contributes
archive/Reference/firmware/Core/Startup/%.o: ../archive/Reference/firmware/Core/Startup/%.s archive/Reference/firmware/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-archive-2f-Reference-2f-firmware-2f-Core-2f-Startup

clean-archive-2f-Reference-2f-firmware-2f-Core-2f-Startup:
	-$(RM) ./archive/Reference/firmware/Core/Startup/startup_stm32wle5jcix.d ./archive/Reference/firmware/Core/Startup/startup_stm32wle5jcix.o

.PHONY: clean-archive-2f-Reference-2f-firmware-2f-Core-2f-Startup

