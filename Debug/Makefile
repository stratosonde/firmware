################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

# PIPE-04 (#265, Phase 6): centralized compile-time definitions, appended to
# every active C compile recipe (see the subdir.mk pattern rules). The bench
# default is EMPTY (debug build); the flight configuration is built WITHOUT
# source mutation:
#   make -C Debug clean
#   make -C Debug all PROJECT_CPPFLAGS=-DSONDE_FLIGHT_BUILD
# tools/check_project_cppflags.py proves every active C recipe honours it.
PROJECT_CPPFLAGS ?=

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Utilities/trace/adv_trace/subdir.mk
-include Utilities/timer/subdir.mk
-include Utilities/sequencer/subdir.mk
-include Utilities/misc/subdir.mk
-include Utilities/lpm/tiny_lpm/subdir.mk
-include Middlewares/Third_Party/h3lite/src/subdir.mk
-include Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Utilities/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Mac/Region/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Mac/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/LmHandler/Packages/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/LmHandler/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Crypto/subdir.mk
-include LoRaWAN/Target/subdir.mk
-include LoRaWAN/App/subdir.mk
-include Drivers/STM32WLxx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Radio_Sonde_E5_HF_EU
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Radio_Sonde_E5_HF_EU.elf \

MAP_FILES += \
Radio_Sonde_E5_HF_EU.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Radio_Sonde_E5_HF_EU.list \


# All Target
all: main-build

# Main-build Target
main-build: Radio_Sonde_E5_HF_EU.elf secondary-outputs

# Tool invocations
# Linker script via a portable RELATIVE path — the CubeMX-generated absolute
# Windows path (C:\...) broke Linux make ("multiple target patterns" from the
# drive-letter colon) and broke any checkout at a different location.
LDSCRIPT ?= ../STM32WLE5JCIX_FLASH.ld

Radio_Sonde_E5_HF_EU.elf Radio_Sonde_E5_HF_EU.map: $(OBJS) $(USER_OBJS) $(LDSCRIPT) Makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Radio_Sonde_E5_HF_EU.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"$(LDSCRIPT)" --specs=nosys.specs -Wl,-Map="Radio_Sonde_E5_HF_EU.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) Makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Radio_Sonde_E5_HF_EU.list: $(EXECUTABLES) Makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Radio_Sonde_E5_HF_EU.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Radio_Sonde_E5_HF_EU.elf Radio_Sonde_E5_HF_EU.list Radio_Sonde_E5_HF_EU.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
