################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Utilities/trace/adv_trace/subdir.mk
-include Utilities/timer/subdir.mk
-include Utilities/sequencer/subdir.mk
-include Utilities/misc/subdir.mk
-include Utilities/lpm/tiny_lpm/subdir.mk
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
-include subdir.mk
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

BUILD_ARTIFACT_NAME := LoRaWAN_End_Node
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LoRaWAN_End_Node.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LoRaWAN_End_Node.list \

OBJCOPY_BIN += \
LoRaWAN_End_Node.bin \


# All Target
all: main-build

# Main-build Target
main-build: LoRaWAN_End_Node.elf secondary-outputs

# Tool invocations
LoRaWAN_End_Node.elf: $(OBJS) $(USER_OBJS) /home/mrieger/STM32CubeIDE/workspace_1.5.1/LoRaWAN_End_Node/LoRaWAN_End_Node/STM32WL55JCIX_FLASH.ld
	arm-none-eabi-gcc -o "LoRaWAN_End_Node.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/mrieger/STM32CubeIDE/workspace_1.5.1/LoRaWAN_End_Node/LoRaWAN_End_Node/STM32WL55JCIX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LoRaWAN_End_Node.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LoRaWAN_End_Node.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LoRaWAN_End_Node.list"
	@echo 'Finished building: $@'
	@echo ' '

LoRaWAN_End_Node.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LoRaWAN_End_Node.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
