################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/MFCC/subdir.mk
-include Src/GRU/subdir.mk
-include Src/DNN/subdir.mk
-include Src/subdir.mk
-include RTT/subdir.mk
-include RK_Shell/ringbuff/ringbuff/src/ringbuff/subdir.mk
-include RK_Shell/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Mainboard.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Mainboard.list \

OBJCOPY_HEX += \
Mainboard.hex \


# All Target
all: Mainboard.elf secondary-outputs

# Tool invocations
Mainboard.elf: $(OBJS) $(USER_OBJS) ../STM32F429IGTX_FLASH.ld
	arm-none-eabi-gcc -o "Mainboard.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"../STM32F429IGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Mainboard.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Mainboard.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Mainboard.list"
	@echo 'Finished building: $@'
	@echo ' '

Mainboard.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Mainboard.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
