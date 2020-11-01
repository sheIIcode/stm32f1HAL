################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
timer.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
timer.list \

OBJCOPY_BIN += \
timer.bin \


# All Target
all:
	+@make --no-print-directory main-build && make --no-print-directory post-build

# Main-build Target
main-build: timer.elf secondary-outputs

# Tool invocations
timer.elf: $(OBJS) $(USER_OBJS) C:\Users\vvilq\Documents\GitProjects\C\timer\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "timer.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\vvilq\Documents\GitProjects\C\timer\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="timer.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

timer.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "timer.list"
	@echo 'Finished building: $@'
	@echo ' '

timer.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "timer.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	arm-none-eabi-objcopy -O ihex timer.elf timer.hex
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
