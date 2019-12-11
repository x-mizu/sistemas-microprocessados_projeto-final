################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
sistemas-microprocessados_projeto-final.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
sistemas-microprocessados_projeto-final.list \


# All Target
all: sistemas-microprocessados_projeto-final.elf secondary-outputs

# Tool invocations
sistemas-microprocessados_projeto-final.elf: $(OBJS) $(USER_OBJS) C:\Users\xmizu\Documents\GitHub\sistemas-microprocessados_projeto-final\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "sistemas-microprocessados_projeto-final.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\xmizu\Documents\GitHub\sistemas-microprocessados_projeto-final\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="sistemas-microprocessados_projeto-final.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

sistemas-microprocessados_projeto-final.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "sistemas-microprocessados_projeto-final.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
