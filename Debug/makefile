################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/ST/rfal/Src/subdir.mk
-include Middlewares/ST/ndef/Src/poller/subdir.mk
-include Middlewares/ST/ndef/Src/message/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/STM32F4xx-Nucleo/subdir.mk
-include Drivers/BSP/NFC05A1/subdir.mk
-include Drivers/BSP/Components/ST25R3911/subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
CUBEIDE-NUCLEO-NFC5.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CUBEIDE-NUCLEO-NFC5.list \

OBJCOPY_BIN += \
CUBEIDE-NUCLEO-NFC5.bin \


# All Target
all: CUBEIDE-NUCLEO-NFC5.elf secondary-outputs

# Tool invocations
CUBEIDE-NUCLEO-NFC5.elf: $(OBJS) $(USER_OBJS) /Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "CUBEIDE-NUCLEO-NFC5.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="CUBEIDE-NUCLEO-NFC5.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CUBEIDE-NUCLEO-NFC5.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CUBEIDE-NUCLEO-NFC5.list"
	@echo 'Finished building: $@'
	@echo ' '

CUBEIDE-NUCLEO-NFC5.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "CUBEIDE-NUCLEO-NFC5.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
