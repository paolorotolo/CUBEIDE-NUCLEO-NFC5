################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/NFC05A1/led.c 

OBJS += \
./Drivers/BSP/NFC05A1/led.o 

C_DEPS += \
./Drivers/BSP/NFC05A1/led.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/NFC05A1/led.o: ../Drivers/BSP/NFC05A1/led.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/rfal/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/STM32F4xx-Nucleo" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/NFC05A1" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components/ST25R3911" -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/NFC05A1/led.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

