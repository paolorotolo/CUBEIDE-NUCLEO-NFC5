################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/ndef/Src/poller/ndef_poller.c \
../Middlewares/ST/ndef/Src/poller/ndef_t2t.c \
../Middlewares/ST/ndef/Src/poller/ndef_t3t.c \
../Middlewares/ST/ndef/Src/poller/ndef_t4t.c \
../Middlewares/ST/ndef/Src/poller/ndef_t5t.c 

OBJS += \
./Middlewares/ST/ndef/Src/poller/ndef_poller.o \
./Middlewares/ST/ndef/Src/poller/ndef_t2t.o \
./Middlewares/ST/ndef/Src/poller/ndef_t3t.o \
./Middlewares/ST/ndef/Src/poller/ndef_t4t.o \
./Middlewares/ST/ndef/Src/poller/ndef_t5t.o 

C_DEPS += \
./Middlewares/ST/ndef/Src/poller/ndef_poller.d \
./Middlewares/ST/ndef/Src/poller/ndef_t2t.d \
./Middlewares/ST/ndef/Src/poller/ndef_t3t.d \
./Middlewares/ST/ndef/Src/poller/ndef_t4t.d \
./Middlewares/ST/ndef/Src/poller/ndef_t5t.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/ndef/Src/poller/ndef_poller.o: ../Middlewares/ST/ndef/Src/poller/ndef_poller.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_STM32F4XX_NUCLEO '-DUSE_LOGGER=LOGGER_ON' -DST25R3911 -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/rfal/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/STM32F4xx-Nucleo" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/NFC05A1" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components/ST25R3911" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/poller" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/message" -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/ndef/Src/poller/ndef_poller.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/ndef/Src/poller/ndef_t2t.o: ../Middlewares/ST/ndef/Src/poller/ndef_t2t.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_STM32F4XX_NUCLEO '-DUSE_LOGGER=LOGGER_ON' -DST25R3911 -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/rfal/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/STM32F4xx-Nucleo" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/NFC05A1" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components/ST25R3911" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/poller" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/message" -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/ndef/Src/poller/ndef_t2t.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/ndef/Src/poller/ndef_t3t.o: ../Middlewares/ST/ndef/Src/poller/ndef_t3t.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_STM32F4XX_NUCLEO '-DUSE_LOGGER=LOGGER_ON' -DST25R3911 -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/rfal/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/STM32F4xx-Nucleo" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/NFC05A1" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components/ST25R3911" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/poller" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/message" -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/ndef/Src/poller/ndef_t3t.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/ndef/Src/poller/ndef_t4t.o: ../Middlewares/ST/ndef/Src/poller/ndef_t4t.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_STM32F4XX_NUCLEO '-DUSE_LOGGER=LOGGER_ON' -DST25R3911 -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/rfal/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/STM32F4xx-Nucleo" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/NFC05A1" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components/ST25R3911" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/poller" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/message" -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/ndef/Src/poller/ndef_t4t.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/ndef/Src/poller/ndef_t5t.o: ../Middlewares/ST/ndef/Src/poller/ndef_t5t.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_STM32F4XX_NUCLEO '-DUSE_LOGGER=LOGGER_ON' -DST25R3911 -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/rfal/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/STM32F4xx-Nucleo" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/NFC05A1" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Drivers/BSP/Components/ST25R3911" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/poller" -I"/Users/paolorotolo/STM32CubeIDE/workspace_1.4.0/CUBEIDE-NUCLEO-NFC5/Middlewares/ST/ndef/Inc/message" -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/ndef/Src/poller/ndef_t5t.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
