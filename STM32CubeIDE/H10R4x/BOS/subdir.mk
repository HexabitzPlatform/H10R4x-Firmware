################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
W:/Hexabitz/H10R4x-Firmware/BOS/BOS.c \
W:/Hexabitz/H10R4x-Firmware/BOS/BOS_CLI.c \
W:/Hexabitz/H10R4x-Firmware/BOS/BOS_CLIcommands.c \
W:/Hexabitz/H10R4x-Firmware/BOS/BOS_dma.c \
W:/Hexabitz/H10R4x-Firmware/BOS/BOS_eeprom.c \
W:/Hexabitz/H10R4x-Firmware/BOS/BOS_freertos.c \
W:/Hexabitz/H10R4x-Firmware/BOS/BOS_inputs.c \
W:/Hexabitz/H10R4x-Firmware/BOS/BOS_messaging.c \
W:/Hexabitz/H10R4x-Firmware/BOS/BOS_msgparser.c 

OBJS += \
./BOS/BOS.o \
./BOS/BOS_CLI.o \
./BOS/BOS_CLIcommands.o \
./BOS/BOS_dma.o \
./BOS/BOS_eeprom.o \
./BOS/BOS_freertos.o \
./BOS/BOS_inputs.o \
./BOS/BOS_messaging.o \
./BOS/BOS_msgparser.o 

C_DEPS += \
./BOS/BOS.d \
./BOS/BOS_CLI.d \
./BOS/BOS_CLIcommands.d \
./BOS/BOS_dma.d \
./BOS/BOS_eeprom.d \
./BOS/BOS_freertos.d \
./BOS/BOS_inputs.d \
./BOS/BOS_messaging.d \
./BOS/BOS_msgparser.d 


# Each subdirectory must supply rules for building sources it contributes
BOS/BOS.o: W:/Hexabitz/H10R4x-Firmware/BOS/BOS.c BOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_CLI.o: W:/Hexabitz/H10R4x-Firmware/BOS/BOS_CLI.c BOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_CLI.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_CLIcommands.o: W:/Hexabitz/H10R4x-Firmware/BOS/BOS_CLIcommands.c BOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_CLIcommands.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_dma.o: W:/Hexabitz/H10R4x-Firmware/BOS/BOS_dma.c BOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_dma.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_eeprom.o: W:/Hexabitz/H10R4x-Firmware/BOS/BOS_eeprom.c BOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_eeprom.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_freertos.o: W:/Hexabitz/H10R4x-Firmware/BOS/BOS_freertos.c BOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_freertos.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_inputs.o: W:/Hexabitz/H10R4x-Firmware/BOS/BOS_inputs.c BOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_inputs.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_messaging.o: W:/Hexabitz/H10R4x-Firmware/BOS/BOS_messaging.c BOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_messaging.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_msgparser.o: W:/Hexabitz/H10R4x-Firmware/BOS/BOS_msgparser.c BOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_msgparser.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

