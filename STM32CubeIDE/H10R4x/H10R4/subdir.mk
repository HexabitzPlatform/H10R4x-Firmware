################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/startup_stm32f091xc.s 

C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_adc.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_dma.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_gpio.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_it.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_rtc.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_timers.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_uart.c 

OBJS += \
./H10R4/H10R4.o \
./H10R4/H10R4_adc.o \
./H10R4/H10R4_dma.o \
./H10R4/H10R4_gpio.o \
./H10R4/H10R4_it.o \
./H10R4/H10R4_rtc.o \
./H10R4/H10R4_timers.o \
./H10R4/H10R4_uart.o \
./H10R4/startup_stm32f091xc.o 

S_DEPS += \
./H10R4/startup_stm32f091xc.d 

C_DEPS += \
./H10R4/H10R4.d \
./H10R4/H10R4_adc.d \
./H10R4/H10R4_dma.d \
./H10R4/H10R4_gpio.d \
./H10R4/H10R4_it.d \
./H10R4/H10R4_rtc.d \
./H10R4/H10R4_timers.d \
./H10R4/H10R4_uart.d 


# Each subdirectory must supply rules for building sources it contributes
H10R4/H10R4.o: D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H10R4/H10R4.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H10R4/H10R4_adc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_adc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H10R4/H10R4_adc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H10R4/H10R4_dma.o: D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H10R4/H10R4_dma.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H10R4/H10R4_gpio.o: D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H10R4/H10R4_gpio.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H10R4/H10R4_it.o: D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H10R4/H10R4_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H10R4/H10R4_rtc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_rtc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H10R4/H10R4_rtc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H10R4/H10R4_timers.o: D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H10R4/H10R4_timers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H10R4/H10R4_uart.o: D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/H10R4_uart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H10R4/H10R4_uart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H10R4/startup_stm32f091xc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H10R4x/H10R4/startup_stm32f091xc.s
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -I../ -x assembler-with-cpp -MMD -MP -MF"H10R4/startup_stm32f091xc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

