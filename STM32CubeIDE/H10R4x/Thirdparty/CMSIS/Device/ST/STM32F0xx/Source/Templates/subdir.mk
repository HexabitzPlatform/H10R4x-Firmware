################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
W:/Current\ work/DTest/H10R4x-Firmware/Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c 

OBJS += \
./Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.o 

C_DEPS += \
./Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.o: W:/Current\ work/DTest/H10R4x-Firmware/Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH10R4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H10R4 -I../../BOS -I../../User -Og -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

