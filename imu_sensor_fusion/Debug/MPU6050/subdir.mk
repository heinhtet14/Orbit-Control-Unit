################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MPU6050/mpu6050.c 

OBJS += \
./MPU6050/mpu6050.o 

C_DEPS += \
./MPU6050/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
MPU6050/%.o: ../MPU6050/%.c MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MPU6050

clean-MPU6050:
	-$(RM) ./MPU6050/mpu6050.d ./MPU6050/mpu6050.o

.PHONY: clean-MPU6050

