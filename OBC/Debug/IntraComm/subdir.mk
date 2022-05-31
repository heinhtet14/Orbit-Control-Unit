################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../IntraComm/IntraSatComm.c \
../IntraComm/es_crc32.c 

OBJS += \
./IntraComm/IntraSatComm.o \
./IntraComm/es_crc32.o 

C_DEPS += \
./IntraComm/IntraSatComm.d \
./IntraComm/es_crc32.d 


# Each subdirectory must supply rules for building sources it contributes
IntraComm/%.o: ../IntraComm/%.c IntraComm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/OBC/IntraComm" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-IntraComm

clean-IntraComm:
	-$(RM) ./IntraComm/IntraSatComm.d ./IntraComm/IntraSatComm.o ./IntraComm/es_crc32.d ./IntraComm/es_crc32.o

.PHONY: clean-IntraComm

