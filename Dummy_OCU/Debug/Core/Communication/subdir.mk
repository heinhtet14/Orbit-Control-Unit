################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Communication/IntraSatComm.c \
../Core/Communication/es_crc32.c \
../Core/Communication/test.c 

OBJS += \
./Core/Communication/IntraSatComm.o \
./Core/Communication/es_crc32.o \
./Core/Communication/test.o 

C_DEPS += \
./Core/Communication/IntraSatComm.d \
./Core/Communication/es_crc32.d \
./Core/Communication/test.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Communication/%.o: ../Core/Communication/%.c Core/Communication/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/Dummy_OCU/Core/Communication" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Communication

clean-Core-2f-Communication:
	-$(RM) ./Core/Communication/IntraSatComm.d ./Core/Communication/IntraSatComm.o ./Core/Communication/es_crc32.d ./Core/Communication/es_crc32.o ./Core/Communication/test.d ./Core/Communication/test.o

.PHONY: clean-Core-2f-Communication

