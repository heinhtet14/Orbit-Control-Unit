################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SPI_Library/MAX_6675.c 

OBJS += \
./SPI_Library/MAX_6675.o 

C_DEPS += \
./SPI_Library/MAX_6675.d 


# Each subdirectory must supply rules for building sources it contributes
SPI_Library/%.o: ../SPI_Library/%.c SPI_Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/stm32-protocol/SPI_Library" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-SPI_Library

clean-SPI_Library:
	-$(RM) ./SPI_Library/MAX_6675.d ./SPI_Library/MAX_6675.o

.PHONY: clean-SPI_Library

