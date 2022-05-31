################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../OBC_SPI_Lib/obc_spi.c 

OBJS += \
./OBC_SPI_Lib/obc_spi.o 

C_DEPS += \
./OBC_SPI_Lib/obc_spi.d 


# Each subdirectory must supply rules for building sources it contributes
OBC_SPI_Lib/%.o: ../OBC_SPI_Lib/%.c OBC_SPI_Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/Optocoupler_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/Thermocouple_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/ADC_DAC_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/PFCV_UART_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/OBC_SPI_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/FIR_DSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-OBC_SPI_Lib

clean-OBC_SPI_Lib:
	-$(RM) ./OBC_SPI_Lib/obc_spi.d ./OBC_SPI_Lib/obc_spi.o

.PHONY: clean-OBC_SPI_Lib

