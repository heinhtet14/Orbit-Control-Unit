################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ADC_DAC_Lib/adc_dac.c 

OBJS += \
./ADC_DAC_Lib/adc_dac.o 

C_DEPS += \
./ADC_DAC_Lib/adc_dac.d 


# Each subdirectory must supply rules for building sources it contributes
ADC_DAC_Lib/%.o: ../ADC_DAC_Lib/%.c ADC_DAC_Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/Optocoupler_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/Thermocouple_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/ADC_DAC_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/PFCV_UART_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/OBC_SPI_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/FIR_DSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ADC_DAC_Lib

clean-ADC_DAC_Lib:
	-$(RM) ./ADC_DAC_Lib/adc_dac.d ./ADC_DAC_Lib/adc_dac.o

.PHONY: clean-ADC_DAC_Lib

