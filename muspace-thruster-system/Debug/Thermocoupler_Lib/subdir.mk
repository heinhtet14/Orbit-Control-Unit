################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Thermocoupler_Lib/thermocouple.c 

OBJS += \
./Thermocoupler_Lib/thermocouple.o 

C_DEPS += \
./Thermocoupler_Lib/thermocouple.d 


# Each subdirectory must supply rules for building sources it contributes
Thermocoupler_Lib/%.o: ../Thermocoupler_Lib/%.c Thermocoupler_Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/Optocoupler_Lib" -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/muspace-thruster-system/Thermocoupler_Lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Thermocoupler_Lib

clean-Thermocoupler_Lib:
	-$(RM) ./Thermocoupler_Lib/thermocouple.d ./Thermocoupler_Lib/thermocouple.o

.PHONY: clean-Thermocoupler_Lib

