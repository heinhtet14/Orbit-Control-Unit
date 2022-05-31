################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Discrete_Convolution/DC.c 

OBJS += \
./Discrete_Convolution/DC.o 

C_DEPS += \
./Discrete_Convolution/DC.d 


# Each subdirectory must supply rules for building sources it contributes
Discrete_Convolution/%.o: ../Discrete_Convolution/%.c Discrete_Convolution/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/DAC_Ben/Discrete_Convolution" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Discrete_Convolution

clean-Discrete_Convolution:
	-$(RM) ./Discrete_Convolution/DC.d ./Discrete_Convolution/DC.o

.PHONY: clean-Discrete_Convolution

