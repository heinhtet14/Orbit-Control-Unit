################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../X-CUBE-AI/App/sine_model.c \
../X-CUBE-AI/App/sine_model_data.c 

OBJS += \
./X-CUBE-AI/App/sine_model.o \
./X-CUBE-AI/App/sine_model_data.o 

C_DEPS += \
./X-CUBE-AI/App/sine_model.d \
./X-CUBE-AI/App/sine_model_data.d 


# Each subdirectory must supply rules for building sources it contributes
X-CUBE-AI/App/%.o: ../X-CUBE-AI/App/%.c X-CUBE-AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/AI/Inc -I../X-CUBE-AI/App -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-X-2d-CUBE-2d-AI-2f-App

clean-X-2d-CUBE-2d-AI-2f-App:
	-$(RM) ./X-CUBE-AI/App/sine_model.d ./X-CUBE-AI/App/sine_model.o ./X-CUBE-AI/App/sine_model_data.d ./X-CUBE-AI/App/sine_model_data.o

.PHONY: clean-X-2d-CUBE-2d-AI-2f-App

