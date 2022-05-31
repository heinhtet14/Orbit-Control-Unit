################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FLASH_MEM/FLASH_MEM.c 

OBJS += \
./FLASH_MEM/FLASH_MEM.o 

C_DEPS += \
./FLASH_MEM/FLASH_MEM.d 


# Each subdirectory must supply rules for building sources it contributes
FLASH_MEM/%.o: ../FLASH_MEM/%.c FLASH_MEM/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/HEIN.H/STM32CubeIDE/workspace_1.8.0/FLASH_F4/FLASH_MEM" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FLASH_MEM

clean-FLASH_MEM:
	-$(RM) ./FLASH_MEM/FLASH_MEM.d ./FLASH_MEM/FLASH_MEM.o

.PHONY: clean-FLASH_MEM

