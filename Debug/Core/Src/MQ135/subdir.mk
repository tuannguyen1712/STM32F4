################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MQ135/MQ135.c 

OBJS += \
./Core/Src/MQ135/MQ135.o 

C_DEPS += \
./Core/Src/MQ135/MQ135.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MQ135/%.o Core/Src/MQ135/%.su Core/Src/MQ135/%.cyclo: ../Core/Src/MQ135/%.c Core/Src/MQ135/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-MQ135

clean-Core-2f-Src-2f-MQ135:
	-$(RM) ./Core/Src/MQ135/MQ135.cyclo ./Core/Src/MQ135/MQ135.d ./Core/Src/MQ135/MQ135.o ./Core/Src/MQ135/MQ135.su

.PHONY: clean-Core-2f-Src-2f-MQ135

