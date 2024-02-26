################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DHT11/dht.c 

OBJS += \
./Core/Src/DHT11/dht.o 

C_DEPS += \
./Core/Src/DHT11/dht.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/DHT11/%.o Core/Src/DHT11/%.su Core/Src/DHT11/%.cyclo: ../Core/Src/DHT11/%.c Core/Src/DHT11/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-DHT11

clean-Core-2f-Src-2f-DHT11:
	-$(RM) ./Core/Src/DHT11/dht.cyclo ./Core/Src/DHT11/dht.d ./Core/Src/DHT11/dht.o ./Core/Src/DHT11/dht.su

.PHONY: clean-Core-2f-Src-2f-DHT11

