################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/minmea/minmea.c 

OBJS += \
./Application/minmea/minmea.o 

C_DEPS += \
./Application/minmea/minmea.d 


# Each subdirectory must supply rules for building sources it contributes
Application/minmea/%.o: ../Application/minmea/%.c Application/minmea/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-minmea

clean-Application-2f-minmea:
	-$(RM) ./Application/minmea/minmea.d ./Application/minmea/minmea.o

.PHONY: clean-Application-2f-minmea

