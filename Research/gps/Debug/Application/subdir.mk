################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/app_main.c \
../Application/gps.c \
../Application/nmea_gps.c \
../Application/write.c 

OBJS += \
./Application/app_main.o \
./Application/gps.o \
./Application/nmea_gps.o \
./Application/write.o 

C_DEPS += \
./Application/app_main.d \
./Application/gps.d \
./Application/nmea_gps.d \
./Application/write.d 


# Each subdirectory must supply rules for building sources it contributes
Application/%.o: ../Application/%.c Application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application

clean-Application:
	-$(RM) ./Application/app_main.d ./Application/app_main.o ./Application/gps.d ./Application/gps.o ./Application/nmea_gps.d ./Application/nmea_gps.o ./Application/write.d ./Application/write.o

.PHONY: clean-Application

