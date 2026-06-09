################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Shu/Documents/GitHub/motor_set/src/drivers/current_sensor.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/drivers/drv8301.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/drivers/encoder_mt6701_abz.c 

OBJS += \
./motor_drivers/current_sensor.o \
./motor_drivers/drv8301.o \
./motor_drivers/encoder_mt6701_abz.o 

C_DEPS += \
./motor_drivers/current_sensor.d \
./motor_drivers/drv8301.d \
./motor_drivers/encoder_mt6701_abz.d 


# Each subdirectory must supply rules for building sources it contributes
motor_drivers/current_sensor.o: C:/Users/Shu/Documents/GitHub/motor_set/src/drivers/current_sensor.c motor_drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
motor_drivers/drv8301.o: C:/Users/Shu/Documents/GitHub/motor_set/src/drivers/drv8301.c motor_drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
motor_drivers/encoder_mt6701_abz.o: C:/Users/Shu/Documents/GitHub/motor_set/src/drivers/encoder_mt6701_abz.c motor_drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-motor_drivers

clean-motor_drivers:
	-$(RM) ./motor_drivers/current_sensor.cyclo ./motor_drivers/current_sensor.d ./motor_drivers/current_sensor.o ./motor_drivers/current_sensor.su ./motor_drivers/drv8301.cyclo ./motor_drivers/drv8301.d ./motor_drivers/drv8301.o ./motor_drivers/drv8301.su ./motor_drivers/encoder_mt6701_abz.cyclo ./motor_drivers/encoder_mt6701_abz.d ./motor_drivers/encoder_mt6701_abz.o ./motor_drivers/encoder_mt6701_abz.su

.PHONY: clean-motor_drivers

