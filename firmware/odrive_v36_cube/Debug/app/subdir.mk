################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Shu/Documents/GitHub/motor_set/src/app/axis0_current_loop_isr.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/app/axis0_default_config.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/app/axis_state_machine.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/app/calibration.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/app/console.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/app/parameter_table.c 

OBJS += \
./app/axis0_current_loop_isr.o \
./app/axis0_default_config.o \
./app/axis_state_machine.o \
./app/calibration.o \
./app/console.o \
./app/parameter_table.o 

C_DEPS += \
./app/axis0_current_loop_isr.d \
./app/axis0_default_config.d \
./app/axis_state_machine.d \
./app/calibration.d \
./app/console.d \
./app/parameter_table.d 


# Each subdirectory must supply rules for building sources it contributes
app/axis0_current_loop_isr.o: C:/Users/Shu/Documents/GitHub/motor_set/src/app/axis0_current_loop_isr.c app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
app/axis0_default_config.o: C:/Users/Shu/Documents/GitHub/motor_set/src/app/axis0_default_config.c app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
app/axis_state_machine.o: C:/Users/Shu/Documents/GitHub/motor_set/src/app/axis_state_machine.c app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
app/calibration.o: C:/Users/Shu/Documents/GitHub/motor_set/src/app/calibration.c app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
app/console.o: C:/Users/Shu/Documents/GitHub/motor_set/src/app/console.c app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
app/parameter_table.o: C:/Users/Shu/Documents/GitHub/motor_set/src/app/parameter_table.c app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-app

clean-app:
	-$(RM) ./app/axis0_current_loop_isr.cyclo ./app/axis0_current_loop_isr.d ./app/axis0_current_loop_isr.o ./app/axis0_current_loop_isr.su ./app/axis0_default_config.cyclo ./app/axis0_default_config.d ./app/axis0_default_config.o ./app/axis0_default_config.su ./app/axis_state_machine.cyclo ./app/axis_state_machine.d ./app/axis_state_machine.o ./app/axis_state_machine.su ./app/calibration.cyclo ./app/calibration.d ./app/calibration.o ./app/calibration.su ./app/console.cyclo ./app/console.d ./app/console.o ./app/console.su ./app/parameter_table.cyclo ./app/parameter_table.d ./app/parameter_table.o ./app/parameter_table.su

.PHONY: clean-app

