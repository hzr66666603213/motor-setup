################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_adc_stm32f405.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_encoder_stm32f405.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_gpio_stm32f405.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_pwm_stm32f405.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_spi_stm32f405.c 

OBJS += \
./hal_stm32f405/hal_adc_stm32f405.o \
./hal_stm32f405/hal_encoder_stm32f405.o \
./hal_stm32f405/hal_gpio_stm32f405.o \
./hal_stm32f405/hal_pwm_stm32f405.o \
./hal_stm32f405/hal_spi_stm32f405.o 

C_DEPS += \
./hal_stm32f405/hal_adc_stm32f405.d \
./hal_stm32f405/hal_encoder_stm32f405.d \
./hal_stm32f405/hal_gpio_stm32f405.d \
./hal_stm32f405/hal_pwm_stm32f405.d \
./hal_stm32f405/hal_spi_stm32f405.d 


# Each subdirectory must supply rules for building sources it contributes
hal_stm32f405/hal_adc_stm32f405.o: C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_adc_stm32f405.c hal_stm32f405/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
hal_stm32f405/hal_encoder_stm32f405.o: C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_encoder_stm32f405.c hal_stm32f405/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
hal_stm32f405/hal_gpio_stm32f405.o: C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_gpio_stm32f405.c hal_stm32f405/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
hal_stm32f405/hal_pwm_stm32f405.o: C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_pwm_stm32f405.c hal_stm32f405/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
hal_stm32f405/hal_spi_stm32f405.o: C:/Users/Shu/Documents/GitHub/motor_set/src/hal/stm32f405/hal_spi_stm32f405.c hal_stm32f405/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-hal_stm32f405

clean-hal_stm32f405:
	-$(RM) ./hal_stm32f405/hal_adc_stm32f405.cyclo ./hal_stm32f405/hal_adc_stm32f405.d ./hal_stm32f405/hal_adc_stm32f405.o ./hal_stm32f405/hal_adc_stm32f405.su ./hal_stm32f405/hal_encoder_stm32f405.cyclo ./hal_stm32f405/hal_encoder_stm32f405.d ./hal_stm32f405/hal_encoder_stm32f405.o ./hal_stm32f405/hal_encoder_stm32f405.su ./hal_stm32f405/hal_gpio_stm32f405.cyclo ./hal_stm32f405/hal_gpio_stm32f405.d ./hal_stm32f405/hal_gpio_stm32f405.o ./hal_stm32f405/hal_gpio_stm32f405.su ./hal_stm32f405/hal_pwm_stm32f405.cyclo ./hal_stm32f405/hal_pwm_stm32f405.d ./hal_stm32f405/hal_pwm_stm32f405.o ./hal_stm32f405/hal_pwm_stm32f405.su ./hal_stm32f405/hal_spi_stm32f405.cyclo ./hal_stm32f405/hal_spi_stm32f405.d ./hal_stm32f405/hal_spi_stm32f405.o ./hal_stm32f405/hal_spi_stm32f405.su

.PHONY: clean-hal_stm32f405

