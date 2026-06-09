################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Shu/Documents/GitHub/motor_set/src/foc/foc_math.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/foc/svpwm.c 

OBJS += \
./foc/foc_math.o \
./foc/svpwm.o 

C_DEPS += \
./foc/foc_math.d \
./foc/svpwm.d 


# Each subdirectory must supply rules for building sources it contributes
foc/foc_math.o: C:/Users/Shu/Documents/GitHub/motor_set/src/foc/foc_math.c foc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
foc/svpwm.o: C:/Users/Shu/Documents/GitHub/motor_set/src/foc/svpwm.c foc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-foc

clean-foc:
	-$(RM) ./foc/foc_math.cyclo ./foc/foc_math.d ./foc/foc_math.o ./foc/foc_math.su ./foc/svpwm.cyclo ./foc/svpwm.d ./foc/svpwm.o ./foc/svpwm.su

.PHONY: clean-foc

