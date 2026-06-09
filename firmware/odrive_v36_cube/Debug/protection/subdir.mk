################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Shu/Documents/GitHub/motor_set/src/protection/fault.c \
C:/Users/Shu/Documents/GitHub/motor_set/src/protection/protection.c 

OBJS += \
./protection/fault.o \
./protection/protection.o 

C_DEPS += \
./protection/fault.d \
./protection/protection.d 


# Each subdirectory must supply rules for building sources it contributes
protection/fault.o: C:/Users/Shu/Documents/GitHub/motor_set/src/protection/fault.c protection/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
protection/protection.o: C:/Users/Shu/Documents/GitHub/motor_set/src/protection/protection.c protection/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-protection

clean-protection:
	-$(RM) ./protection/fault.cyclo ./protection/fault.d ./protection/fault.o ./protection/fault.su ./protection/protection.cyclo ./protection/protection.d ./protection/protection.o ./protection/protection.su

.PHONY: clean-protection

