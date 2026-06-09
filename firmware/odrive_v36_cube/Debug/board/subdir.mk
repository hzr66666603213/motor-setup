################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Shu/Documents/GitHub/motor_set/src/board/board_odrive_v36.c 

OBJS += \
./board/board_odrive_v36.o 

C_DEPS += \
./board/board_odrive_v36.d 


# Each subdirectory must supply rules for building sources it contributes
board/board_odrive_v36.o: C:/Users/Shu/Documents/GitHub/motor_set/src/board/board_odrive_v36.c board/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Shu/Documents/GitHub/motor_set/include" -I"C:/Users/Shu/Documents/GitHub/motor_set/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-board

clean-board:
	-$(RM) ./board/board_odrive_v36.cyclo ./board/board_odrive_v36.d ./board/board_odrive_v36.o ./board/board_odrive_v36.su

.PHONY: clean-board

