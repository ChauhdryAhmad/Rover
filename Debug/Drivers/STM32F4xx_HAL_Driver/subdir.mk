################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32F4xx_HAL_Driver/my_uart.c 

OBJS += \
./Drivers/STM32F4xx_HAL_Driver/my_uart.o 

C_DEPS += \
./Drivers/STM32F4xx_HAL_Driver/my_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F4xx_HAL_Driver/%.o Drivers/STM32F4xx_HAL_Driver/%.su Drivers/STM32F4xx_HAL_Driver/%.cyclo: ../Drivers/STM32F4xx_HAL_Driver/%.c Drivers/STM32F4xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32F4xx_HAL_Driver

clean-Drivers-2f-STM32F4xx_HAL_Driver:
	-$(RM) ./Drivers/STM32F4xx_HAL_Driver/my_uart.cyclo ./Drivers/STM32F4xx_HAL_Driver/my_uart.d ./Drivers/STM32F4xx_HAL_Driver/my_uart.o ./Drivers/STM32F4xx_HAL_Driver/my_uart.su

.PHONY: clean-Drivers-2f-STM32F4xx_HAL_Driver

