################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/crc32.c \
../Src/inet.c \
../Src/main.c \
../Src/message.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c 

OBJS += \
./Src/crc32.o \
./Src/inet.o \
./Src/main.o \
./Src/message.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o 

C_DEPS += \
./Src/crc32.d \
./Src/inet.d \
./Src/main.d \
./Src/message.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F411xE -I"C:/Users/vojis/Documents/stm_projects/USART_TLV/Inc" -I"C:/Users/vojis/Documents/stm_projects/USART_TLV/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/vojis/Documents/stm_projects/USART_TLV/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/vojis/Documents/stm_projects/USART_TLV/Drivers/CMSIS/Include" -I"C:/Users/vojis/Documents/stm_projects/USART_TLV/Drivers/CMSIS/Device/ST/STM32F4xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


