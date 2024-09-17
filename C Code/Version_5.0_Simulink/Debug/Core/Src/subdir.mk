################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/analog_in.c \
../Core/Src/analog_out.c \
../Core/Src/controller.c \
../Core/Src/filter.c \
../Core/Src/global_variables.c \
../Core/Src/main.c \
../Core/Src/moving_average.c \
../Core/Src/processes.c \
../Core/Src/pwm.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/timer_interrupt.c 

OBJS += \
./Core/Src/analog_in.o \
./Core/Src/analog_out.o \
./Core/Src/controller.o \
./Core/Src/filter.o \
./Core/Src/global_variables.o \
./Core/Src/main.o \
./Core/Src/moving_average.o \
./Core/Src/processes.o \
./Core/Src/pwm.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/timer_interrupt.o 

C_DEPS += \
./Core/Src/analog_in.d \
./Core/Src/analog_out.d \
./Core/Src/controller.d \
./Core/Src/filter.d \
./Core/Src/global_variables.d \
./Core/Src/main.d \
./Core/Src/moving_average.d \
./Core/Src/processes.d \
./Core/Src/pwm.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/timer_interrupt.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I"C:/Users/setm/OneDrive - KTH/Desktop/random/bno_test_h503_v1" -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/analog_in.cyclo ./Core/Src/analog_in.d ./Core/Src/analog_in.o ./Core/Src/analog_in.su ./Core/Src/analog_out.cyclo ./Core/Src/analog_out.d ./Core/Src/analog_out.o ./Core/Src/analog_out.su ./Core/Src/controller.cyclo ./Core/Src/controller.d ./Core/Src/controller.o ./Core/Src/controller.su ./Core/Src/filter.cyclo ./Core/Src/filter.d ./Core/Src/filter.o ./Core/Src/filter.su ./Core/Src/global_variables.cyclo ./Core/Src/global_variables.d ./Core/Src/global_variables.o ./Core/Src/global_variables.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/moving_average.cyclo ./Core/Src/moving_average.d ./Core/Src/moving_average.o ./Core/Src/moving_average.su ./Core/Src/processes.cyclo ./Core/Src/processes.d ./Core/Src/processes.o ./Core/Src/processes.su ./Core/Src/pwm.cyclo ./Core/Src/pwm.d ./Core/Src/pwm.o ./Core/Src/pwm.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/timer_interrupt.cyclo ./Core/Src/timer_interrupt.d ./Core/Src/timer_interrupt.o ./Core/Src/timer_interrupt.su

.PHONY: clean-Core-2f-Src

