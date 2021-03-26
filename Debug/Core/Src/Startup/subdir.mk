################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Src/Startup/startup_stm32f405vgtx.s 

OBJS += \
./Core/Src/Startup/startup_stm32f405vgtx.o 

S_DEPS += \
./Core/Src/Startup/startup_stm32f405vgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Startup/startup_stm32f405vgtx.o: ../Core/Src/Startup/startup_stm32f405vgtx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Src/Startup/startup_stm32f405vgtx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

