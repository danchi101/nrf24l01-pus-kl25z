################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/SystickDemo.c" \
"../Sources/main.c" \
"../Sources/mcg.c" \
"../Sources/sa_mtb.c" \

C_SRCS += \
../Sources/SystickDemo.c \
../Sources/main.c \
../Sources/mcg.c \
../Sources/sa_mtb.c \

OBJS += \
./Sources/SystickDemo.o \
./Sources/main.o \
./Sources/mcg.o \
./Sources/sa_mtb.o \

C_DEPS += \
./Sources/SystickDemo.d \
./Sources/main.d \
./Sources/mcg.d \
./Sources/sa_mtb.d \

OBJS_QUOTED += \
"./Sources/SystickDemo.o" \
"./Sources/main.o" \
"./Sources/mcg.o" \
"./Sources/sa_mtb.o" \

C_DEPS_QUOTED += \
"./Sources/SystickDemo.d" \
"./Sources/main.d" \
"./Sources/mcg.d" \
"./Sources/sa_mtb.d" \

OBJS_OS_FORMAT += \
./Sources/SystickDemo.o \
./Sources/main.o \
./Sources/mcg.o \
./Sources/sa_mtb.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/SystickDemo.o: ../Sources/SystickDemo.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/SystickDemo.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/SystickDemo.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/main.o: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/main.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/main.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/mcg.o: ../Sources/mcg.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/mcg.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/mcg.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/sa_mtb.o: ../Sources/sa_mtb.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/sa_mtb.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/sa_mtb.o"
	@echo 'Finished building: $<'
	@echo ' '


