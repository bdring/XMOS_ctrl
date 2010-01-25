################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
XC_SRCS += \
../xmos_ctrl.xc 

XN_SRCS += \
../XK-1.xn 

OBJS += \
./xmos_ctrl.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.xc
	@echo 'Building file: $<'
	@echo 'Invoking: XMOS XC Compiler'
	xcc -O0 -g -Wall -c -o "$@" "$<" "../XK-1.xn"
	@echo 'Finished building: $<'
	@echo ' '


