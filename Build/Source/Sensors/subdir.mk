################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Sensors/Odom.cpp \
../Source/Sensors/Sensor.cpp \
../Source/Sensors/amcl_laser.cpp 

OBJS += \
./Source/Sensors/Odom.o \
./Source/Sensors/Sensor.o \
./Source/Sensors/amcl_laser.o 

CPP_DEPS += \
./Source/Sensors/Odom.d \
./Source/Sensors/Sensor.d \
./Source/Sensors/amcl_laser.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Sensors/%.o: ../Source/Sensors/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I"/home/cybernik/Development/Projects/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


