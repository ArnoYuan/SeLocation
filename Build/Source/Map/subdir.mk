################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Source/Map/map.c \
../Source/Map/map_range.c \
../Source/Map/map_store.c 

CPP_SRCS += \
../Source/Map/map_cspace.cpp 

OBJS += \
./Source/Map/map.o \
./Source/Map/map_cspace.o \
./Source/Map/map_range.o \
./Source/Map/map_store.o 

C_DEPS += \
./Source/Map/map.d \
./Source/Map/map_range.d \
./Source/Map/map_store.d 

CPP_DEPS += \
./Source/Map/map_cspace.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Map/%.o: ../Source/Map/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-openwrt-linux-muslgnueabi-gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Source/Map/%.o: ../Source/Map/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I"/home/cybernik/Development/Projects/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


