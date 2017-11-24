################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Source/ParticleFilter/eig3.c \
../Source/ParticleFilter/pf.c \
../Source/ParticleFilter/pf_kdtree.c \
../Source/ParticleFilter/pf_pdf.c \
../Source/ParticleFilter/pf_vector.c 

OBJS += \
./Source/ParticleFilter/eig3.o \
./Source/ParticleFilter/pf.o \
./Source/ParticleFilter/pf_kdtree.o \
./Source/ParticleFilter/pf_pdf.o \
./Source/ParticleFilter/pf_vector.o 

C_DEPS += \
./Source/ParticleFilter/eig3.d \
./Source/ParticleFilter/pf.d \
./Source/ParticleFilter/pf_kdtree.d \
./Source/ParticleFilter/pf_pdf.d \
./Source/ParticleFilter/pf_vector.d 


# Each subdirectory must supply rules for building sources it contributes
Source/ParticleFilter/%.o: ../Source/ParticleFilter/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-openwrt-linux-muslgnueabi-gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


