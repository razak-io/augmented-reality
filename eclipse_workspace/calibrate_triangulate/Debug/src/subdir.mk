################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/calibrate.cpp \
../src/main.cpp \
../src/rectify.cpp \
../src/stereo_calibrate.cpp \
../src/triangulate.cpp 

OBJS += \
./src/calibrate.o \
./src/main.o \
./src/rectify.o \
./src/stereo_calibrate.o \
./src/triangulate.o 

CPP_DEPS += \
./src/calibrate.d \
./src/main.d \
./src/rectify.d \
./src/stereo_calibrate.d \
./src/triangulate.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv4 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


