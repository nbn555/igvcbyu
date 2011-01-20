################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Compass.cpp \
../src/GPS.cpp \
../src/JoystickCommand.cpp \
../src/MotorCommand.cpp \
../src/MotorCommandInterface.cpp \
../src/MotorController.cpp \
../src/PoseEstimator.cpp \
../src/WaypointPlanner.cpp 

OBJS += \
./src/Compass.o \
./src/GPS.o \
./src/JoystickCommand.o \
./src/MotorCommand.o \
./src/MotorCommandInterface.o \
./src/MotorController.o \
./src/PoseEstimator.o \
./src/WaypointPlanner.o 

CPP_DEPS += \
./src/Compass.d \
./src/GPS.d \
./src/JoystickCommand.d \
./src/MotorCommand.d \
./src/MotorCommandInterface.d \
./src/MotorController.d \
./src/PoseEstimator.d \
./src/WaypointPlanner.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/tallred3/ecen493/YClopsLib/inc" -I/usr/include/mrpt/base/include/ -I/usr/include/mrpt/mrpt-config/ -I/usr/include/mrpt/bayes/include/ -I/usr/include/mrpt/detectors/include/ -I/usr/include/mrpt/vision/include/ -I/usr/include/mrpt/maps/include/ -I/usr/include/mrpt/gui/include/ -I/usr/include/mrpt/slam/include/ -I/usr/include/mrpt/opengl/include/ -I/usr/include/mrpt/obs/include/ -I/usr/include/mrpt/scanmatching/include/ -I/usr/include/mrpt/hmtslam/include/ -I/usr/include/mrpt/hwdrivers/include/ -I/usr/include/mrpt/reactivenav/include/ -I/usr/include/mrpt/topography/include/ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


