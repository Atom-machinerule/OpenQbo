cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(libdxl src/dynamixel_sdk/src/dxl_hal.c)
rosbuild_add_library(dynamixel src/dynamixel_sdk/src/dynamixel.c)
target_link_libraries(dynamixel libdxl)

rosbuild_add_library(arduqbo_instructions_lib src/driver/arduqbo_instructions.cpp)
rosbuild_add_library(qboduino_driver_lib src/driver/qboduino_driver.cpp)
target_link_libraries(qboduino_driver_lib arduqbo_instructions_lib)

rosbuild_add_library(servos_lib src/servos.cpp)
target_link_libraries(servos_lib dynamixel libdxl)

rosbuild_add_library(base_controller_lib src/controllers/base_controller.cpp)
rosbuild_add_library(battery_controller_lib src/controllers/battery_controller.cpp)
rosbuild_add_library(joint_controller_lib src/controllers/joint_controller.cpp)
target_link_libraries(joint_controller_lib servos_lib)
rosbuild_add_library(lcd_controller_lib src/controllers/lcd_controller.cpp)
rosbuild_add_library(mics_controller_lib src/controllers/mics_controller.cpp)
rosbuild_add_library(mouth_controller_lib src/controllers/mouth_controller.cpp)
rosbuild_add_library(nose_controller_lib src/controllers/nose_controller.cpp)
rosbuild_add_library(srf10_controller_lib src/controllers/srf10_controller.cpp)
rosbuild_add_library(imu_controller_lib src/controllers/imu_controller.cpp)
rosbuild_add_library(irs_controller_lib src/controllers/infra_red_recievers_controller.cpp)

rosbuild_add_executable(qbo_arduqbo src/qbo_arduqbo.cpp)
target_link_libraries(qbo_arduqbo cereal_port qboduino_driver_lib base_controller_lib battery_controller_lib joint_controller_lib lcd_controller_lib mics_controller_lib mouth_controller_lib nose_controller_lib srf10_controller_lib imu_controller_lib irs_controller_lib pthread dynamixel libdxl)
