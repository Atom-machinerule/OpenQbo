cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(FindPkgConfig)

rosbuild_add_executable(qbo_video_record src/video_record.cpp)
