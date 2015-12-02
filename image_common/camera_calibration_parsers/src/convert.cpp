#include "camera_calibration_parsers/parse.h"
#include <ros/console.h>
#include <cstdio>

using namespace camera_calibration_parsers;

int main(int argc, char** argv)
{
  if (argc < 3) {
    printf("Usage: %s input.yml output.ini\n"
           "       %s input.ini output.yml\n", argv[0], argv[0]);
    return 0;
  }

  std::string name;
  sensor_msgs::CameraInfo cam_info;
  if (!readCalibration(argv[1], name, cam_info)) {
    ROS_ERROR("Failed to load camera model from file %s", argv[1]);
    return -1;
  }
  if (!writeCalibration(argv[2], name, cam_info)) {
    ROS_ERROR("Failed to save camera model to file %s", argv[2]);
    return -1;
  }
  
  ROS_INFO("Saved %s", argv[2]);
  return 0;
}
