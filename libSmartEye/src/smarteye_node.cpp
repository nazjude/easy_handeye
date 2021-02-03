#include <ros/ros.h>

#include "libSmartEye/RosApi.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "smarteye_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  smart_eye::RosApi ros_api(nh, pnh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
