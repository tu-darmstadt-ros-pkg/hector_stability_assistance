#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "stability_visualization node");

  ros::spin();
  return 0;
}
