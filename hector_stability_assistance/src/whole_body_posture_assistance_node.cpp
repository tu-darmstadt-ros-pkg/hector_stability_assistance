#include <ros/ros.h>

#include <hector_stability_assistance/whole_body_posture_assistance.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_posture_assistance_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  hector_stability_assistance::WholeBodyPostureAssistance controller(nh, pnh);
  if (!controller.init()) {
    return 0;
  }

  ros::spin();
  return 0;
}
