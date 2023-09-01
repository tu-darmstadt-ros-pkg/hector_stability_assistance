#include <ros/ros.h>

#include <hector_stability_assistance/speed_controller.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "stability_visualization node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  hector_stability_assistance::SpeedController controller(nh, pnh);
  if (!controller.init()) {
    return 0;
  }

  ros::spin();
  return 0;
}
