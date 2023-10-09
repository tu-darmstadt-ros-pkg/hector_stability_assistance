#include <ros/ros.h>

#include <hector_stability_assistance/stability_visualization.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "stability_visualization_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  hector_stability_assistance::StabilityVisualization viz(nh, pnh);
  if (!viz.init()) {
    return 0;
  }

  ros::spin();
  return 0;
}
