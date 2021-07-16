#ifndef HECTOR_STABILITY_ASSISTANCE_STABILITY_VISUALIZATION_H
#define HECTOR_STABILITY_ASSISTANCE_STABILITY_VISUALIZATION_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>

#include <hector_pose_prediction_interface/pose_predictor.h>

namespace hector_stability_assistance {

class StabilityVisualization {
public:
  StabilityVisualization(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  bool init();
private:
  void timerCallback(const ros::TimerEvent&);
  void gridMapCallback(const grid_map_msgs::GridMapConstPtr& grid_map);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg);

  void update();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters
  double update_frequency_;
  std::string elevation_layer_name_;

  ros::Timer timer_;
  ros::Subscriber grid_map_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher stability_pub_;
  ros::Publisher traction_pub_;
  ros::Publisher support_polygon_pub_;
  ros::Publisher com_pub_;

  grid_map_msgs::GridMapConstPtr latest_grid_map_;

  std::shared_ptr<hector_pose_prediction_interface::PosePredictor<float>> pose_predictor_;
  urdf::Model urdf_model_;
  std::set<std::string> missing_joint_states_;
  std::unordered_map<std::string, float> joint_states_;
};

template <typename T>
std::string setToString(const std::set<T>& set) {
  std::stringstream ss;
  ss << "[";
  for (auto entry: set) {
    ss << entry << ",";
  }
  ss << "]";
  return ss.str();
}

}

#endif