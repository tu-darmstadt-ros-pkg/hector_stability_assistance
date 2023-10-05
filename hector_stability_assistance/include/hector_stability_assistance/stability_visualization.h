#ifndef HECTOR_STABILITY_ASSISTANCE_STABILITY_VISUALIZATION_H
#define HECTOR_STABILITY_ASSISTANCE_STABILITY_VISUALIZATION_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <voxblox_ros/esdf_server.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
//#include <grid_map_msgs/GridMap.h>


#include <hector_pose_prediction_interface/pose_predictor.h>

#include <hector_stability_assistance/robot_state_provider.h>

namespace hector_stability_assistance {

class StabilityVisualization {
public:
  StabilityVisualization(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  bool init();
private:
  void timerCallback(const ros::TimerEvent&);
//  void gridMapCallback(const grid_map_msgs::GridMapConstPtr& grid_map);

  void update();

  bool estimateRobotPose(Eigen::Isometry3d& robot_pose,
                         hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                         hector_pose_prediction_interface::ContactInformation<double>& contact_information,
                         bool predict_pose);
  void computeStabilityMargin(const Eigen::Isometry3d& robot_pose, hector_pose_prediction_interface::SupportPolygon<double>& support_polygon);

  void publishCOM() const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters
  double update_frequency_;
  std::string elevation_layer_name_;
  bool predict_pose_;

  std::string world_frame_;
  std::string base_frame_;

  ros::Timer timer_;
//  ros::Subscriber grid_map_sub_;
  ros::Publisher stability_margin_pub_;
  ros::Publisher stability_margins_pub_;
  ros::Publisher traction_pub_;
  ros::Publisher support_polygon_pub_;
  ros::Publisher com_pub_;
  ros::Publisher robot_heightmap_pub_;
  ros::Publisher submap_pub_;

  std::shared_ptr<voxblox::EsdfServer> esdf_server_;
  std::shared_ptr<sdf_contact_estimation::SdfModel> sdf_model_;

//  grid_map_msgs::GridMapConstPtr latest_grid_map_;

  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  std::shared_ptr<urdf::Model> urdf_;
  std::shared_ptr<RobotStateProvider> state_provider_;
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