#ifndef HECTOR_STABILITY_ASSISTANCE_STABILITY_VISUALIZATION_H
#define HECTOR_STABILITY_ASSISTANCE_STABILITY_VISUALIZATION_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

//#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <voxblox_ros/esdf_server.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>

#include <hector_pose_prediction_interface/pose_predictor.h>

namespace hector_stability_assistance {

class StabilityVisualization {
public:
  StabilityVisualization(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  bool init();
private:
  void timerCallback(const ros::TimerEvent&);
//  void gridMapCallback(const grid_map_msgs::GridMapConstPtr& grid_map);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg);
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& twist_msg);

  void update();

  bool initializeRobotModel();

  bool estimateRobotPose(Eigen::Isometry3d& robot_pose,
                         hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                         hector_pose_prediction_interface::ContactInformation<double>& contact_information,
                         bool predict_pose);
  void computeStabilityMargin(const Eigen::Isometry3d& robot_pose, hector_pose_prediction_interface::SupportPolygon<double>& support_polygon);

  bool getRobotPose(Eigen::Isometry3d& robot_pose) const;
  Eigen::Isometry3d computeDiffDriveTransform(double linear_speed, double angular_speed, double time_delta) const;

  void publishCOM() const;
  void publishEdgeStabilities(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                              ros::Publisher& publisher) const;
  void publishMinStability(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                           ros::Publisher& publisher) const;
  void publishSupportPolygon(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                             const hector_pose_prediction_interface::ContactInformation<double>& contact_information,
                             ros::Publisher& publisher) const;

  void publishRobotModel(const Eigen::Isometry3d& robot_pose,
                         const std::unordered_map<std::string, double>& joint_state,
                         ros::Publisher& publisher) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters
  double update_frequency_;
  std::string elevation_layer_name_;
  bool predict_pose_;
  double prediction_time_delta_;

  ros::Timer timer_;
  ros::Subscriber grid_map_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher stability_margin_pub_;
  ros::Publisher stability_margins_pub_;
  ros::Publisher traction_pub_;
  ros::Publisher support_polygon_pub_;
  ros::Publisher com_pub_;
  ros::Publisher robot_heightmap_pub_;
  ros::Publisher submap_pub_;

  ros::Publisher predicted_stability_margin_pub_;
  ros::Publisher predicted_stability_margins_pub_;
  ros::Publisher predicted_traction_pub_;
  ros::Publisher predicted_support_polygon_pub_;
  ros::Publisher predicted_robot_model_pub_;

  std::shared_ptr<voxblox::EsdfServer> esdf_server_;
  std::shared_ptr<sdf_contact_estimation::SdfModel> sdf_model_;

  geometry_msgs::Twist latest_twist_;
//  grid_map_msgs::GridMapConstPtr latest_grid_map_;

  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  std::shared_ptr<urdf::Model> urdf_;
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

  std::set<std::string> missing_joint_states_;
  std::unordered_map<std::string, double> joint_states_;
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