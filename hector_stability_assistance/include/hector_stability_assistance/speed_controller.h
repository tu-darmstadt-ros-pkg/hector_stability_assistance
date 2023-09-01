#ifndef HECTOR_STABILITY_ASSISTANCE_SPEED_CONTROLLER_H
#define HECTOR_STABILITY_ASSISTANCE_SPEED_CONTROLLER_H

#include <ros/ros.h>
#include <unordered_map>
#include <hector_pose_prediction_interface/types.h>
#include <voxblox_ros/esdf_server.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf/model.h>
#include <hector_pose_prediction_interface/pose_predictor.h>

namespace hector_stability_assistance {

struct RobotTerrainState {
  Eigen::Isometry3d robot_pose;
  std::unordered_map<std::string, double> joint_positions;

  double minimum_stability;
  Eigen::Vector3d center_of_mass;
  hector_pose_prediction_interface::SupportPolygon<double> support_polygon;
  hector_pose_prediction_interface::ContactInformation<double> contact_information;
};

class SpeedController {
public:
  SpeedController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  bool init();
private:
  bool loadParameters(const ros::NodeHandle& nh);
  bool initRobotModel();
  bool initPosePredictor();

  void computeSpeedCommand(double& linear, double& angular);
  std::vector<RobotTerrainState> predictTerrainInteraction(double linear, double angular);
  void publishTerrainInteraction(const std::vector<RobotTerrainState>& robot_states);
  double computeSpeedScaling(const std::vector<RobotTerrainState>& robot_states);

  bool estimateRobotPose(RobotTerrainState& robot_terrain_state);
  bool getRobotPose(Eigen::Isometry3d& robot_pose) const;
  void cmdVelCallback(geometry_msgs::Twist twist_msg);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  /// Parameters
  double prediction_horizon_;
  double sample_resolution_;
  double critical_stability_threshold_;
  double warn_stability_threshold_;

  std::string world_frame_;
  std::string base_frame_;

  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  std::shared_ptr<urdf::Model> urdf_;
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

  std::shared_ptr<voxblox::EsdfServer> esdf_server_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::set<std::string> missing_joint_states_;
  std::unordered_map<std::string, double> joint_states_;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber joint_state_sub_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher robot_display_pub_;
  ros::Publisher support_polygon_pub_;
};

}  // namespace hector_stability_assistance

#endif  // HECTOR_STABILITY_ASSISTANCE_SPEED_CONTROLLER_H