#ifndef HECTOR_STABILITY_ASSISTANCE_WHOLE_BODY_POSTURE_ASSISTANCE_H
#define HECTOR_STABILITY_ASSISTANCE_WHOLE_BODY_POSTURE_ASSISTANCE_H

#include <ros/ros.h>

#include <urdf/model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <voxblox_ros/esdf_server.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

#include <hector_stability_assistance/robot_state_provider.h>
#include <std_msgs/Bool.h>

namespace hector_stability_assistance {

class WholeBodyPostureAssistance {
public:
  WholeBodyPostureAssistance(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  bool init();
  void update();
  bool mapReceived() const;
private:
  bool loadParameters(const ros::NodeHandle& nh);
  bool initRobotModel();
  bool initPostureOptimization();

  void timerCallback(const ros::TimerEvent& event);

  robot_trajectory::RobotTrajectory createTrajectory(const moveit::core::RobotState& start_state, const moveit::core::RobotState& end_state) const;
  bool executeJointTrajectory(const robot_trajectory::RobotTrajectory& trajectory, ros::Time start_time=ros::Time());

  void cmdVelCallback(const geometry_msgs::TwistConstPtr& twist_msg);
  void enableCallback(const std_msgs::BoolConstPtr& bool_msg);
  void publishEnabledStatus();


  void publishRobotStateDisplay(const robot_state::RobotStatePtr& robot_state);

  // Parameters
  double control_rate_;
  bool enabled_;
  std::string move_group_;

  std::string base_frame_;
  std::string world_frame_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Timer timer_;
  std::shared_ptr<RobotStateProvider> state_provider_;

  std::shared_ptr<urdf::Model> urdf_;
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

  std::shared_ptr<voxblox::EsdfServer> esdf_server_;
  std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;
  std::shared_ptr<whole_body_posture_optimization::PostureOptimizationResult> last_result_;

  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;

  ros::Subscriber cmd_vel_sub_;
  geometry_msgs::Twist latest_twist_;
  bool last_twist_zero_;

  ros::Subscriber enable_sub_;
  ros::Publisher enabled_status_pub_;

  ros::Publisher robot_display_pub_;
  ros::Publisher robot_marker_pub_;
};

}  // namespace hector_stability_assistance

#endif  // HECTOR_STABILITY_ASSISTANCE_WHOLE_BODY_POSTURE_ASSISTANCE_H
