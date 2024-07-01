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
#include <nav_msgs/Odometry.h>
#include <condition_variable>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

namespace hector_stability_assistance {

namespace ba = boost::accumulators;

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
  void spinEsdfUpdate();

  robot_trajectory::RobotTrajectory createTrajectory(const moveit::core::RobotState& start_state, const moveit::core::RobotState& end_state) const;
  bool executeJointTrajectory(const robot_trajectory::RobotTrajectory& trajectory, ros::Time start_time=ros::Time());

  void cmdVelCallback(geometry_msgs::Twist twist_msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  void enableCallback(const std_msgs::BoolConstPtr& bool_msg);
  void publishEnabledStatus();
  void publishOptimizationStatus(bool success);


  void publishRobotStateDisplay(const robot_state::RobotStatePtr& robot_state, bool failed);
  void hideRobotStateDisplay();
  void publishSupportPolygon(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon);

  double computeSpeedScaling(double linear_speed, double angular_speed);
  double approximateTimeForStateChange(const robot_state::RobotState& state_a, const robot_state::RobotState& state_b);

  // Parameters
  bool stop_on_optimization_failure_{true};
  double control_rate_duration_{0.1};
  std::atomic<bool> enabled_{true};
  std::string move_group_;
  double prediction_distance_{0.15};
  double prediction_angle_{0.2};
  int stagnation_mean_window_size_{50};

  std::string base_frame_;
  std::string world_frame_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Timer timer_;
  ros::CallbackQueue timer_queue_;
  ros::AsyncSpinner timer_spinner_{0, &timer_queue_};
  std::shared_ptr<RobotStateProvider> state_provider_;

  std::shared_ptr<urdf::Model> urdf_;
  moveit::core::RobotModelConstPtr robot_model_;

  std::shared_ptr<voxblox::EsdfServer> esdf_server_;
  mutable std::mutex esdf_update_mutex_;
  ros::CallbackQueue esdf_server_queue_;
  std::unique_ptr<std::thread> esdf_update_thread_;

  std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;
  robot_state::RobotStatePtr last_result_state_;

  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;
  robot_trajectory::RobotTrajectoryPtr trajectory_;
  mutable std::mutex trajectory_mutex_;
  mutable std::mutex execution_mutex_;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber odom_sub_;
  geometry_msgs::Twist latest_twist_;
  geometry_msgs::Twist latest_twist_output_;
  mutable std::mutex twist_update_mutex_;
  std::atomic<bool> last_twist_zero_{false};
  std::atomic<bool> last_optimization_successful_{true};
  ba::accumulator_set<double, ba::stats<ba::tag::rolling_mean>> stagnation_mean_acc_;


  double stagnation_{0.0};

  ros::Subscriber enable_sub_;
  ros::Publisher enabled_status_pub_;

  ros::Publisher robot_display_pub_;
  ros::Publisher query_pose_pub_;
  ros::Publisher robot_marker_pub_;
  ros::Publisher support_polygon_pub_;
  ros::Publisher stagnation_pub_;
  ros::Publisher optimization_status_pub_;

  ros::Publisher cmd_vel_pub_;
};

}  // namespace hector_stability_assistance

#endif  // HECTOR_STABILITY_ASSISTANCE_WHOLE_BODY_POSTURE_ASSISTANCE_H
