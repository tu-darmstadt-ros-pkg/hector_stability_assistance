#ifndef HECTOR_STABILITY_ASSISTANCE_ROBOT_STATE_PROVIDER_H
#define HECTOR_STABILITY_ASSISTANCE_ROBOT_STATE_PROVIDER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <Eigen/Eigen>

namespace hector_stability_assistance {

class RobotStateProvider {
public:
  RobotStateProvider(const ros::NodeHandle& nh, const std::vector<std::string>& joint_names,
                     std::string  world_frame, std::string  base_frame);
  bool getRobotPose(Eigen::Isometry3d& robot_pose) const;
  bool jointStateComplete() const;
  const std::unordered_map<std::string, double>& getJointState() const;
private:
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg);

  ros::NodeHandle nh_;
  std::string world_frame_;
  std::string base_frame_;

  std::set<std::string> missing_joint_states_;
  std::unordered_map<std::string, double> joint_state_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber joint_state_sub_;
};

}  // namespace hector_stability_assistance

#endif  // HECTOR_STABILITY_ASSISTANCE_ROBOT_STATE_PROVIDER_H
