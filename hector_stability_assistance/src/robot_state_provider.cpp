#include <hector_stability_assistance/robot_state_provider.h>

#include <utility>
#include <eigen_conversions/eigen_msg.h>

namespace hector_stability_assistance {

RobotStateProvider::RobotStateProvider(const ros::NodeHandle &nh, const std::vector<std::string> &joint_names,
                                       std::string world_frame, std::string base_frame)
  : nh_(nh),
  world_frame_(std::move(world_frame)),
  base_frame_(std::move(base_frame)),
  tf_listener_(tf_buffer_),
  missing_joint_states_(joint_names.begin(), joint_names.end())
{
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &RobotStateProvider::jointStateCallback, this);
}

bool RobotStateProvider::getRobotPose(Eigen::Isometry3d &robot_pose) const {
  geometry_msgs::TransformStamped transform_msg;
  try{
    transform_msg = tf_buffer_.lookupTransform(world_frame_, base_frame_,
                                               ros::Time(0), ros::Duration(1.0));
  }
  catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1, "%s",ex.what());
    return false;
  }
  tf::transformMsgToEigen(transform_msg.transform, robot_pose);
  return true;
}

const std::unordered_map<std::string, double>& RobotStateProvider::getJointState() const
{
  return joint_state_;
}

void RobotStateProvider::jointStateCallback(const sensor_msgs::JointStateConstPtr &joint_state_msg) {
  // Mark seen joints
  if (!missing_joint_states_.empty()) {
    for (const auto & joint_name : joint_state_msg->name) {
      auto it = missing_joint_states_.find(joint_name);
      if (it != missing_joint_states_.end()) {
        missing_joint_states_.erase(it);
      }
    }
  }
  // Update state
  for (unsigned int i = 0; i < joint_state_msg->name.size(); ++i) {
    joint_state_[joint_state_msg->name[i]] = joint_state_msg->position[i];
  }
}

bool RobotStateProvider::jointStateComplete() const {
  return missing_joint_states_.empty();
}
}  // namespace hector_stability_assistance