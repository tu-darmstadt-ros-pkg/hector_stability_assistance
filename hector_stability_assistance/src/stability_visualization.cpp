#include <hector_stability_assistance/stability_visualization.h>

#include <hector_heightmap_pose_prediction/heightmap_pose_predictor.h>
#include <hector_heightmap_pose_prediction/robot_heightmap_providers/urdf_robot_heightmap_provider.h>
#include <eigen_conversions/eigen_msg.h>

namespace hector_stability_assistance {

StabilityVisualization::StabilityVisualization(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
: nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_), update_frequency_(10.0) {}

bool StabilityVisualization::init() {
  // Parameters
  update_frequency_ = pnh_.param("update_frequency", 10.0);

  // Load urdf model
  if (!urdf_model_.initParam("robot_description")) {
    ROS_ERROR("Failed to load robot_description");
    return false;
  }

  for (const auto &kv : urdf_model_.joints_)
  {
    missing_joint_states_.insert( kv.first );
  }

  // Subscriber and Publisher
  stability_pub_ = pnh_.advertise<std_msgs::Float64>("stability", 1);
  traction_pub_ = pnh_.advertise<std_msgs::Float64>("traction", 1);
  support_polygon_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("support_polygon", 1);
  com_pub_ = pnh_.advertise<geometry_msgs::PointStamped>("center_of_mass", 1);

  grid_map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("elevation_map", 1, &StabilityVisualization::gridMapCallback, this);
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &StabilityVisualization::jointStateCallback, this);
  timer_ = nh_.createTimer(ros::Duration(1.0/update_frequency_), &StabilityVisualization::timerCallback, this);
  return true;
}

void StabilityVisualization::gridMapCallback(const grid_map_msgs::GridMapConstPtr& grid_map) {
  latest_grid_map_ = grid_map;
}

void StabilityVisualization::timerCallback(const ros::TimerEvent&) {
  update();
}

void StabilityVisualization::update() {
  // Update robot state
  if (!missing_joint_states_.empty()) {
    ROS_WARN_STREAM_THROTTLE(1, "Can't update stability estimation: The following joint states are still missing: ");
    return;
  }
  auto robot_height_map = std::make_shared<hector_heightmap_pose_prediction::UrdfRobotHeightmapProvider<SCALAR_TYPE>>(urdf_model_, joint_states_);

  // Update map
  if (!latest_grid_map_) {
    ROS_WARN_STREAM_THROTTLE(1, "Can't update stability estimation: No elevation map has been received yet.");
    return;
  }
  hector_world_heightmap::WorldHeightmapInterface<SCALAR_TYPE>::Ptr height_map;

  // Get robot position
  geometry_msgs::TransformStamped transform_msg;
  try{
    transform_msg = tf_buffer_.lookupTransform(latest_grid_map_->info.header.frame_id, "base_link",
                                             ros::Time(0), ros::Duration(1.0));
  }
  catch (const tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return;
  }

  Eigen::Matrix<SCALAR_TYPE, 3, 1> robot_position;
  Eigen::Quaternion<SCALAR_TYPE> robot_orientation;
  tf::vectorMsgToEigen(transform_msg.transform.translation, robot_position);
  tf::quaternionMsgToEigen(transform_msg.transform.rotation, robot_orientation);
  hector_pose_prediction_interface::math::Pose<SCALAR_TYPE> robot_pose(robot_position, robot_orientation);


  // Generate pose prediction with current state
  pose_predictor_ = std::make_shared<hector_heightmap_pose_prediction::HeightmapPosePredictor<SCALAR_TYPE, false>>(height_map, robot_height_map);

  hector_pose_prediction_interface::math::SupportPolygon<SCALAR_TYPE> support_polygon;
  hector_pose_prediction_interface::ContactInformation<SCALAR_TYPE> contact_information;
  if (!pose_predictor_->estimateContactInformation(robot_pose, support_polygon, contact_information)) {
    ROS_WARN("Support polygon estimation failed.");
    return;
  }

  // Publish results
}

void StabilityVisualization::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg) {
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
    joint_states_[joint_state_msg->name[i]] = joint_state_msg->position[i];
  }
}



}