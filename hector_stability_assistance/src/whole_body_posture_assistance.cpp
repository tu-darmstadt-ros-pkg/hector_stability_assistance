#include <hector_stability_assistance/whole_body_posture_assistance.h>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <moveit/robot_state/conversions.h>

#include <hector_stability_assistance/visualization.h>
#include <hector_stability_assistance/util.h>

namespace hector_stability_assistance {

WholeBodyPostureAssistance::WholeBodyPostureAssistance(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
: nh_(nh), pnh_(pnh), last_twist_zero_(false)
{
  latest_twist_.linear.x = 0.0;
  latest_twist_.angular.z = 0.0;
}

bool WholeBodyPostureAssistance::init() {
  if (!loadParameters(pnh_)) {
    return false;
  }
  if (!initRobotModel()) {
    return false;
  }
  if (!initPostureOptimization()) {
    return false;
  }

  std::vector<std::string> joint_names;
  joint_names.reserve(urdf_->joints_.size());
  for (const auto &kv : urdf_->joints_)
  {
    if (kv.second->type != urdf::Joint::FIXED && kv.second->type != urdf::Joint::UNKNOWN && !kv.second->mimic) {
      joint_names.push_back( kv.first );
      ROS_DEBUG_STREAM("Adding required joint: " << kv.first << " type: " << kv.second->type);
    }
  }
  state_provider_ = std::make_shared<RobotStateProvider>(nh_, joint_names, world_frame_, base_frame_);

  moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(pnh_);

  // Publishers
  robot_display_pub_ = pnh_.advertise<moveit_msgs::DisplayRobotState>("optimized_robot_state", 10);
  robot_marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("optimized_robot_state_marker", 10);
  enabled_status_pub_ = pnh_.advertise<std_msgs::Bool>("enabled_status", 10, true);
  publishEnabledStatus();

  // Subscribers
  cmd_vel_sub_ = pnh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &WholeBodyPostureAssistance::cmdVelCallback, this);
  enable_sub_ = pnh_.subscribe<std_msgs::Bool>("enable", 10, &WholeBodyPostureAssistance::enableCallback, this);

  timer_ = nh_.createTimer(ros::Duration(1/control_rate_), &WholeBodyPostureAssistance::timerCallback, this, false, true);

  return true;
}

bool WholeBodyPostureAssistance::loadParameters(const ros::NodeHandle &nh) {
  enabled_ = nh.param("enabled", enabled_);
  control_rate_ = nh.param("control_rate", control_rate_);
  if (control_rate_ <= 0) {
    ROS_ERROR("control_rate must be greater 0.");
    return false;
  }
  if (!nh.getParam("move_group", move_group_)) {
    ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/move_group is missing");
    return false;
  }
  return true;
}

bool WholeBodyPostureAssistance::initRobotModel() {
  if (!robot_model_) {
    urdf_ = std::make_shared<urdf::Model>();
    if (!urdf_->initParam("/robot_description")) {
      return false;
    }
    base_frame_ = urdf_->getRoot()->name;
    auto srdf = std::make_shared<srdf::Model>();
    std::string semantic_description;
    if (pnh_.getParam("/robot_description_semantic", semantic_description)) {
      srdf->initString(*urdf_, semantic_description);
    }

    try {
      robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_, srdf);
      robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    } catch (std::exception& e) {
      ROS_ERROR_STREAM( "Failed to initialize robot model: " << e.what());
      return false;
    }
    robot_state_->setToDefaultValues();
  }
  return true;
}
bool WholeBodyPostureAssistance::initPostureOptimization() {
  // Create SDF Model
  ros::NodeHandle esdf_server_pnh(pnh_, "esdf_server");
  esdf_server_ = std::make_shared<voxblox::EsdfServer>(nh_, esdf_server_pnh);
  std::shared_ptr<sdf_contact_estimation::SdfModel> sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(pnh_);
  sdf_model->loadEsdf(esdf_server_->getEsdfMapPtr(), esdf_server_->getEsdfMaxDistance(), false);
  world_frame_ = esdf_server_->getWorldFrame();

  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor;
  std::string pose_predictor_name = pnh_.param<std::string>("pose_predictor", "sdf_contact_estimation::SDFContactEstimation");
  if (pose_predictor_name == "sdf_contact_estimation::SDFContactEstimation") {
    // Create robot model
    ros::NodeHandle pose_predictor_nh(pnh_, "sdf_pose_predictor");
    ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
    auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);

    // Create pose predictor
    auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pose_predictor_nh, shape_model, sdf_model);
    sdf_pose_predictor->enableVisualisation(false);
    pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);
  } else if (pose_predictor_name == "hector_heightmap_pose_prediction::HeightmapPosePredictor") {
    ROS_ERROR_STREAM("Not implemented.");
    return false;

    //    auto robot_heightmap_provider = std::make_shared<hector_heightmap_pose_prediction::UrdfRobotHeightmapProvider<double>>(map_bag->resolution(), urdf_model_);
    //    robot_heightmap_provider->disableHeightmapCache();
    //    auto heightmap_pose_predictor = std::make_shared<hector_heightmap_pose_prediction::HeightmapPosePredictor<double>>(map_bag, robot_heightmap_provider);
    //    pose_predictor_ = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(heightmap_pose_predictor);

    //    robot_heightmap_pub_ = pnh_.advertise<sensor_msgs::PointCloud>("robot_heightmap", 1);
    //    submap_pub_ = pnh_.advertise<grid_map_msgs::GridMap>("submap", 1);
    //    grid_map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("elevation_map", 1, &StabilityVisualization::gridMapCallback, this);
  } else {
    ROS_ERROR_STREAM("Unknown pose predictor '" << pose_predictor_name << "'.");
    return false;
  }

  optimizer_ = std::make_shared<whole_body_posture_optimization::WholeBodyPostureOptimization>(pnh_, pose_predictor, sdf_model);

  return true;
}

void WholeBodyPostureAssistance::timerCallback(const ros::TimerEvent &event) {
  update();
}

void WholeBodyPostureAssistance::update() {
  if (!enabled_) {
    return;
  }
  // Stop sending commands when twist is zero (repeatedly)
  if (latest_twist_.linear.x == 0 && latest_twist_.angular.z == 0) {
    if (last_twist_zero_) {
      return;
    }
    last_twist_zero_ = true;
  } else {
    last_twist_zero_ = false;
  }
  if (!mapReceived()) {
    ROS_WARN_STREAM_THROTTLE(1, "No map received yet");
    return;
  }
  if (!last_result_) {
    auto current_state = std::make_shared<moveit::core::RobotState>(robot_model_);
    if (state_provider_->getRobotState(*current_state)) {
      last_result_ = std::make_shared<whole_body_posture_optimization::PostureOptimizationResult>();
      last_result_->result_state = current_state;
    } else {
      ROS_WARN_STREAM_THROTTLE(1, "Waiting for initial state");
      return;
    }
  }

  Eigen::Isometry3d current_robot_pose;
  if (!state_provider_->getRobotPose(current_robot_pose)) {
    return;
  }
  double distance = 0.15;
  double time;
  double linear_abs = std::abs(latest_twist_.linear.x);
  if (linear_abs > 0.0) {
    time = distance / linear_abs;
  } else {
    time = 0.5;
  }
  Eigen::Isometry3d movement_delta_transform = util::computeDiffDriveTransform(latest_twist_.linear.x, latest_twist_.angular.z, time);
  Eigen::Isometry3d query_pose = util::pose3Dto2D(current_robot_pose) * movement_delta_transform;
  auto result = optimizer_->findOptimalPosture(query_pose, optimizer_->getDefaultJointPositions(), last_result_->result_state);
  last_result_ = std::make_shared<whole_body_posture_optimization::PostureOptimizationResult>(result);
  publishRobotStateDisplay(result.result_state);

  // Execute trajectory
  if (result.success && result.result_state) {
    moveit::core::RobotState current_state(robot_model_);
    if (!state_provider_->getRobotState(current_state)) {
      ROS_ERROR_STREAM("Failed to retrieve current state");
      return;
    }
    robot_trajectory::RobotTrajectory trajectory = createTrajectory(current_state, *result.result_state);
    executeJointTrajectory(trajectory, ros::Time::now());
  }
}

bool WholeBodyPostureAssistance::mapReceived() const {
  return esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() != 0;
}

void WholeBodyPostureAssistance::publishRobotStateDisplay(const moveit::core::RobotStatePtr &robot_state) {
  moveit_msgs::DisplayRobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg.state);
  robot_display_pub_.publish(robot_state_msg);
}

bool WholeBodyPostureAssistance::executeJointTrajectory(const robot_trajectory::RobotTrajectory &trajectory, ros::Time start_time) {
  // Check if there are controllers that can handle the execution
  if (!moveit_cpp_ptr_->getTrajectoryExecutionManager()->ensureActiveControllersForGroup(trajectory.getGroupName())) {
    ROS_ERROR("Execution failed! No active controllers configured for group '%s'", trajectory.getGroupName().c_str());
    return false;
  }

  // Execute trajectory
  moveit_msgs::RobotTrajectory robot_trajectory_msg;
  trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);
  robot_trajectory_msg.joint_trajectory.header.stamp = start_time;
  return moveit_cpp_ptr_->getTrajectoryExecutionManager()->pushAndExecute(robot_trajectory_msg);
}

robot_trajectory::RobotTrajectory WholeBodyPostureAssistance::createTrajectory(const moveit::core::RobotState &start_state,
                                                  const moveit::core::RobotState &end_state) const
{
  robot_trajectory::RobotTrajectory trajectory(robot_model_, move_group_);
  trajectory.addSuffixWayPoint(start_state, 0.0);
  trajectory.addSuffixWayPoint(end_state, 1/control_rate_);
  return trajectory;
}

void WholeBodyPostureAssistance::cmdVelCallback(const geometry_msgs::TwistConstPtr &twist_msg) {
  latest_twist_ = *twist_msg;
}

void WholeBodyPostureAssistance::enableCallback(const std_msgs::BoolConstPtr &bool_msg) {
  enabled_ = bool_msg->data;
  ROS_INFO_STREAM((enabled_ ? "Enabling " : "Disabling ") << " whole body posture assistance.");
  publishEnabledStatus();
}

void WholeBodyPostureAssistance::publishEnabledStatus() {
  std_msgs::Bool bool_msg;
  bool_msg.data = enabled_;
  enabled_status_pub_.publish(bool_msg);
}

}  // namespace hector_stability_assistance