#include <hector_stability_assistance/whole_body_posture_assistance.h>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <moveit/robot_state/conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <hector_stability_assistance/visualization.h>
#include <hector_stability_assistance/util.h>
#include <hector_pose_prediction_ros/visualization.h>

namespace hector_stability_assistance {

WholeBodyPostureAssistance::WholeBodyPostureAssistance(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
: nh_(nh),
  pnh_(pnh),
  enabled_(true),
  control_rate_duration_(0.1),
  prediction_distance_(0.15),
  prediction_angle_(0.2),
  last_twist_zero_(false),
  timer_spinner_(0, &timer_queue_)
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
  query_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("query_pose", 10);
  robot_marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("optimized_robot_state_marker", 10);
  enabled_status_pub_ = pnh_.advertise<std_msgs::Bool>("enabled_status", 10, true);
  cmd_vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 10, false);
  support_polygon_pub_ = pnh_.advertise<visualization_msgs::MarkerArray >("optimized_support_polygon", 10);
  stagnation_pub_ = pnh_.advertise<std_msgs::Float64>("stagnation", 10);
  optimization_status_pub_ = pnh_.advertise<std_msgs::Bool>("optimization_status", 10, true);
  publishEnabledStatus();
  publishOptimizationStatus(true);

  // Subscribers
  cmd_vel_sub_ = pnh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &WholeBodyPostureAssistance::cmdVelCallback, this);
  odom_sub_ = pnh_.subscribe<nav_msgs::Odometry>("/odom", 10, &WholeBodyPostureAssistance::odomCallback, this);
  enable_sub_ = pnh_.subscribe<std_msgs::Bool>("enable", 10, &WholeBodyPostureAssistance::enableCallback, this);


  // Timer on separate thread
  ros::NodeHandle timer_nh(nh_);
  timer_nh.setCallbackQueue(&timer_queue_);
  timer_ = timer_nh.createTimer(ros::Duration(control_rate_duration_), &WholeBodyPostureAssistance::timerCallback, this, false, true);
  timer_spinner_.start();

  return true;
}

bool WholeBodyPostureAssistance::loadParameters(const ros::NodeHandle &nh) {
  bool enabled = enabled_;
  enabled = nh.param("enabled", enabled);
  enabled_ = enabled;
  double control_rate = nh.param("control_rate", 10);
  if (control_rate <= 0) {
    ROS_ERROR("control_rate must be greater 0.");
    return false;
  }
  control_rate_duration_ = 1.0 / control_rate;

  prediction_distance_ = nh.param("prediction_distance", prediction_distance_);
  prediction_angle_ = nh.param("prediction_angle", prediction_angle_);
  if (!nh.getParam("move_group", move_group_)) {
    ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/move_group is missing");
    return false;
  }
  stop_on_optimization_failure_ = nh.param("stop_on_optimization_failure", stop_on_optimization_failure_);

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
  ros::NodeHandle esdf_server_nh(nh_);
  esdf_server_nh.setCallbackQueue(&esdf_server_queue_);
  ros::NodeHandle esdf_server_pnh(pnh_, "esdf_server");
  esdf_server_pnh.setCallbackQueue(&esdf_server_queue_);
  esdf_server_ = std::make_shared<voxblox::EsdfServer>(esdf_server_nh, esdf_server_pnh);

  std::shared_ptr<sdf_contact_estimation::SdfModel> sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(pnh_);
  sdf_model->loadEsdf(esdf_server_->getEsdfMapPtr(), esdf_server_->getEsdfMaxDistance(), false);
  world_frame_ = esdf_server_->getWorldFrame();

  esdf_update_thread_ = std::make_unique<std::thread>(&WholeBodyPostureAssistance::spinEsdfUpdate, this);
  esdf_update_thread_->detach();

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
  double event_delay = event.current_real.toSec() - event.current_expected.toSec();
  if (std::abs(event_delay) > control_rate_duration_) {
    ROS_WARN_STREAM("Update has been called with a delay of " << event_delay);
  }
  update();
}

void WholeBodyPostureAssistance::update() {
  if (!enabled_) {
    return;
  }
  // Stop sending commands when twist is zero (repeatedly)
  double linear_speed, angular_speed;
  {
    std::lock_guard<std::mutex> lock(twist_update_mutex_);
    linear_speed = latest_twist_.linear.x;
    angular_speed = latest_twist_.angular.z;
  }

  if (linear_speed == 0 && angular_speed == 0) {
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
  Eigen::Isometry3d current_pose_2d = util::pose3Dto2D(current_robot_pose);

  Eigen::Isometry3d query_pose;
  double linear_abs = std::abs(linear_speed);
  double angular_abs = std::abs(angular_speed);

  double epsilon = 1e-5;
  if (linear_abs < epsilon && angular_abs < epsilon) {
    query_pose = current_pose_2d;
  } else {
    double prediction_distance = prediction_distance_ + stagnation_ * prediction_distance_;
    double time_linear = linear_abs > 0.0 ? prediction_distance / linear_abs : std::numeric_limits<double>::max();
    double time_angular = angular_abs > 0.0 ? prediction_angle_ / angular_abs : std::numeric_limits<double>::max();
    double time = std::min(time_linear, time_angular);

    Eigen::Isometry3d movement_delta_transform = util::computeDiffDriveTransform(linear_speed, angular_speed, time);
    query_pose =  current_pose_2d * movement_delta_transform;
  }

  // Publish query pose
  visualization::publishPose(query_pose, world_frame_, query_pose_pub_);

  whole_body_posture_optimization::PostureOptimizationResult result;
  {
    std::unique_lock<std::mutex> esdf_update_lock(esdf_update_mutex_);
    result = optimizer_->findOptimalPosture(query_pose, optimizer_->getDefaultJointPositions(), last_result_->result_state);
  }
  bool success = result.success && result.result_state;
  publishOptimizationStatus(success);
  if (success) {
    publishRobotStateDisplay(result.result_state, false);
    publishSupportPolygon(result.support_polygon);
    last_result_ = std::make_shared<whole_body_posture_optimization::PostureOptimizationResult>(result);
    // Execute trajectory
    moveit::core::RobotState current_state(robot_model_);
    if (!state_provider_->getRobotState(current_state)) {
      ROS_ERROR_STREAM("Failed to retrieve current state");
      return;
    }
    robot_trajectory::RobotTrajectory trajectory = createTrajectory(current_state, *result.result_state);
    executeJointTrajectory(trajectory, ros::Time::now());
  } else {
    publishRobotStateDisplay(last_result_->result_state, true);
    last_result_ = nullptr;
  }
}

bool WholeBodyPostureAssistance::mapReceived() const {
  std::unique_lock<std::mutex> esdf_update_lock(esdf_update_mutex_);
  return esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() != 0;
}

void WholeBodyPostureAssistance::publishRobotStateDisplay(const moveit::core::RobotStatePtr &robot_state, bool failed) {
  moveit_msgs::DisplayRobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg.state);

  // Set color
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  if (failed) {
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0;
  } else {
    color.r = 0;
    color.g = 1.0;
    color.b = 0;
  }
  for (const auto& link: robot_model_->getLinkModelNames()) {
    moveit_msgs::ObjectColor object_color;
    object_color.color = color;
    object_color.id = link;
    robot_state_msg.highlight_links.push_back(object_color);
  }
  robot_display_pub_.publish(robot_state_msg);
}

void WholeBodyPostureAssistance::publishSupportPolygon(
    const hector_pose_prediction_interface::SupportPolygon<double> &support_polygon)
{
  visualization::deleteAllMarkers(support_polygon_pub_);
  visualization_msgs::MarkerArray support_polygon_marker_array;
  hector_pose_prediction_interface::visualization::addSupportPolygonToMarkerArray(support_polygon_marker_array, support_polygon, world_frame_);
  support_polygon_pub_.publish(support_polygon_marker_array);
}

bool WholeBodyPostureAssistance::executeJointTrajectory(const robot_trajectory::RobotTrajectory &trajectory, ros::Time start_time) {
  // Check if there are controllers that can handle the execution
  if (!moveit_cpp_ptr_->getTrajectoryExecutionManager()->ensureActiveControllersForGroup(trajectory.getGroupName())) {
    ROS_ERROR("Execution failed! No active controllers configured for group '%s'", trajectory.getGroupName().c_str());
    return false;
  }

  std::unique_lock<std::mutex> trajectory_lock(trajectory_mutex_);
  trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory, true);

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
  trajectory.addSuffixWayPoint(end_state, control_rate_duration_);
  return trajectory;
}

void WholeBodyPostureAssistance::cmdVelCallback(geometry_msgs::Twist twist_msg) {
  latest_twist_ = twist_msg;

  double scaling = computeSpeedScaling(latest_twist_.linear.x, latest_twist_.angular.z);
  twist_msg.linear.x *= scaling;
  twist_msg.angular.z *= scaling;

  latest_twist_output_ = twist_msg;
  cmd_vel_pub_.publish(latest_twist_output_);
}

void WholeBodyPostureAssistance::enableCallback(const std_msgs::BoolConstPtr &bool_msg) {
  enabled_ = bool_msg->data;
  ROS_INFO_STREAM((enabled_ ? "Enabling " : "Disabling ") << " whole body posture assistance.");
  publishEnabledStatus();
  if (!enabled_) {
    moveit_cpp_ptr_->getTrajectoryExecutionManager()->stopExecution();

  }
}

void WholeBodyPostureAssistance::publishEnabledStatus() {
  std_msgs::Bool bool_msg;
  bool_msg.data = enabled_;
  enabled_status_pub_.publish(bool_msg);
}

double WholeBodyPostureAssistance::approximateTimeForStateChange(const robot_state::RobotState &state_a,
                                                                 const robot_state::RobotState &state_b) {
  double required_time = 0.0;
  const robot_model::RobotModelConstPtr& model = state_a.getRobotModel();
  for (unsigned int i = 0; i < model->getJointModelCount(); ++i) {
    const moveit::core::JointModel* joint = model->getJointModel(i);
    if (joint->getVariableBounds().size() != 1 ||
        !joint->getVariableBounds()[0].velocity_bounded_) {
      continue;
    }
    double velocity_limit = joint->getVariableBounds()[0].max_velocity_;
    const double* pos_a = state_a.getJointPositions(joint);
    const double* pos_b = state_b.getJointPositions(joint);
    double distance = joint->distance(pos_a, pos_b);
    double time = distance / velocity_limit; // assume movement with constant max. velocity
    if (time > required_time) {
      required_time = time;
    }
  }

  return required_time;
}

double WholeBodyPostureAssistance::computeSpeedScaling(double linear_speed, double angular_speed) {
  // Stop on failed optimization
  if (!last_optimization_successful_ && stop_on_optimization_failure_) {
    return 0.0;
  }

  std::unique_lock<std::mutex> trajectory_lock(trajectory_mutex_);
  if (!trajectory_ || !enabled_) {
    return 1.0;
  }

  moveit::core::RobotState current_state(robot_model_);
  if (!state_provider_->getRobotState(current_state)) {
    ROS_ERROR_STREAM("Failed to retrieve current state");
    return 1.0;
  }

  double required_time = approximateTimeForStateChange(current_state, trajectory_->getLastWayPoint());

  // Distance of base
  Eigen::Isometry3d start_pose = current_state.getJointTransform("world_virtual_joint");
  Eigen::Isometry3d end_pose = trajectory_->getLastWayPoint().getJointTransform("world_virtual_joint");
  double base_distance = (start_pose.translation().block<2, 1>(0, 0) - end_pose.translation().block<2, 1>(0, 0)).norm();

  double max_base_speed = base_distance / required_time;

  double linear_speed_abs = std::abs(linear_speed);
  double scaling = max_base_speed / linear_speed_abs;
  scaling = std::min(scaling, 1.0); // Do not speed up
  return scaling;
}

void WholeBodyPostureAssistance::spinEsdfUpdate() {
  ros::WallRate rate(10);
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(esdf_update_mutex_);
      esdf_server_queue_.callAvailable();
    }
    rate.sleep();
  }
}

void WholeBodyPostureAssistance::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  if (latest_twist_output_.linear.x > 0.05) {
    double pct = odom_msg->twist.twist.linear.x / latest_twist_output_.linear.x;
    pct = hector_math::clamp(pct, 0.0, 1.0);
    stagnation_ = 1 - pct;
  } else {
    stagnation_ = 0.0;
  }
  std_msgs::Float64 stagnation_msg;
  stagnation_msg.data = stagnation_;
  stagnation_pub_.publish(stagnation_msg);
}

void WholeBodyPostureAssistance::publishOptimizationStatus(bool success) {
  last_optimization_successful_ = success;
  std_msgs::Bool bool_msg;
  bool_msg.data = success;
  optimization_status_pub_.publish(bool_msg);
}

}  // namespace hector_stability_assistance