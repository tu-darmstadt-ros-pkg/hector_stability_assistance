#include <hector_stability_assistance/speed_controller.h>

#include <hector_stability_assistance/visualization.h>
#include <hector_stability_assistance/util.h>

#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/util/utils.h>
#include <hector_rviz_plugins_msgs/DisplayMultiRobotState.h>
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include <moveit/robot_state/conversions.h>
#include <hector_pose_prediction_ros/visualization.h>

namespace hector_stability_assistance {

SpeedController::SpeedController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
: nh_(nh),
  pnh_(pnh),
  enabled_(true),
  control_rate_(20.0),
  prediction_horizon_(2.0),
  maximum_time_step_(0.25),
  safety_distance_(0.0),
  sample_resolution_(0.05),
  angular_sample_resolution_(0.15),
  critical_stability_threshold_(0.0),
  warn_stability_threshold_(1.0),
  last_twist_zero_(false),
  command_received_(false),
  virtual_inertia_factor_(0.0)
{
  latest_twist_.linear.x = 0.0;
  latest_twist_.angular.z = 0.0;
}

bool SpeedController::init() {
  if (!initRobotModel()) {
    return false;
  }

  if (!loadParameters(pnh_)) {
    return false;
  }

  if (!initPosePredictor()) {
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

  // Publishers
  support_polygon_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("support_polygon", 10);
  predicted_path_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("predicted_path", 10);
  robot_display_pub_ = pnh_.advertise<hector_rviz_plugins_msgs::DisplayMultiRobotState>("predicted_robot_states", 10);
  robot_marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("predicted_robot_markers", 10);
  speed_scaling_pub_ = pnh_.advertise<std_msgs::Float64>("speed_scaling", 10);
  cmd_vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);
  enabled_status_pub_ = pnh_.advertise<std_msgs::Bool>("enabled_status", 10, true);
  publishEnabledStatus();

  // Subscribers
  cmd_vel_sub_ = pnh_.subscribe<geometry_msgs::Twist>("cmd_vel_in", 10, &SpeedController::cmdVelCallback, this);
  enable_sub_ = pnh_.subscribe<std_msgs::Bool>("enable", 10, &SpeedController::enableCallback, this);

  timer_ = nh_.createTimer(ros::Duration(1/control_rate_), &SpeedController::timerCallback, this, false, true);

  return true;
}

bool SpeedController::loadParameters(const ros::NodeHandle& nh) {
  enabled_ = nh.param("enabled", enabled_);
  control_rate_ = nh.param("control_rate", control_rate_);
  if (control_rate_ <= 0) {
    ROS_ERROR("control_rate must be greater 0.");
    return false;
  }
  prediction_horizon_ = nh.param("prediction_horizon", prediction_horizon_);
  if (prediction_horizon_ < 0) {
    ROS_ERROR("prediction_horizon must be greater or equal 0.");
    return false;
  }
  maximum_time_step_ = nh.param("maximum_time_step", maximum_time_step_);
  if (maximum_time_step_ <= 0) {
    ROS_ERROR("maximum_time_step must be greater 0.");
    return false;
  }
  safety_distance_ = nh.param("safety_distance", safety_distance_);
  if (safety_distance_ < 0) {
    ROS_ERROR("safety_distance must be greater or equal 0.");
    return false;
  }
  sample_resolution_ = nh.param("sample_resolution", sample_resolution_);
  if (sample_resolution_ <= 0) {
    ROS_ERROR("sample_resolution must be greater than 0.");
    return false;
  }
  angular_sample_resolution_ = nh.param("angular_sample_resolution", angular_sample_resolution_);
  if (angular_sample_resolution_ <= 0) {
    ROS_ERROR("angular_sample_resolution must be greater than 0.");
    return false;
  }
  virtual_inertia_factor_ = nh.param("virtual_inertia_factor", virtual_inertia_factor_);
  if (virtual_inertia_factor_ < 0) {
    ROS_ERROR("virtual_inertia_factor must be greater or equal 0.");
    return false;
  }
  critical_stability_threshold_ = nh.param("critical_stability_threshold", critical_stability_threshold_);
  warn_stability_threshold_ = nh.param("warn_stability_threshold", warn_stability_threshold_);

  return loadJoints(nh);
}

bool SpeedController::loadJoints(const ros::NodeHandle& nh) {
  XmlRpc::XmlRpcValue joints;
  if (!nh.getParam("joints", joints)) {
    return true;
  }
  if (joints.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR_STREAM(nh.getNamespace() << "/joints is not a struct.");
    return false;
  }
  const std::vector<std::string>& joint_names = robot_model_->getJointModelNames();
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = joints.begin(); it != joints.end(); ++it) {
    const std::string& joint_name = it->first;
    if (std::find(joint_names.begin(), joint_names.end(), joint_name) == joint_names.end()) {
      ROS_ERROR_STREAM("Joint " << joint_name << " is not part of the robot model.");
      return false;
    }
    const XmlRpc::XmlRpcValue& joint_dict = it->second;
    if (joint_dict.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_STREAM(nh.getNamespace() << "/" << joint_name << " is not a struct.");
      continue;
    }
    if (joint_dict["in"].getType() != XmlRpc::XmlRpcValue::TypeString) {
      ROS_ERROR_STREAM(nh.getNamespace() << "/" << joint_name << "/in is not a string.");
      continue;
    }
    const std::string in_topic = static_cast<std::string>(joint_dict["in"]);
    if (joint_dict["out"].getType() != XmlRpc::XmlRpcValue::TypeString) {
      ROS_ERROR_STREAM(nh.getNamespace() << "/" << joint_name << "/out is not a string.");
      continue;
    }
    const std::string out_topic = static_cast<std::string>(joint_dict["out"]);

    flipper_cmd_subs_[joint_name] = nh_.subscribe<std_msgs::Float64>(in_topic, 10, std::bind(&SpeedController::flipperCmdCallback, this, joint_name, std::placeholders::_1));
    flipper_cmd_pubs_[joint_name] = nh_.advertise<std_msgs::Float64>(out_topic, 10, false);
    latest_flipper_speed_[joint_name] = 0.0;
  }

  return true;
}

bool SpeedController::initRobotModel() {
  if (!robot_model_) {
    urdf_ = std::make_shared<urdf::Model>();
    if (!urdf_->initParam("/robot_description")) {
      return false;
    }
    base_frame_ = urdf_->getRoot()->name;
    auto srdf = std::make_shared<srdf::Model>();

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

bool SpeedController::initPosePredictor() {
  std::string pose_predictor_name = pnh_.param<std::string>("pose_predictor", "sdf_contact_estimation::SDFContactEstimation");
  if (pose_predictor_name == "sdf_contact_estimation::SDFContactEstimation") {
    // Create SDF Model
    ros::NodeHandle esdf_server_pnh(pnh_, "esdf_server");
    esdf_server_ = std::make_shared<voxblox::EsdfServer>(nh_, esdf_server_pnh);
    std::shared_ptr<sdf_contact_estimation::SdfModel> sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(pnh_);
    sdf_model->loadEsdf(esdf_server_->getEsdfMapPtr(), esdf_server_->getEsdfMaxDistance(), false);
    world_frame_ = esdf_server_->getWorldFrame();

    // Create robot model
    ros::NodeHandle pose_predictor_nh(pnh_, "sdf_pose_predictor");
    ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
    auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);

    // Create pose predictor
    auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pose_predictor_nh, shape_model, sdf_model);
    sdf_pose_predictor->enableVisualisation(false);
    pose_predictor_ = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);
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
  return true;
}


void SpeedController::timerCallback(const ros::TimerEvent& /*event*/) {
  if (!command_received_) {
    return;
  }
  geometry_msgs::Twist twist = latest_twist_;
  std::unordered_map<std::string, double> joint_speeds = latest_flipper_speed_;

  command_received_ = false;
  if (enabled_) {
    auto start = std::chrono::system_clock::now();
    //  ROS_INFO_STREAM("Input speed [" << twist_msg.linear.x << ", " << twist_msg.angular.z << "]");
    computeSpeedCommand(twist.linear.x, twist.angular.z, joint_speeds);
    //  ROS_INFO_STREAM("Output speed [" << twist_msg.linear.x << ", " << twist_msg.angular.z << "]");
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    if (elapsed.count() > 50) {
      ROS_WARN_STREAM_THROTTLE(5, "Command delay has reached " << elapsed.count() <<
                                      " ms. Consider reducing the prediction horizon or resolution.");
    }
  }
  if (twist.linear.x == 0 && twist.angular.z == 0) {
    if (!last_twist_zero_) {
      cmd_vel_pub_.publish(twist);
    }
    last_twist_zero_ = true;
  } else {
    cmd_vel_pub_.publish(twist);
    last_twist_zero_ = false;
  }

  std_msgs::Float64 speed_msg;
  for (const auto& joint_speed: joint_speeds) {
    speed_msg.data = joint_speed.second;
    flipper_cmd_pubs_[joint_speed.first].publish(speed_msg);
  }
}

void SpeedController::cmdVelCallback(const geometry_msgs::TwistConstPtr& twist_msg) {
  latest_twist_ = *twist_msg;
  command_received_ = true;
}

void SpeedController::flipperCmdCallback(const std::string& joint, const std_msgs::Float64ConstPtr& float_msg) {
  latest_flipper_speed_[joint] = float_msg->data;
  command_received_ = true;
}

void SpeedController::computeSpeedCommand(double& linear, double& angular, std::unordered_map<std::string, double>& joint_speeds) {
  std::vector<RobotTerrainState> robot_states = predictTerrainInteraction(linear, angular, joint_speeds);
  publishTerrainInteraction(robot_states);
  double speed_scaling = computeSpeedScaling(linear, angular, robot_states);
  linear *= speed_scaling;
  angular *= speed_scaling;
  for (auto& joint_speed: joint_speeds) {
    joint_speed.second *= speed_scaling;
  }
}

std::vector<RobotTerrainState> SpeedController::predictTerrainInteraction(double linear, double angular, const std::unordered_map<std::string, double>& joint_speeds) {
  // Get current robot pose and joint state
  Eigen::Isometry3d current_robot_pose;
  if (!state_provider_->getRobotPose(current_robot_pose)) {
    return {};
  }
  // Update robot state
  if (!state_provider_->jointStateComplete()) {
    ROS_WARN_STREAM_THROTTLE(1, "Can't update stability estimation: The following joint states are still missing: " << visualization::setToString(state_provider_->getMissingJointStates()));
    return {};
  }
  const std::unordered_map<std::string, double>& joint_state = state_provider_->getJointState();

  std::vector<RobotTerrainState> robot_states;
  // Estimate at current robot pose
  RobotTerrainState current_terrain_state;
  current_terrain_state.time_delta = 0.0;
  if (critical_stability_threshold_ > 0) {
    if (!estimateRobotPose(current_robot_pose, joint_state, current_terrain_state, false)) {
      current_terrain_state.minimum_stability = -1;
      ROS_DEBUG_STREAM("Failed to predict support polygon at current pose");
      return {std::move(current_terrain_state)};
    }
  } else {
    current_terrain_state.robot_pose = current_robot_pose;
    current_terrain_state.minimum_stability = 10000.0;
    current_terrain_state.joint_positions = joint_state;
    pose_predictor_->robotModel()->updateJointPositions(joint_state);
    current_terrain_state.center_of_mass = pose_predictor_->robotModel()->centerOfMass();
  }
  robot_states.push_back(std::move(current_terrain_state));


  // Predict along robot trajectory (assumes constant velocity)
  double time_step;
  double linear_abs = std::abs(linear);
  double angular_abs = std::abs(angular);
  double epsilon = 1e-5;
  if (linear_abs < epsilon && angular_abs < epsilon) {
    time_step = maximum_time_step_;
  } else {
    double time_step_linear = linear_abs > 0.0 ? sample_resolution_ / linear_abs : std::numeric_limits<double>::max();
    double time_step_angular = angular_abs > 0.0 ? angular_sample_resolution_ / angular_abs : std::numeric_limits<double>::max();
    time_step = std::min(time_step_linear, time_step_angular);
    time_step = std::min(time_step, maximum_time_step_);
  }
  unsigned int steps = std::floor(prediction_horizon_ / time_step);
  robot_states.reserve(steps);

  Eigen::Isometry3d robot_delta_motion = util::computeDiffDriveTransform(linear, angular, time_step);
  for (unsigned int i = 1; i <= steps; ++i) {
    // last result + fk
    Eigen::Isometry3d predicted_pose = robot_states.back().robot_pose * robot_delta_motion;
    auto extrapolated_joint_positions = state_provider_->extrapolateJointPositions(joint_state, joint_speeds, time_step * i);

    // pose prediction
    RobotTerrainState robot_terrain_state;
    robot_terrain_state.time_delta = time_step * i;
    if (!estimateRobotPose(predicted_pose, extrapolated_joint_positions, robot_terrain_state, true)) {
      robot_terrain_state.minimum_stability = -1;
      ROS_DEBUG_STREAM("Failed to predict support polygon at t = " << robot_terrain_state.time_delta);
      robot_states.push_back(std::move(robot_terrain_state));
      break;
    }
    Eigen::Vector3d force = robot_states.back().robot_pose.linear() * Eigen::Vector3d::UnitX() * linear_abs * virtual_inertia_factor_;
    computeStabilityMargin(robot_terrain_state, force);
    robot_states.push_back(std::move(robot_terrain_state));
  }
//  for (unsigned int i = 0; i < robot_states.size(); ++i) {
//    ROS_INFO_STREAM("State " << i << ": Stability: " << robot_states[i].minimum_stability);
//  }
  return robot_states;
}

double SpeedController::computeSpeedScaling(double linear, double angular,
                                            const std::vector<RobotTerrainState>& robot_states) {
  if (robot_states.empty()) {
//    ROS_INFO_STREAM("No robot state");
    return 0.0;
  }

  bool critical = false;
  double critical_time_delta = 0.0;
  for (const auto& state: robot_states) {
    if (state.minimum_stability < critical_stability_threshold_) {
      ROS_DEBUG_STREAM("Found critical state at t = " << state.time_delta << " (" << state.minimum_stability << " < " << critical_stability_threshold_ << ")");
      critical = true;
      critical_time_delta = state.time_delta;
      break;
    }
  }

  double speed_scaling = 1.0;
  if (critical) {
    // Linearly reduce speed up to a safety distance in front of the critical state
    speed_scaling = computeLinearSpeedReduction(linear, critical_time_delta);
  }

//  ROS_INFO_STREAM("speed scaling: " << speed_scaling << ", critical time delta: " << critical_time_delta);

  std_msgs::Float64 float_msg;
  float_msg.data = speed_scaling;
  speed_scaling_pub_.publish(float_msg);
  return speed_scaling;
}

double SpeedController::computeLinearSpeedReduction(double linear, double time_delta_critical) const {
  // Stop immediately when there is no prediction horizon
  // This implies, that the current state is critical
  if (prediction_horizon_ <= 0) {
    return 0.0;
  }

  double time_until_stop = 0; // Time until the robot has to stop to be safe
  if (safety_distance_ <= 0) {
    time_until_stop = time_delta_critical;
  } else {
    double linear_abs = std::abs(linear);
    if (linear_abs < 1e-5) {
      // If there is a safety distance but our linear speed is slow, we have to stop immediately
      // because the critical state is inside the safety distance
      return 0.0;
    }
    double time_safety = safety_distance_ / linear_abs;
    time_until_stop = std::max(time_delta_critical - time_safety, 0.0);
  }

  double speed_scaling =  time_until_stop / prediction_horizon_;
  speed_scaling = std::min(speed_scaling, 1.0);
  return speed_scaling;
}

bool SpeedController::estimateRobotPose(const Eigen::Isometry3d& robot_pose,
                                        const std::unordered_map<std::string, double>& joint_positions,
                                        RobotTerrainState& robot_terrain_state, bool predict_pose) {
  pose_predictor_->robotModel()->updateJointPositions(joint_positions);
  robot_terrain_state.joint_positions = joint_positions;
  robot_terrain_state.center_of_mass = pose_predictor_->robotModel()->centerOfMass();
  hector_math::Pose<double> robot_pose_type(robot_pose);
  bool success;
  if (!predict_pose) {
    success = pose_predictor_->estimateContactInformation(robot_pose_type, robot_terrain_state.support_polygon, robot_terrain_state.contact_information);
  } else {
    robot_terrain_state.minimum_stability = pose_predictor_->predictPoseAndContactInformation(robot_pose_type, robot_terrain_state.support_polygon, robot_terrain_state.contact_information);
    success = !std::isnan(robot_terrain_state.minimum_stability);
  }
  robot_terrain_state.robot_pose = robot_pose_type.asTransform();
  if (!success || robot_terrain_state.support_polygon.contact_hull_points.empty()) {
    return false;
  }
  if (!predict_pose) {
    computeStabilityMargin(robot_terrain_state);
  }
  return true;
}

void SpeedController::computeStabilityMargin(RobotTerrainState& robot_terrain_state, const Eigen::Vector3d& external_force) {
  if (robot_terrain_state.support_polygon.contact_hull_points.empty()) {
    return;
  }
  Eigen::Vector3d com = robot_terrain_state.robot_pose * robot_terrain_state.center_of_mass;
  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  Eigen::Vector3d net_force = external_force + gravity;
  hector_stability_metrics::non_differentiable::computeForceAngleStabilityMeasure<double>(robot_terrain_state.support_polygon.contact_hull_points, robot_terrain_state.support_polygon.edge_stabilities, com, net_force);
  auto min_it = std::min_element(robot_terrain_state.support_polygon.edge_stabilities.begin(), robot_terrain_state.support_polygon.edge_stabilities.end());
  robot_terrain_state.minimum_stability = *min_it;
}

void SpeedController::publishTerrainInteraction(const std::vector<RobotTerrainState>& robot_states) {
  publishMultiRobotState(robot_states);
  publishMultiRobotMarker(robot_states);
  publishSupportPolygon(robot_states);
  publishPredictedPath(robot_states);
}

void SpeedController::publishMultiRobotState(const std::vector<RobotTerrainState>& robot_states) const {
  if (robot_display_pub_.getNumSubscribers() == 0) {
    return;
  }
  hector_rviz_plugins_msgs::DisplayMultiRobotState display_msg;
  display_msg.header.frame_id = world_frame_;
  display_msg.robots.reserve(robot_states.size());

  for (unsigned int i = 0; i < robot_states.size(); ++i) {
    const auto& state = robot_states[i];
    hector_rviz_plugins_msgs::MultiRobotStateEntry entry;
    entry.id = "robot_state_" + std::to_string(i);

    tf::poseEigenToMsg(state.robot_pose, entry.pose.pose);
    robot_state_->setVariablePositions(std::map<std::string, double>(state.joint_positions.begin(), state.joint_positions.end()));
    moveit::core::robotStateToRobotStateMsg(*robot_state_, entry.robot_state.state, false);

    for (const auto& link: robot_model_->getLinkModelNames()) {
      moveit_msgs::ObjectColor object_color;
      object_color.color = stabilityToColorMsg(state.minimum_stability);
      object_color.id = link;
      entry.robot_state.highlight_links.push_back(object_color);
    }

    display_msg.robots.push_back(entry);
  }
  robot_display_pub_.publish(display_msg);
}

void SpeedController::publishMultiRobotMarker(const std::vector<RobotTerrainState>& robot_states) const {
  if (robot_marker_pub_.getNumSubscribers() == 0) {
    return;
  }
  visualization_msgs::MarkerArray marker_array;
  visualization::deleteAllMarkers(marker_array);
  for (unsigned int i = 0; i < robot_states.size(); ++i) {
    const auto& state = robot_states[i];
    robot_state_->setVariablePositions(std::map<std::string, double>(state.joint_positions.begin(), state.joint_positions.end()));

    std_msgs::ColorRGBA color = stabilityToColorMsg(state.minimum_stability);
    std::string ns = "robot_state_" + std::to_string(i);
    visualization_msgs::MarkerArray marker_array_temp;
    robot_state_->getRobotMarkers(marker_array_temp, robot_model_->getLinkModelNames(), color, ns, ros::Duration(0));
    // Transform from base to world frame
    for (auto& marker: marker_array_temp.markers) {
      Eigen::Isometry3d marker_pose_base;
      tf::poseMsgToEigen(marker.pose, marker_pose_base);
      Eigen::Isometry3d marker_pose_world = state.robot_pose * marker_pose_base;
      tf::poseEigenToMsg(marker_pose_world, marker.pose);
      marker.header.frame_id = world_frame_;
    }
    marker_array.markers.insert(marker_array.markers.end(), marker_array_temp.markers.begin(), marker_array_temp.markers.end());
  }
  visualization::fixIds(marker_array);
  robot_marker_pub_.publish(marker_array);
}

void SpeedController::publishSupportPolygon(const std::vector<RobotTerrainState>& robot_states) const {
  if (support_polygon_pub_.getNumSubscribers() == 0) {
    return;
  }
  visualization_msgs::MarkerArray support_polygon_marker_array;
  visualization::deleteAllMarkers(support_polygon_marker_array);
//  for (const auto& state: robot_states) {
//    hector_pose_prediction_interface::visualization::addSupportPolygonWithContactInformationToMarkerArray(
//        support_polygon_marker_array, state.support_polygon, state.contact_information, world_frame_);
//  }
  hector_pose_prediction_interface::visualization::addSupportPolygonWithContactInformationToMarkerArray(
      support_polygon_marker_array, robot_states.back().support_polygon, robot_states.back().contact_information, world_frame_);
  visualization::fixIds(support_polygon_marker_array);
  support_polygon_pub_.publish(support_polygon_marker_array);
}

void SpeedController::publishPredictedPath(const std::vector<RobotTerrainState>& robot_states) const {
  if (predicted_path_pub_.getNumSubscribers() == 0) {
    return;
  }
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = world_frame_;
  path_marker.ns = "path";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.scale.x = 0.02;
  path_marker.color.r = 0.0;
  path_marker.color.g = 0.0;
  path_marker.color.b = 1.0;
  path_marker.color.a = 1.0;
  path_marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker sphere_list_marker;
  sphere_list_marker.header.frame_id = world_frame_;
  sphere_list_marker.ns = "poses";
  sphere_list_marker.id = 1;
  sphere_list_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  sphere_list_marker.action = visualization_msgs::Marker::ADD;
  sphere_list_marker.scale.x = 0.05;
  sphere_list_marker.scale.y = 0.05;
  sphere_list_marker.scale.z = 0.05;
  sphere_list_marker.pose.orientation.w = 1.0;


  for (const auto & robot_state : robot_states) {
    geometry_msgs::Point point;
    tf::pointEigenToMsg(robot_state.robot_pose.translation(), point);
    sphere_list_marker.points.push_back(point);
    path_marker.points.push_back(point);
    std_msgs::ColorRGBA color = stabilityToColorMsg(robot_state.minimum_stability);
    sphere_list_marker.colors.push_back(color);
  }

  if (path_marker.points.size() < 2) {
    path_marker.action = visualization_msgs::Marker::DELETE;
    path_marker.points.clear();
  }

  visualization_msgs::MarkerArray array;
  array.markers.push_back(path_marker);
  array.markers.push_back(sphere_list_marker);
  predicted_path_pub_.publish(array);
}
std_msgs::ColorRGBA SpeedController::stabilityToColorMsg(double stability) const {
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  if (stability <= critical_stability_threshold_) {
    // critical
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0;
  } else if (stability <= warn_stability_threshold_) {
    // warn
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0;
  } else {
    // good
    color.r = 0;
    color.g = 1.0;
    color.b = 0;
  }
  return color;
}

void SpeedController::enableCallback(const std_msgs::BoolConstPtr& bool_msg) {
  enabled_ = bool_msg->data;
  ROS_INFO_STREAM((enabled_ ? "Enabling " : "Disabling ") << "stability speed controller.");
  publishEnabledStatus();
}

void SpeedController::publishEnabledStatus() {
  std_msgs::Bool bool_msg;
  bool_msg.data = enabled_;
  enabled_status_pub_.publish(bool_msg);
}

}  // namespace hector_stability_assistance