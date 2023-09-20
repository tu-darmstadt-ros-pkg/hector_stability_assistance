#include <hector_stability_assistance/speed_controller.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/util/utils.h>
#include <hector_rviz_plugins_msgs/DisplayMultiRobotState.h>
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include <moveit/robot_state/conversions.h>
#include <hector_pose_prediction_ros/visualization.h>
#include <std_msgs/Float64.h>
#include "hector_stability_assistance/visualization.h"

namespace hector_stability_assistance {

SpeedController::SpeedController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
: nh_(nh),
  pnh_(pnh),
  prediction_horizon_(2.0),
  sample_resolution_(0.05),
  critical_stability_threshold_(0.5),
  warn_stability_threshold_(1.0)
{
}

bool SpeedController::init() {
  if (!loadParameters(pnh_)) {
    return false;
  }

  // Load moveit and urdf model
  if (!initRobotModel()) {
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
  speed_scaling_pub_ = pnh_.advertise<std_msgs::Float64>("speed_scaling", 10);
  cmd_vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);

  // Subscribers
  cmd_vel_sub_ = pnh_.subscribe<geometry_msgs::Twist>("cmd_vel_in", 10, &SpeedController::cmdVelCallback, this);

  return true;
}

bool SpeedController::loadParameters(const ros::NodeHandle& nh) {
  prediction_horizon_ = nh.param("prediction_horizon", prediction_horizon_);
  safety_distance_ = nh.param("safety_distance", prediction_horizon_);
  sample_resolution_ = nh.param("sample_resolution", sample_resolution_);
  critical_stability_threshold_ = nh.param("critical_stability_threshold", critical_stability_threshold_);
  warn_stability_threshold_ = nh.param("warn_stability_threshold", warn_stability_threshold_);
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

void SpeedController::cmdVelCallback(geometry_msgs::Twist twist_msg) {
  ROS_INFO_STREAM("Input speed [" << twist_msg.linear.x << ", " << twist_msg.angular.z << "]");
  auto start = std::chrono::system_clock::now();
  computeSpeedCommand(twist_msg.linear.x, twist_msg.angular.z);
  ROS_INFO_STREAM("Output speed [" << twist_msg.linear.x << ", " << twist_msg.angular.z << "]");
  auto end = std::chrono::system_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  ROS_INFO_STREAM("Cmd vel delay: " << elapsed.count() << " ms.");
  cmd_vel_pub_.publish(twist_msg);
}

void SpeedController::computeSpeedCommand(double& linear, double& angular) {
  std::vector<RobotTerrainState> robot_states = predictTerrainInteraction(linear, angular);
  publishTerrainInteraction(robot_states);
  double speed_scaling = computeSpeedScaling(linear, angular, robot_states);
  linear *= speed_scaling;
  angular *= speed_scaling;
}

std::vector<RobotTerrainState> SpeedController::predictTerrainInteraction(double linear, double angular) {
  // Get current robot pose and joint state
  Eigen::Isometry3d current_robot_pose;
  if (!state_provider_->getRobotPose(current_robot_pose)) {
    return {};
  }
  if (!state_provider_->jointStateComplete()) {
    return {};
  }
  const std::unordered_map<std::string, double>& joint_state = state_provider_->getJointState();

  std::vector<RobotTerrainState> robot_states;
  // Estimate at current robot pose
  RobotTerrainState current_terrain_state;
  current_terrain_state.time_delta = 0.0;
  if (!estimateRobotPose(current_robot_pose, joint_state, current_terrain_state, false)) {
    current_terrain_state.minimum_stability = -1;
    return {std::move(current_terrain_state)};
  }
  robot_states.push_back(std::move(current_terrain_state));

  // Predict along robot trajectory
  double abs_linear = std::abs(linear);
  double prediction_distance = prediction_horizon_ * abs_linear;
  unsigned int steps = std::floor(prediction_distance / sample_resolution_);
  robot_states.reserve(steps);
  double time_step = sample_resolution_ / abs_linear;
  Eigen::Isometry3d robot_delta_motion = computeDiffDriveTransform(linear, angular, time_step);
  for (unsigned int i = 1; i <= steps; ++i) {
    // last result + fk
    Eigen::Isometry3d predicted_pose = robot_states.back().robot_pose * robot_delta_motion;

    // pose prediction
    RobotTerrainState robot_terrain_state;
    robot_terrain_state.time_delta = time_step * i;
    if (!estimateRobotPose(predicted_pose, joint_state, robot_terrain_state, true)) {
      robot_terrain_state.minimum_stability = -1;
      ROS_WARN_STREAM("Failed to predict support polygon at t = " << robot_terrain_state.time_delta);
      robot_states.push_back(std::move(robot_terrain_state));
      break;
    }
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
    ROS_INFO_STREAM("No robot state");
    return 0.0;
  }

//  bool warn = false;
  bool critical = false;
  double critical_time_delta = 0.0;
  double stability_margin;
  for (const auto& state: robot_states) {
//    if (state.minimum_stability < warn_stability_threshold_) {
//      warn = true;
//    }
    if (state.minimum_stability < critical_stability_threshold_) {
      ROS_INFO_STREAM("Found critical state at t = " << state.time_delta);
      critical = true;
      critical_time_delta = state.time_delta;
      stability_margin = state.minimum_stability;
      break;
    }
  }

  double speed_scaling = 1.0;
  if (critical) {
    if (prediction_horizon_ > 0) {
      // Linearly reduce speed up to a safety distance in front of the critical state
      double time_safety = safety_distance_ / std::abs(linear);
      double time_until_stop = std::max(critical_time_delta - time_safety, 0.0);
      speed_scaling =  time_until_stop / prediction_horizon_;
      speed_scaling = std::min(speed_scaling, 1.0);
    } else {
      speed_scaling = 0.0;
    }
  }

  ROS_INFO_STREAM("speed scaling: " << speed_scaling << ", critical time delta: " << critical_time_delta);

  std_msgs::Float64 float_msg;
  float_msg.data = speed_scaling;
  speed_scaling_pub_.publish(float_msg);
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
  if (!success || robot_terrain_state.support_polygon.contact_hull_points.empty()) {
//    ROS_WARN_STREAM("Failed to estimate support polygon.");
    return false;
  }
  robot_terrain_state.robot_pose = robot_pose_type.asTransform();
  if (!predict_pose) {
    computeStabilityMargin(robot_terrain_state);
  }
  return true;
}
void SpeedController::computeStabilityMargin(RobotTerrainState& robot_terrain_state) {
  if (robot_terrain_state.support_polygon.contact_hull_points.empty()) {
    return;
  }
  Eigen::Vector3d com = robot_terrain_state.robot_pose * robot_terrain_state.center_of_mass;
  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  hector_stability_metrics::non_differentiable::computeForceAngleStabilityMeasure<double>(robot_terrain_state.support_polygon.contact_hull_points, robot_terrain_state.support_polygon.edge_stabilities, com, gravity);
  auto min_it = std::min_element(robot_terrain_state.support_polygon.edge_stabilities.begin(), robot_terrain_state.support_polygon.edge_stabilities.end());
  robot_terrain_state.minimum_stability = *min_it;
}

void SpeedController::publishTerrainInteraction(const std::vector<RobotTerrainState>& robot_states) {
  publishMultiRobotState(robot_states);
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

void SpeedController::publishSupportPolygon(const std::vector<RobotTerrainState>& robot_states) const {
  if (support_polygon_pub_.getNumSubscribers() == 0) {
    return;
  }
  visualization::deleteAllMarkers(support_polygon_pub_);
  visualization_msgs::MarkerArray support_polygon_marker_array;
  for (const auto& state: robot_states) {
    hector_pose_prediction_interface::visualization::addSupportPolygonWithContactInformationToMarkerArray(
        support_polygon_marker_array, state.support_polygon, state.contact_information, world_frame_);
  }
  visualization::fixIds(support_polygon_marker_array);
  support_polygon_pub_.publish(support_polygon_marker_array);
}

Eigen::Isometry3d SpeedController::computeDiffDriveTransform(double linear_speed, double angular_speed, double time_delta) const
{
  double x;
  double y;
  double theta = angular_speed * time_delta;
  if (angular_speed < 1e-6) {
    x = linear_speed * time_delta;
    y = 0;
  } else {
    x = linear_speed / angular_speed * std::sin(theta);
    y = linear_speed / angular_speed * (1 - std::cos(theta));
  }

  Eigen::Isometry3d transform(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  transform.translation().x() = x;
  transform.translation().y() = y;
  transform.translation().z() = 0;

  return transform;
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

}  // namespace hector_stability_assistance