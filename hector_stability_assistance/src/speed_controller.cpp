#include <hector_stability_assistance/speed_controller.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <hector_rviz_plugins_msgs/DisplayMultiRobotState.h>
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include <moveit/robot_state/conversions.h>

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
  robot_display_pub_ = pnh_.advertise<hector_rviz_plugins_msgs::DisplayMultiRobotState>("predicted_robot_states", 10);
  cmd_vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);

  // Subscribers
  cmd_vel_sub_ = pnh_.subscribe<geometry_msgs::Twist>("cmd_vel_in", 10, &SpeedController::cmdVelCallback, this);

  return true;
}

bool SpeedController::loadParameters(const ros::NodeHandle& nh) {
  prediction_horizon_ = nh.param("prediction_horizon", prediction_horizon_);
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
  computeSpeedCommand(twist_msg.linear.x, twist_msg.angular.z);
  ROS_INFO_STREAM("Output speed [" << twist_msg.linear.x << ", " << twist_msg.angular.z << "]");
  cmd_vel_pub_.publish(twist_msg);
}

void SpeedController::computeSpeedCommand(double& linear, double& angular) {
  std::vector<RobotTerrainState> robot_states = predictTerrainInteraction(linear, angular);
  publishTerrainInteraction(robot_states);
  double speed_scaling = computeSpeedScaling(robot_states);
  linear *= speed_scaling;
  angular *= speed_scaling;
}

std::vector<RobotTerrainState> SpeedController::predictTerrainInteraction(double linear, double angular) {
  // Predict robot-terrain interaction at sampled poses along the path
  std::vector<RobotTerrainState> robot_states;

  RobotTerrainState prediction;
  Eigen::Isometry3d robot_pose;
  if (!state_provider_->getRobotPose(robot_pose)) {
    return {};
  }
  if (!state_provider_->jointStateComplete()) {
    return {};
  }
  if (estimateRobotPose(robot_pose, state_provider_->getJointState(), prediction, false)) {
    robot_states.push_back(std::move(prediction));
  }

//  // get current pose and estimate support polygon
//  // TODO replace with multiplication
//  double time_step = sample_resolution_ / linear;
//  unsigned int steps = std::floor(prediction_horizon_ / time_step);
//  robot_states.reserve(steps);
//  // compute diff drive fk
//  for (unsigned int i = 0; i < steps; ++i) {
//    // compute input pose from last result
//    // pose prediction
//    // save to array
//  }
  return robot_states;
}

double SpeedController::computeSpeedScaling(const std::vector<RobotTerrainState>& robot_states) {
  if (robot_states.empty()) {
    ROS_INFO_STREAM("No robot state");
    return 0.0;
  }

  double speed_scaling;
  if (robot_states.front().minimum_stability < critical_stability_threshold_) {
    speed_scaling = 0.0;
  } else {
    speed_scaling = 1.0;
  }
  ROS_INFO_STREAM("Stability: " << robot_states.front().minimum_stability << ", threshold: " << critical_stability_threshold_ << ", speed scaling: " << speed_scaling);
  // Determine speed based on predicted terrain interaction
  // Start simple:
  // if stability below critical value in critical area, send 0
  // if stability below critical value beyond critical area, scale velocity based on distance to critical area
  return speed_scaling;
}

bool SpeedController::estimateRobotPose(const Eigen::Isometry3d& robot_pose,
                                        const std::unordered_map<std::string, double>& joint_positions,
                                        RobotTerrainState& robot_terrain_state, bool predict_pose) {
  pose_predictor_->robotModel()->updateJointPositions(joint_positions);
  robot_terrain_state.joint_positions = joint_positions;
  hector_math::Pose<double> robot_pose_type(robot_pose);
  bool success;
  if (!predict_pose) {
    success = pose_predictor_->estimateContactInformation(robot_pose_type, robot_terrain_state.support_polygon, robot_terrain_state.contact_information);
  } else {
    robot_terrain_state.minimum_stability = pose_predictor_->predictPoseAndContactInformation(robot_pose_type, robot_terrain_state.support_polygon, robot_terrain_state.contact_information);
    success = !std::isnan(robot_terrain_state.minimum_stability);
  }
  if (!success || robot_terrain_state.support_polygon.contact_hull_points.empty()) {
    ROS_WARN_STREAM("Failed to estimate support polygon.");
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
}

void SpeedController::publishMultiRobotState(const std::vector<RobotTerrainState>& robot_states) const {
  std_msgs::ColorRGBA color_good;
  color_good.r = 0;
  color_good.g = 1.0;
  color_good.b = 0;
  color_good.a = 1.0;
  std_msgs::ColorRGBA color_warn;
  color_warn.r = 1.0;
  color_warn.g = 1.0;
  color_warn.b = 0;
  color_warn.a = 1.0;
  std_msgs::ColorRGBA color_critical;
  color_critical.r = 1.0;
  color_critical.g = 0.0;
  color_critical.b = 0;
  color_critical.a = 1.0;

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

    std_msgs::ColorRGBA color;
    if (state.minimum_stability <= critical_stability_threshold_) {
      color = color_critical;
    } else if (state.minimum_stability <= warn_stability_threshold_) {
      color = color_warn;
    } else {
      color = color_good;
    }
    for (const auto& link: robot_model_->getLinkModelNames()) {
      moveit_msgs::ObjectColor object_color;
      object_color.color = color;
      object_color.id = link;
      entry.robot_state.highlight_links.push_back(object_color);
    }

    display_msg.robots.push_back(entry);
  }

  robot_display_pub_.publish(display_msg);
}

}  // namespace hector_stability_assistance