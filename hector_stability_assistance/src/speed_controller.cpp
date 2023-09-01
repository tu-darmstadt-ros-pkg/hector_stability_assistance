#include <hector_stability_assistance/speed_controller.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <hector_rviz_plugins_msgs/DisplayMultiRobotState.h>

namespace hector_stability_assistance {

SpeedController::SpeedController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
: nh_(nh),
  pnh_(pnh),
  tf_listener_(tf_buffer_),
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

  for (const auto &kv : urdf_->joints_)
  {
    if (kv.second->type != urdf::Joint::FIXED && kv.second->type != urdf::Joint::UNKNOWN && !kv.second->mimic) {
      missing_joint_states_.insert( kv.first );
      ROS_DEBUG_STREAM("Adding required joint: " << kv.first << " type: " << kv.second->type);
    }
  }

  if (!initPosePredictor()) {
    return false;
  }

  // Publishers
  support_polygon_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("support_polygon", 10);
  robot_display_pub_ = pnh_.advertise<hector_rviz_plugins_msgs::DisplayMultiRobotState>("predicted_robot_states", 10);
  cmd_vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);



  // Subscribers
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &SpeedController::jointStateCallback, this);
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
  computeSpeedCommand(twist_msg.linear.x, twist_msg.angular.z);
  cmd_vel_pub_.publish(twist_msg);
}

void SpeedController::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg) {
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

void SpeedController::computeSpeedCommand(double& linear, double& angular) {
  std::vector<RobotTerrainState> robot_states = predictTerrainInteraction(linear, angular);
  publishTerrainInteraction(robot_states);
  double speed_scaling = computeSpeedScaling(robot_states);
  linear *= speed_scaling;
  angular += speed_scaling;
}

bool SpeedController::getRobotPose(Eigen::Isometry3d& robot_pose) const {
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

std::vector<RobotTerrainState> SpeedController::predictTerrainInteraction(double linear, double angular) {
  // Predict robot-terrain interaction at sampled poses along the path
  std::vector<RobotTerrainState> robot_states;

  // get current pose and estimate support polygon

  double time_step = sample_resolution_ / linear;
  unsigned int steps = std::floor(prediction_horizon_ / time_step);
  robot_states.reserve(steps);
  // compute diff drive fk
  for (unsigned int i = 0; i < steps; ++i) {
    // compute input pose from last result
    // pose prediction
    // save to array
  }
  return robot_states;
}

void SpeedController::publishTerrainInteraction(const std::vector<RobotTerrainState>& robot_states) {
}

double SpeedController::computeSpeedScaling(const std::vector<RobotTerrainState>& robot_states) {
  // Determine speed based on predicted terrain interaction
  // Start simple:
  // if stability below critical value in critical area, send 0
  // if stability below critical value beyond critical area, scale velocity based on distance to critical area
  return 0;
}

}  // namespace hector_stability_assistance