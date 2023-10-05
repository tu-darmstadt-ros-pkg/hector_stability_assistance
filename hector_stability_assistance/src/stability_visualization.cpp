#include <hector_stability_assistance/stability_visualization.h>

#include <chrono>
//#include <hector_heightmap_pose_prediction/heightmap_pose_predictor.h>
//#include <hector_heightmap_pose_prediction/robot_heightmap_providers/urdf_robot_heightmap_provider.h>
#include <hector_pose_prediction_ros/visualization.h>
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
//#include <hector_world_heightmap_ros/message_conversions/map.h>
#include <eigen_conversions/eigen_msg.h>
//#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float64MultiArray.h>

//#include <hector_stability_assistance/local_grid_map.h>
#include <hector_stability_assistance/visualization.h>
#include <hector_rviz_plugins_msgs/DisplayMultiRobotState.h>
#include <moveit/robot_state/conversions.h>


namespace hector_stability_assistance {

StabilityVisualization::StabilityVisualization(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
: nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_), update_frequency_(10.0), predict_pose_(false) {

}

bool StabilityVisualization::init() {
  // Parameters
  update_frequency_ = pnh_.param("update_frequency", 10.0);
  elevation_layer_name_ = pnh_.param("elevation_layer_name", std::string("elevation"));
  predict_pose_ = pnh_.param("predict_pose", false);

  // Load urdf model
  urdf_ = std::make_shared<urdf::Model>();
  if (!urdf_->initParam("/robot_description")) {
    return false;
  }

  for (const auto &kv : urdf_->joints_)
  {
    if (kv.second->type != urdf::Joint::FIXED && kv.second->type != urdf::Joint::UNKNOWN && !kv.second->mimic) {
      missing_joint_states_.insert( kv.first );
      ROS_DEBUG_STREAM("Adding required joint: " << kv.first << " type: " << kv.second->type);
    }
  }

  // Initialize pose predictor
  std::string pose_predictor_name = pnh_.param<std::string>("pose_predictor", "sdf_contact_estimation::SDFContactEstimation");
  if (pose_predictor_name == "sdf_contact_estimation::SDFContactEstimation") {
    // Create SDF Model
    ros::NodeHandle esdf_server_pnh(pnh_, "esdf_server");
    esdf_server_ = std::make_shared<voxblox::EsdfServer>(nh_, esdf_server_pnh);
    sdf_model_ = std::make_shared<sdf_contact_estimation::SdfModel>(pnh_);
    sdf_model_->loadEsdf(esdf_server_->getEsdfMapPtr(), esdf_server_->getEsdfMaxDistance(), false);

    // Create robot model
    ros::NodeHandle pose_predictor_nh(pnh_, "sdf_pose_predictor");
    ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
    auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);

    // Create pose predictor
    auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pose_predictor_nh, shape_model, sdf_model_);
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

  // Subscriber and Publisher
  stability_margin_pub_ = pnh_.advertise<std_msgs::Float64>("stability_margin", 1);
  stability_margins_pub_ = pnh_.advertise<std_msgs::Float64MultiArray>("stability_margins", 1);
  traction_pub_ = pnh_.advertise<std_msgs::Float64>("traction", 1);
  support_polygon_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("support_polygon", 1);
  com_pub_ = pnh_.advertise<geometry_msgs::PointStamped>("center_of_mass", 1);

  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &StabilityVisualization::jointStateCallback, this);

  timer_ = nh_.createTimer(ros::Duration(1.0/update_frequency_), &StabilityVisualization::timerCallback, this, false);
  return true;
}

//void StabilityVisualization::gridMapCallback(const grid_map_msgs::GridMapConstPtr& grid_map) {
//  latest_grid_map_ = grid_map;
//}

void StabilityVisualization::timerCallback(const ros::TimerEvent&) {
  update();
}

void StabilityVisualization::update() {
  // Update robot state
  if (!missing_joint_states_.empty()) {
    ROS_WARN_STREAM_THROTTLE(1, "Can't update stability estimation: The following joint states are still missing: " << setToString(missing_joint_states_));
    return;
  }
  pose_predictor_->robotModel()->updateJointPositions(joint_states_);

  // Center of mass
  publishCOM();

  // Get robot position
  Eigen::Isometry3d robot_pose_eigen;
  if (!getRobotPose(robot_pose_eigen)) {
    return;
  }

  // Evaluate current pose
  hector_pose_prediction_interface::SupportPolygon<double> support_polygon;
  hector_pose_prediction_interface::ContactInformation<double> contact_information;
  if (!estimateRobotPose(robot_pose_eigen, support_polygon, contact_information, predict_pose_)) {
    return;
  }

  // Publish results
  // Stability
  computeStabilityMargin(robot_pose_eigen, support_polygon);
  visualization::publishEdgeStabilities(support_polygon, stability_margins_pub_);
  visualization::publishMinStability(support_polygon, stability_margin_pub_);

  // Support polygon
  visualization::publishSupportPolygon(support_polygon, contact_information, support_polygon_pub_);

  // Traction
  // TODO

  /*
   * DEBUG
   */
  // Robot Heightmap
//  hector_world_heightmap::HeightmapRef<float>::ConstPtr robot_height_map =
//      robot_height_map_provider->computeHeightmap( height_map->resolution(), robot_orientation.cast<float>() );
//  Eigen::Matrix3Xf pointcloud = hector_world_heightmap_ros::message_conversions::heightmapToPointcloud<float>(robot_height_map);
//  sensor_msgs::PointCloud pointcloud_msg;
//  pointcloud_msg.header.frame_id = latest_grid_map_->info.header.frame_id;
//  pointcloud_msg.header.stamp = ros::Time::now();
//  for (Eigen::Index i = 0; i < pointcloud.cols(); ++i) {
//    const Eigen::Vector3f hm_point = robot_pose.translation() + pointcloud.col(i);
//    geometry_msgs::Point32 point;
//    point.x = hm_point.x();
//    point.y = hm_point.y();
//    point.z = hm_point.z();
//    pointcloud_msg.points.push_back( point );
//  }
//  robot_heightmap_pub_.publish(pointcloud_msg);

  // Submap
//  hector_world_heightmap::HeightmapRef<float>::ConstPtr ground_submap =
//      height_map->getSubMap(robot_pose.translation() + robot_height_map->origin(),
//                            robot_height_map->map().rows(), robot_height_map->map().cols());
//  if (ground_submap) {
//    submap_pub_.publish( hector_world_heightmap_ros::message_conversions::heightmapToMsg<float>(ground_submap));
//  } else {
//    ROS_WARN_STREAM("Failed to retrieve sub map");
//  }
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

void StabilityVisualization::publishCOM() const
{
  geometry_msgs::PointStamped point_msg;
  if(urdf_->getRoot()) {
    point_msg.header.frame_id = urdf_->getRoot()->name;
  }
  point_msg.header.stamp = ros::Time::now();
  tf::pointEigenToMsg(pose_predictor_->robotModel()->centerOfMass(), point_msg.point);
  com_pub_.publish(point_msg);
}

bool StabilityVisualization::getRobotPose(Eigen::Isometry3d& robot_pose) const
{
  geometry_msgs::TransformStamped transform_msg;
  try{
    transform_msg = tf_buffer_.lookupTransform("world", "base_link",
                                               ros::Time(0), ros::Duration(1.0)); //TODO remove hardcoded frames
  }
  catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1, "%s",ex.what());
    return false;
  }
  tf::transformMsgToEigen(transform_msg.transform, robot_pose);
  return true;
}

bool StabilityVisualization::estimateRobotPose(Eigen::Isometry3d& robot_pose,
                                               hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                                               hector_pose_prediction_interface::ContactInformation<double>& contact_information,
                                               bool predict_pose)
{
  hector_math::Pose<double> robot_pose_type(robot_pose);
  bool success;
  if (!predict_pose) {
    success = pose_predictor_->estimateContactInformation(robot_pose_type, support_polygon, contact_information);
  } else {
    success = !std::isnan(pose_predictor_->predictPoseAndContactInformation(robot_pose_type, support_polygon, contact_information));
  }
  if (!success || support_polygon.contact_hull_points.empty()) {
    ROS_WARN_STREAM("Failed to estimate support polygon.");
    return false;
  }
  robot_pose = robot_pose_type.asTransform();
  return true;
}

void StabilityVisualization::computeStabilityMargin(const Eigen::Isometry3d& robot_pose, hector_pose_prediction_interface::SupportPolygon<double>& support_polygon)
{
  Eigen::Vector3d com = robot_pose * pose_predictor_->robotModel()->centerOfMass();
  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  hector_stability_metrics::non_differentiable::computeForceAngleStabilityMeasure<double>(support_polygon.contact_hull_points, support_polygon.edge_stabilities, com, gravity);
}

}