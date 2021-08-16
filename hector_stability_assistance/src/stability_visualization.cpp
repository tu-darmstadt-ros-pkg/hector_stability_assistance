#include <hector_stability_assistance/stability_visualization.h>

#include <chrono>
#include <hector_heightmap_pose_prediction/heightmap_pose_predictor.h>
#include <hector_heightmap_pose_prediction/robot_heightmap_providers/urdf_robot_heightmap_provider.h>
#include <hector_world_heightmap_ros/message_conversions/map.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/PointCloud.h>

#include <hector_stability_assistance/local_grid_map.h>
#include <hector_stability_assistance/visualization.h>

namespace hector_stability_assistance {

StabilityVisualization::StabilityVisualization(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
: nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_), update_frequency_(10.0) {}

bool StabilityVisualization::init() {
  // Parameters
  update_frequency_ = pnh_.param("update_frequency", 10.0);
  elevation_layer_name_ = pnh_.param("elevation_layer_name", std::string("elevation"));


  // Load urdf model
  if (!urdf_model_.initParam("robot_description")) {
    ROS_ERROR("Failed to load robot_description");
    return false;
  }

  for (const auto &kv : urdf_model_.joints_)
  {
    if (kv.second->type != urdf::Joint::FIXED && kv.second->type != urdf::Joint::UNKNOWN && !kv.second->mimic) {
      missing_joint_states_.insert( kv.first );
      ROS_DEBUG_STREAM("Adding required joint: " << kv.first << " type: " << kv.second->type);
    }
  }

  // Subscriber and Publisher
  stability_pub_ = pnh_.advertise<std_msgs::Float64>("stability", 1);
  traction_pub_ = pnh_.advertise<std_msgs::Float64>("traction", 1);
  support_polygon_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("support_polygon", 1);
  com_pub_ = pnh_.advertise<geometry_msgs::PointStamped>("center_of_mass", 1);
  robot_heightmap_pub_ = pnh_.advertise<sensor_msgs::PointCloud>("robot_heightmap", 1);
  submap_pub_ = pnh_.advertise<grid_map_msgs::GridMap>("submap", 1);

  grid_map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("elevation_map", 1, &StabilityVisualization::gridMapCallback, this);
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &StabilityVisualization::jointStateCallback, this);
  timer_ = nh_.createTimer(ros::Duration(1.0/update_frequency_), &StabilityVisualization::timerCallback, this, false);
  return true;
}

void StabilityVisualization::gridMapCallback(const grid_map_msgs::GridMapConstPtr& grid_map) {
  latest_grid_map_ = grid_map;
}

void StabilityVisualization::timerCallback(const ros::TimerEvent&) {
  update();
}

void StabilityVisualization::update() {
  ROS_INFO_STREAM("Updating stability");
  // Update robot state
  if (!missing_joint_states_.empty()) {
    ROS_WARN_STREAM_THROTTLE(1, "Can't update stability estimation: The following joint states are still missing: " << setToString(missing_joint_states_));
    return;
  }
  auto robot_height_map_provider = std::make_shared<hector_heightmap_pose_prediction::UrdfRobotHeightmapProvider<float>>(urdf_model_, joint_states_);

  // Update map
  if (!latest_grid_map_) {
    ROS_WARN_STREAM_THROTTLE(1, "Can't update stability estimation: No elevation map has been received yet.");
    return;
  }
  hector_world_heightmap::WorldHeightmapInterface<float>::Ptr height_map = std::make_shared<LocalGridMap>(latest_grid_map_, elevation_layer_name_);


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

  Eigen::Matrix<double, 3, 1> robot_position;
  Eigen::Quaternion<double> robot_orientation;
  tf::vectorMsgToEigen(transform_msg.transform.translation, robot_position);
  tf::quaternionMsgToEigen(transform_msg.transform.rotation, robot_orientation);
  hector_pose_prediction_interface::math::Pose<float> robot_pose(robot_position.cast<float>(), robot_orientation.cast<float>());



  // Generate pose prediction with current state
  pose_predictor_ = std::make_shared<hector_heightmap_pose_prediction::HeightmapPosePredictor<float, true>>(height_map, robot_height_map_provider);

  hector_pose_prediction_interface::math::SupportPolygon<float> support_polygon;
  hector_pose_prediction_interface::ContactInformation<float> contact_information;
//  if (!pose_predictor_->estimateContactInformation(robot_pose, support_polygon, contact_information)) {
  if (pose_predictor_->estimateSupportPolygon(robot_pose, support_polygon)) {
    // Publish results
    // Stability
    auto min_it = std::min_element(support_polygon.edge_stabilities.begin(), support_polygon.edge_stabilities.end());
    std_msgs::Float64 stability_msg;
    stability_msg.data = *min_it;
    stability_pub_.publish(stability_msg);

    // Support polygon
    visualization::deleteAllMarkers(support_polygon_pub_);
    visualization_msgs::MarkerArray support_polygon_marker_array;
    visualization::appendPointVisualization(support_polygon.contact_hull_points, support_polygon_marker_array, urdf_model_.getRoot()->name, "contact_points", Eigen::Vector3f(1, 0, 0), 0.03);
    visualization::appendSupportPolygonVisualization(support_polygon, support_polygon_marker_array, urdf_model_.getRoot()->name, "support_polygon");
    support_polygon_pub_.publish(support_polygon_marker_array);
  } else {
    ROS_WARN("Support polygon estimation failed.");
  }



  // Center of mass
  geometry_msgs::PointStamped point_msg;
  if(urdf_model_.getRoot()) {
    point_msg.header.frame_id = urdf_model_.getRoot()->name;
  }
  point_msg.header.stamp = ros::Time::now();
  tf::pointEigenToMsg(robot_height_map_provider->centerOfMass().cast<double>(), point_msg.point);
  com_pub_.publish(point_msg);

  /*
   * DEBUG
   */
  // Robot Heightmap
  hector_world_heightmap::HeightmapRef<float>::ConstPtr robot_height_map =
      robot_height_map_provider->computeHeightmap( height_map->resolution(), robot_orientation.cast<float>() );
  Eigen::Matrix3Xf pointcloud = hector_world_heightmap_ros::message_conversions::heightmapToPointcloud<float>(robot_height_map);
  sensor_msgs::PointCloud pointcloud_msg;
  pointcloud_msg.header.frame_id = latest_grid_map_->info.header.frame_id;
  pointcloud_msg.header.stamp = ros::Time::now();
  for (Eigen::Index i = 0; i < pointcloud.cols(); ++i) {
    const Eigen::Vector3f hm_point = robot_pose.translation() + pointcloud.col(i);
    geometry_msgs::Point32 point;
    point.x = hm_point.x();
    point.y = hm_point.y();
    point.z = hm_point.z();
    pointcloud_msg.points.push_back( point );
  }
  robot_heightmap_pub_.publish(pointcloud_msg);

  // Submap
  hector_world_heightmap::HeightmapRef<float>::ConstPtr ground_submap =
      height_map->getSubMap(robot_pose.translation() + robot_height_map->origin(),
                            robot_height_map->map().rows(), robot_height_map->map().cols());
  if (ground_submap) {
    submap_pub_.publish( hector_world_heightmap_ros::message_conversions::heightmapToMsg<float>(ground_submap));
  } else {
    ROS_WARN_STREAM("Failed to retrieve sub map");
  }
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