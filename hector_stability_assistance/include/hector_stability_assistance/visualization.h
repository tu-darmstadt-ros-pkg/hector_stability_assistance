#ifndef HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H
#define HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <hector_pose_prediction_interface/types.h>

void deleteAllMarkers(ros::Publisher &pub)
{
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(marker);
  pub.publish(array);
}

template <typename T>
void appendPointVisualization(const hector_pose_prediction_interface::math::Vector3List<T> &points, visualization_msgs::MarkerArray &marker_array,
                              const std::string& frame_id, const std::string& ns, const Eigen::Vector3f &color, double size)
{
  for (size_t j = 0; j < points.size(); j++) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color.a = 1.0;
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);

    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = static_cast<int>(j);

    Eigen::Affine3d marker_pose = Eigen::Affine3d::Identity();
    marker_pose.translation() = points[j].template cast<double>();
    tf::poseEigenToMsg(marker_pose, marker.pose);
    marker_array.markers.push_back(marker);
  }
}

template <typename T>
void appendSupportPolygonVisualization(const hector_pose_prediction_interface::math::SupportPolygon<T>& support_polygon, visualization_msgs::MarkerArray &marker_array, const std::string& frame_id, const std::string& ns)
{
  for (size_t i = 0; i < support_polygon.contact_hull_points.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.00001;
    marker.scale.z = 0.00001;

    // calculate stability color
    T stability_pct = 1.0f;
    if (i < support_polygon.edge_stabilities.size()) {
      T max_stability = 6.0f;
      T clamped_stability = std::max(0.0f, std::min(support_polygon.edge_stabilities[i], max_stability));
      stability_pct = clamped_stability/max_stability;
    }
    marker.color.a = 1.0;
    marker.color.r = 1.0f - static_cast<float>(stability_pct);
    marker.color.g = static_cast<float>(stability_pct);
    marker.color.b = 0;

    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = static_cast<int>(i);

    Eigen::Affine3d marker_pose = Eigen::Affine3d::Identity();
    tf::poseEigenToMsg(marker_pose, marker.pose);

    geometry_msgs::Point point_start;
    tf::pointEigenToMsg(support_polygon.contact_hull_points[i].template cast<double>(), point_start);
    marker.points.push_back(point_start);
    int ip1 = (i + 1) % support_polygon.contact_hull_points.size(); // wrap around
    geometry_msgs::Point point_end;
    tf::pointEigenToMsg(support_polygon.contact_hull_points[ip1].template cast<double>(), point_end);
    marker.points.push_back(point_end);
    marker_array.markers.push_back(marker);
  }
}





#endif //HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H
