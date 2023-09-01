#include <hector_stability_assistance/visualization.h>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <hector_pose_prediction_ros/visualization.h>
#include <hector_rviz_plugins_msgs/DisplayMultiRobotState.h>

namespace hector_stability_assistance {
namespace visualization {

void deleteAllMarkers(ros::Publisher &pub) {
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(marker);
  pub.publish(array);
}

void publishSupportPolygon(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                           const hector_pose_prediction_interface::ContactInformation<double>& contact_information,
                           ros::Publisher& publisher)
{
  visualization::deleteAllMarkers(publisher);
  visualization_msgs::MarkerArray support_polygon_marker_array;
  hector_pose_prediction_interface::visualization::addSupportPolygonWithContactInformationToMarkerArray(
      support_polygon_marker_array, support_polygon, contact_information, "world"); //TODO map name
  publisher.publish(support_polygon_marker_array);
}

void publishMinStability(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                                                 ros::Publisher& publisher)
{
  auto min_it = std::min_element(support_polygon.edge_stabilities.begin(), support_polygon.edge_stabilities.end());
  std_msgs::Float64 stability_msg;
  stability_msg.data = *min_it;
  publisher.publish(stability_msg);
}

void publishEdgeStabilities(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                                                    ros::Publisher& publisher)
{
  std_msgs::Float64MultiArray stability_margins_msg;
  stability_margins_msg.data = support_polygon.edge_stabilities;
  publisher.publish(stability_margins_msg);
}

void deleteRobotModels(ros::Publisher& publisher)
{
  hector_rviz_plugins_msgs::DisplayMultiRobotState display_msg;
  display_msg.header.frame_id = "world";
  publisher.publish(display_msg);
}

}
}


