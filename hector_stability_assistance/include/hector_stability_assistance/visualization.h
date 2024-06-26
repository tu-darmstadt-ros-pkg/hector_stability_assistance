#ifndef HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H
#define HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H

#include <ros/ros.h>
#include <hector_pose_prediction_interface/types.h>
#include <visualization_msgs/MarkerArray.h>


namespace hector_stability_assistance {
namespace visualization {

void deleteAllMarkers(const ros::Publisher &pub);
void deleteAllMarkers(visualization_msgs::MarkerArray& array);

void fixIds(visualization_msgs::MarkerArray& array);

void publishEdgeStabilities(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                            ros::Publisher& publisher);
void publishMinStability(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                         ros::Publisher& publisher);
void publishSupportPolygon(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                           const hector_pose_prediction_interface::ContactInformation<double>& contact_information,
                           ros::Publisher& publisher);
void publishDouble(double value, ros::Publisher& publisher);
void publishPose(const Eigen::Isometry3d& pose, const std::string& frame_id, ros::Publisher& pub);

template <typename T>
std::string setToString(const std::set<T>& set) {
  std::stringstream ss;
  ss << "[";
  for (auto entry: set) {
    ss << entry << ",";
  }
  ss << "]";
  return ss.str();
}

}
}







#endif //HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H
