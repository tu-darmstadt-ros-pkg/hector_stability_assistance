#ifndef HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H
#define HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H

#include <ros/ros.h>
#include <hector_pose_prediction_interface/types.h>
#include <visualization_msgs/MarkerArray.h>


namespace hector_stability_assistance {
namespace visualization {

void deleteAllMarkers(const ros::Publisher &pub);

void fixIds(visualization_msgs::MarkerArray& array);

void publishEdgeStabilities(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                            ros::Publisher& publisher);
void publishMinStability(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                         ros::Publisher& publisher);
void publishSupportPolygon(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                           const hector_pose_prediction_interface::ContactInformation<double>& contact_information,
                           ros::Publisher& publisher);
void publishDouble(double value, ros::Publisher& publisher);

}
}







#endif //HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H
