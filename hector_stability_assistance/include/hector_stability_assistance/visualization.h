#ifndef HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H
#define HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H

#include <ros/ros.h>
#include <hector_pose_prediction_interface/types.h>

namespace hector_stability_assistance {
namespace visualization {

void deleteAllMarkers(ros::Publisher &pub);

void publishEdgeStabilities(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                            ros::Publisher& publisher);
void publishMinStability(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                         ros::Publisher& publisher);
void publishSupportPolygon(const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                           const hector_pose_prediction_interface::ContactInformation<double>& contact_information,
                           ros::Publisher& publisher);

}
}







#endif //HECTOR_STABILITY_ASSISTANCE_VISUALIZATION_H
