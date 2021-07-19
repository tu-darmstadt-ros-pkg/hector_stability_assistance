#include <hector_stability_assistance/visualization.h>

namespace hector_stability_assistance {
namespace visualization {

void deleteAllMarkers(ros::Publisher &pub) {
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(marker);
  pub.publish(array);
}

}
}


