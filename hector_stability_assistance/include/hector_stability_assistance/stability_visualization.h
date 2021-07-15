#ifndef HECTOR_STABILITY_ASSISTANCE_STABILITY_VISUALIZATION_H
#define HECTOR_STABILITY_ASSISTANCE_STABILITY_VISUALIZATION_H

#include <ros/ros.h>

namespace hector_stability_assistance {

class StabilityVisualization {
public:
  StabilityVisualization(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};

}

#endif