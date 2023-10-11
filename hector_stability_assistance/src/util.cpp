#include <hector_stability_assistance/util.h>
#include <sdf_contact_estimation/util/utils.h>

namespace hector_stability_assistance {
namespace util {

Eigen::Isometry3d pose3Dto2D(const Eigen::Isometry3d& pose) {
  Eigen::Vector3d pose_rpy = sdf_contact_estimation::rotToNormalizedRpy(pose.linear());
  Eigen::Isometry3d pose_2d(Eigen::AngleAxisd(pose_rpy(2), Eigen::Vector3d::UnitZ()));
  pose_2d.translation() = pose.translation();
  return pose_2d;
}

Eigen::Isometry3d computeDiffDriveTransform(double linear_speed, double angular_speed, double time_delta) {
  double x;
  double y;
  double theta = angular_speed * time_delta;
  if (angular_speed < 1e-6) {
    x = linear_speed * time_delta;
    y = 0;
  } else {
    x = linear_speed / angular_speed * std::sin(theta);
    y = linear_speed / angular_speed * (1 - std::cos(theta));
  }

  Eigen::Isometry3d transform(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  transform.translation().x() = x;
  transform.translation().y() = y;
  transform.translation().z() = 0;

  return transform;
}
}
}
