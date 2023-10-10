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
}
}
