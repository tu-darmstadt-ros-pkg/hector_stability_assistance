#ifndef HECTOR_STABILITY_ASSISTANCE_UTIL_H
#define HECTOR_STABILITY_ASSISTANCE_UTIL_H

#include <Eigen/Eigen>

namespace hector_stability_assistance {
namespace util {
  Eigen::Isometry3d pose3Dto2D(const Eigen::Isometry3d& pose);
}
}

#endif  // HECTOR_STABILITY_ASSISTANCE_UTIL_H
