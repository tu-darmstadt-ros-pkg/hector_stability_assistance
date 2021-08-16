#ifndef HECTOR_STABILITY_ASSISTANCE_SUB_MAP_H
#define HECTOR_STABILITY_ASSISTANCE_SUB_MAP_H

#include <hector_world_heightmap/world_heightmap_interface.h>
#include <hector_world_heightmap/helpers/eigen.h>

#include <grid_map_ros/grid_map_ros.hpp>

namespace hector_stability_assistance {

typedef float SCALAR;

class SubMap : public hector_world_heightmap::HeightmapRef<SCALAR> {
public:
  SubMap(const std::shared_ptr<grid_map::GridMap>& map, const std::string& layer_name, const grid_map::Index& start_index, Eigen::Index rows, Eigen::Index cols )
      : map_(map), layer_name_(layer_name), start_index_(start_index), rows_( rows ), cols_( cols ), row_offset_(0), col_offset_(0)
  {
    assert(start_index[0] < map_->getSize()[0]);
    assert(start_index[1] < map_->getSize()[1]);
    // Compute origin (center) of sub map
//    SCALAR x_offset = (start_index[0] + rows / 2.f) * map_->getResolution() - map_->getLength().x() / 2.f;
//    SCALAR y_offset = (start_index[1] + cols / 2.f) * map_->getResolution() - map_->getLength().y() / 2.f;
//    origin_ << static_cast<SCALAR>(map_->getPosition().x()) + x_offset, static_cast<SCALAR>(map_->getPosition().y()) + y_offset, 0;

    grid_map::Index origin_index(start_index[0] + rows/2.f, start_index[1] + cols/2.f);
    grid_map::Position position;
    map_->getPosition(origin_index, position);
    origin_.x() = position.x();
    origin_.y() = position.y();
    origin_.z() = 0;
    ROS_INFO("Submap origin at [%f, %f, %f]", origin_.x(), origin_.y(), origin_.z());

    // Check if the start index is > 0, otherwise add an offset
    if (start_index[0] < 0) {
      row_offset_ = -start_index[0];
      start_index_[0] = 0;
      ROS_INFO_STREAM("Row offset: " << row_offset_);
    }

    if (start_index[1] < 0 ) {
      col_offset_ = -start_index[1];
      start_index_[1] = 0;
      ROS_INFO_STREAM("Col offset: " << col_offset_);
    }

  }

  Eigen::Ref<const hector_world_heightmap::math::GridMap<SCALAR>> map() const override {
    // Check if submap is completely in map
    if ( row_offset_ == 0 && col_offset_ == 0 && start_index_[0] + rows_ <= map_->getSize()[0] &&
        start_index_[0] + cols_ <= map_->getSize()[1]) {
      ROS_INFO("Simple map from [%d, %d] to [%ld, %ld]", start_index_[0], start_index_[1], start_index_[0] + rows_, start_index_[1] + cols_);
      return hector_world_heightmap::eigen::flip(map_->get(layer_name_).block(start_index_[0], start_index_[1], rows_, cols_).array());
    }

    // Otherwise do fancy stuff
    ROS_INFO_STREAM("Padding map");
    Eigen::Index rows = std::min(rows_, static_cast<long>(map_->getSize()[0] - start_index_[0])) - row_offset_;
    Eigen::Index cols = std::min(cols_, static_cast<long>(map_->getSize()[1] - start_index_[1])) - col_offset_;
    return hector_world_heightmap::eigen::wrapWithConstant(map_->get(layer_name_).block(start_index_[0], start_index_[1], rows, cols).array(),
                                    std::numeric_limits<SCALAR>::quiet_NaN(), rows_, cols_, row_offset_, col_offset_ );
  }

  hector_world_heightmap::math::Vector3<SCALAR> origin() const override { return origin_; }

  std::string frame() const override { return map_->getFrameId(); }

  SCALAR resolution() const override { return static_cast<SCALAR>(map_->getResolution()); }

  long long timestamp() const override { return 0; }

private:
  std::shared_ptr<grid_map::GridMap> map_;
  std::string layer_name_;
  hector_world_heightmap::math::Vector3<SCALAR> origin_;
  grid_map::Index start_index_;
  Eigen::Index rows_;
  Eigen::Index cols_;
  Eigen::Index row_offset_;
  Eigen::Index col_offset_;
};


};



#endif //HECTOR_STABILITY_ASSISTANCE_SUB_MAP_H
