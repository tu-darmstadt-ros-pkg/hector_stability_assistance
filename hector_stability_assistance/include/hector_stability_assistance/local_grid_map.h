#ifndef HECTOR_STABILITY_ASSISTANCE_LOCAL_GRID_MAP_H
#define HECTOR_STABILITY_ASSISTANCE_LOCAL_GRID_MAP_H

#include <hector_world_heightmap/world_heightmap_interface.h>
#include <hector_world_heightmap/heightmap_functions.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include "sub_map.h"

namespace hector_stability_assistance  {

class LocalGridMap : public hector_world_heightmap::WorldHeightmapInterface<float> {
public:
  LocalGridMap(const grid_map_msgs::GridMapConstPtr& grid_map_msg, const std::string& layer_name) {
    grid_map_ = std::make_shared<grid_map::GridMap>();
    grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, *grid_map_, std::vector<std::string>(1, layer_name));
  }

  bool hasMapAt(const hector_world_heightmap::math::Vector3<float> &location) const override {
    grid_map::Position position = location.topRows<2>().cast<double>();
    grid_map::Index index;
    grid_map_->getIndex(position, index);
    return grid_map_->isValid(index);
  }

  hector_world_heightmap::HeightmapRef<float>::ConstPtr getSubMap(const hector_world_heightmap::math::Vector3<float> &origin,
                                                                  Eigen::Index rows,
                                                                  Eigen::Index cols) const override {
    if (!hasMapAt(origin)) {
      return nullptr;
    }


    hector_world_heightmap::math::BlockIndices indices = hector_world_heightmap::math::blockCoordinatesToIndices<float>(grid_map_->getPosition().cast<float>(),
                                                                          grid_map_->getSize()[0], grid_map_->getSize()[1],
                                                                          origin.topRows<2>(), rows, cols,
                                                                          grid_map_->getResolution());
    grid_map::Index start_index;
    start_index.x() = indices.x0;
    start_index.y() = indices.y0;

    grid_map::Size size = grid_map_->getSize();
    // Return nullptr if no overlap at all
    if (start_index.x() + rows <= 0 || start_index.y() + cols <= 0 || start_index.x() >= size.x() || start_index.y() >= size.y())
      return nullptr;

    return std::make_shared<SubMap>(grid_map_, layer_name_, start_index, rows, cols);
  }

  float resolution() const override {
    return static_cast<float>(grid_map_->getResolution());
  };
private:
  std::shared_ptr<grid_map::GridMap> grid_map_;
  std::string layer_name_;
};

}

#endif //HECTOR_STABILITY_ASSISTANCE_LOCAL_GRID_MAP_H
