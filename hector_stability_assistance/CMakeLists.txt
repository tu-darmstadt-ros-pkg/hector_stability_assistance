cmake_minimum_required(VERSION 3.0.2)
project(hector_stability_assistance)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  urdf
  tf2_ros
#  hector_heightmap_pose_prediction
  hector_pose_prediction_interface
#  hector_rviz_overlay
  visualization_msgs
#  grid_map_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  hector_rviz_plugins_msgs
  eigen_conversions
#  grid_map_ros
#  hector_world_heightmap_ros
  voxblox_ros
  sdf_contact_estimation
  moveit_core
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_stability_assistance
  CATKIN_DEPENDS
    tf2_ros
    urdf
#    hector_heightmap_pose_prediction
    hector_pose_prediction_interface
#    hector_rviz_overlay
    visualization_msgs
#    grid_map_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    hector_rviz_plugins_msgs
    eigen_conversions
#    grid_map_ros
#    hector_world_heightmap_ros
    voxblox_ros
    sdf_contact_estimation
    moveit_core
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/stability_visualization.h
#  include/${PROJECT_NAME}/local_grid_map.h
#  include/${PROJECT_NAME}/sub_map.h
  include/${PROJECT_NAME}/visualization.h
  include/${PROJECT_NAME}/speed_controller.h


  src/stability_visualization.cpp
#  src/local_grid_map.cpp
#  src/sub_map.cpp
  src/visualization.cpp
  src/speed_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(stability_visualization_node src/stability_visualization_node.cpp)
add_dependencies(stability_visualization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stability_visualization_node ${PROJECT_NAME})

add_executable(speed_controller_node src/speed_controller_node.cpp)
add_dependencies(speed_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_controller_node ${PROJECT_NAME})

#############
## Install ##
#############

# install(DIRECTORY 
#   config
#   cfg
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(TARGETS ${PROJECT_NAME} stability_visualization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# install(FILES 
#   nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hector_stability_assistance.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
