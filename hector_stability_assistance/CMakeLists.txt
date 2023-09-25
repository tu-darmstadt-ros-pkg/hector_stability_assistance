cmake_minimum_required(VERSION 3.0.2)
project(hector_stability_assistance)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  urdf
  tf2_ros
  hector_pose_prediction_interface
  hector_stability_metrics
  hector_pose_prediction_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  hector_rviz_plugins_msgs
  eigen_conversions
  voxblox_ros
  sdf_contact_estimation
  moveit_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_stability_assistance
  CATKIN_DEPENDS
   urdf
   tf2_ros
   hector_pose_prediction_interface
   hector_stability_metrics
   hector_pose_prediction_ros
   std_msgs
   sensor_msgs
   geometry_msgs
   visualization_msgs
   hector_rviz_plugins_msgs
   eigen_conversions
   voxblox_ros
   sdf_contact_estimation
   moveit_core
#  DEPENDS system_lib
)

catkin_add_env_hooks(50.export.hector_stability_assistance SHELLS bash)

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
  include/${PROJECT_NAME}/robot_state_provider.h

  src/stability_visualization.cpp
#  src/local_grid_map.cpp
#  src/sub_map.cpp
  src/visualization.cpp
  src/speed_controller.cpp
  src/robot_state_provider.cpp

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

install(DIRECTORY
   launch
   config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

install(TARGETS ${PROJECT_NAME} stability_visualization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} speed_controller_node
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
