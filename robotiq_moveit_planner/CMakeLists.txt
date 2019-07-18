cmake_minimum_required(VERSION 2.8.3)
project(robotiq_moveit_planner)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    roscpp
    rospy
    rviz_visual_tools
    moveit_visual_tools
    )

find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    roscpp
    rospy
    rviz_visual_tools
    moveit_visual_tools
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

#add_executable(${PROJECT_NAME}_node src/bioik_planning.cpp)
add_executable(gripper_test src/gripper_test.cpp)
target_link_libraries(gripper_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})


## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   robotic_tools
#)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

