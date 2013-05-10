cmake_minimum_required(VERSION 2.8.3)
project(creativa_moveit_tests)

find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES pr2_moveit_tutorials
  # CATKIN_DEPENDS.
  #   moveit_core
  #   moveit_ros_planning_interface
  #   interactive_markers
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(creativa_tutorial src/creativa_tutorial.cpp)
target_link_libraries(creativa_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS creativa_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
