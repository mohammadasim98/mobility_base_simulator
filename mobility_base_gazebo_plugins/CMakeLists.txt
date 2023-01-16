cmake_minimum_required(VERSION 2.8.3)
project(mobility_base_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  tf
  gazebo_ros
  gazebo_plugins
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mobility_base_core_msgs
)
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS tf gazebo_ros gazebo_plugins roscpp std_msgs sensor_msgs geometry_msgs mobility_base_core_msgs
  DEPENDS gazebo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/MobilityBasePlugin.cpp)
add_dependencies(${PROJECT_NAME} mobility_base_core_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
