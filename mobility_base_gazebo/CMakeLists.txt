cmake_minimum_required(VERSION 2.8.3)
project(mobility_base_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
