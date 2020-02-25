cmake_minimum_required(VERSION 2.8.3)
project(husky_cartographer_navigation)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/cartographer_demo.launch)
endif()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)