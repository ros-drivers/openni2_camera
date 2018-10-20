cmake_minimum_required(VERSION 2.8.3)
project(openni2_launch)

find_package(catkin)
catkin_python_setup()
catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
#  roslaunch_add_file_check(launch/includes)  # 20181003 this fails.
endif()
