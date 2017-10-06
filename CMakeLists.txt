cmake_minimum_required(VERSION 2.8.3)
project(openni2_camera)

find_package(catkin REQUIRED COMPONENTS camera_info_manager
                                        dynamic_reconfigure
                                        image_transport
                                        nodelet
                                        sensor_msgs
                                        roscpp
                                        pal_detection_msgs
                                        cv_bridge
                                        tf)

find_package(Boost REQUIRED COMPONENTS system thread)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}")

add_definitions(-D__linux__)

find_package(OpenNI2 REQUIRED)
find_package(NiTE2 REQUIRED)

generate_dynamic_reconfigure_options(cfg/OpenNI2.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES openni2_wrapper
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp pal_detection_msgs cv_bridge tf
)
link_directories(/usr/lib/OpenNI2/Drivers)

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS}
                    ${OPENNI2_INCLUDE_DIRS}
                    ${NITE2_INCLUDE_DIRS}
                    )

add_library(openni2_wrapper
   src/openni2_convert.cpp
   src/openni2_device.cpp 
   src/openni2_timer_filter.cpp 
   src/openni2_frame_listener.cpp
   src/nite2_hand_tracker_frame_listener.cpp
   src/nite2_user_tracker_frame_listener.cpp
   src/openni2_device_manager.cpp
   src/openni2_exception.cpp
   src/openni2_video_mode.cpp
)

target_link_libraries(openni2_wrapper openni2_wrapper ${catkin_LIBRARIES}
                                                      ${OPENNI2_LIBRARIES}
                                                      ${NITE2_LIBRARIES}
                                                      ${Boost_LIBRARIES} )

add_executable(test_wrapper test/test_wrapper.cpp )
target_link_libraries(test_wrapper openni2_wrapper ${Boost_LIBRARIES} ${NITE2_LIBRARIES})

add_library(openni2_driver_lib
   src/openni2_driver.cpp
)
add_dependencies(openni2_driver_lib pal_detection_msgs_generate_messages_cpp)
target_link_libraries(openni2_driver_lib openni2_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(openni2_driver_lib ${PROJECT_NAME}_gencfg)

add_library(openni2_camera_nodelet
   ros/openni2_camera_nodelet.cpp
)
target_link_libraries(openni2_camera_nodelet openni2_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(openni2_camera_nodelet ${PROJECT_NAME}_gencfg)

add_executable(openni2_camera_node
   ros/openni2_camera_node.cpp
)
target_link_libraries(openni2_camera_node openni2_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(openni2_camera_node ${PROJECT_NAME}_gencfg)

install(TARGETS openni2_wrapper openni2_camera_nodelet openni2_camera_node openni2_driver_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES openni2_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES etc/NiTE.ini
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc
)

