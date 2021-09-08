cmake_minimum_required(VERSION 2.8.10)
project(sjtu_drone)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    image_transport
    message_generation
    gazebo_ros
    gazebo_plugins
    )

find_package(gazebo REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(OGRE OGRE)
    pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()


link_directories(${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${OGRE_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}
                    ${OGRE-Terrain_INCLUDE_DIRS}
                    include)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

###################################
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy image_transport std_msgs sensor_msgs geometry_msgs message_runtime
)

################# 1. A system plugin for gazebo to initialize ROS  #############
add_library( plugin_ros_init SHARED
    src/plugin_ros_init.cpp
)
target_link_libraries( plugin_ros_init
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${OGRE_LIBRARY_DIRS}
)

set_target_properties( plugin_ros_init
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################## 2. A simple model controller for the quadrotor #############
add_library( plugin_drone SHARED
    src/plugin_drone.cpp
    src/pid_controller.cpp
    include/plugin_drone.h
    include/pid_controller.h
)

target_link_libraries( plugin_drone
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

set_target_properties( plugin_drone
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################## 3. A IMU sensor plugin for the quadrotor ####################
add_library( plugin_ros_imu SHARED
    src/plugin_ros_imu_native.cpp
    include/plugin_ros_imu_native.h
)

add_dependencies(plugin_ros_imu plugin_imu_ros_gencpp)

target_link_libraries( plugin_ros_imu
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

set_target_properties( plugin_ros_imu
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################# 4. A  sensor plugin for the camera  #########################
add_library( plugin_ros_cam SHARED
    src/plugin_ros_cam.cpp
    src/util_ros_cam.cpp
    include/plugin_ros_cam.h
    include/util_ros_cam.h
)
target_link_libraries( plugin_ros_cam
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${OGRE_LIBRARY_DIRS}
   CameraPlugin
)

set_target_properties( plugin_ros_cam
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)


################## 7. A sonar sensor plugin for the quadrotor ####################
add_library( plugin_ros_sonar SHARED
    src/plugin_ros_sonar.cpp
    include/plugin_ros_sonar.h
)

target_link_libraries( plugin_ros_sonar
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

set_target_properties( plugin_ros_sonar
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)


install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
