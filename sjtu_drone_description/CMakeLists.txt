cmake_minimum_required(VERSION 3.8)
project(sjtu_drone_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Gazebo Harmonic (gz-sim8) — try vendor packages first, fall back to system
find_package(gz_sim_vendor QUIET)
find_package(gz-sim8 REQUIRED)
find_package(gz_plugin_vendor QUIET)
find_package(gz-plugin2 REQUIRED)

################## Minimal gz-sim8 System plugin (Phase 1 skeleton) ############
add_library(plugin_drone SHARED
  src/plugin_drone_gz.cpp
  src/pid_controller.cpp
  src/plugin_drone_private.cpp
)

target_compile_features(plugin_drone PUBLIC cxx_std_17)
target_include_directories(plugin_drone PRIVATE include)
target_link_libraries(plugin_drone
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)
ament_target_dependencies(plugin_drone
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs
)

install(TARGETS plugin_drone
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid_controller test/test_pid_controller.cpp src/pid_controller.cpp)
  target_include_directories(test_pid_controller PRIVATE include)
  target_link_libraries(test_pid_controller gz-sim8::gz-sim8)

  ament_add_gtest(test_plugin_drone_configure
    test/test_plugin_drone_configure.cpp
    src/pid_controller.cpp
    src/plugin_drone_private.cpp
    src/plugin_drone_gz.cpp
  )
  target_include_directories(test_plugin_drone_configure PRIVATE include)
  target_link_libraries(test_plugin_drone_configure gz-sim8::gz-sim8)
  ament_target_dependencies(test_plugin_drone_configure
    rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs
  )

  ament_add_gtest(test_plugin_drone_dynamics
    test/test_plugin_drone_dynamics.cpp
    src/pid_controller.cpp
    src/plugin_drone_private.cpp
    src/plugin_drone_gz.cpp
  )
  target_include_directories(test_plugin_drone_dynamics PRIVATE include)
  target_link_libraries(test_plugin_drone_dynamics gz-sim8::gz-sim8)
  ament_target_dependencies(test_plugin_drone_dynamics
    rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs
  )

  ament_add_gtest(test_world_loads test/test_world_loads.cpp)
  target_include_directories(test_world_loads PRIVATE include)
  target_link_libraries(test_world_loads gz-sim8::gz-sim8)
  target_compile_definitions(test_world_loads PRIVATE WORLD_DIR="${CMAKE_CURRENT_SOURCE_DIR}/worlds")

  ament_add_gtest(test_model_sdf test/test_model_sdf.cpp)
  target_include_directories(test_model_sdf PRIVATE include)
  target_link_libraries(test_model_sdf gz-sim8::gz-sim8)
  target_compile_definitions(test_model_sdf PRIVATE
    MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models/sjtu_drone"
  )

  find_package(tinyxml2 REQUIRED)
  ament_add_gtest(test_integration test/test_integration.cpp)
  target_include_directories(test_integration PRIVATE include)
  target_link_libraries(test_integration tinyxml2::tinyxml2)
  target_compile_definitions(test_integration PRIVATE
    SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    WORLD_DIR="${CMAKE_CURRENT_SOURCE_DIR}/worlds"
    MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models/sjtu_drone"
  )
endif()

ament_package()
