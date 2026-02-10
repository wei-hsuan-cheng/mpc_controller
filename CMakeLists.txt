cmake_minimum_required(VERSION 3.14)
project(mpc_controller LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(mobile_manipulator_mpc REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/control/OCS2Controller.cpp
  src/visualization/MobileManipulatorVisualization.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  geometry_msgs
  visualization_msgs
  tf2_ros
  control_msgs
  nav_msgs
  ocs2_core
  ocs2_mpc
  ocs2_sqp
  ocs2_ros_interfaces
  mobile_manipulator_mpc
  ocs2_self_collision_visualization
)

pluginlib_export_plugin_description_file(controller_interface mpc_controller_plugin.xml)

set(OCS2_CONTROL_NODE_DEPENDENCIES
  rclcpp
  mobile_manipulator_mpc
  ocs2_sqp
  ocs2_ros_interfaces
  ocs2_msgs
  std_srvs
  geometry_msgs
  tf2_ros
  visualization_msgs
)

# MPC node
add_executable(mobile_manipulator_mpc_node src/mpc/MobileManipulatorMpcNode.cpp)
ament_target_dependencies(mobile_manipulator_mpc_node ${OCS2_CONTROL_NODE_DEPENDENCIES} ocs2_core ocs2_mpc)
target_link_libraries(mobile_manipulator_mpc_node yaml-cpp)

# SQP MPC node
add_executable(mobile_manipulator_sqp_mpc_node src/mpc/MobileManipulatorSqpMpcNode.cpp)
ament_target_dependencies(mobile_manipulator_sqp_mpc_node ${OCS2_CONTROL_NODE_DEPENDENCIES} ocs2_core ocs2_sqp)
target_link_libraries(mobile_manipulator_sqp_mpc_node yaml-cpp)

add_executable(mobile_manipulator_marker_target src/command/MobileManipulatorMarkerTarget.cpp)
ament_target_dependencies(mobile_manipulator_marker_target ${OCS2_CONTROL_NODE_DEPENDENCIES})
target_link_libraries(mobile_manipulator_marker_target yaml-cpp)

add_executable(mobile_manipulator_twist_target src/command/MobileManipulatorTwistTarget.cpp)
ament_target_dependencies(mobile_manipulator_twist_target ${OCS2_CONTROL_NODE_DEPENDENCIES})
target_link_libraries(mobile_manipulator_twist_target yaml-cpp)

add_executable(mobile_manipulator_trajectory_target src/command/MobileManipulatorTrajectoryTarget.cpp)
ament_target_dependencies(mobile_manipulator_trajectory_target ${OCS2_CONTROL_NODE_DEPENDENCIES})
target_link_libraries(mobile_manipulator_trajectory_target yaml-cpp)

add_executable(fake_base_odom_node src/sim/FakeBaseOdomNode.cpp)
ament_target_dependencies(fake_base_odom_node rclcpp geometry_msgs nav_msgs tf2_ros)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  mobile_manipulator_mpc_node
  mobile_manipulator_sqp_mpc_node
  mobile_manipulator_marker_target
  mobile_manipulator_twist_target
  mobile_manipulator_trajectory_target
  fake_base_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES mpc_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(DIRECTORY auto_generated/
  DESTINATION share/${PROJECT_NAME}/auto_generated
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  geometry_msgs
  visualization_msgs
  tf2_ros
  control_msgs
  nav_msgs
  ocs2_core
  ocs2_mpc
  ocs2_sqp
  ocs2_ros_interfaces
  mobile_manipulator_mpc
  ocs2_self_collision_visualization
  ocs2_msgs
  std_srvs
)

ament_package()
