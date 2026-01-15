cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0167 OLD)

project(panda_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(msg_files
  "msg/CubeDetector.msg"
 
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(PROGRAMS
  scripts/so101.py
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(include)

add_executable(panda_ik_node src/panda_ik.cpp   src/trajectory.cpp)
ament_target_dependencies(
  panda_ik_node
  rclcpp
  std_msgs
  ament_index_cpp
)
target_link_libraries(
  panda_ik_node
  pinocchio::pinocchio
  fcl
)
target_link_libraries(panda_ik_node ompl ${cpp_typesupport_target})
target_include_directories(panda_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/ompl-1.5
)



add_executable(cube_detector
  src/perception/cube_detector.cpp)

target_link_libraries(cube_detector ${cpp_typesupport_target})
ament_target_dependencies(cube_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV)

add_executable(cube_tf src/perception/cube_tf_transform.cpp)
ament_target_dependencies(cube_tf
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
)

target_link_libraries(cube_tf ${cpp_typesupport_target})


# Teleop node

add_executable(panda_teleop
  src/panda_teleop.cpp
)

ament_target_dependencies(
  panda_teleop
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(
  panda_teleop
  pinocchio::pinocchio
)

target_include_directories(panda_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# UNIVERSAL ROBOTS
add_executable(home_pos_ur5 src/home_pos.cpp)
ament_target_dependencies(
  home_pos_ur5
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
)

add_executable(ur5_teleop
  src/panda_teleop.cpp
)

ament_target_dependencies(
  ur5_teleop
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(
  ur5_teleop
  pinocchio::pinocchio
)
install(TARGETS panda_ik_node
  cube_detector
  cube_tf
  panda_teleop
  home_pos_ur5
  ur5_teleop
  DESTINATION lib/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install robot models (MJCF / URDF / meshes)
install(
  DIRECTORY franka_emika_panda ur5
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
