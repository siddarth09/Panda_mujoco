cmake_minimum_required(VERSION 3.8)
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
target_link_libraries(panda_ik_node ompl)
target_include_directories(panda_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/ompl-1.5
)

install(TARGETS panda_ik_node
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
  DIRECTORY franka_emika_panda
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
