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

install(PROGRAMS
  scripts/so101.py
  DESTINATION lib/${PROJECT_NAME}
)



add_executable(panda_ik_node src/panda_ik.cpp)

ament_target_dependencies(
  panda_ik_node
  rclcpp
  std_msgs
  ament_index_cpp
)

target_link_libraries(
  panda_ik_node
  pinocchio::pinocchio
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
