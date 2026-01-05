#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "panda_mujoco/trajectory_utils.hpp"
#include "panda_mujoco/motion_planner.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>

#include <memory>
#include <vector>

// Cube message
#include <panda_mujoco/msg/cube_detector.hpp>

class PandaMotionPlanner;




struct CubeInfo
{
  std::string color;
  Eigen::Vector3d pos;
  rclcpp::Time stamp;
};




class PandaIKNode : public rclcpp::Node
{
public:
  PandaIKNode();

private:
  // ----- ROS callbacks / loop -----
  void cubeCallback(const panda_mujoco::msg::CubeDetector::SharedPtr msg);
  void controlLoop();

  // ----- Motion helpers -----
  void planAndExecute(const Eigen::Vector3d& target_pos);
  void solveIK_SE3(const Eigen::Vector3d& target_pos);

  // ----- Command publishing -----
  void publishArm(const Eigen::VectorXd& q7);
  void publishGripper(double finger_pos);
  void openGripper();
  void closeGripper();
  bool selectNextCube(Eigen::Vector3d& out_pos, std::string& color);

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  // ----- OMPL / collision helpers -----
  Eigen::VectorXd omplStateToEigen(const ompl::base::State* st) const;
  Eigen::VectorXd armToFullQ(const Eigen::VectorXd& q7) const;
  bool isCollisionFree(const Eigen::VectorXd& q7);

  // ----- ROS interfaces -----
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Subscription<panda_mujoco::msg::CubeDetector>::SharedPtr cube_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

  // ----- Pinocchio -----
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_{0};

  Eigen::VectorXd q_;
  Eigen::VectorXd q_min_, q_max_;

  // Collision
  pinocchio::GeometryModel geom_model_;
  pinocchio::GeometryData geom_data_;

  // Planner
  std::unique_ptr<PandaMotionPlanner> planner_;

  // ----- State -----
  bool have_cube_{false};
  bool executing_{false};
  bool have_js_{false};
  Eigen::Vector3d cube_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pregrasp_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d grasp_{Eigen::Vector3d::Zero()};

  // ----- Control params -----
  double control_hz_{100.0};

  // Gripper setpoints (ROS2 control: finger_joint1 position)
  double gripper_open_{0.04};
  double gripper_closed_{0.0};

  std::vector<CubeInfo> cube_buffer_;
  bool picking_{false};
};
