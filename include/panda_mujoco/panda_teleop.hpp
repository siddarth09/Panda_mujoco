#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <Eigen/Dense>

#include <memory>
#include <vector>
#include <string>

class PandaIKNode : public rclcpp::Node
{
public:
  PandaIKNode();

private:
  // ---------------- ROS callbacks ----------------
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ---------------- Control loop ----------------
  void teleopStep();

  // ---------------- IK ----------------
  void solveIK_SE3(const Eigen::Vector3d& target_pos);                         // position-only
  void solveIK_SE3(const Eigen::Vector3d& target_pos, const Eigen::Matrix3d& target_rot); // pose

  // ---------------- Helpers ----------------
  Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& R) const;

  // ---------------- Command publishing ----------------
  void publishArm(const Eigen::VectorXd& q7);
  void publishGripper(double finger_pos);
  void openGripper();
  void closeGripper();

  // ---------------- ROS interfaces ----------------
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------------- Pinocchio ----------------
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_{0};

  Eigen::VectorXd q_;
  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;

  // ---------------- Teleop state ----------------
  sensor_msgs::msg::Joy last_joy_;
  bool have_joy_{false};
  bool have_js_{false};

  // Hold command once we have JS (prevents startup collapse)
  bool holding_position_{false};

  // LeRobot-style persistent EE target (THE KEY FIX)
  Eigen::Vector3d ee_target_pos_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d ee_target_rot_{Eigen::Matrix3d::Identity()};
  bool ee_target_initialized_{false};

  // ---------------- Filters ----------------
  double dx_filt_{0.0};
  double dy_filt_{0.0};
  double dz_filt_{0.0};
  double dyaw_filt_{0.0};
  double dpitch_filt_{0.0};

  // ---------------- Gripper ----------------
  double gripper_open_{0.04};
  double gripper_closed_{0.01};
};
