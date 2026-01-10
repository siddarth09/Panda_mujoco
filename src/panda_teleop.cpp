#include "panda_mujoco/panda_teleop.hpp"

#include <chrono>
#include <stdexcept>
#include <algorithm>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/log.hxx>   // pinocchio::log3

using namespace std::chrono_literals;

// -------------------- small helpers --------------------
static inline double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline double deadzone(double v, double dz)
{
  return (std::abs(v) < dz) ? 0.0 : v;
}

static inline double ema(double prev, double x, double alpha)
{
  return alpha * x + (1.0 - alpha) * prev;
}

// ------------------ ctor ------------------
PandaIKNode::PandaIKNode()
: rclcpp::Node("panda_teleop")
, data_(model_) // placeholder; overwritten after model build
{
  arm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arm_controller/commands", 10);

  gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/gripper_controller/commands", 10);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&PandaIKNode::joyCallback, this, std::placeholders::_1));

  js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&PandaIKNode::jointStateCallback, this, std::placeholders::_1));

  // Load Panda model
  const std::string pkg =
    ament_index_cpp::get_package_share_directory("panda_mujoco");
  const std::string urdf = pkg + "/franka_emika_panda/panda.urdf";

  pinocchio::urdf::buildModel(urdf, model_);
  data_ = pinocchio::Data(model_);

  ee_frame_ = model_.getFrameId("panda_ee");
  if (ee_frame_ == (size_t)-1)
    throw std::runtime_error("Frame 'panda_ee' not found in model");

  q_min_ = model_.lowerPositionLimit;
  q_max_ = model_.upperPositionLimit;

  // neutral includes fingers too, but we only command first 7
  q_ = pinocchio::neutral(model_);

  // Teleop loop at 100 Hz
  timer_ = create_wall_timer(10ms, std::bind(&PandaIKNode::teleopStep, this));

  RCLCPP_INFO(get_logger(), "Panda teleop node started (LeRobot-style persistent target)");
}

// ------------------ joystick ------------------
void PandaIKNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_ = *msg;
  have_joy_ = true;
}

// ------------------ joint states ------------------
void PandaIKNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  static const std::vector<std::string> names = {
    "joint1","joint2","joint3","joint4","joint5","joint6","joint7"
  };

  for (int i = 0; i < 7; ++i) {
    for (size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name[j] == names[i]) {
        if (j < msg->position.size()) q_[i] = msg->position[j];
        break;
      }
    }
  }

  have_js_ = true;
}

// ------------------ SO(3) projection ------------------
Eigen::Matrix3d PandaIKNode::projectToSO3(const Eigen::Matrix3d& R) const
{
  // SVD projection: R_proj = U * V^T (ensures det=+1)
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  Eigen::Matrix3d Rproj = U * V.transpose();
  if (Rproj.determinant() < 0.0) {
    U.col(2) *= -1.0;
    Rproj = U * V.transpose();
  }
  return Rproj;
}

// ------------------ teleop loop ------------------
void PandaIKNode::teleopStep()
{
  if (!have_js_ || !have_joy_)
    return;

  // 0) Hold current joint position once to prevent collapse on startup
  if (!holding_position_) {
    publishArm(q_.head(7));
    holding_position_ = true;
    return;
  }

  // 1) Initialize persistent target ONCE (LeRobot style)
  if (!ee_target_initialized_) {
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    const auto& T = data_.oMf[ee_frame_];

    ee_target_pos_ = T.translation();
    ee_target_rot_ = T.rotation();
    ee_target_initialized_ = true;

    RCLCPP_INFO(get_logger(), "Initialized EE target from current pose.");
  }

  // 2) Read enable + mode
  // Your Joy print showed LB = buttons[4] (goes 1 when pressed)
  const bool enable =
    (last_joy_.buttons.size() > 4 && last_joy_.buttons[4]);

  const bool orient_mode =
    (last_joy_.buttons.size() > 5 && last_joy_.buttons[5]);

  // 3) Tunables (feel free to tune)
  constexpr double DZ       = 0.10;
  constexpr double pos_step = 0.005;  // meters per tick
  constexpr double rot_step = 0.015;  // radians per tick
  constexpr double alpha    = 0.35;   // EMA smoothing

  // 4) Read sticks (Xbox: axes[0]=LSx, axes[1]=LSy, axes[3]=RSx, axes[4]=RSy)
  double lsx = (last_joy_.axes.size() > 0) ? last_joy_.axes[0] : 0.0;
  double lsy = (last_joy_.axes.size() > 1) ? last_joy_.axes[1] : 0.0;
  double rsx = (last_joy_.axes.size() > 3) ? last_joy_.axes[3] : 0.0;
  double rsy = (last_joy_.axes.size() > 4) ? last_joy_.axes[4] : 0.0;

  lsx = deadzone(lsx, DZ);
  lsy = deadzone(lsy, DZ);
  rsx = deadzone(rsx, DZ);
  rsy = deadzone(rsy, DZ);

  // 5) LeRobot-style teleop outputs deltas ONLY
  double dx = 0.0, dy = 0.0, dz = 0.0;
  double dyaw = 0.0, dpitch = 0.0;

  if (!enable) {
    // Dead-man released => output is exactly zero deltas
    dx_filt_ = dy_filt_ = dz_filt_ = 0.0;
    dyaw_filt_ = dpitch_filt_ = 0.0;
  } else {
    if (!orient_mode) {
      // TRANSLATION MODE (RB NOT held)
      // LS X/Y -> planar motion (X-Y)
      // RS vertical -> Z
      dx =  pos_step * (lsy);   // forward/back
      dy =  pos_step * (lsx);   // left/right
      dz =  pos_step * (-rsy);  // up/down (invert so up on stick raises)

      dx_filt_ = ema(dx_filt_, dx, alpha);
      dy_filt_ = ema(dy_filt_, dy, alpha);
      dz_filt_ = ema(dz_filt_, dz, alpha);
    } else {
      // ORIENTATION MODE (RB held)
      // RS horizontal -> yaw around Z
      // RS vertical   -> pitch around Y
      dyaw   = rot_step * (rsx);
      dpitch = rot_step * (-rsy);

      dyaw_filt_   = ema(dyaw_filt_, dyaw, alpha);
      dpitch_filt_ = ema(dpitch_filt_, dpitch, alpha);
    }
  }

  // 6) Integrate deltas into persistent target (THIS IS THE KEY FIX)
  ee_target_pos_.x() += dx_filt_;
  ee_target_pos_.y() += dy_filt_;
  ee_target_pos_.z() += dz_filt_;

  // Simple safety clamp (tune to your scene)
  constexpr double TABLE_Z = 0.40;
  constexpr double CLEAR   = 0.03;
  ee_target_pos_.z() = std::clamp(ee_target_pos_.z(), TABLE_Z + CLEAR, 0.85);

  // Apply orientation deltas always (they’re zero if not in orient mode)
  Eigen::Matrix3d R_delta =
    Eigen::AngleAxisd(dyaw_filt_,   Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    Eigen::AngleAxisd(dpitch_filt_, Eigen::Vector3d::UnitY()).toRotationMatrix();

  ee_target_rot_ = R_delta * ee_target_rot_;
  ee_target_rot_ = projectToSO3(ee_target_rot_);

  // 7) Always run IK and publish (even when dead-man off => holds pose)
  solveIK_SE3(ee_target_pos_, ee_target_rot_);
  publishArm(q_.head(7));

  // 8) Gripper (A open, B close; change mapping if needed)
  if (last_joy_.buttons.size() > 0 && last_joy_.buttons[0]) openGripper();
  if (last_joy_.buttons.size() > 1 && last_joy_.buttons[1]) closeGripper();

  // 9) Debug
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "LB=%d RB=%d | MODE=%s | LS(%.2f %.2f) RS(%.2f %.2f) | tgt z=%.3f",
    (int)enable, (int)orient_mode,
    orient_mode ? "ORIENT" : "TRANS",
    lsx, lsy, rsx, rsy,
    ee_target_pos_.z()
  );
}

// ------------------ IK (pose) ------------------
void PandaIKNode::solveIK_SE3(
  const Eigen::Vector3d& target_pos,
  const Eigen::Matrix3d& target_rot)
{
  constexpr double lambda = 1e-3;
  constexpr double w_pos  = 1.0;
  constexpr double w_rot  = 0.4;

  for (int it = 0; it < 60; ++it)
  {
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    const auto& T = data_.oMf[ee_frame_];

    const Eigen::Vector3d pos_err = target_pos - T.translation();
    const Eigen::Matrix3d R_err   = target_rot * T.rotation().transpose();
    const Eigen::Vector3d rot_err = pinocchio::log3(R_err);

    if (pos_err.norm() < 1e-4 && rot_err.norm() < 1e-4)
      break;

    Eigen::MatrixXd J_full(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q_, ee_frame_,
      pinocchio::LOCAL_WORLD_ALIGNED, J_full);

    // Use only arm joints
    Eigen::MatrixXd J = J_full.leftCols(7);

    Eigen::VectorXd err(6);
    err.head<3>() = w_pos * pos_err;
    err.tail<3>() = w_rot * rot_err;

    // Classic damped least squares:
    // dq = J^T (J J^T + λ I)^-1 err
    Eigen::MatrixXd A = (J * J.transpose())
                      + lambda * Eigen::MatrixXd::Identity(6,6);

    Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(err);

    dq = dq.cwiseMax(-0.05).cwiseMin(0.05);
    q_.head(7) += dq;

    q_.head(7) =
      q_.head(7)
        .cwiseMax(q_min_.head(7))
        .cwiseMin(q_max_.head(7));
  }
}

// ------------------ IK (position-only) ------------------
void PandaIKNode::solveIK_SE3(const Eigen::Vector3d& target_pos)
{
  // position-only version just keeps current target orientation
  solveIK_SE3(target_pos, ee_target_rot_);
}

// ------------------ publishing ------------------
void PandaIKNode::publishArm(const Eigen::VectorXd& q7)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(q7.data(), q7.data() + 7);
  arm_pub_->publish(msg);

  // keep internal q consistent with what we command
  q_.head(7) = q7;
}

void PandaIKNode::publishGripper(double v)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {v};
  gripper_pub_->publish(msg);
}

void PandaIKNode::openGripper()
{
  publishGripper(gripper_open_);
}

void PandaIKNode::closeGripper()
{
  publishGripper(gripper_closed_);
}

// ------------------ main ------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PandaIKNode>());
  rclcpp::shutdown();
  return 0;
}
