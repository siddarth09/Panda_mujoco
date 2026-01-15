#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/log.hxx>

#include <Eigen/Dense>

#include <chrono>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

using namespace std::chrono_literals;

// -------------------- small helpers --------------------
static inline double deadzone(double v, double dz)
{
  return (std::abs(v) < dz) ? 0.0 : v;
}

static inline double ema(double prev, double x, double alpha)
{
  return alpha * x + (1.0 - alpha) * prev;
}

class UR5TeleopNode : public rclcpp::Node
{
public:
  UR5TeleopNode()
  : rclcpp::Node("ur5_teleop"),
    data_(model_)
  {
    // ----------- EDIT THESE FOR YOUR SETUP -----------
    command_topic_ = "/ur5_arm_controller/commands";   // controller command topic
    ee_frame_name_ = "tool0";                          // end-effector frame in URDF

    joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };
    // -------------------------------------------------

    arm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&UR5TeleopNode::joyCallback, this, std::placeholders::_1));

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&UR5TeleopNode::jointStateCallback, this, std::placeholders::_1));

    // Load UR5 model
    // Change package + path to your URDF
    const std::string pkg = ament_index_cpp::get_package_share_directory("ur5_mujoco");
    const std::string urdf = pkg + "/urdf/ur5.urdf";

    pinocchio::urdf::buildModel(urdf, model_);
    data_ = pinocchio::Data(model_);

    ee_frame_ = model_.getFrameId(ee_frame_name_);
    if (ee_frame_ == (size_t)-1) {
      throw std::runtime_error("Frame '" + ee_frame_name_ + "' not found in model");
    }

    q_min_ = model_.lowerPositionLimit;
    q_max_ = model_.upperPositionLimit;

    q_ = pinocchio::neutral(model_);
    // Make sure q_ has at least 6 DoF; if your URDF includes extra joints, we still only command first 6.
    if ((int)q_.size() < 6) {
      throw std::runtime_error("Model q size < 6; UR5 model seems wrong.");
    }

    timer_ = create_wall_timer(10ms, std::bind(&UR5TeleopNode::teleopStep, this));
    RCLCPP_INFO(get_logger(), "UR5 teleop node started (persistent EE target, LeRobot-style deltas)");
  }

private:
  // ------------------ joystick ------------------
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_ = *msg;
    have_joy_ = true;
  }

  // ------------------ joint states ------------------
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // map joint_state positions into q_.head(6)
    for (int i = 0; i < 6; ++i) {
      for (size_t j = 0; j < msg->name.size(); ++j) {
        if (msg->name[j] == joint_names_[i]) {
          if (j < msg->position.size()) q_[i] = msg->position[j];
          break;
        }
      }
    }
    have_js_ = true;
  }

  // ------------------ SO(3) projection ------------------
  Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& R) const
  {
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
  void teleopStep()
  {
    if (!have_js_ || !have_joy_) return;

    // 0) Hold current joint position once (startup)
    if (!holding_position_) {
      publishArm(q_.head(6));
      holding_position_ = true;
      return;
    }

    // 1) Initialize persistent target ONCE
    if (!ee_target_initialized_) {
      pinocchio::forwardKinematics(model_, data_, q_);
      pinocchio::updateFramePlacements(model_, data_);
      const auto& T = data_.oMf[ee_frame_];
      ee_target_pos_ = T.translation();
      ee_target_rot_ = T.rotation();
      ee_target_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Initialized UR5 EE target from current pose.");
    }

    // 2) Read enable + mode
    // Same mapping as your Panda: LB = dead-man, RB = orientation mode
    const bool enable =
      (last_joy_.buttons.size() > 4 && last_joy_.buttons[4]);

    const bool orient_mode =
      (last_joy_.buttons.size() > 5 && last_joy_.buttons[5]);

    // 3) Tunables
    constexpr double DZ       = 0.10;
    constexpr double pos_step = 0.005;  // meters per tick
    constexpr double rot_step = 0.015;  // rad per tick
    constexpr double alpha    = 0.35;

    // 4) Sticks
    double lsx = (last_joy_.axes.size() > 0) ? last_joy_.axes[0] : 0.0;
    double lsy = (last_joy_.axes.size() > 1) ? last_joy_.axes[1] : 0.0;
    double rsx = (last_joy_.axes.size() > 3) ? last_joy_.axes[3] : 0.0;
    double rsy = (last_joy_.axes.size() > 4) ? last_joy_.axes[4] : 0.0;

    lsx = deadzone(lsx, DZ);
    lsy = deadzone(lsy, DZ);
    rsx = deadzone(rsx, DZ);
    rsy = deadzone(rsy, DZ);

    // 5) Produce deltas (LeRobot-style)
    double dx = 0.0, dy = 0.0, dz = 0.0;
    double dyaw = 0.0, dpitch = 0.0;

    if (!enable) {
      dx_filt_ = dy_filt_ = dz_filt_ = 0.0;
      dyaw_filt_ = dpitch_filt_ = 0.0;
    } else {
      if (!orient_mode) {
        dx =  pos_step * (lsy);
        dy =  pos_step * (lsx);
        dz =  pos_step * (-rsy);

        dx_filt_ = ema(dx_filt_, dx, alpha);
        dy_filt_ = ema(dy_filt_, dy, alpha);
        dz_filt_ = ema(dz_filt_, dz, alpha);
      } else {
        dyaw   = rot_step * (rsx);
        dpitch = rot_step * (-rsy);

        dyaw_filt_   = ema(dyaw_filt_, dyaw, alpha);
        dpitch_filt_ = ema(dpitch_filt_, dpitch, alpha);
      }
    }

    // 6) Integrate into persistent target
    ee_target_pos_.x() += dx_filt_;
    ee_target_pos_.y() += dy_filt_;
    ee_target_pos_.z() += dz_filt_;

    // Clamp z for safety (tune to your world)
    ee_target_pos_.z() = std::clamp(ee_target_pos_.z(), 0.05, 1.20);

    Eigen::Matrix3d R_delta =
      Eigen::AngleAxisd(dyaw_filt_,   Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(dpitch_filt_, Eigen::Vector3d::UnitY()).toRotationMatrix();

    ee_target_rot_ = R_delta * ee_target_rot_;
    ee_target_rot_ = projectToSO3(ee_target_rot_);

    // 7) IK + publish (even with dead-man off => holds last target)
    solveIK_SE3(ee_target_pos_, ee_target_rot_);
    publishArm(q_.head(6));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "LB=%d RB=%d | MODE=%s | tgt=(%.3f %.3f %.3f)",
      (int)enable, (int)orient_mode,
      orient_mode ? "ORIENT" : "TRANS",
      ee_target_pos_.x(), ee_target_pos_.y(), ee_target_pos_.z()
    );
  }

  // ------------------ IK ------------------
  void solveIK_SE3(const Eigen::Vector3d& target_pos,
                   const Eigen::Matrix3d& target_rot)
  {
    constexpr double lambda = 1e-3;
    constexpr double w_pos  = 1.0;
    constexpr double w_rot  = 0.4;

    for (int it = 0; it < 60; ++it) {
      pinocchio::forwardKinematics(model_, data_, q_);
      pinocchio::computeJointJacobians(model_, data_, q_);
      pinocchio::updateFramePlacements(model_, data_);

      const auto& T = data_.oMf[ee_frame_];

      const Eigen::Vector3d pos_err = target_pos - T.translation();
      const Eigen::Matrix3d R_err   = target_rot * T.rotation().transpose();
      const Eigen::Vector3d rot_err = pinocchio::log3(R_err);

      if (pos_err.norm() < 1e-4 && rot_err.norm() < 1e-4) break;

      Eigen::MatrixXd J_full(6, model_.nv);
      pinocchio::computeFrameJacobian(
        model_, data_, q_, ee_frame_,
        pinocchio::LOCAL_WORLD_ALIGNED, J_full);

      Eigen::MatrixXd J = J_full.leftCols(6);

      Eigen::VectorXd err(6);
      err.head<3>() = w_pos * pos_err;
      err.tail<3>() = w_rot * rot_err;

      Eigen::MatrixXd A = (J * J.transpose())
                        + lambda * Eigen::MatrixXd::Identity(6, 6);

      Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(err);

      dq = dq.cwiseMax(-0.05).cwiseMin(0.05);
      q_.head(6) += dq;

      q_.head(6) =
        q_.head(6)
          .cwiseMax(q_min_.head(6))
          .cwiseMin(q_max_.head(6));
    }
  }

  // ------------------ publish ------------------
  void publishArm(const Eigen::VectorXd& q6)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(q6.data(), q6.data() + 6);
    arm_pub_->publish(msg);

    q_.head(6) = q6;
  }

private:
  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Topics/frames/joints
  std::string command_topic_;
  std::string ee_frame_name_;
  std::vector<std::string> joint_names_;

  // Teleop state
  sensor_msgs::msg::Joy last_joy_;
  bool have_joy_{false};
  bool have_js_{false};
  bool holding_position_{false};
  bool ee_target_initialized_{false};

  // Filters
  double dx_filt_{0.0}, dy_filt_{0.0}, dz_filt_{0.0};
  double dyaw_filt_{0.0}, dpitch_filt_{0.0};

  // Pinocchio
  pinocchio::Model model_;
  pinocchio::Data data_;
  size_t ee_frame_{(size_t)-1};

  Eigen::VectorXd q_;
  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;

  // Persistent EE target
  Eigen::Vector3d ee_target_pos_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d ee_target_rot_{Eigen::Matrix3d::Identity()};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5TeleopNode>());
  rclcpp::shutdown();
  return 0;
}
