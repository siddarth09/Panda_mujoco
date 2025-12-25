#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>  // for log3

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

class PandaIKNode : public rclcpp::Node
{
public:
  PandaIKNode() : Node("panda_ik_node")
  {
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_controller/commands", 10);

    const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("panda_mujoco");
    const std::string urdf_path =
      pkg_share + "/franka_emika_panda/panda.urdf";

    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    RCLCPP_INFO(
      this->get_logger(),
      "Model loaded. nq=%d nv=%d nframes=%zu",
      model_.nq, model_.nv, model_.frames.size());

    // End-effector frame
    const std::string ee_name = "hand";
    ee_frame_ = model_.getFrameId(ee_name);
    if (ee_frame_ == (pinocchio::FrameIndex)-1) {
      RCLCPP_ERROR(
        this->get_logger(),
        "EE frame '%s' not found. Available frames:",
        ee_name.c_str());
      for (const auto & f : model_.frames) {
        RCLCPP_ERROR(this->get_logger(), "  %s", f.name.c_str());
      }
      throw std::runtime_error("Invalid end-effector frame name");
    }

    // Joint limits
    q_min_ = model_.lowerPositionLimit;
    q_max_ = model_.upperPositionLimit;

    // Start at neutral
    q_ = pinocchio::neutral(model_);
    RCLCPP_INFO(this->get_logger(), "Neutral q size=%ld", (long)q_.size());

    // Optional: print frames (fix parent format)
    for (const auto & f : model_.frames) {
      RCLCPP_INFO(
        this->get_logger(),
        "Frame: %-25s | parent=%lu | type=%d",
        f.name.c_str(),
        static_cast<unsigned long>(f.parent),
        static_cast<int>(f.type));
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PandaIKNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Panda IK node started");
  }

private:
  void controlLoop()
  {
    
    const Eigen::Vector3d target(0.70, 0.18, 0.415);

    solveIK_SE3(target);

    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(q_.data(), q_.data() + q_.size());
    pub_->publish(msg);
  }

  void solveIK_SE3(const Eigen::Vector3d & target_pos)
  {
    // Tuning knobs
    const double lambda   = 1e-3;   // damping
    const double alpha    = 0.5;    // step size
    const double tol_pos  = 1e-4;   // position tolerance
    const double tol_rot  = 1e-4;   // rotation tolerance (rad)
    const double max_step = 0.10;   // max rad per joint per iter

    // Desired EE orientation (example): flip about X so tool points down-ish.
    const Eigen::Matrix3d R_des =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

    for (int iter = 0; iter < 100; ++iter)
    {
      // Required update order in Pinocchio
      pinocchio::forwardKinematics(model_, data_, q_);
      pinocchio::computeJointJacobians(model_, data_, q_);
      pinocchio::updateFramePlacements(model_, data_);

      const auto & T = data_.oMf[ee_frame_];  // EE pose in world

      // Position error
      const Eigen::Vector3d pos_err = target_pos - T.translation();

      // Orientation error via SO(3) log map
      const Eigen::Matrix3d R_err_mat = R_des * T.rotation().transpose();
      const Eigen::Vector3d rot_err = pinocchio::log3(R_err_mat);

      // Convergence check
      if (pos_err.norm() < tol_pos && rot_err.norm() < tol_rot) {
        break;
      }

      // Frame Jacobian (6x7) in a stable reference frame
      Eigen::MatrixXd J(6, model_.nv);
      pinocchio::computeFrameJacobian(
        model_, data_, q_, ee_frame_,
        pinocchio::LOCAL_WORLD_ALIGNED, J);

      // 6D task error (weight rotation)
      Eigen::VectorXd err6(6);
      err6.head<3>() = pos_err;
      err6.tail<3>() = 0.3 * rot_err;  // orientation weight

      // Damped Least Squares: dq = J^T (J J^T + λI)^-1 e
      Eigen::MatrixXd A = (J * J.transpose())
                        + lambda * Eigen::MatrixXd::Identity(6, 6);

      Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(err6);

      // Clamp per-joint step
      for (int k = 0; k < dq.size(); ++k) {
        dq[k] = std::clamp(dq[k], -max_step, max_step);
      }

      // Integrate & clamp to limits
      q_ += alpha * dq;
      q_ = q_.cwiseMax(q_min_).cwiseMin(q_max_);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_{0};

  Eigen::VectorXd q_;
  Eigen::VectorXd q_min_, q_max_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PandaIKNode>());
  rclcpp::shutdown();
  return 0;
}
