#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <Eigen/Dense>

class PandaIKNode : public rclcpp::Node
{
public:
  PandaIKNode() : Node("panda_ik_node")
    {
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_controller/commands", 10);

    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("panda_mujoco");

    std::string urdf =
        pkg_share + "/franka_emika_panda/panda.urdf";

    pinocchio::urdf::buildModel(urdf, model_);
    
    data_ = pinocchio::Data(model_);

    const std::string ee_name = "hand";
    ee_frame_ = model_.getFrameId(ee_name);

    if (ee_frame_ == (pinocchio::FrameIndex)-1)
    {
    RCLCPP_ERROR(this->get_logger(), "EE frame '%s' not found. Available frames:", ee_name.c_str());
    for (const auto &f : model_.frames)
    {
        RCLCPP_ERROR(this->get_logger(), "  %s", f.name.c_str());
    }
    throw std::runtime_error("Invalid end-effector frame name");
    }
    if (ee_frame_ == (pinocchio::FrameIndex)-1)
    {
        RCLCPP_FATAL(this->get_logger(), "EE frame not found");
        throw std::runtime_error("Invalid EE frame");
    }

    RCLCPP_INFO(this->get_logger(),
                "Model loaded. nq=%d nv=%d nframes=%zu",
                model_.nq, model_.nv, model_.frames.size());

    q_ = pinocchio::neutral(model_);
    RCLCPP_INFO(this->get_logger(), "Neutral q size=%ld", (long)q_.size());

    for (const auto &f : model_.frames)
    {
    RCLCPP_INFO(
        this->get_logger(),
        "Frame: %-25s | parent=%d | type=%d",
        f.name.c_str(),
        f.parent,
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
    Eigen::Vector3d target(0.5,0.18,0.03);
    solveIK(target);

    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(q_.data(), q_.data() + q_.size());
    pub_->publish(msg);
  }

  void solveIK(const Eigen::Vector3d &target)
  {
    for (int i = 0; i < 100; ++i)
    {
      pinocchio::forwardKinematics(model_, data_, q_);
      pinocchio::updateFramePlacements(model_, data_);

      const auto &T = data_.oMf[ee_frame_];
      Eigen::Vector3d err = target - T.translation();

      if (err.norm() < 1e-4)
        break;

      Eigen::MatrixXd J(6, model_.nv);
      pinocchio::computeFrameJacobian(
        model_, data_, q_, ee_frame_,
        pinocchio::WORLD, J);

      Eigen::VectorXd dq =
        J.topRows<3>().completeOrthogonalDecomposition().solve(err);

      q_ += 0.1 * dq;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  pinocchio::Model model_;
  pinocchio::Data data_;

  Eigen::VectorXd q_;
  pinocchio::FrameIndex ee_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PandaIKNode>());
  rclcpp::shutdown();
  return 0;
}
