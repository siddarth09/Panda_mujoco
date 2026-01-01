#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pinocchio/fwd.hpp>          // forward declarations for pinocchio::Model/Data
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include "pinocchio/collision/collision.hpp"
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "panda_mujoco/trajectory_utils.hpp"
#include "panda_mujoco/motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>

#include <memory>
#include <vector>

class PandaMotionPlanner;

class PandaIKNode : public rclcpp::Node
{
public:
  PandaIKNode();

  

private:
    void controlLoop();
    void solveIK_SE3(const Eigen::Vector3d & target_pos);
    void publishJointCommand(const Eigen::VectorXd& q7);
        // Convert OMPL state to joint vector (7 DoF arm)
    Eigen::VectorXd omplStateToEigen(const ompl::base::State* st) const;
    Eigen::VectorXd armToFullQ(const Eigen::VectorXd& q7) const;
    bool isCollisionFree(const Eigen::VectorXd& q);
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_{0};

    Eigen::VectorXd q_;
    Eigen::VectorXd q_min_, q_max_;

    std::unique_ptr<PandaMotionPlanner> planner_;

    std::vector<Eigen::VectorXd> exec_path_;
    std::vector<int> exec_repeats_;
    size_t exec_idx_{0};
    int exec_tick_{0};
    bool executing_{false};


    pinocchio::GeometryModel geom_model_;
    pinocchio::GeometryData geom_data_;
  

    double control_hz_{100.0};
};

