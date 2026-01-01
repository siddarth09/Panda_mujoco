#include "panda_mujoco/panda_ik.hpp"



#include <chrono>
#include <iostream>

// Project a 3x3 matrix to the nearest valid rotation matrix (SO(3))
// This prevents log3() from asserting when numerical drift happens.
static Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& M)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    M, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
  // Ensure det(R)=+1
  if (R.determinant() < 0.0) {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    R = U * svd.matrixV().transpose();
  }
  return R;
}

PandaIKNode::PandaIKNode()
: rclcpp::Node("panda_ik_node")
{
  // ROS2 publisher (7 arm joints)
  pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arm_controller/commands", 10);

  // Load URDF into Pinocchio
  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("panda_mujoco");
  const std::string urdf_path =
    pkg_share + "/franka_emika_panda/panda.urdf";

  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);

  // Build collision geometry model from URDF
  pinocchio::urdf::buildGeom(
    model_,
    urdf_path,
    pinocchio::COLLISION,
    geom_model_);

  // IMPORTANT: generate collision pairs
  geom_model_.addAllCollisionPairs();
  geom_data_ = pinocchio::GeometryData(geom_model_);

  RCLCPP_INFO(
    this->get_logger(),
    "Model loaded. nq=%d nv=%d nframes=%zu",
    model_.nq, model_.nv, model_.frames.size());

  // End-effector frame
  const std::string ee_name = "panda_ee";
  ee_frame_ = model_.getFrameId(ee_name);
  if (ee_frame_ == (size_t)-1) {
    for (const auto & f : model_.frames) {
      RCLCPP_ERROR(this->get_logger(), "Frame: %s", f.name.c_str());
    }
    throw std::runtime_error("Invalid end-effector frame");
  }

  // Joint limits
  q_min_ = model_.lowerPositionLimit;
  q_max_ = model_.upperPositionLimit;

  // Start at neutral (size nq = 9)
  q_ = pinocchio::neutral(model_);

  // OMPL planner (7 arm joints only)
  planner_ = std::make_unique<PandaMotionPlanner>(
    q_min_.head(7),
    q_max_.head(7),
    [this](const ompl::base::State* st) {

      Eigen::VectorXd q7 = omplStateToEigen(st);

      // joint limits for arm
      for (int i = 0; i < 7; ++i)
        if (q7[i] < q_min_[i] || q7[i] > q_max_[i])
          return false;

      // collision check using full q (size 9)
      return isCollisionFree(q7);
    });

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / control_hz_)),
    std::bind(&PandaIKNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Panda IK node started");
}

void PandaIKNode::controlLoop()
{
  if (!executing_) {
    // Your target
    const Eigen::Vector3d target(0.85, -0.22, 0.415);

    Eigen::VectorXd q_start = q_.head(7);

    solveIK_SE3(target);
    Eigen::VectorXd q_goal = q_.head(7);

    std::vector<Eigen::VectorXd> raw_path;
    if (!planner_->plan(q_start, q_goal, raw_path, 1.0)) {
      RCLCPP_WARN(this->get_logger(), "OMPL planning failed");
      return;
    }

    exec_path_ = densifyPathMaxStep(raw_path, 0.02);

    Eigen::VectorXd dq_max(7);
    dq_max.setConstant(1.2);
    exec_repeats_ = timeScaleToFixedRate(exec_path_, dq_max, control_hz_);

    exec_idx_ = 0;
    exec_tick_ = 0;
    executing_ = true;

    RCLCPP_INFO(this->get_logger(),
      "Planned %zu waypoints", exec_path_.size());
  }

  if (exec_idx_ >= exec_path_.size()) {
    executing_ = false;
    return;
  }

  publishJointCommand(exec_path_[exec_idx_]);

  if (++exec_tick_ >= exec_repeats_[exec_idx_]) {
    exec_tick_ = 0;
    exec_idx_++;
  }
}

void PandaIKNode::solveIK_SE3(const Eigen::Vector3d & target_pos)
{
  // Damped least squares IK (arm only)
  const double lambda   = 1e-3;
  const double alpha    = 0.5;
  const double tol_pos  = 1e-4;
  const double tol_rot  = 1e-4;
  const double max_step = 0.10;

  // Desired orientation (flip around X)
  const Eigen::Matrix3d R_des =
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

  for (int iter = 0; iter < 100; ++iter) {

    // FK / jacobians
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    const auto & T = data_.oMf[ee_frame_];

    // --- Position error ---
    Eigen::Vector3d pos_err = target_pos - T.translation();

    // --- Rotation error (safe) ---
    Eigen::Matrix3d R_cur = T.rotation();
    Eigen::Matrix3d R_err = R_des * R_cur.transpose();
    R_err = projectToSO3(R_err); // prevent NaNs inside log3
    Eigen::Vector3d rot_err = pinocchio::log3(R_err);

    if (pos_err.norm() < tol_pos && rot_err.norm() < tol_rot)
      break;

    // Full Jacobian is 6 x nv (nv=9), but we ONLY use first 7 arm columns.
    Eigen::MatrixXd Jfull(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q_, ee_frame_,
      pinocchio::LOCAL_WORLD_ALIGNED, Jfull);

    Eigen::MatrixXd J = Jfull.leftCols(7); // arm-only Jacobian

    Eigen::VectorXd err6(6);
    err6.head<3>() = pos_err;
    err6.tail<3>() = 0.3 * rot_err;

    Eigen::MatrixXd A =
      J * J.transpose() +
      lambda * Eigen::MatrixXd::Identity(6, 6);

    Eigen::VectorXd dq7 =
      J.transpose() * A.ldlt().solve(err6);

    for (int k = 0; k < dq7.size(); ++k)
      dq7[k] = std::clamp(dq7[k], -max_step, max_step);

    // Update ONLY first 7 joints in q_ (keep fingers fixed)
    q_.head(7) += alpha * dq7;

    // clamp only the arm joints
    q_.head(7) = q_.head(7).cwiseMax(q_min_.head(7)).cwiseMin(q_max_.head(7));
  }
}

Eigen::VectorXd PandaIKNode::armToFullQ(const Eigen::VectorXd& q7) const
{
  Eigen::VectorXd q_full = q_;     // current full config (size 9)
  q_full.head<7>() = q7;           // overwrite arm joints
  return q_full;
}

bool PandaIKNode::isCollisionFree(const Eigen::VectorXd& q7)
{
  const Eigen::VectorXd q_full = armToFullQ(q7);

  // Pinocchio official API (C++):
  // computeCollisions(model, data, geom_model, geom_data, q, stopAtFirstCollision)
  pinocchio::computeCollisions(
    model_, data_, geom_model_, geom_data_, q_full,
    true // stop at first collision for speed
  );

  // Check results
  for (size_t i = 0; i < geom_data_.collisionResults.size(); ++i) {
    if (geom_data_.collisionResults[i].isCollision()) {
      return false;
    }
  }
  return true;
}

Eigen::VectorXd PandaIKNode::omplStateToEigen(
  const ompl::base::State* st) const
{
  const auto* rv =
    st->as<ompl::base::RealVectorStateSpace::StateType>();

  Eigen::VectorXd q(7);
  for (int i = 0; i < 7; ++i)
    q[i] = (*rv)[i];

  return q;
}

void PandaIKNode::publishJointCommand(const Eigen::VectorXd& q7)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(7);
  for (int i = 0; i < 7; ++i)
    msg.data[i] = q7[i];
  pub_->publish(msg);

  // keep internal state consistent with what we command
  q_.head(7) = q7;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PandaIKNode>());
  rclcpp::shutdown();
  return 0;
}
