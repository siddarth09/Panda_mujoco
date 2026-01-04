#include "panda_mujoco/panda_ik.hpp"

#include <chrono>
#include <iostream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <panda_mujoco/msg/cube_detector.hpp>

// ------------------ utility ------------------
static Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& M)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    M, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
  if (R.determinant() < 0.0) {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    R = U * svd.matrixV().transpose();
  }
  return R;
}

// ------------------ constructor ------------------
PandaIKNode::PandaIKNode()
: rclcpp::Node("panda_ik_node")
{
  // Arm controller
  arm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arm_controller/commands", 10);

  // Gripper controller
  gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/gripper_controller/commands", 10);

  // Cube pose subscriber
  cube_sub_ = create_subscription<panda_mujoco::msg::CubeDetector>(
    "/cube_table_to_base", 10,
    std::bind(&PandaIKNode::cubeCallback, this, std::placeholders::_1));

  js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
  "/joint_states", 10,
  std::bind(&PandaIKNode::jointStateCallback, this, std::placeholders::_1));


  // Load Panda model
  const std::string pkg =
    ament_index_cpp::get_package_share_directory("panda_mujoco");
  const std::string urdf = pkg + "/franka_emika_panda/panda.urdf";

  pinocchio::urdf::buildModel(urdf, model_);
  data_ = pinocchio::Data(model_);

  pinocchio::urdf::buildGeom(
    model_, urdf, pinocchio::COLLISION, geom_model_);
  geom_model_.addAllCollisionPairs();
  geom_data_ = pinocchio::GeometryData(geom_model_);

  ee_frame_ = model_.getFrameId("panda_ee");
  if (ee_frame_ == (size_t)-1)
    throw std::runtime_error("panda_ee not found");

  q_min_ = model_.lowerPositionLimit;
  q_max_ = model_.upperPositionLimit;
  q_ = pinocchio::neutral(model_);

  planner_ = std::make_unique<PandaMotionPlanner>(
    q_min_.head(7),
    q_max_.head(7),
    [this](const ompl::base::State* st) {
      Eigen::VectorXd q7 = omplStateToEigen(st);
      return isCollisionFree(q7);
    });

  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&PandaIKNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "Panda IK + Gripper node started");
}

// ------------------ cube callback ------------------
void PandaIKNode::cubeCallback(
  const panda_mujoco::msg::CubeDetector::SharedPtr msg)
{
  
  if (msg->color == "blue")
    return;

 
  if (msg->position.z < 0.2)
    return;

  constexpr double HALF_CUBE = 0.025;  // 5cm cube

  Eigen::Vector3d pos(
    msg->position.x,
    msg->position.y,
    msg->position.z - HALF_CUBE);

  for (const auto& c : cube_buffer_) {
    if ((c.pos - pos).norm() < 0.05)  // 5 cm
      return;
  }

  RCLCPP_INFO(get_logger(), "Cube received: %s",
            msg->color.c_str());

  cube_buffer_.push_back({
    msg->color,
    pos,
    msg->header.stamp
  });
  have_cube_ = true;
}

bool PandaIKNode::selectNextCube(Eigen::Vector3d& out_pos, std::string& color)
{
  if (cube_buffer_.empty())
    return false;

  auto it = std::min_element(
    cube_buffer_.begin(), cube_buffer_.end(),
    [](const CubeInfo& a, const CubeInfo& b) {
      return a.pos.norm() < b.pos.norm();
    });

  out_pos = it->pos;
  color = it->color;

  cube_buffer_.erase(it);

  return true;
}


// ------------------ control loop ------------------
void PandaIKNode::controlLoop()
{
  if (!have_cube_ || !have_js_ || picking_)
    return;

  // ---------- CONSTANTS ----------
  constexpr double PLACE_X = 0.10;
  constexpr double PLACE_Y = 0.10;

  // Empirical TF / camera bias
  constexpr double XY_BIAS_X = 0.06;
  constexpr double XY_BIAS_Y = -0.05;

  constexpr double CUBE_HEIGHT      = 0.05;
  constexpr double CUBE_HALF_HEIGHT = 0.025;
  constexpr double GRASP_CLEARANCE  = 0.002;

  constexpr double TABLE_Z = 0.435;   // derived from observations

  static int stack_count = 0;

  // ---------- SELECT CUBE ----------
  Eigen::Vector3d target;
  std::string color;

  if (!selectNextCube(target, color))
    return;

  RCLCPP_INFO(
    get_logger(),
    "Selected cube %s at [%.3f %.3f %.3f]",
    color.c_str(), target.x(), target.y(), target.z());

  picking_ = true;
  cube_pos_ = target;

  // ---------- PICK ----------
  openGripper();
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // 1) Hover above cube
  Eigen::Vector3d pregrasp =
      cube_pos_ + Eigen::Vector3d(0, 0, 0.03);
  RCLCPP_INFO(get_logger(), "Hovering above cube");
  planAndExecute(pregrasp);

  // 2) Descend to grasp depth
  Eigen::Vector3d grasp =
      cube_pos_
      - Eigen::Vector3d(0, XY_BIAS_Y, CUBE_HALF_HEIGHT - GRASP_CLEARANCE);

  RCLCPP_INFO(get_logger(), "Descending to grasp");
  planAndExecute(grasp);

  // 3) Close gripper
  RCLCPP_INFO(get_logger(), "Closing gripper");
  closeGripper();
  rclcpp::sleep_for(std::chrono::milliseconds(400));

  // 4) Lift cube
  Eigen::Vector3d lift =
      cube_pos_ + Eigen::Vector3d(0, 0, 0.20);
  RCLCPP_INFO(get_logger(), "Lifting cube");
  planAndExecute(lift);

  // ---------- PLACE ----------
  Eigen::Vector3d place_hover(
    PLACE_X + XY_BIAS_X,
    PLACE_Y + XY_BIAS_Y,
    TABLE_Z + 0.20 + stack_count * CUBE_HEIGHT);

  Eigen::Vector3d place_down(
    PLACE_X + XY_BIAS_X,
    PLACE_Y + XY_BIAS_Y,
    TABLE_Z + CUBE_HALF_HEIGHT + stack_count * CUBE_HEIGHT);

  // 5) Move above stack
  RCLCPP_INFO(get_logger(), "Moving to place hover");
  planAndExecute(place_hover);

  // 6) Lower onto stack
  RCLCPP_INFO(get_logger(), "Placing cube");
  planAndExecute(place_down);

  // 7) Open gripper
  openGripper();
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // 8) Retreat upward
  planAndExecute(place_hover);

  stack_count++;

  RCLCPP_INFO(get_logger(), "Pick & place complete (stack height = %d)", stack_count);

  picking_ = false;
}




// ------------------ planning ------------------
void PandaIKNode::planAndExecute(const Eigen::Vector3d& target)
{
  Eigen::VectorXd q_start = q_.head(7);
  solveIK_SE3(target);
  Eigen::VectorXd q_goal = q_.head(7);

  std::vector<Eigen::VectorXd> path;
  if (!planner_->plan(q_start, q_goal, path, 1.0)) {
    RCLCPP_WARN(get_logger(), "Planning failed");
    return;
  }

  auto exec = densifyPathMaxStep(path, 0.02);
  for (const auto& q : exec) {
    publishArm(q);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

// ------------------ IK ------------------
void PandaIKNode::solveIK_SE3(const Eigen::Vector3d& target)
{
  const double lambda = 1e-3;
  const Eigen::Matrix3d R_des =
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

  for (int i = 0; i < 100; ++i)
  {
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    const auto& T = data_.oMf[ee_frame_];

    Eigen::Vector3d pos_err = target - T.translation();

    Eigen::Matrix3d R_err =
      (R_des * T.rotation().transpose()).eval();
    Eigen::Vector3d rot_err = pinocchio::log3(R_err);

    if (pos_err.norm() < 1e-4 && rot_err.norm() < 1e-4)
      break;

    // Full Jacobian
    Eigen::MatrixXd J_full(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q_, ee_frame_,
      pinocchio::LOCAL_WORLD_ALIGNED, J_full);

    // Arm-only Jacobian (ignore fingers)
    Eigen::MatrixXd J = J_full.leftCols(7);

    Eigen::VectorXd err(6);
    err.head<3>() = pos_err;
    err.tail<3>() = 0.3 * rot_err;

    Eigen::VectorXd dq =
      J.transpose() *
      (J * J.transpose()
       + lambda * Eigen::MatrixXd::Identity(6, 6))
        .ldlt().solve(err);

    q_.head(7) += dq;

    // Joint limits (arm only)
    q_.head(7) = q_.head(7)
      .cwiseMax(q_min_.head(7))
      .cwiseMin(q_max_.head(7));
  }
}


// ------------------ publishing ------------------
void PandaIKNode::publishArm(const Eigen::VectorXd& q7)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(q7.data(), q7.data() + 7);
  arm_pub_->publish(msg);
  q_.head(7) = q7;
}

void PandaIKNode::publishGripper(double finger_pos)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = { finger_pos };
  gripper_pub_->publish(msg);
}

void PandaIKNode::closeGripper()
{
  publishGripper(gripper_closed_);
}


void PandaIKNode::openGripper()
{
  publishGripper(gripper_open_);
}

void PandaIKNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Map joint names to indices once (recommended), but here’s a simple approach:
  auto find_index = [&](const std::string& name)->int {
    for (size_t i=0;i<msg->name.size();++i) if (msg->name[i]==name) return (int)i;
    return -1;
  };

  static const std::vector<std::string> arm_names = {
    "joint1","joint2","joint3","joint4","joint5","joint6","joint7"
  };

  for (int i=0;i<7;++i) {
    int idx = find_index(arm_names[i]);
    if (idx >= 0 && idx < (int)msg->position.size())
      q_[i] = msg->position[idx];
  }

  have_js_ = true;
}

// ------------------ collision ------------------
bool PandaIKNode::isCollisionFree(const Eigen::VectorXd& q7)
{
  Eigen::VectorXd q = q_;
  q.head(7) = q7;

  pinocchio::computeCollisions(
    model_, data_, geom_model_, geom_data_, q, true);

  for (auto& c : geom_data_.collisionResults)
    if (c.isCollision()) return false;

  return true;
}

Eigen::VectorXd PandaIKNode::omplStateToEigen(
    const ompl::base::State* st) const
{
  const auto* rv =
    st->as<ompl::base::RealVectorStateSpace::StateType>();

  Eigen::VectorXd q7(7);
  for (int i = 0; i < 7; ++i)
    q7[i] = rv->values[i];

  return q7;
}
// ------------------ main ------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PandaIKNode>());
  rclcpp::shutdown();
  return 0;
}
