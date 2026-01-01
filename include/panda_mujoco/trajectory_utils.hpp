#pragma once

#include <Eigen/Dense>
#include <vector>

// Densify a joint-space path so that no joint moves more than
// max_step_rad between consecutive waypoints.
std::vector<Eigen::VectorXd> densifyPathMaxStep(
  const std::vector<Eigen::VectorXd>& path,
  double max_step_rad = 0.02);

// Compute repeat counts for fixed-rate execution given joint velocity limits.
std::vector<int> timeScaleToFixedRate(
  const std::vector<Eigen::VectorXd>& path,
  const Eigen::VectorXd& dq_max,   // rad/s per joint
  double control_hz,
  double min_dt = 1e-3);
