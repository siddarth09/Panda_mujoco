#include "panda_mujoco/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>

std::vector<Eigen::VectorXd> densifyPathMaxStep(
  const std::vector<Eigen::VectorXd>& path,
  double max_step_rad)
{
  if (path.size() < 2) return path;

  std::vector<Eigen::VectorXd> out;
  out.reserve(path.size() * 5);
  out.push_back(path.front());

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::VectorXd& a = path[i];
    const Eigen::VectorXd& b = path[i + 1];

    Eigen::VectorXd d = (b - a).cwiseAbs();
    double max_d = d.maxCoeff();

    int steps = std::max(1, static_cast<int>(
      std::ceil(max_d / max_step_rad)));

    for (int s = 1; s <= steps; ++s) {
      double t = static_cast<double>(s) / steps;
      out.push_back((1.0 - t) * a + t * b);
    }
  }
  return out;
}

std::vector<int> timeScaleToFixedRate(
  const std::vector<Eigen::VectorXd>& path,
  const Eigen::VectorXd& dq_max,
  double control_hz,
  double min_dt)
{
  if (path.size() < 2) return {};

  // Validate dq_max size matches path dimensions
  if (dq_max.size() != path[0].size()) {
    return {};  // Return empty vector on dimension mismatch
  }

  std::vector<int> repeats(path.size(), 1);
  const double dt_tick = 1.0 / control_hz;

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    Eigen::VectorXd dq = path[i + 1] - path[i];
    double dt = min_dt;

    for (int j = 0; j < dq.size(); ++j) {
      double req = std::abs(dq[j]) / std::max(1e-6, dq_max[j]);
      dt = std::max(dt, req);
    }

    repeats[i] = std::max(
      1, static_cast<int>(std::ceil(dt / dt_tick)));
  }

  repeats.back() = 1;
  return repeats;
}
