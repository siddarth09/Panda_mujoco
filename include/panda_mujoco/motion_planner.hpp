/*
This code uses Open Motion Planning Library (OMPL) to form a collision-free
joint-space path and return it as a sequence of joint vectors.

Pipeline:
Define a space → set bounds → define validity →
choose planner → set start/goal → solve → simplify →
sample states → return path.
*/

#pragma once // Header guard (prevents multiple inclusion)

#include <ompl/base/State.h>                        // OMPL base state
#include <ompl/base/spaces/RealVectorStateSpace.h> // Defines joint space
#include <ompl/geometric/SimpleSetup.h>             // Manages planner, problem, solution
#include <ompl/geometric/planners/rrt/RRTConnect.h> // RRT-Connect algorithm

#include <memory>      // Smart pointers
#include <functional>  // std::function
#include <vector>      // Dynamic arrays
#include <algorithm>

#include <Eigen/Dense> // Linear algebra library

class PandaMotionPlanner
{
public:
  /*
    OMPL itself does NOT know what "collision" means.
    It only asks a single question:

      → Is this state valid?

    Therefore, collision checking (and joint-limit checking)
    must be supplied from outside as a callback.
  */
  using StateValidityFn =
    std::function<bool(const ompl::base::State*)>;

  // Constructor
  PandaMotionPlanner(const Eigen::VectorXd& q_min,
                     const Eigen::VectorXd& q_max,
                     StateValidityFn validity_fn)
    : q_min_(q_min), q_max_(q_max)
  {
    using namespace ompl; // Convenience

    // Creating a 7-DOF joint-space for Panda
    auto space = std::make_shared<base::RealVectorStateSpace>(7);

    // Setting joint limits so that OMPL understands bounds
    base::RealVectorBounds bounds(7);
    for (int i = 0; i < 7; ++i) {
      bounds.setLow(i, q_min_[i]);
      bounds.setHigh(i, q_max_[i]);
    }
    space->setBounds(bounds);

    // Creates the planning pipeline around the space
    ss_ = std::make_unique<geometric::SimpleSetup>(space);

    // Given a point in joint space, is it valid?
    // (joint limits + collision checking happen in the callback)
    ss_->setStateValidityChecker(validity_fn);

    // Choosing the planner (RRT-Connect)
    ss_->setPlanner(
      std::make_shared<geometric::RRTConnect>(
        ss_->getSpaceInformation()));
  }

  /*
    Plan from q_start → q_goal and return a dense joint-space path
  */
  bool plan(const Eigen::VectorXd& q_start,
            const Eigen::VectorXd& q_goal,
            std::vector<Eigen::VectorXd>& path_out,
            double planning_time_sec = 1.0)
  {
    using namespace ompl;

    path_out.clear();

    // Helper that allocates OMPL states and frees them automatically
    base::ScopedState<base::RealVectorStateSpace>
      start(ss_->getStateSpace()),
      goal(ss_->getStateSpace());

    // Copy Eigen vectors into OMPL states
    for (int i = 0; i < 7; ++i) {
      start[i] = q_start[i];
      goal[i]  = q_goal[i];
    }

    ss_->clear();
    ss_->setStartAndGoalStates(start, goal);

    // Solve the planning problem
    if (!ss_->solve(planning_time_sec))
      return false;

    // Shorten and smooth the path
    ss_->simplifySolution();

    // Retrieve the solution path
    auto path = ss_->getSolutionPath();

    // Interpolate so that we get many intermediate states
    path.interpolate(1000);

    // Copy OMPL states into Eigen vectors
    for (std::size_t k = 0; k < path.getStateCount(); ++k) {
      const auto* st =
        path.getState(k)
          ->as<base::RealVectorStateSpace::StateType>();

      Eigen::VectorXd q(7);
      for (int i = 0; i < 7; ++i)
        q[i] = (*st)[i];

      path_out.push_back(q);
    }

    return !path_out.empty();
  }

private:
  Eigen::VectorXd q_min_, q_max_;
  std::unique_ptr<ompl::geometric::SimpleSetup> ss_;
};
