//
// Created by giuseppe on 20.07.21.
//

#include <ros/ros.h>
#include <map>

#include "safety_filter/constraints/joint_limits.hpp"
#include "safety_filter/constraints/passivity_constraint.hpp"
#include "safety_filter/filter/filter.hpp"

namespace manipulation {

class PandaMobileJointLimitsConstraints
    : public safety_filter::JointLimitsConstraints {
 public:
  // TODO(giuseppe) does not account for non-holonomic case
  PandaMobileJointLimitsConstraints(
      const size_t nc, const size_t nx,
      const safety_filter::JointLimitsConstraintsSetting& settings);

  void update_jacobian(const Eigen::VectorXd& x) override;
};

struct PandaMobileSafetyFilterSettings {
  Eigen::Matrix<double, 10, 1> u_min;
  Eigen::Matrix<double, 10, 1> u_max;
  Eigen::Matrix<double, 10, 1> q_min;
  Eigen::Matrix<double, 10, 1> q_max;

  double max_reach = 0.8;
  double min_dist = 0.15;

  bool joint_limits = true;
  bool input_limits = true;
  bool cartesian_limits = false;

  bool passivity_constraint = false;
  double tank_initial_energy = 1.0;
  double tank_lower_energy_bound = 1e-3;
  double tank_integration_dt = 0.01;

  bool verbose = false;

  bool init_from_ros(ros::NodeHandle& nh);
};

class PandaMobileSafetyFilter {
 public:
  using passivity_ptr_t = std::shared_ptr<safety_filter::PassivityConstraint>;

  PandaMobileSafetyFilter(const std::string& urdf_string,
                          const PandaMobileSafetyFilterSettings& settings);

  void update(const Eigen::VectorXd& x,
                                       const Eigen::VectorXd& u,
                                       const Eigen::VectorXd& torque);
  bool apply(Eigen::VectorXd& u_opt);

  // TODO(giuseppe) brittle implementation -> what if this is not created?
  inline passivity_ptr_t& passivity_constraint() {
    return passivity_constraint_ptr_;
  }

 private:
  Eigen::VectorXd x_;
  Eigen::VectorXd u_;
  Eigen::VectorXd torque_;

  PandaMobileSafetyFilterSettings settings_;
  std::shared_ptr<safety_filter::SafetyFilter> filter_;
  passivity_ptr_t passivity_constraint_ptr_;
};

}  // namespace panda_mobile