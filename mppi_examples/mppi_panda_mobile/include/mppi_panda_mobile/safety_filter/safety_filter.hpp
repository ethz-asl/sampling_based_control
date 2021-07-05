//
// Created by giuseppe on 03.07.21.
//

#include <ros/ros.h>
#include <map>

#include "safety_filter/constraints/cartesian_limit.hpp"
#include "safety_filter/constraints/constraints_manager.hpp"
#include "safety_filter/constraints/input_limits.hpp"
#include "safety_filter/constraints/joint_limits.hpp"
#include "safety_filter/filter/filter.hpp"

namespace panda_mobile {

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

  bool verbose = false;

  bool init_from_ros(ros::NodeHandle& nh);
};

class PandaMobileSafetyFilter {
 public:
  PandaMobileSafetyFilter(const std::string& urdf_string,
                          const PandaMobileSafetyFilterSettings& settings);

  bool apply(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
             Eigen::VectorXd& u_opt);

 private:
  PandaMobileSafetyFilterSettings settings_;
  std::shared_ptr<safety_filter::SafetyFilter> filter_;
};

}  // namespace panda_mobile