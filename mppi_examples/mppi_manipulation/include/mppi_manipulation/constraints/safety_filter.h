//
// Created by giuseppe on 20.07.21.
//
#pragma once

#include <ros/ros.h>
#include <map>

#include <mppi/core/filter.h>

#include "safety_filter/constraints/joint_limits.hpp"
#include "safety_filter/filter/filter.hpp"

#include "mppi_manipulation/params/filter_params.h"

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

class PandaMobileSafetyFilter : public mppi::Filter {
 public:
  PandaMobileSafetyFilter(const FilterParams& settings);

  void update(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
              const double t);
  void update_violation(const Eigen::VectorXd& x);
  void reset_constraints();
  bool apply(Eigen::VectorXd& u_opt);

  void reset(const mppi::observation_t& x, const double t) override;
  void apply(const mppi::observation_t& x, mppi::input_t& u,
             const double t) override;

  inline const FilterParams get_settings() const { return params_; }
  inline const std::string get_urdf_string() const { return params_.urdf; }
  inline const std::unique_ptr<safety_filter::SafetyFilter>& get_filter() {
    return filter_;
  }

 public:
  std::map<std::string, std::shared_ptr<safety_filter::ConstraintBase>>
      constraints_;

 private:
  std::string urdf_;
  FilterParams params_;
  std::unique_ptr<safety_filter::SafetyFilter> filter_;
  Eigen::VectorXd u_f_;
};

}  // namespace manipulation