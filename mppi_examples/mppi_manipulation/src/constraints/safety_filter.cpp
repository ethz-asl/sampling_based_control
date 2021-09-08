//
// Created by giuseppe on 20.07.21.
//

#include "mppi_manipulation/constraints/safety_filter.h"
#include "mppi_manipulation/constraints/passivity.h"
#include "safety_filter/constraints/cartesian_limit.hpp"
#include "safety_filter/constraints/constraints_manager.hpp"
#include "safety_filter/constraints/first_derivative_limit.hpp"
#include "safety_filter/constraints/input_limits.hpp"
#include "safety_filter/constraints/second_derivative_limit.hpp"

using namespace safety_filter;
using namespace manipulation;

PandaMobileJointLimitsConstraints::PandaMobileJointLimitsConstraints(
    const size_t nc, const size_t nx,
    const JointLimitsConstraintsSetting& settings)
    : JointLimitsConstraints(nc, nx, settings){};

void PandaMobileJointLimitsConstraints::update_jacobian(
    const Eigen::VectorXd& x) {
  jacobian_.block(0, 0, 2, 2) << std::cos(x(2)), -std::sin(x(2)),
      std::sin(x(2)), std::cos(x(2));
}

PandaMobileSafetyFilter::PandaMobileSafetyFilter(const FilterParams& params)
    : params_(params) {
  std::cout << params_ << std::endl;
  ConstraintsManager cm(10);

  if (params_.input_limits) {
    safety_filter::InputLimitsSettings ul_settings;
    ul_settings.u_min = params_.u_min;
    ul_settings.u_max = params_.u_max;
    std::shared_ptr<ConstraintBase> ul_const =
        std::make_shared<InputLimits>(ul_settings);
    cm.add_constraint("input_limits", ul_const);
    constraints_["input_limits"] = ul_const;
  }

  if (params_.first_derivative_limits) {
    safety_filter::FirstDerivativeLimitSettings dl_settings;
    dl_settings.ud_min = params_.ud_min;
    dl_settings.ud_max = params_.ud_max;
    std::shared_ptr<ConstraintBase> dl_const =
        std::make_shared<FirstDerivativeLimits>(dl_settings);
    cm.add_constraint("first_derivative_limits", dl_const);
  }

  if (params_.second_derivative_limits) {
    safety_filter::SecondDerivativeLimitSettings ddl_settings;
    ddl_settings.udd_min = params_.udd_min;
    ddl_settings.udd_max = params_.udd_max;
    std::shared_ptr<ConstraintBase> ddl_const =
        std::make_shared<SecondDerivativeLimits>(ddl_settings);
    cm.add_constraint("second_derivative_limits", ddl_const);
  }

  if (params_.joint_limits) {
    JointLimitsConstraintsSetting jl_setting;
    jl_setting.q_min = params_.q_min;
    jl_setting.q_max = params_.q_max;
    std::shared_ptr<ConstraintBase> jl_const =
        std::make_shared<PandaMobileJointLimitsConstraints>(10, 10, jl_setting);
    cm.add_constraint("joint_limits", jl_const);
    constraints_["joint_limits"] = jl_const;
  }

  if (params_.cartesian_limits) {
    CartesianLimitSettings end_effector_self_collision;
    end_effector_self_collision.direction = CartesianLimitSettings::COLLISION;
    end_effector_self_collision.frame_a = "panda_link1";
    end_effector_self_collision.frame_b = "panda_hand";
    end_effector_self_collision.distance = params_.min_dist;

    CartesianLimitSettings end_effector_reach;
    end_effector_reach.direction = CartesianLimitSettings::REACH;
    end_effector_reach.P(2, 2) = 0;  // only 2d
    end_effector_reach.frame_a = "panda_link0";
    end_effector_reach.frame_b = "panda_hand";
    end_effector_reach.distance = params_.max_reach;

    CartesianLimitConstraintSettings cart_const_settings;
    cart_const_settings.urdf_string = params_.urdf;
    cart_const_settings.verbosity = params_.verbose;
    cart_const_settings.limits.push_back(end_effector_self_collision);
    cart_const_settings.limits.push_back(end_effector_reach);
    std::shared_ptr<ConstraintBase> cart_const =
        std::make_shared<CartesianLimitConstraints>(10, cart_const_settings);
    cm.add_constraint("cartesian_limits", cart_const);
    constraints_["cartesian_limits"] = cart_const;
  }

  if (params_.passivity_constraint) {
    std::shared_ptr<ConstraintBase> pass_const =
        std::make_shared<PassivityConstraint>(10, params_.min_tank_energy);
    cm.add_constraint("passivity_constraint", pass_const);
    constraints_["passivity_constraint"] = pass_const;
  }

  filter_ = std::make_unique<SafetyFilter>(cm);
}

void PandaMobileSafetyFilter::update(const Eigen::VectorXd& x,
                                     const Eigen::VectorXd& u, const double t) {
  filter_->update_observation(x, u, t);
  filter_->update_problem(x.head<10>(), u.head<10>());
}

void PandaMobileSafetyFilter::update_violation(const Eigen::VectorXd& x) {
  for (auto const& constraint : constraints_) {
    constraint.second->update_violation(x);
  }
}

bool PandaMobileSafetyFilter::apply(Eigen::VectorXd& u_opt) {
  if (params_.verbose) filter_->print_problem();
  if (!filter_->solve(u_opt)) {
    filter_->print_problem();
    throw std::runtime_error("Failed to find solution to problem.");
  }
  return true;
}

void PandaMobileSafetyFilter::reset_constraints() {
  filter_->reset_constraints();
}
