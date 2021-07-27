//
// Created by giuseppe on 03.07.21.
//

#include "mppi_panda_mobile/safety_filter/safety_filter.hpp"
#include "safety_filter/constraints/cartesian_limit.hpp"
#include "safety_filter/constraints/constraints_manager.hpp"
#include "safety_filter/constraints/first_derivative_limit.hpp"
#include "safety_filter/constraints/input_limits.hpp"
#include "safety_filter/constraints/second_derivative_limit.hpp"

using namespace safety_filter;
using namespace panda_mobile;

PandaMobileJointLimitsConstraints::PandaMobileJointLimitsConstraints(
    const size_t nc, const size_t nx,
    const JointLimitsConstraintsSetting& settings)
    : JointLimitsConstraints(nc, nx, settings){};

void PandaMobileJointLimitsConstraints::update_jacobian(
    const Eigen::VectorXd& x) {
  jacobian_.block(0, 0, 2, 2) << std::cos(x(2)), -std::sin(x(2)),
      std::sin(x(2)), std::cos(x(2));
}

bool PandaMobileSafetyFilterSettings::init_from_ros(ros::NodeHandle& nh) {
  if (!nh.param("safety_filter/dt", dt, 0.01) || dt <= 0) {
    ROS_ERROR("Failed to parse safety_filter/dt or invalid");
    return false;
  }

  if (!nh.param("safety_filter/verbose", verbose, false)) {
    ROS_WARN("Failed to parse safety_filter/verbose");
    return false;
  }

  if (!nh.param("safety_filter/joint_limits", joint_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/joint_limits");
    return false;
  }

  if (joint_limits) {
    std::vector<double> q_min_v, q_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/q_min", q_min_v, {})) {
      ROS_WARN("Failed to parse safety_filter/q_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/q_max", q_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/q_max");
      return false;
    }

    int q_size = q_min_v.size();
    if (q_size != q_max_v.size()) {
      throw std::runtime_error("Joint limits have different size!");
    }

    q_min.resize(q_size);
    q_max.resize(q_size);
    for (int i = 0; i < q_size; i++) {
      q_min[i] = q_min_v[i];
      q_max[i] = q_max_v[i];
    }
  }

  if (!nh.param("safety_filter/input_limits", input_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/input_limits");
    return false;
  }

  if (input_limits) {
    std::vector<double> u_min_v, u_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/u_min", u_min_v, {})) {
      ROS_WARN("Failed to parse safety_filter/u_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/u_max", u_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/u_max");
      return false;
    }

    int u_size = u_min_v.size();
    if (u_size != u_max_v.size()) {
      throw std::runtime_error("Joint limits have different size!");
    }

    u_min.resize(u_size);
    u_max.resize(u_size);
    for (int i = 0; i < u_size; i++) {
      u_min[i] = u_min_v[i];
      u_max[i] = u_max_v[i];
    }
  }

  if (!nh.param("safety_filter/first_derivative_limits",
                first_derivative_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/first_derivative_limits");
    return false;
  }

  if (first_derivative_limits) {
    std::vector<double> ud_min_v, ud_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/ud_min", ud_min_v, {})) {
      ROS_WARN("Failed to parse safety_filter/ud_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/ud_max", ud_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/ud_max");
      return false;
    }

    int u_size = ud_min_v.size();
    if (u_size != ud_max_v.size()) {
      throw std::runtime_error("First derivative limits have different size!");
    }

    ud_min.resize(u_size);
    ud_max.resize(u_size);
    for (int i = 0; i < u_size; i++) {
      ud_min[i] = ud_min_v[i];
      ud_max[i] = ud_max_v[i];
    }
  }

  if (!nh.param("safety_filter/second_derivative_limits",
                second_derivative_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/second_derivative_limits");
    return false;
  }

  if (second_derivative_limits) {
    std::vector<double> udd_min_v, udd_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/udd_min", udd_min_v,
                                       {})) {
      ROS_WARN("Failed to parse safety_filter/udd_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/ud_max", udd_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/udd_max");
      return false;
    }

    int u_size = udd_min_v.size();
    if (u_size != udd_max_v.size()) {
      throw std::runtime_error("Second derivative limits have different size!");
    }

    udd_min.resize(u_size);
    udd_max.resize(u_size);
    for (int i = 0; i < u_size; i++) {
      udd_min[i] = udd_min_v[i];
      udd_max[i] = udd_max_v[i];
    }
  }

  if (!nh.param("safety_filter/cartesian_limits", cartesian_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/cartesian_limits");
    return false;
  }

  if (cartesian_limits) {
    if (!nh.param("safety_filter/max_reach", max_reach, 0.80)) {
      ROS_WARN("Failed to parse safety_filter/max_reach");
      return false;
    }

    if (!nh.param("safety_filter/min_distance", min_dist, 0.15)) {
      ROS_WARN("Failed to parse safety_filter/min_distance");
      return false;
    }
  }

  return true;
}

std::ostream& operator<<(std::ostream& os,
                         const PandaMobileSafetyFilterSettings& settings) {
  // clang-format off
  os << "PandaMobileSafetyFilterSettings: " << std::endl;
  os << "dt = " << settings.dt << std::endl;
  os << "joint_limits: " << settings.joint_limits << std::endl;
  os << " >> q_min=" << settings.q_min.transpose() << std::endl;
  os << " >> q_max=" << settings.q_max.transpose() << std::endl;

  os << "input_limits: " << settings.input_limits << std::endl;
  os << " >> u_min=" << settings.u_min.transpose() << std::endl;
  os << " >> u_max=" << settings.u_max.transpose() << std::endl;

  os << "first_derivative_limits: " << settings.first_derivative_limits << std::endl;
  os << " >> ud_min=" << settings.ud_min.transpose() << std::endl;
  os << " >> ud_max=" << settings.ud_max.transpose() << std::endl;

  os << "second_derivative_limits: " << settings.second_derivative_limits << std::endl;
  os << " >> udd_min=" << settings.udd_min.transpose() << std::endl;
  os << " >> udd_max=" << settings.udd_max.transpose() << std::endl;

  os << "cartesian_limits: " << settings.cartesian_limits << std::endl;
  os << " >> max_reach: " << settings.max_reach << std::endl;
  os << " >> min_distance: " << settings.min_dist << std::endl;

  os << "verbose: " << settings.verbose << std::endl;
  // clang-format on
  return os;
}

PandaMobileSafetyFilter::PandaMobileSafetyFilter(
    const std::string& urdf_string,
    const PandaMobileSafetyFilterSettings& settings)
    : settings_(settings) {
  std::cout << settings_ << std::endl;
  ConstraintsManager cm(10);

  if (settings_.input_limits) {
    safety_filter::InputLimitsSettings ul_settings;
    ul_settings.u_min = settings_.u_min;
    ul_settings.u_max = settings_.u_max;
    std::shared_ptr<ConstraintBase> ul_const =
        std::make_shared<InputLimits>(ul_settings);
    cm.add_constraint("input_limits", ul_const);
  }

  if (settings_.first_derivative_limits) {
    safety_filter::FirstDerivativeLimitSettings dl_settings;
    dl_settings.dt = settings_.dt;
    dl_settings.ud_min = settings_.ud_min;
    dl_settings.ud_max = settings_.ud_max;
    std::shared_ptr<ConstraintBase> dl_const =
        std::make_shared<FirstDerivativeLimits>(dl_settings);
    cm.add_constraint("first_derivative_limits", dl_const);
  }

  if (settings_.second_derivative_limits) {
    safety_filter::SecondDerivativeLimitSettings ddl_settings;
    ddl_settings.dt = settings_.dt;
    ddl_settings.udd_min = settings_.udd_min;
    ddl_settings.udd_max = settings_.udd_max;
    std::shared_ptr<ConstraintBase> ddl_const =
        std::make_shared<SecondDerivativeLimits>(ddl_settings);
    cm.add_constraint("second_derivative_limits", ddl_const);
  }

  if (settings_.joint_limits) {
    JointLimitsConstraintsSetting jl_setting;
    jl_setting.q_min = settings_.q_min;
    jl_setting.q_max = settings_.q_max;
    std::shared_ptr<ConstraintBase> jl_const =
        std::make_shared<PandaMobileJointLimitsConstraints>(10, 10, jl_setting);
    cm.add_constraint("joint_limits", jl_const);
  }

  if (settings_.cartesian_limits) {
    CartesianLimitSettings end_effector_self_collision;
    end_effector_self_collision.direction = CartesianLimitSettings::COLLISION;
    end_effector_self_collision.frame_a = "panda_link1";
    end_effector_self_collision.frame_b = "panda_hand";
    end_effector_self_collision.distance = settings_.min_dist;

    CartesianLimitSettings end_effector_reach;
    end_effector_reach.direction = CartesianLimitSettings::REACH;
    end_effector_reach.P(2, 2) = 0;  // only 2d
    end_effector_reach.frame_a = "panda_link0";
    end_effector_reach.frame_b = "panda_hand";
    end_effector_reach.distance = settings_.max_reach;

    CartesianLimitConstraintSettings cart_const_settings;
    cart_const_settings.urdf_string = urdf_string;
    cart_const_settings.verbosity = settings_.verbose;
    cart_const_settings.limits.push_back(end_effector_self_collision);
    cart_const_settings.limits.push_back(end_effector_reach);
    std::shared_ptr<ConstraintBase> cart_const =
        std::make_shared<CartesianLimitConstraints>(10, cart_const_settings);
    cm.add_constraint("cartesian_limits", cart_const);
  }

  filter_ = std::make_shared<SafetyFilter>(cm);
}

bool PandaMobileSafetyFilter::apply(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& u,
                                    Eigen::VectorXd& u_opt) {
  filter_->update_problem(x, u);
  if (settings_.verbose) filter_->print_problem();
  return filter_->solve(u_opt);
}
