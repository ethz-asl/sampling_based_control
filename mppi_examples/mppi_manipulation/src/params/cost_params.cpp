//
// Created by giuseppe on 09.08.21.
//

#include "mppi_manipulation/params/cost_params.h"

using namespace manipulation;

bool CostParams::init_from_config(const manipulation::Config& config){
  YAML::Node cost = config.data["cost"]; 
  if (!cost) {
    std::cout << "Failed to load cost YAML node." << std::endl;
    return false;
  }
  bool sf = true;
  Qreg = config.parse_key<double>(cost, "regularization", sf).value_or(0.0);
  Qt = config.parse_key<double>(cost, "linear_weight", sf).value_or(0.0);    // translation cost
  Qt2 = config.parse_key<double>(cost, "linear_weight_opening", sf).value_or(0.0);
  Qr = config.parse_key<double>(cost, "angular_weight", sf).value_or(0.0);  // rotation cost
  Qr2 = config.parse_key<double>(cost, "angular_weight_opening", sf).value_or(0.0);
  Qo = config.parse_key<double>(cost, "obstacle_weight", sf).value_or(0.0);   // obstacle cost
  Qos = config.parse_key<double>(cost, "obstacle_weight_slope", sf).value_or(0.0);  // obstacle cost slope
  Qc = config.parse_key<double>(cost, "contact_weight", sf).value_or(0.0);   // contact cost
  ro = config.parse_key<double>(cost, "obstacle_radius", sf).value_or(0.0);   // obstacle radius
  max_reach = config.parse_key<double>(cost, "max_reach", sf).value_or(0.0);
  min_dist = config.parse_key<double>(cost, "min_dist", sf).value_or(0.0);  // min distance from base for collision avoidance
  Q_reach = config.parse_key<double>(cost, "reach_weight", sf).value_or(0.0);
  Q_reachs = config.parse_key<double>(cost, "reach_weight_slope", sf).value_or(0.0);
  Q_obj = config.parse_key<double>(cost, "object_weight", sf).value_or(0.0);
  Q_tol = config.parse_key<double>(cost, "object_tolerance", sf).value_or(0.0);
  Q_j = config.parse_key<double>(cost, "arm_joint_weight", sf).value_or(0.0);
  Q_bp = config.parse_key<double>(cost, "robot_base_position_weight", sf).value_or(0.0);
  Q_bo = config.parse_key<double>(cost, "robot_base_orientation_weight", sf).value_or(0.0);
  std::vector<double> trans = config.parse_key<std::vector<double>>(cost, "grasp_translation_offset", sf).value_or(std::vector<double>{0.0, 0.0, 0.0});
  std::vector<double> rot = config.parse_key<std::vector<double>>(cost, "grasp_orientation_offset", sf).value_or(std::vector<double>{0, -0.7071068, 0, 0.7071068});
  Eigen::Vector3d t(trans[0], trans[1], trans[2]);
  Eigen::Quaterniond q(rot[3], rot[0], rot[1], rot[2]);
  grasp_offset = mppi_pinocchio::Pose(t, q);
  debug_prints = config.parse_key<bool>(cost, "debug_prints", sf).value_or(true);
  
  Q_joint_limit = config.parse_key<double>(cost, "joint_limit_cost", sf).value_or(0.0);;
  Q_joint_limit_slope = config.parse_key<double>(cost, "joint_limit_slope", sf).value_or(0.0);;
  upper_joint_limits = config.parse_key<std::vector<double>>(cost, "upper_joint_limits", sf).value_or(std::vector<double>{2.0, 2.0, 6.28, 2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973});
  lower_joint_limits = config.parse_key<std::vector<double>>(cost, "lower_joint_limits", sf).value_or(std::vector<double>{-2.0, -2.0, -6.28, -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973});
  
  // power cost
  max_power = config.parse_key<double>(cost, "max_power", sf).value_or(0.0);
  Q_power = config.parse_key<double>(cost, "power_weight", sf).value_or(0.0);
  
  // frames
  handle_frame = config.parse_key<std::string>(cost, "handle_frame", sf).value_or("");
  tracked_frame = config.parse_key<std::string>(cost, "tracked_frame", sf).value_or("");
  arm_base_frame = config.parse_key<std::string>(cost, "arm_base_frame", sf).value_or("");
  
  collision_link_0 = config.parse_key<std::string>(cost, "collision_link_0", sf).value_or("");
  collision_link_1 = config.parse_key<std::string>(cost, "collision_link_1", sf).value_or("");
  collision_threshold = config.parse_key<double>(cost, "collision_threshold", sf).value_or(0.0);
  Q_collision = config.parse_key<double>(cost, "collision_weight", sf).value_or(0.0);

  robot_description = config.robot_description;
  object_description = config.object_description;
  return sf;
}

bool CostParams::init_from_ros(const ros::NodeHandle& nh) {
  if (!nh.getParam("/robot_description", robot_description) ||
      robot_description.empty()) {
    ROS_ERROR("Failed to parse /robot_description or invalid!");
    return false;
  }

  if (!nh.getParam("/object_description", object_description) ||
      object_description.empty()) {
    ROS_ERROR("Failed to parse /object_description or invalid!");
    return false;
  }

  if (!nh.getParam("cost/regularization", Qreg) || Qreg < 0) {
    ROS_ERROR("Failed to parse cost/regularization or invalid!");
    return false;
  }
  if (!nh.getParam("cost/obstacle_weight", Qo) || Qo < 0) {
    ROS_ERROR("Failed to parse cost/obstacle_weight or invalid!");
    return false;
  }

  if (!nh.getParam("cost/obstacle_weight_slope", Qos) || Qos < 0) {
    ROS_ERROR("Filed to parse cost/obstacle_weight_slope or invalid!");
    return false;
  }

  if (!nh.getParam("cost/linear_weight", Qt) || Qt < 0) {
    ROS_ERROR("Filed to parse cost/linear_weight or invalid!");
    return false;
  }

  if (!nh.getParam("cost/angular_weight", Qr) || Qr < 0) {
    ROS_ERROR("Filed to parse cost/angular_weight or invalid!");
    return false;
  }

  if (!nh.getParam("cost/arm_joint_weight", Q_j) || Q_j < 0) {
    ROS_WARN("Filed to parse cost/arm_joint_weight or invalid!");
    return false;
  }

  if (!nh.getParam("cost/robot_base_position_weight", Q_bp) || Q_bp < 0) {
    ROS_WARN("Filed to parse cost/robot_base_position_weight or invalid!");
    return false;
  }

  if (!nh.getParam("cost/robot_base_orientation_weight", Q_bo) || Q_bo < 0) {
    ROS_WARN("Filed to parse cost/robot_base_orientation_weight or invalid!");
    return false;
  }

  if (!nh.getParam("cost/contact_weight", Qc) || Qc < 0) {
    ROS_ERROR("Filed to parse contact_weight or invalid!");
    return false;
  }

  std::vector<double> trans;
  if (!nh.getParam("cost/grasp_translation_offset", trans) ||
      trans.size() != 3) {
    ROS_ERROR("Failed to parse cost/grasp_translation_offset or invalid!");
    return false;
  }

  std::vector<double> rot;
  if (!nh.getParam("cost/grasp_orientation_offset", rot) || rot.size() != 4) {
    ROS_ERROR("Failed to parse cost/grasp_orientation_offset or invalid!");
    return false;
  }
  Eigen::Vector3d t(trans[0], trans[1], trans[2]);
  Eigen::Quaterniond q(rot[3], rot[0], rot[1], rot[2]);
  grasp_offset = mppi_pinocchio::Pose(t, q);

  if (!nh.getParam("cost/obstacle_radius", ro) || ro < 0) {
    ROS_ERROR("Failed to parse cost/obstacle_radius or invalid!");
    return false;
  }

  if (!nh.getParam("cost/max_reach", max_reach) || max_reach < 0) {
    ROS_ERROR("Failed to parse cost/max_reach or invalid!");
    return false;
  }

  if (!nh.getParam("cost/min_dist", min_dist)) {
    ROS_ERROR("Failed to parse cost/min_dist or invalid!");
    return false;
  }

  if (!nh.getParam("cost/reach_weight", Q_reach) || Q_reach < 0) {
    ROS_ERROR("Failed to parse cost/reach_weight or invalid!");
    return false;
  }

  if (!nh.getParam("cost/reach_weight_slope", Q_reachs) || Q_reachs < 0) {
    ROS_ERROR("Failed to parse cost/reach_weight_slope or invalid!");
    return false;
  }

  if (!nh.getParam("cost/upper_joint_limits", upper_joint_limits) ||
      upper_joint_limits.size() != 10) {
    ROS_ERROR("Failed to parse cost/upper_joint_limits or invalid!");
    return false;
  }

  if (!nh.getParam("cost/lower_joint_limits", lower_joint_limits) ||
      lower_joint_limits.size() != 10) {
    ROS_ERROR("Failed to parse cost/lower_joint_limits or invalid!");
    return false;
  }

  if (!nh.getParam("cost/joint_limit_cost", Q_joint_limit) ||
      Q_joint_limit < 0) {
    ROS_ERROR("Failed to parse cost/joint_limit_cost or invalid!");
    return false;
  }

  if (!nh.getParam("cost/joint_limit_slope", Q_joint_limit_slope) ||
      Q_joint_limit_slope < 0) {
    ROS_ERROR("Failed to parse cost/joint_limit_slope or invalid!");
    return false;
  }

  if (!nh.getParam("cost/object_weight", Q_obj) || Q_obj < 0) {
    ROS_ERROR("Failed to parse cost/object_weight or invalid!");
    return false;
  }

  if (!nh.getParam("cost/object_tolerance", Q_tol) || Q_tol < 0) {
    ROS_ERROR("Failed to parse cost/object_tolerance or invalid!");
    return false;
  }

  if (!nh.getParam("cost/linear_weight_opening", Qt2) || Qt2 < 0) {
    ROS_ERROR("Failed to parse cost/linear_weight_opening or invalid!");
    return false;
  }

  if (!nh.getParam("cost/angular_weight_opening", Qr2) || Qr2 < 0) {
    ROS_ERROR("Failed to parse cost/angular_weight_opening or invalid!");
    return false;
  }

  if (!nh.getParam("cost/power_weight", Q_power) || Q_power < 0) {
    ROS_ERROR("Failed to parse cost/power_weight or invalid!");
    return false;
  }

  if (!nh.getParam("cost/max_power", max_power) || max_power < 0) {
    ROS_ERROR("Failed to parse cost/max_power or invalid!");
    return false;
  }

  if (!nh.getParam("cost/handle_frame", handle_frame) || handle_frame.empty()) {
    ROS_ERROR("Failed to parse cost/handle_frame or invalid!");
    return false;
  }

  if (!nh.getParam("cost/tracked_frame", tracked_frame) ||
      tracked_frame.empty()) {
    ROS_ERROR("Failed to parse cost/tracked_frame or invalid!");
    return false;
  }

  if (!nh.getParam("cost/arm_base_frame", arm_base_frame) ||
      arm_base_frame.empty()) {
    ROS_ERROR("Failed to parse cost/arm_base_frame or invalid!");
    return false;
  }

  if (!nh.getParam("cost/collision_link_0", collision_link_0) ||
      collision_link_0.empty()) {
    ROS_ERROR("Failed to parse cost/collision_link_0 or invalid!");
    return false;
  }

  if (!nh.getParam("cost/collision_link_1", collision_link_1) ||
      collision_link_1.empty()) {
    ROS_ERROR("Failed to parse cost/collision_link_1 or invalid!");
    return false;
  }

  if (!nh.getParam("cost/collision_threshold", collision_threshold)) {
    ROS_ERROR("Failed to parse cost/collision_threshold or invalid!");
    return false;
  }

  if (!nh.getParam("cost/collision_weight", Q_collision)) {
    ROS_ERROR("Failed to parse cost/collision_weight or invalid!");
    return false;
  }

  return true;
}

std::ostream& operator<<(std::ostream& os,
                         const manipulation::CostParams& param) {
  if (param.debug_prints) {
    // clang-format off
    os << "========================================" << std::endl;
    os << "        Panda Cost Parameters           " << std::endl;
    os << "========================================" << std::endl;
    os << " regularization weight: "   << param.Qreg << std::endl;
    os << " obstacle_weight: "         << param.Qo << std::endl;
    os << " obstacle_radius: "         << param.ro << std::endl;
    os << " obstacle_weight_slope: "   << param.Qos << std::endl;
    os << " linear_weight: "           << param.Qt << std::endl;
    os << " linear_weight_opening: "   << param.Qt2 << std::endl;
    os << " angular_weight: "          << param.Qr << std::endl;
    os << " angular_weight_opening: "  << param.Qr2 << std::endl;
    os << " contact_weight: "          << param.Qc << std::endl;
    os << " reach weight: "            << param.Q_reach << std::endl;
    os << " reach weight slope: "      << param.Q_reachs << std::endl;
    os << " object_weight: "           << param.Q_obj << std::endl;
    os << " object_weight_tolerance: " << param.Q_tol << std::endl;
    os << " grasp offset: "            << std::endl;
    os << " >> translation = "         << param.grasp_offset.translation.transpose() << std::endl;
    os << " >> rotation = "            << param.grasp_offset.rotation.coeffs().transpose() << std::endl;
    os << " joint limit weight: "      << param.Q_joint_limit << std::endl;
    os << " joint limit slope: "       << param.Q_joint_limit_slope << std::endl;
    os << " upper joint limits: ";
    for (size_t i=0; i<7; i++) os << param.upper_joint_limits[i] << " ";
    os << std::endl;
    os << " lower joint limits: ";
    for (size_t i=0; i<7; i++) os << param.lower_joint_limits[i] << " ";
    os << std::endl;
    os << "========================================" << std::endl;
  }else{
    os << "";
  }
  // clang-format on

  return os;
}