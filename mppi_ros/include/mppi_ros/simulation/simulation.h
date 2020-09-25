/*!
 * @file     simulation.h
 * @author   Giuseppe Rizzi
 * @date     13.08.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <ros/ros.h>

// TODO
class PathIntegralSimulation{
 public:
  PathIntegralSimulation(const ros::NodeHandlePtr nh) : nh_(nh){};

 private:
  ros::NodeHandlePtr nh_;
};

/*
 * // ros interface
  ros::init(argc, argv, "panda_control_node");
  ros::NodeHandle nh("~");
  auto ros_interface = PandaRosInterface(nh);

  // dynamics
  auto config = PandaDynamicsConfig();
  config.substeps = nh.param<double>("dynamics/substeps", 1.0);
  bool kinematic_simulation = nh.param<bool>("dynamics/kinematic_simulation", true);
  auto panda_dynamics = PandaDynamics(kinematic_simulation);
  panda_dynamics.set_dynamic_properties(config);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(PandaDim::STATE_DIMENSION);
  auto initial_configuration = nh.param<std::vector<double>>("initial_configuration", {});
  for(size_t i=0; i<initial_configuration.size(); i++)
    x(i) = initial_configuration[i];
  panda_dynamics.reset(x);
  mppi::DynamicsBase::dynamics_ptr dynamics = panda_dynamics.clone();

  // cost
  PandaCost panda_cost;
  double linear_weight = nh.param<double>("linear_weight", 10.0);
  panda_cost.set_linear_weight(linear_weight);
  double angular_weight = nh.param<double>("angular_weight", 10.0);
  panda_cost.set_angular_weight(angular_weight);
  double obstacle_radius = nh.param<double>("obstacle_radius", 0.2);
  panda_cost.set_obstacle_radius(obstacle_radius);
  mppi::CostBase::cost_ptr cost = std::shared_ptr<PandaCost>(&panda_cost);

  // sampler
  auto noise_sampler = mppi::GaussianSampler(PandaDim::INPUT_DIMENSION);
  auto noise_sigma = nh.param<std::vector<double>>("noise_variance", {});
  Eigen::VectorXd cov_diagonal = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(noise_sigma.data(), noise_sigma.size());
  noise_sampler.set_noise_variance(cov_diagonal);
  mppi::SamplerBase::sampler_ptr sampler = std::shared_ptr<mppi::GaussianSampler>(&noise_sampler);
  std::cout << "Sampler covariance: " << sampler->sigma() << std::endl;

  // solver config
  auto solver_config = mppi::SolverConfig();
  std::string config_file = nh.param<std::string>("config_file", "");
  if (!solver_config.init_from_file(config_file))
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);

  // solver
  auto solver = mppi::PathIntegral(dynamics, cost, sampler, solver_config);

  double dt;
  mppi::DynamicsBase::input_t u;
  u = panda_dynamics.get_zero_input(x);

  double r;
  double stage_cost;

  bool reference_ever_set = false;
  pinocchio::SE3 end_effector_pose;
  pinocchio::SE3 end_effector_pose_desired;
  pinocchio::SE3 obstacle_pose;

  // control loop
  ros::Duration(5.0).sleep();
  while (ros::ok()){

    // reference
    if (ros_interface.get_reference(end_effector_pose_desired)){
      reference_ever_set = true;
      panda_cost.set_reference(end_effector_pose_desired, ros::Time::now().toSec(), x);
    }

    // obstacle
    if (ros_interface.get_obstacle(obstacle_pose))
      panda_cost.set_obstacle(obstacle_pose);

    // control loop
    auto start =  std::chrono::steady_clock::now();
    if (reference_ever_set){
      u = solver.control(x, ros::Time::now().toSec());
    }
    else{
      ROS_WARN_STREAM_THROTTLE(2.0, "Reference never received");
    }
    x = dynamics->step(u, solver_config.step_size);


    // publish to ros
    r = solver.get_rollout_min_cost();
    stage_cost = panda_cost.get_stage_cost(x, ros::Time::now().toSec());
    end_effector_pose = panda_cost.get_pose_end_effector(x);
    ros_interface.publish_state(x);
    ros_interface.publish_input(u);
    ros_interface.publish_cost(stage_cost);
    ros_interface.publish_rollout_cost(r);
    ros_interface.publish_end_effector_pose(end_effector_pose);
    ros_interface.publish_obstacle_marker(obstacle_radius);

    // timing
    auto end = std::chrono::steady_clock::now();
    dt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if (dt > solver_config.step_size*1000){
      ROS_WARN_STREAM_THROTTLE(3.0, "Not real time (current dt=" << dt << " ms)");
    }
    else{
      ros::Duration(solver_config.step_size - dt/1000.).sleep();
    }
    ros::spinOnce();
 */