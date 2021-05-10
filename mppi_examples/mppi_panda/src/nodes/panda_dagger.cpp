/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */
#include "mppi_panda/controller_interface.h"
#include "mppi_panda/dynamics.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>

#include <policy_learning/panda_expert.h>
#include <policy_learning/hdf5_dataset.h>
#include <policy_learning/torch_script_policy.h>

using namespace panda;

// helper functions to determine termination
bool isTerminated(geometry_msgs::PoseStamped& conv_pose, double& start_time,
  geometry_msgs::PoseStamped current_pose, double current_time,
  bool reference_set) {
  // check if robot converges to a postion in which it stays for a certain time
  double epsilon = 0.05;
  double time_thresh = 2.0;
  // read values from the "convergence" pose
  double x = conv_pose.pose.position.x;
  double y = conv_pose.pose.position.y;
  double z = conv_pose.pose.position.z;
  double qx = conv_pose.pose.orientation.x;
  double qy = conv_pose.pose.orientation.y;
  double qz = conv_pose.pose.orientation.z;
  double qw = conv_pose.pose.orientation.w;
  // values from current pose
  double x_curr = current_pose.pose.position.x;
  double y_curr = current_pose.pose.position.y;
  double z_curr = current_pose.pose.position.z;
  double qx_curr = current_pose.pose.orientation.x;
  double qy_curr = current_pose.pose.orientation.y;
  double qz_curr = current_pose.pose.orientation.z;
  double qw_curr = current_pose.pose.orientation.w;

  bool x_cond = abs(x-x_curr) < epsilon;
  bool y_cond = abs(y-y_curr) < epsilon;
  bool z_cond = abs(z-z_curr) < epsilon;
  bool qx_cond = abs(qx-qx_curr) < epsilon;
  bool qy_cond = abs(qy-qy_curr) < epsilon;
  bool qz_cond = abs(qz-qz_curr) < epsilon;
  bool qw_cond = abs(qw-qw_curr) < epsilon;
  bool is_in_threshold = x_cond && y_cond && z_cond && qx_cond && qy_cond
    && qz_cond && qw_cond;
  bool time_condition = (current_time - start_time) > time_thresh;
  if(!reference_set) start_time = current_time;
  if (is_in_threshold && time_condition && reference_set) {
    return true;
  }
  else if (is_in_threshold) return false;
  else {
    conv_pose = current_pose;
    start_time = current_time;
    return false;
  }
}

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "panda_dagger");
  ros::NodeHandle nh("~");

  auto sequential = nh.param<bool>("sequential", false);
  // controller has a learned expert in it, but we can turn it off by using the
  // param learned_ratio --> we also load an additional learned policy instance
  auto controller = PandaControllerInterface(nh);

  mppi::DynamicsBase::dynamics_ptr simulation;
  std::string robot_description =
      nh.param<std::string>("/robot_description", "");
  simulation = std::make_shared<PandaDynamics>(robot_description);

  // -------------------------------
  // learner
  // -------------------------------
  std::unique_ptr<Policy> policy_ptr = nullptr;
  std::string torchscript_model_path;
  if (nh.param<std::string>("torchscript_model_path",
      torchscript_model_path, "")) {
    policy_ptr = std::make_unique<TorchScriptPolicy>(
      torchscript_model_path
    );
    ROS_INFO_STREAM("Using Torch script from " << torchscript_model_path);
  }
  else {
    ROS_ERROR_STREAM("Failed to load the expert policy. " <<
      torchscript_model_path << " is not a path to a model. Provide proper path to" <<
      " model parameters.");
    throw std::runtime_error("Failed to initialzied controller!");
  }
  std::unique_ptr<Dataset> dataset_ptr = nullptr;
  // std::string learned_expert_output_path;
  // if (nh.param<std::string>("learned_expert_output_path",
  //     learned_expert_output_path, "")) {
  //   dataset_ptr = std::make_unique<Hdf5Dataset>(
  //     learned_expert_output_path
  //   );
  //   ROS_INFO_STREAM("HDF5 output path: " << learned_expert_output_path);
  // }
  // else {
  //   ROS_ERROR_STREAM("Failed to open dataset storage location. Provide proper path to" <<
  //     " save collected data.");
  //   throw std::runtime_error("Failed to initialzied data saving!");
  // }

  auto learner = std::make_shared<PandaExpert>(
    std::move(policy_ptr),
    std::move(dataset_ptr),
    robot_description
  );

  Eigen::VectorXd x = Eigen::VectorXd::Zero(PandaDim::STATE_DIMENSION);
  Eigen::VectorXd x_nom = Eigen::VectorXd::Zero(PandaDim::STATE_DIMENSION);

  // set initial state
  auto x0 = nh.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < x0.size(); i++) x(i) = x0[i];
  simulation->reset(x);

  // init control input
  mppi::DynamicsBase::input_t u;
  mppi::DynamicsBase::input_t u_expert;
  u = simulation->get_zero_input(x);
  u_expert = simulation->get_zero_input(x);

  ros::Publisher state_publisher =
      nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ros::Publisher ee_publisher =
      nh.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);
  ros::Publisher ee_desired_publisher =
      nh.advertise<geometry_msgs::PoseStamped>("/ee_desired_nominal", 10);

  sensor_msgs::JointState joint_state;
  joint_state.name = {"panda_joint1", "panda_joint2", "panda_joint3",
                      "panda_joint4", "panda_joint5", "panda_joint6",
                      "panda_joint7"};
  joint_state.position.resize(7);
  joint_state.header.frame_id = "world";

  geometry_msgs::PoseStamped ee_pose;
  geometry_msgs::PoseStamped ee_pose_desired;

  bool static_optimization = nh.param<bool>("static_optimization", false);
  double sim_dt = nh.param<double>("sim_dt", 0.01);

  double sim_time = 0.0;

  // init the controller
  bool ok = controller.init();
  if (!ok) {
    throw std::runtime_error("Failed to initialzied controller!");
  }

  // set the very first observation
  controller.set_observation(x, sim_time);

  if (!sequential) controller.start();

  // helper variable to terminate episode
  bool terminate = false;
  geometry_msgs::PoseStamped final_pose =
    controller.get_pose_end_effector_ros(x);
  double start_time = 0.0;
  bool reference_set;

  // ensure we don't sample any learned trajectories in expert
  if (controller.config_.learned_rollout_ratio != 0) {
    ROS_ERROR_STREAM("Aborting Dagger, turn off sampling from NN in params!");
    throw std::runtime_error("Failed to initialzied controller!");
  }

  while (ros::ok()) {
    auto start = std::chrono::steady_clock::now();

    controller.update_reference();
    controller.set_observation(x, sim_time);
    // loop this ? -> i think controlling over substeps would be better, then we only have one parameter
    // check out function "offline_control" which would have the same effect.
    // Controller is our "expert" --> thus if we loop this we are basically
    // modifying the substeps of the expert. If we do this, I think we should do
    // it explicitly via substeps.
    controller.update_policy();
    //u = learner->get_action(x); // segfault if reference in learner is not set!!!
    // check if this then also saves this state action pair
    // controller.get_input(x, u_expert, sim_time);
    controller.get_input(x, u, sim_time);
    controller.publish_ros_default();
    controller.publish_ros();

    if (!static_optimization) {
      x = simulation->step(u, sim_dt);
      sim_time += sim_dt;
    }

    for (size_t i = 0; i < 7; i++) joint_state.position[i] = x(i);
    joint_state.header.stamp = ros::Time::now();
    state_publisher.publish(joint_state);

    ee_pose = controller.get_pose_end_effector_ros(x);
    ee_publisher.publish(ee_pose);

    ee_pose_desired = controller.get_pose_end_effector_ros(x_nom);
    ee_desired_publisher.publish(ee_pose_desired);

    auto end = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count() /
        1000.0;
    if (sim_dt - elapsed > 0)
      ros::Duration(sim_dt - elapsed).sleep();
    else {
      ROS_WARN_STREAM_THROTTLE(
          3.0, "Simulation slower than real time: last dt=" << elapsed << "s.");
    }

    reference_set = controller.get_reference_set();
    terminate = isTerminated(final_pose, start_time, ee_pose, sim_time,
      reference_set);

    if (terminate) {
      ros::shutdown();
    }

    ros::spinOnce();
  }
}
