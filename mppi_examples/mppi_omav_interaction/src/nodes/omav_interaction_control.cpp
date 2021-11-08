/*!
 * @file    omav_interaction_control.cpp
 * @author  Matthias Studiger
 * @date    10.04.2021
 * @version 1.0
 * @brief   description
 */
#include <mppi_omav_interaction/omav_trajectory_generator.h>

using namespace omav_interaction;

template <class T>
void getParam(const ros::NodeHandle &nh, const std::string &id, T &var,
              const T &val_def) {
  if (!nh.param<T>(id, var, val_def)) {
    ROS_FATAL_STREAM("Could not find parameter " << id);
    ros::shutdown();
    exit(1);
  } else {
    ROS_INFO_STREAM("Successfully read " << id);
  }
}

int main(int argc, char **argv) {
  // Initialize Ros Node
  ros::init(argc, argv, "omav_control_node");
  ros::NodeHandle nh("~"), nh_public;

  const double sim_dt = 0.015;
  const double control_rate = 250.0;

  // Load ROS parameters
  // Check if running with omav interface
  bool running_rotors;
  std::string robot_description_raisim;
  std::string robot_description_pinocchio;
  std::string object_description;
  std::vector<double> x0;
  bool sequential;

  getParam<bool>(nh, "running_rotors", running_rotors, false);
  getParam<std::string>(nh, "/robot_description_raisim",
                        robot_description_raisim, "");
  getParam<std::string>(nh, "/robot_description_pinocchio",
                        robot_description_pinocchio, "");
  getParam<std::string>(nh, "/object_description", object_description, "");
  getParam<std::vector<double>>(nh, "initial_configuration", x0, {});
  getParam<bool>(nh, "sequential", sequential, false);

  ROS_INFO_STREAM("Robot & Object Description Raisim Loaded");

  // ros interface
  std::shared_ptr<omav_interaction::OmavTrajectoryGenerator>
      omav_trajectory_node(
          new omav_interaction::OmavTrajectoryGenerator(nh_public, nh));
  auto controller = OMAVControllerInterface(nh, nh_public);
  ROS_INFO_STREAM("Controller Created");
  auto simulation = std::make_shared<OMAVVelocityDynamicsRos>(
      nh, robot_description_raisim, object_description, sim_dt);
  ROS_INFO_STREAM("Simulation Created");

  // Set nominal state
  observation_t state_nom = observation_t::Zero(simulation->get_state_dimension());
  // set initial state
  observation_t state = observation_t::Zero(simulation->get_state_dimension());
  for (size_t i = 0; i < x0.size(); i++) state(i) = x0[i];
  ROS_INFO_STREAM("Resetting initial state to " << state.transpose());
  simulation->reset(state);

  // init control input
  mppi::DynamicsBase::input_t input = input_t::Zero(6);

  // init the controller
  bool ok = controller.init();
  if (!ok) throw std::runtime_error("Failed to initialize controller");
  controller.set_observation(state, 0.0);
  if (running_rotors) {
    while (omav_trajectory_node->odometry_bool_) {
      ROS_INFO_STREAM_ONCE("No Odometry received yet");
      ros::spinOnce();
    }
  }
  ROS_INFO_STREAM("First Odometry received");

  // Set first odometry value as reference
  if (running_rotors) {
    omav_trajectory_node->get_odometry(state);
    controller.set_initial_reference(state);
    omav_trajectory_node->initialize_integrators(state);
  }

  // start controller
  if (!sequential) controller.start();

  // Initialize values for the manual timing
  double sim_time = 0.0;
  int index_temp = 0;

  // do some timing
  double elapsed;
  std::chrono::time_point<std::chrono::steady_clock> t_start, t_end;
  ros::Rate r_control(control_rate);
  while (ros::ok()) {
    t_start = std::chrono::steady_clock::now();
    if (omav_trajectory_node->rqt_cost_bool_) {
      omav_trajectory_node->rqt_cost_bool_ =
          controller.update_cost_param(omav_trajectory_node->rqt_cost_);
      ROS_INFO_STREAM("New Cost Param Set");
    }
    if (omav_trajectory_node->reset_object_) {
      state(13) = 0;
      simulation->reset(state);
      omav_trajectory_node->reset_object_ = false;
      ROS_INFO_STREAM("Reset Object");
    }
    if (sequential) {
      controller.update_reference();
      // controller.set_observation(state, sim_time);
      controller.update_policy();
      controller.get_input_state(state, state_nom, input, sim_time);
      controller.publish_ros_default();
      if (omav_trajectory_node->target_state_time_ > 0.1) {
        controller.publish_ros();
      }
      // Additional publisher for additional visualization
      controller.publish_optimal_rollout();
      controller.publish_all_trajectories();
    } else if (running_rotors) {
      omav_trajectory_node->get_odometry(state);
      sim_time += 1.0 / control_rate;
      omav_trajectory_node->target_state_time_ += 1.0 / control_rate;
    }

    if (!running_rotors) {
      state = simulation->step(input, sim_dt);
      simulation->publish_ros();
      sim_time += sim_dt;
      omav_trajectory_node->target_state_time_ += sim_dt;

      t_end = std::chrono::steady_clock::now();
      elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
              .count() /
          1000.0;
    }
    // After the first trajectory is sent input timing is started
    if (omav_trajectory_node->first_trajectory_sent_) {
      // Calculation of the index where we are in the last sent trajectory based
      // on the time the last trajectory that was sent
      omav_trajectory_node->shift_index_ =
          std::ceil(omav_trajectory_node->target_state_time_ / sim_dt);
      // Input does only have to be shifted if the trajectory index changed and
      // exception is made when we are close to 0.1s when its crucial the
      // trajectory optimized is continuous
      if (omav_trajectory_node->shift_index_ != index_temp &&
          omav_trajectory_node->shift_index_ < 4) {
        index_temp = omav_trajectory_node->shift_index_;
        // Input is shifted in the MPPI as well as the initial values of the
        // desired trajectories of the integrators
        controller.manually_shift_input(index_temp);
        omav_trajectory_node->set_target(
            omav_trajectory_node->current_trajectory_.points[index_temp]);
      } else if (omav_trajectory_node->shift_index_ != index_temp &&
                 !omav_trajectory_node->shift_lock_) {
        // To ensure the trajectories are continuous even if the controller
        // takes longer than 0.015 to run the "final state" is set earlier
        omav_interaction::conversions::InterpolateTrajectoryPoints(
            omav_trajectory_node->current_trajectory_.points[6],
            omav_trajectory_node->current_trajectory_.points[7],
            &omav_trajectory_node->target_state_);
        controller.manually_shift_input(7);
        omav_trajectory_node->shift_lock_ = true;
      } else if (omav_trajectory_node->target_state_time_ > 0.09) {
        // Experienced some problems where I ran into problems due to
        // multithreading, so to ensure no funny business happening added this
        // safeguard
        controller.manually_shift_input(0);
      }
    }

    // Set new observation
    controller.set_observation(state, sim_time);
    controller.get_input_state(state, state_nom, input, sim_time);
    // Timing Tasks
    if (running_rotors) {
      r_control.sleep();
    } else if (sim_dt - elapsed > 0) {
      ros::Duration(sim_dt - elapsed).sleep();
    } else {
      ROS_INFO_STREAM_THROTTLE(
          3.0, "Slower than real-time: " << elapsed / sim_dt << "x slower.");
    }
    ros::spinOnce();
  }
}
