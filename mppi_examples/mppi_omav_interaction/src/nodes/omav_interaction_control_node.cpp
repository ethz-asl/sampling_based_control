
#include <mppi_omav_interaction/nodes/omav_interaction_control_node.h>

namespace omav_interaction {

InteractionControlNode::InteractionControlNode(ros::NodeHandle& nh,
                                               ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), controller_(private_nh, nh) {
  timed_command_ = nh_.createTimer(
      ros::Duration(1. / kControl_rate),
      &InteractionControlNode::TimedCommandCallback, this, false, true);

  // load parameters
  // InitializeControllerParams();
  InitializeNodeParams();
}

InteractionControlNode::~InteractionControlNode() {}

bool InteractionControlNode::InitializeNodeParams() {
  // ros interface
  omav_trajectory_node_ =
      std::make_shared<OmavTrajectoryGenerator>(nh_, private_nh_);

  // Load ROS parameters
  // Check if running with omav interface
  std::string robot_description_raisim;
  std::string object_description;
  std::vector<double> x0;
  std::string object_name;
  getParam<bool>(private_nh_, "running_rotors", running_rotors_, true);
  getParam<std::string>(private_nh_, "object_name", object_name, "shelf");
  getParam<std::string>(private_nh_, "/robot_description_raisim",
                        robot_description_raisim, "");
  getParam<std::string>(private_nh_, "/object_description", object_description,
                        "");
  getParam<std::vector<double>>(private_nh_, "initial_configuration", x0, {});
  getParam<bool>(private_nh_, "sequential", sequential_, false);

  ROS_INFO_STREAM(
      "[mppi_omav_interaction] Robot & Object Description Raisim Loaded");

  controller_.setTask(object_name);

  // ROS_INFO_STREAM("Controller Created");
  simulation_ = std::make_shared<OMAVVelocityDynamicsRos>(
      private_nh_, robot_description_raisim, object_description, kSim_dt);
  // ROS_INFO_STREAM("Simulation Created");

  // Set nominal state
  state_nom_ = observation_t::Zero(simulation_->get_state_dimension());
  // set initial state
  state_ = observation_t::Zero(simulation_->get_state_dimension());
  for (size_t i = 0; i < x0.size(); i++) state_(i) = x0[i];
  // ROS_INFO_STREAM("Resetting initial state to " << state.transpose());
  simulation_->reset(state_);

  // init control input
  input_ = input_t::Zero(6);

  // init the controller
  if (!controller_.init())
    throw std::runtime_error("Failed to initialize controller");
  controller_.set_observation(state_, 0.0);
  ROS_INFO_STREAM("[mppi_omav_interaction] First Odometry received");

  // Set first odometry value as reference
  if (running_rotors_) {
    omav_trajectory_node_->get_odometry(state_);
    controller_.set_initial_reference(state_);
    omav_trajectory_node_->initialize_integrators(state_);
  }

  // Initialize values for the manual timing
  sim_time_ = 0.0;

  return true;
}

bool InteractionControlNode::InitializeControllerParams() { return true; }

void InteractionControlNode::TimedCommandCallback(const ros::TimerEvent& e) {
  computeCommand(e.current_real);
}

bool InteractionControlNode::computeCommand(const ros::Time& t_now) {
  // Check if there are any parameter updates (from rqt)
  if (controller_.getTask() == InteractionTask::Shelf) {
    if (omav_trajectory_node_->rqt_cost_shelf_bool_) {
      ROS_INFO("[mppi_omav_interaction] Setting new Cost Param");
      omav_trajectory_node_->rqt_cost_shelf_bool_ =
          controller_.update_cost_param_shelf(
              omav_trajectory_node_->rqt_cost_shelf_);
      ROS_INFO("[mppi_omav_interaction] New Cost Param Set");
    }
  } else if (controller_.getTask() == InteractionTask::Valve) {
    // Set valve reference value to current angle
    controller_.updateValveReference(state_(13) + 0.5);
    if (omav_trajectory_node_->rqt_cost_valve_bool_) {
      omav_trajectory_node_->rqt_cost_valve_bool_ =
          controller_.update_cost_param_valve(
              omav_trajectory_node_->rqt_cost_valve_);
      ROS_INFO("[mppi_omav_interaction] New Cost Param Set");
    }
  }

  // Check if the object should be reset
  if (omav_trajectory_node_->reset_object_) {
    state_(13) = 0;
    simulation_->reset(state_);
    omav_trajectory_node_->reset_object_ = false;
    ROS_INFO("[mppi_omav_interaction] Reset Object");
  }

  // Get odometry and target state
  double t_odom;
  // Time the last odometry was received:
  omav_trajectory_node_->get_odometry(state_, t_odom);
  if (t_odom < 0) {
    ROS_WARN_THROTTLE(
        1.0, "[mppi_omav_interaction] Bad odom time. Is odometry available?");
    return false;
  } else if (!controller_running_) {
    // Start controller once odometry has been received
    controller_.start();
    controller_running_ = true;
  }

  omav_trajectory_node_->target_state_time_ += 1.0 / kControl_rate;
  const double t_since_last_target =
      (t_now - omav_trajectory_node_->last_target_received_).toSec();

  // After the first trajectory is sent input timing is started
  if (omav_trajectory_node_->first_trajectory_sent_) {
    // Calculation of the index where we are in the last sent trajectory based
    // on the time the last trajectory that was sent
    const int shift_index = std::ceil(t_since_last_target / kSim_dt);
    std::cout << "t_odom: " << t_odom
              << ", tst: " << omav_trajectory_node_->target_state_time_
              << ", tslt: " << t_since_last_target
              << ", shift_index: " << shift_index << std::endl;
    // omav_trajectory_node_->shift_index_ =
    //     std::ceil(t_since_last_target / kSim_dt);
    // ROS_INFO_STREAM(
    //     "Target state time = " << t_since_last_target);
    // ROS_INFO_STREAM("Shift index
    // = " << omav_trajectory_node_->shift_index_);
    // Input does only have to be shifted if the trajectory index changed and
    // exception is made when we are close to 0.1s when its crucial the
    // trajectory optimized is continuous
    if (shift_index != index_temp_ && shift_index < 4) {
      index_temp_ = shift_index;
    static int index_temp = 0;
    if (shift_index != index_temp && shift_index < 4) {
      index_temp = shift_index;
      // Input is shifted in the MPPI as well as the initial values of the
      // desired trajectories of the integrators
      controller_.manually_shift_input(index_temp);
      if (index_temp <
          omav_trajectory_node_->current_trajectory_.points.size()) {
        omav_trajectory_node_->set_target(
            omav_trajectory_node_->current_trajectory_.points[index_temp]);
      } else {
        ROS_WARN_THROTTLE(
            1.0, "[mppi_omav_interaction] Wrong size of current trajectory.");
      }
    } else if (shift_index != index_temp &&
               !omav_trajectory_node_->shift_lock_) {
      // To ensure the trajectories are continuous even if the controller
      // takes longer than 0.015 to run the "final state" is set earlier
      mav_msgs::EigenTrajectoryPoint target_state;
      omav_interaction::conversions::InterpolateTrajectoryPoints(
          omav_trajectory_node_->current_trajectory_.points[6],
          omav_trajectory_node_->current_trajectory_.points[7], &target_state);
      omav_trajectory_node_->set_target(target_state);
      controller_.manually_shift_input(7);
      omav_trajectory_node_->shift_lock_ = true;
    } else if (t_since_last_target > 0.09) {
      // Experienced some problems where I ran into problems due to
      // multithreading, so to ensure no funny business happening added this
      // safeguard
      controller_.manually_shift_input(0);
    }
  }

  sim_time_ += 1.0 / kControl_rate;
  ROS_INFO_STREAM("Sim time: " << sim_time_ << ", odom time: " << t_odom
                               << ", ros time: " << t_now.toSec());

  // Set new observation
  controller_.set_observation(state_, t_odom);

  return true;
}

template <class T>
void InteractionControlNode::getParam(const ros::NodeHandle& nh,
                                      const std::string& id, T& var,
                                      const T& val_def) const {
  if (!nh.param<T>(id, var, val_def)) {
    ROS_FATAL_STREAM("[mppi_omav_interaction] Could not find parameter " << id);
    ros::shutdown();
    exit(1);
  } else {
    ROS_INFO_STREAM("[mppi_omav_interaction] Successfully read " << id);
  }
}

}  // namespace omav_interaction

int main(int argc, char** argv) {
  ros::init(argc, argv, "interaction_controller_node");

  ros::NodeHandle nh, private_nh("~");

  // spawn the controller node
  std::shared_ptr<omav_interaction::InteractionControlNode> controller_node(
      new omav_interaction::InteractionControlNode(nh, private_nh));

  ros::spin();

  return 0;
}
