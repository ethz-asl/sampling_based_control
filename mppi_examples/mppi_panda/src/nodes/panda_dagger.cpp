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
#include <actionlib/server/simple_action_server.h>
#include <chrono>

#include <policy_learning/panda_expert.h>
#include <policy_learning/hdf5_dataset.h>
#include <policy_learning/torch_script_policy.h>
#include <policy_learning/collect_rolloutAction.h>

using namespace panda;

class PandaDataCollector{
  public:
    PandaDataCollector(ros::NodeHandle nh):
        nh_(nh),
        controller_(nh),
        action_server_(nh, "collector", false)
    {
      // ensure we don't sample any learned trajectories in expert
      if (controller_.config_.learned_rollout_ratio != 0) {
        ROS_ERROR_STREAM("Aborting Dagger, turn off sampling from NN in params!");
        throw std::runtime_error("Failed to initialzied controller!");
      }

      std::string robot_description = 
        nh.param<std::string>("/robot_description", "");
      simulation_ = std::make_shared<PandaDynamics>(robot_description);

      state_publisher_ =
        nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
      ee_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);

      action_server_.registerGoalCallback(
        std::bind(&PandaDataCollector::action_callback, this));
      action_server_.start();      

      // init the controller
      bool ok = controller_.init();
      if (!ok) {
        throw std::runtime_error("Failed to initialzied controller!");
      }
    }

  void run(){
    set_initial_state();
    controller_.set_observation(x_, sim_time_);


    // TODO: decide if as fast as possible or not...
    double sim_dt = nh_.param<double>("sim_dt", 0.01);

    ros::Rate idle_rate(idle_frequency_);

    // // helper variable to terminate episode
    // bool terminate = false;
    // geometry_msgs::PoseStamped final_pose =
    //   controller.get_pose_end_effector_ros(x);
    // double start_time = 0.0;
    // bool reference_set;


    mppi::DynamicsBase::input_t u;
    mppi::DynamicsBase::input_t u_expert;
    u = simulation_->get_zero_input(x_);
    u_expert = simulation_->get_zero_input(x_);
    bool terminated = false;

    while (ros::ok()){
      if (action_active_){ 

        controller_.update_reference();
        controller_.set_observation(x_, sim_time_);
        controller_.update_policy();


        // if (controller.get_reference_set()) {
        //   u = learner->get_action(x_); // segfault if reference in learner is not set!!!
        // }

        controller_.get_input(x_, u, sim_time_);
        x_ = simulation_->step(u, sim_dt);
        
        simulation_->reset(x_);
        publish_ros(x_);
        sim_time_ += sim_dt;

        policy_learning::collect_rolloutFeedback feedback;
        feedback.sim_time = sim_time_;
        action_server_.publishFeedback(feedback);

        // if goal is reached, wait some time until stopping
        if (goal_reached(controller_.get_pose_end_effector_ros(x_),
                         controller_.get_target_pose_ros()) &&
            !terminated) {
          terminated = true;
          timeout_ = std::min(timeout_, sim_time_ + time_to_shutdown);
          ROS_INFO("Goal reached, waiting for %f seconds before ending rollout.", 
                   time_to_shutdown);
        }

        if (sim_time_ > timeout_ || 
            action_server_.isPreemptRequested() || 
            !ros::ok()){
          policy_learning::collect_rolloutResult result;
          if (terminated){
            result.goal_reached = true;
            action_server_.setSucceeded(result);
          } else {
            result.goal_reached = false;
            action_server_.setPreempted(result);
          }
          action_active_ = false;
          terminated = false;
        }
      } else {
        publish_ros(x_);
        idle_rate.sleep();
        terminated = false;
      }
      ros::spinOnce();

    }
  }

  private:
    void set_initial_state(){
      auto x0 = nh_.param<std::vector<double>>("initial_configuration", {});
      for (size_t i = 0; i < x0.size(); i++) x_(i) = x0[i];
      simulation_->reset(x_);
    }

    void publish_ros(const Eigen::VectorXd& x){
      sensor_msgs::JointState joint_state;
      joint_state.name = {"panda_joint1", "panda_joint2", "panda_joint3",
                          "panda_joint4", "panda_joint5", "panda_joint6",
                          "panda_joint7"};
      joint_state.position.resize(7);
      joint_state.header.frame_id = "world";
      for (size_t i = 0; i < 7; i++) joint_state.position[i] = x(i);
      joint_state.header.stamp = ros::Time::now();
      state_publisher_.publish(joint_state);

      geometry_msgs::PoseStamped ee_pose;
          ee_pose = controller_.get_pose_end_effector_ros(x);
      ee_publisher_.publish(ee_pose);

      controller_.publish_ros_default();
      controller_.publish_ros();
    }


    void action_callback(){
      auto goal = action_server_.acceptNewGoal();
      ROS_INFO("Received goal with: \n- timeout %f,\n- use_policy %d,\n"
               "- dataset_path %s,\n-policy_path %s\n",
               goal->timeout, goal->use_policy, 
               goal->dataset_path.c_str(), goal->policy_path.c_str());
    
      timeout_ = sim_time_ + goal->timeout;
      controller_.get_controller()->initialize_rollouts();
      // reset to random state


      use_policy_ = goal->use_policy;
      // Set learner and so on
      // auto learner = controller.get_controller()-> get_learned_expert();
      // controller.get_controller()->set_learned_expert(learner);
      action_active_ = true;
    };

    bool goal_reached(const geometry_msgs::PoseStamped& conv_pose, 
                      const geometry_msgs::PoseStamped& current_pose) {
      double d_position = sqrt(
        pow(conv_pose.pose.position.x - current_pose.pose.position.x, 2) + 
        pow(conv_pose.pose.position.y - current_pose.pose.position.y, 2) + 
        pow(conv_pose.pose.position.z - current_pose.pose.position.z, 2)) ;

      Eigen::Quaterniond q_conv(conv_pose.pose.orientation.w,
                                conv_pose.pose.orientation.x,
                                conv_pose.pose.orientation.y,
                                conv_pose.pose.orientation.z);
      Eigen::Quaterniond q_current(current_pose.pose.orientation.w,
                                   current_pose.pose.orientation.x,
                                   current_pose.pose.orientation.y,
                                   current_pose.pose.orientation.z);   

      double d_angle = q_conv.angularDistance(q_current);

      if (d_position < goal_position_threshold &&
          d_angle < goal_angular_threshold) {
        return true;
      } else {
        return false;
      }
    }

  private:
    ros::NodeHandle nh_;
    PandaControllerInterface controller_;
    actionlib::SimpleActionServer<policy_learning::collect_rolloutAction>
      action_server_;

    mppi::DynamicsBase::dynamics_ptr simulation_;

    ros::Publisher state_publisher_;
    ros::Publisher ee_publisher_;
    ros::Publisher ee_desired_publisher_;

    float idle_frequency_ = 1.0; // Hz    

    bool action_active_ = false;
    bool use_policy_ = false;
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(PandaDim::STATE_DIMENSION);
    double sim_time_ = 0.0;
    double timeout_ = INFINITY;

    double time_to_shutdown = 0.5; // s

    double goal_position_threshold = 0.05; // m = 5 cm
    double goal_angular_threshold = 0.087; // rad = 5 deg

};


int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "panda_dagger");
  ros::NodeHandle nh("~");
  PandaDataCollector data_collection(nh);
  data_collection.run();
}
