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
#include <random>

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

      robot_description_ = 
        nh.param<std::string>("/robot_description", "");
      simulation_ = std::make_shared<PandaDynamics>(robot_description_);
      state_publisher_ =
        nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
      ee_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);   
      ee_goal_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("/end_effector_pose_desired", 10);
      obstacle_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("/obstacle", 10);   

      // init the controller
      bool ok = controller_.init();
      if (!ok) {
        throw std::runtime_error("Failed to initialzied controller!");
      }

      action_server_.registerGoalCallback(
        std::bind(&PandaDataCollector::action_callback, this));
      action_server_.start();   
    }

  void run(){
    set_initial_state();
    simulation_->reset(x_);
    controller_.set_observation(x_, sim_time_);

    // Somehow the first published goal is ignored, publish a dummy goal here.
    set_obstacle_pose();
    set_random_goal();

    double sim_dt = nh_.param<double>("sim_dt", 0.01);

    ros::Rate idle_rate(idle_frequency_);

    mppi::DynamicsBase::input_t u;
    mppi::DynamicsBase::input_t u_expert;
    u = simulation_->get_zero_input(x_);
    u_expert = simulation_->get_zero_input(x_);
    bool terminated = false;

    while (ros::ok()){
      if (action_active_){ 
        simulation_->reset(x_);
        controller_.set_observation(x_, sim_time_);
        controller_.update_reference();
        controller_.update_policy();

        if (learner_) u = learner_->get_action(x_); 
        controller_.get_input(x_, u_expert, sim_time_);  

        if (learner_ && learner_->collect_data()){
          learner_->save_state_action(x_, u_expert);
        }

        if (use_policy_) x_ = simulation_->step(u, sim_dt);
        else x_ = simulation_->step(u_expert, sim_dt);

        sim_time_ += sim_dt;
        publish_ros(x_);
        
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

          // Force write by destructing learner... A bit hacky and ugly :(
          learner_ = nullptr;
          controller_.get_controller()->set_learned_expert(learner_);

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
      } else if (callback_received_){
        // reset cached trajectories
        controller_.get_controller()->initialize_rollouts();

        // set state and goal at random
        set_random_state();
        set_random_goal();
        set_obstacle_pose();

        try {
          learner_ = std::make_shared<PandaExpert>(
            std::make_unique<TorchScriptPolicy>(policy_path_),
            std::make_unique<Hdf5Dataset>(dataset_path_),
            robot_description_
          );
          controller_.get_controller()->set_learned_expert(learner_);
          action_active_ = true;
        } catch (const std::runtime_error& error) {
          ROS_ERROR_STREAM("Could not initialze learner: " << error.what());
          action_server_.setAborted();
          action_active_ = false;
        }
        callback_received_ = false;
        action_active_ = true;
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
    }

    void set_random_state(){
      
      std::normal_distribution<double> distribution(0, joint_state_std_dev);

      set_initial_state();
      for (size_t i = 0; i < x_.size(); i++) x_(i) += distribution(generator_);
    }

    void set_random_goal(){
      Eigen::Quaterniond rot = Eigen::Quaterniond::UnitRandom();
      double max_rad = 0.6;
      double min_rad = 0.2;
      Eigen::Vector3d center; 
      center << 0, 0, 0.4;

      Eigen::Vector3d position;
      while (true){
        position = max_rad * Eigen::Vector3d::Random();
        if (position.norm() >= min_rad && position.norm() <= max_rad){
          for (size_t i = 0; i < 3; i++) position(i) += center(i);
          if (position(0) > 0 && position(2) > 0) break;
        }
      }
      geometry_msgs::PoseStamped goal;
      goal.header.frame_id = "world";
      goal.pose.position.x = position(0);
      goal.pose.position.y = position(1);
      goal.pose.position.z = position(2);
      goal.pose.orientation.w = rot.w();
      goal.pose.orientation.x = rot.x();
      goal.pose.orientation.y = rot.y();
      goal.pose.orientation.z = rot.z();

      ee_goal_publisher_.publish(goal);
    }

    void set_obstacle_pose(){
      geometry_msgs::PoseStamped obstacle;
      obstacle.header.frame_id = "world";
      obstacle.pose.position.x = 100;
      obstacle.pose.position.y = 100;
      obstacle.pose.position.z = 100;

      obstacle_publisher_.publish(obstacle);
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
      use_policy_ = goal->use_policy;
      policy_path_ = goal->policy_path;
      dataset_path_ = goal->dataset_path;

      action_active_ = false;
      callback_received_ = true;
    }

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
    std::string robot_description_;

    ros::Publisher state_publisher_;
    ros::Publisher ee_publisher_;
    ros::Publisher ee_goal_publisher_;
    ros::Publisher obstacle_publisher_;

    float idle_frequency_ = 1.0; // Hz    

    bool action_active_ = false;
    bool callback_received_ = false;
    bool use_policy_ = false;
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(PandaDim::STATE_DIMENSION);
    double sim_time_ = 0.0;

    double timeout_ = INFINITY;
    double time_to_shutdown = 2; // s
    double goal_position_threshold = 0.01; // m = 1 cm
    double goal_angular_threshold = 0.087; // rad = 5 deg
    double joint_state_std_dev = 0.5;

    std::default_random_engine generator_;
    std::shared_ptr<PandaExpert> learner_ = nullptr;  
    std::string dataset_path_;
    std::string policy_path_;
    
};


int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "panda_dagger");
  ros::NodeHandle nh("~");
  PandaDataCollector data_collection(nh);
  data_collection.run();
}
