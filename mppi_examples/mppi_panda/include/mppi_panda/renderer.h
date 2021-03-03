/*!
 * @file     renderer.h
 * @author   Giuseppe Rizzi
 * @date     09.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <mppi/visualization/rederer.h>

#include <mppi_pinocchio/model.h>
#include <mppi_pinocchio/ros_conversions.h>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace panda {
class RendererPanda : public mppi::Renderer {
 public:
  RendererPanda(ros::NodeHandle& nh, const std::string& robot_description)
      : nh_(nh) {
    rollouts_publisher_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/rollouts_paths", 10);
    path_point_.header.frame_id = "world";
    path_point_.type = visualization_msgs::Marker::ARROW;
    path_point_.scale.x = 0.001;
    path_point_.scale.y = 0.002;
    path_point_.pose.orientation.x = 0.0;
    path_point_.pose.orientation.y = 0.0;
    path_point_.pose.orientation.z = 0.0;
    path_point_.pose.orientation.w = 1.0;
    path_point_.color.a = 1.0;
    path_point_.points.push_back(start_);
    path_point_.points.push_back(end_);
    path_point_.action = visualization_msgs::Marker::MODIFY;

    robot_model_.init_from_xml(robot_description);
  };

  ~RendererPanda() = default;

  void render(const std::vector<mppi::Rollout>& rollouts) override {
    id_ = 0;
    for (size_t i = 0; i < rollouts.size(); i++) {
      if (i % paths_decimation == 0) {
        add_rollout(rollouts[i]);
      }
      rollouts_publisher_.publish(paths_);
    }
  }

  void add_rollout(const mppi::Rollout& roll) {
    paths_.markers.clear();
    mppi_pinocchio::Pose start_pose, end_pose;
    robot_model_.update_state(roll.xx[0].head<7>());
    start_pose = robot_model_.get_pose("panda_hand");

    for (size_t i = 1; i < roll.xx.size(); i++) {
      if (i % path_points_decimation == 0) {
        robot_model_.update_state(roll.xx[i].head<7>());
        end_pose = robot_model_.get_pose("panda_hand");
        start_.x = start_pose.translation.x();
        start_.y = start_pose.translation.y();
        start_.z = start_pose.translation.z();
        end_.x = end_pose.translation.x();
        end_.y = end_pose.translation.y();
        end_.z = end_pose.translation.z();
        start_pose = end_pose;
        path_point_.id = id_;
        path_point_.points[0] = start_;
        path_point_.points[1] = end_;
        paths_.markers.push_back(path_point_);
        id_++;
      }
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher rollouts_publisher_;
  visualization_msgs::MarkerArray paths_;
  visualization_msgs::Marker path_point_;
  geometry_msgs::Point start_, end_;

  mppi_pinocchio::RobotModel robot_model_;

  int id_ = 0;
  const int paths_decimation = 10;
  const int path_points_decimation = 5;
};
}  // namespace panda