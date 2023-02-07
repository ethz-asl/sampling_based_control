#include <mppi_omav_interaction/reference_pose_publisher.h>

namespace omav_interaction {
ReferencePosePublisher::ReferencePosePublisher(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      reference_pose_reconfigure_server_(
          ros::NodeHandle(private_nh, "reference_pose")) {
  reference_pose_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/mppi_pose_desired", 1);
  odometry_subscriber_ =
      nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                    &ReferencePosePublisher::odometryCallback, this);

  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavReferenceConfig>::CallbackType f;
  f = boost::bind(&ReferencePosePublisher::referencePoseCallback, this,
                  boost::placeholders::_1, boost::placeholders::_2);
  reference_pose_reconfigure_server_.setCallback(f);
}

void ReferencePosePublisher::referencePoseCallback(
    mppi_omav_interaction::MPPIOmavReferenceConfig& config,
    [[maybe_unused]] uint32_t level) const {
  if (config.reset) {
    config.reset = false;

    config.ref_pos_x = current_odometry_.position_W.x();
    config.ref_pos_y = current_odometry_.position_W.y();
    config.ref_pos_z = current_odometry_.position_W.z();

    Eigen::Vector3d euler_angles;
    current_odometry_.getEulerAngles(&euler_angles);
    config.ref_roll = rad2deg(euler_angles(0));
    config.ref_pitch = rad2deg(euler_angles(1));
    config.ref_yaw = rad2deg(euler_angles(2));
  }

  geometry_msgs::PoseStamped rqt_pose_msg;
  Eigen::VectorXd rqt_pose(7);
  Eigen::Quaterniond q;
  omav_interaction::conversions::RPYtoQuaterniond(
      config.ref_roll, config.ref_pitch, config.ref_yaw, q);
  rqt_pose << config.ref_pos_x, config.ref_pos_y, config.ref_pos_z, q.w(),
      q.x(), q.y(), q.z();
  omav_interaction::conversions::PoseStampedMsgFromVector(rqt_pose,
                                                          rqt_pose_msg);
  reference_pose_publisher_.publish(rqt_pose_msg);
}

void ReferencePosePublisher::odometryCallback(
    const nav_msgs::Odometry& odometry_msg) {
  mav_msgs::eigenOdometryFromMsg(odometry_msg, &current_odometry_);
}
}  // namespace omav_interaction