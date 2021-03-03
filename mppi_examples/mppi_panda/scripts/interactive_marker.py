#! /usr/bin/env  python

import copy
import rospy

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped, Pose
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *


class SingleMarkerBroadcaster:
    def __init__(self):
        server_name = rospy.get_param("~marker_server_name",
                                      "interactive_marker")
        self.server = InteractiveMarkerServer(server_name)
        self.initial_pose = PoseStamped()
        self.pose = PoseStamped()

        self.initialized = False
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.frame_id = rospy.get_param('~frame_id', 'odom')
        target_frame = rospy.get_param("~target_frame", "")
        if not target_frame:
            self.init_pose()
        elif not self.init_pose_from_ros(target_frame):
            rospy.logerr(
                "Failed to initialize interactive_marker from tf tree")

        target_pose_topic = rospy.get_param("~target_pose_topic",
                                            "/target_pose_marker")
        self.pose_pub = rospy.Publisher(target_pose_topic,
                                        PoseStamped,
                                        queue_size=1)

    def init_pose(self):
        self.initial_pose = PoseStamped()
        self.initial_pose.pose.position.x = 1.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.position.z = 0.5
        self.initialized = True

    def init_pose_from_ros(self, frame_id):
        max_attempts = 10
        attempts = 0
        while attempts < max_attempts:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id, frame_id, rospy.Time(0), rospy.Duration(10))
                self.initial_pose = PoseStamped()
                self.initial_pose.header.frame_id = self.frame_id
                self.initial_pose.header.stamp = rospy.Time.now()
                self.initial_pose.pose.position.x = transform.transform.translation.x
                self.initial_pose.pose.position.y = transform.transform.translation.y
                self.initial_pose.pose.position.z = transform.transform.translation.z
                self.initial_pose.pose.orientation.x = transform.transform.rotation.x
                self.initial_pose.pose.orientation.y = transform.transform.rotation.y
                self.initial_pose.pose.orientation.z = transform.transform.rotation.z
                self.initial_pose.pose.orientation.w = transform.transform.rotation.w
                self.initialized = True
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                rospy.logwarn(exc)
                rospy.logwarn("Attempting again")
                attempts += 1

        rospy.logerr("Failed to lookup transform: {}".format(exc))
        rospy.logwarn("Initializing marker in hard coded pose")
        self.init_pose()
        return False

    def make_box_control(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_box(msg))
        msg.controls.append(control)
        return control

    def make_box(self, msg):
        marker = Marker()

        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.5
        return marker

    def create_marker(self):
        while not self.initialized:
            rospy.loginfo_throttle(
                10.0, "Waiting the first pose to initialize marker.")
        rospy.loginfo("Initializing marker at {}".format(str(
            self.initial_pose)))

        self.int_marker = InteractiveMarker()
        int_marker = self.int_marker
        int_marker.header.frame_id = self.frame_id
        int_marker.header.stamp = rospy.Time.now()
        int_marker.pose = self.initial_pose.pose
        int_marker.scale = 0.2
        int_marker.name = "Pose Target"
        int_marker.description = "Pose target for the end effector"

        self.make_box_control(int_marker)
        control = InteractiveMarkerControl()

        # Custom move on plane
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        # make a box which also moves on the plane
        control.markers.append(self.make_box(int_marker))
        control.always_visible = True
        int_marker.controls.append(copy.deepcopy(control))

        self.server.insert(int_marker, self.update_pose_callback)
        rospy.loginfo("\n\n\nMarker created\n\n\n")

    def apply_changes(self):
        self.server.applyChanges()

    def update_pose_callback(self, feedback):
        self.pose.header.frame_id = self.frame_id
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose = feedback.pose
        self.pose_pub.publish(self.pose)


if __name__ == '__main__':
    rospy.init_node('interactive_marker_node')

    try:
        interactiveTargetPose = SingleMarkerBroadcaster()
        interactiveTargetPose.create_marker()
        interactiveTargetPose.apply_changes()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
