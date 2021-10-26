#!usr/env/bin python

import rospy
import time
from std_msgs.msg import Float64MultiArray, Float64
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench, Point
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker
from copy import deepcopy

# So to find the additional path to utilities
import sys
from os.path import dirname
print(f"Appending {dirname(__file__)}")
sys.path.append(dirname(__file__))
import numpy as np
from utilities import get_data

# For jacobian computation
import pinocchio as pin

# For plotting
import matplotlib.pyplot as plt

# For filtering estimated wrench
from scipy.signal import medfilt


def create_arrow(x, y, z, frame):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.points.append(Point())
    marker.points.append(Point())
    marker.pose.orientation.w = 1.0
    marker.points[1].x = x
    marker.points[1].y = y
    marker.points[1].z = z
    marker.scale.x = 0.05
    marker.scale.y = 0.1
    marker.scale.z = 0.2

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker


class Robot:
    def __init__(self):
        robot_urdf = rospy.get_param("/robot_description")
        self.model = pin.buildModelFromXML(robot_urdf)
        self.data = self.model.createData()

    def wrench_from_torque(self, frame, q, tau):
        frame_id = self.model.getFrameId(frame)
        pin.framesForwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.computeFrameJacobian(self.model, self.data, q, frame_id,
                                     pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        c, s = np.cos(q[2]), np.sin(q[2])
        J[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return np.linalg.pinv(J.T).dot(tau)


robot = Robot()
q = np.zeros((12, 1))
tau = np.zeros((12, 1))
wrench = np.zeros((6, 1))

REQUIRED_FIELDS = [
    "log/opt_time", "log/power_from_interaction", "log/stage_cost",
    "log/torque_command", "log/external_torque", "log/position_desired",
    "log/position_measured", "log/velocity_command", "log/velocity_measured",
    "log/velocity_mppi", "log/cartesian_limits_violation",
    "log/joint_limits_violation", "log/tank_state", "log/power_channels",
    "log/state"
]

data_for_plot = {
    "wrench_norm": [],
    "object_position": [],
    "dissipated_power": [],
}

log_prefixes = ["no_filter_1", "filter_in_out_11", "filter_in_out_5"]
log_tags = ["no filter", "filter_in_out", "filter_out"]
rospy.init_node("experiment_replay")

joint_state = JointState()
joint_state.name = [
    "x_base_joint", "y_base_joint", "pivot_joint", "panda_joint1",
    "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5",
    "panda_joint6", "panda_joint7", "panda_finger_joint1",
    "panda_finger_joint1"
]
joint_state.position = [0.0] * 12
joint_state.velocity = [0.0] * 12
joint_state.effort = [0.0] * 12
joint_state_publisher = rospy.Publisher("/joint_states",
                                        JointState,
                                        queue_size=1)

articulation_state = JointState()
articulation_state.name = ["articulation_joint"]
articulation_state.position = [0.0]
articulation_state.velocity = [0.0]
articulation_state_publisher = rospy.Publisher("/object_state",
                                               JointState,
                                               queue_size=1)

wrench_ros = Wrench()
wrench_publisher = rospy.Publisher("/wrench", Wrench, queue_size=1)
base_force_publisher = rospy.Publisher("/base_force", Marker, queue_size=1)

rospy.set_param("/use_sim_time", True)
ros_time = Clock()
clock_publisher = rospy.Publisher("/clock", Clock, queue_size=1)

pause = False


def pause_cb(req):
    global pause
    pause = not pause
    return EmptyResponse()


pause_service = rospy.Service("/pause", Empty, pause_cb)


def reset_cb(msg):
    """ TF does not like jumps back in time """
    global i
    i_temp = int(msg.data / 0.01)
    if i_temp < 0:
        i = 0
    elif i_temp > steps:
        i = steps
    else:
        i = i_temp
    rospy.loginfo(f"Resetting to time {msg.data}")


reset_subscriber = rospy.Subscriber("/reset", Float64, reset_cb, queue_size=1)

publish_ros = False
experiment_data = {prefix: deepcopy(data_for_plot) for prefix in log_prefixes}
for prefix in log_prefixes:
    data = get_data("/home/giuseppe/logs_tests_26_10_21",
                    required_fields=REQUIRED_FIELDS,
                    log_prefix=prefix)
    topics = [topic for topic in data.keys()]
    publishers = [
        rospy.Publisher(topic, Float64MultiArray, queue_size=1)
        for topic in topics
    ]
    msgs = [Float64MultiArray()] * len(publishers)

    i = 0
    steps = len(data[topics[0]][0])
    runtime = 0.0
    wrench = np.zeros((6, 1))
    print(f"The experiments last for {steps} steps.")
    while i < steps:
        for topic, pub, msg in zip(topics, publishers, msgs):
            val = data[topic][0][i]
            if isinstance(val, (float, int, np.float64)):
                val = [val]

            if topic == "state":
                for j in range(12):
                    q[j] = val[j]
                    tau[j] = val[-12 + j]
                    wrench = robot.wrench_from_torque("panda_grasp", q, tau)

                experiment_data[prefix]["object_position"].append(val[24])
                experiment_data[prefix]["wrench_norm"].append(
                    np.linalg.norm(wrench))

            if topic == "power_from_interaction":
                experiment_data[prefix]["dissipated_power"].append(np.sum(val))

            if publish_ros:
                msg.data = val
                pub.publish(msg)

                wrench_ros.force.x = wrench[0]
                wrench_ros.force.y = wrench[1]
                wrench_ros.force.z = wrench[2]
                wrench_ros.torque.x = wrench[3]
                wrench_ros.torque.y = wrench[4]
                wrench_ros.torque.z = wrench[5]
                wrench_publisher.publish(wrench_ros)

                joint_state.header.stamp = rospy.get_rostime()
                for j in range(12):
                    joint_state.position[j] = q[j]
                    joint_state.effort[j] = tau[j]
                joint_state_publisher.publish(joint_state)

                articulation_state.header.stamp = rospy.get_rostime()
                articulation_state.position[0] = val[24]
                articulation_state.velocity[0] = val[25]
                articulation_state_publisher.publish(articulation_state)

        ros_time = rospy.Time.from_sec(runtime)
        clock_publisher.publish(ros_time)

        marker = create_arrow(tau[0] / 10., tau[1] / 10., 0.0, "base_link")
        base_force_publisher.publish(marker)

        while pause:
            time.sleep(0.1)

        runtime = i * 0.015
        #time.sleep(0.01)
        i += 1


# Plot data inferring time from length of data
def set_style(ax, title, xlabel, ylabel):
    ax.set_title(title, fontsize=40)
    ax.set_ylabel(ylabel, fontsize=30)
    ax.set_xlabel(xlabel, fontsize=30)
    ax.tick_params(axis='both', which='both', labelsize=40)
    ax.legend(fontsize=35, loc='upper right')
    ax.grid(True)


def add_subplot_axes(ax, rect, facecolor='w'):
    fig = ax.figure
    box = ax.get_position()
    width = box.width
    height = box.height
    inax_position = ax.transAxes.transform(rect[0:2])
    transFigure = fig.transFigure.inverted()
    infig_position = transFigure.transform(inax_position)
    x = infig_position[0]
    y = infig_position[1]
    width *= rect[2]
    height *= rect[3]  # <= Typo was here
    subax = fig.add_axes([x, y, width, height],
                         facecolor=facecolor)  # matplotlib 2.0+
    x_labelsize = subax.get_xticklabels()[0].get_size()
    y_labelsize = subax.get_yticklabels()[0].get_size()
    x_labelsize *= rect[2]**0.5
    y_labelsize *= rect[3]**0.5
    subax.xaxis.set_tick_params(labelsize=x_labelsize)
    subax.yaxis.set_tick_params(labelsize=y_labelsize)
    return subax


# Axis for object position
fig, ax1 = plt.subplots()

# Axis for tank energy
min_energy = 2
fig, ax2 = plt.subplots()
ax_sub = add_subplot_axes(ax2, [0.1, 0.1, 0.6, 0.6])
ax_sub.axhline(y=min_energy, ls="--", lw=6, c='r', label="min energy")

# Axis for wrench norm
fig, ax3 = plt.subplots()

for prefix, log_tag in zip(log_prefixes, log_tags):
    t = np.arange(0, len(experiment_data[prefix]["dissipated_power"])) * 0.015

    # filter wrench
    wrench_filt = medfilt(experiment_data[prefix]["wrench_norm"], 7)

    ax1.plot(t,
             experiment_data[prefix]["object_position"],
             lw=6,
             label=log_tag)
    set_style(ax1, "Object Position", "t[s]", "deg")

    initial_energy = 10
    tank_energy = initial_energy + np.cumsum(
        experiment_data[prefix]["dissipated_power"]) * 0.015
    ax2.plot(t, tank_energy, lw=6, label=log_tag)

    ax_sub.set_xlim(25, 60)
    ax_sub.set_ylim(-1, 11)
    ax_sub.plot(t, tank_energy, lw=6)
    ax_sub.legend(fontsize=40)
    ax_sub.tick_params(axis='both', which='both', labelsize=30)

    set_style(ax2, "", "time [s]", "energy [J]")
    #ax2.axvspan(30, 41, alpha=0.2, color='red')

    ax3.plot(t, wrench_filt, lw=6, label=log_tag)
    set_style(ax3, "", "time [s]", "wrench norm [N]")

    print(
        f"Average interaction wrench for {log_tag}: {np.mean(wrench_filt[wrench_filt>2])} N"
    )

plt.show()
