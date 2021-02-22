#! /usr/bin/env  python

import os
from rospkg import RosPack
import rosbag
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

import seaborn as sns
sns.set_theme()
sns.set_context("paper")

from manipulation_msgs.msg import State

pkg_dir = RosPack().get_path("mppi_manipulation")


class Manipulator:
    def __init__(self, model, data):
        self.model = model
        self.data = data

    def get_frame_velocity(self, q, v, frame_id) -> pin.Motion:
        pin.forwardKinematics(self.model, self.data, q, v)

        return pin.getFrameVelocity(self.model, self.data, self.model.getFrameId(frame_id),
                                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)


def get_manipulator_model():
    urdf_filename = pkg_dir + '/data/panda.urdf'
    model = pin.buildModelFromUrdf(urdf_filename)
    data = model.createData()
    print("Created manipulator model: nq={nq}, nv={nv}".format(nq=model.nq, nv=model.nv))
    return Manipulator(model, data)


# Open bag
bag_file = os.path.join(pkg_dir, "data", "bags", "exp_mobile_3.bag")
bag = rosbag.Bag(bag_file)

time = []
time_data = []
arm_joint_positions = []
arm_joint_velocities = []
base_twist = []

for topic, msg, t in bag.read_messages(topics=['/x_nom', '/joint_states', '/cost', '/mode', '/mppi_data']):
    if topic == '/x_nom':
        time.append(t.to_sec())
        arm_joint_positions.append(np.array(msg.arm_state.position))
        arm_joint_velocities.append(np.array(msg.arm_state.velocity))
        base_twist.append(np.array([msg.base_twist.linear.x, msg.base_twist.linear.y, msg.base_twist.linear.z]))
    if topic == '/mppi_data':
        time_data.append(t.to_sec())

time_data = np.array(time_data)
rate = 1. / (time_data[1:] - time_data[:-1])

bag.close()
time = np.array(time) - time[0]

ee_velocity_norm = []
base_velocity_norm = []
manipulator: Manipulator = get_manipulator_model()
for aq, av, bt in zip(arm_joint_positions, arm_joint_velocities, base_twist):
    ee_vel : pin.Motion = manipulator.get_frame_velocity(aq, av, "panda_hand")
    ee_velocity_norm.append(np.linalg.norm(ee_vel.linear))
    base_velocity_norm.append(np.linalg.norm(bt))
print(time[1:] - time[:-1])

ee_velocity_norm_filt = savgol_filter(ee_velocity_norm, 2501, 1)
base_velocity_norm_filt = savgol_filter(base_velocity_norm, 2501, 1)

print("Plotting data")
fig, ax = plt.subplots()
steps = len(ee_velocity_norm_filt)
t = np.array(range(0, steps-15000))/800.0
ax.plot(time, ee_velocity_norm_filt, label="end effector", linewidth=2)
ax.plot(time, base_velocity_norm_filt, label="base", linewidth=2)

ax.set_xlabel("time [s]", fontsize=40)
ax.set_ylabel("velocity norm", fontsize=40)
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
ax.yaxis.set_minor_locator(AutoMinorLocator(10))
ax.xaxis.set_minor_locator(AutoMinorLocator(10))


ax.set_ybound(lower=0, upper=0.15)
#ax.set_xticks([])

ax.grid(which='major')
ax.grid(which='minor')
# plt.plot(ee_velocity_norm, linestyle="--", label="end effector velocity norm")
# plt.plot(base_velocity_norm, linestyle="--", label="base velocity norm")

fig, ax = plt.subplots()
ax.plot(time_data[1:], rate)

plt.grid()
plt.yticks(fontsize=18)
plt.xticks(fontsize=18)
plt.legend(fontsize=30)
plt.show()
