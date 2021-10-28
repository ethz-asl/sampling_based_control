#!usr/env/bin python

import os
import sys
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from scipy.signal import medfilt
from copy import deepcopy

# find the additional path to utilities
from os.path import dirname
print(f"Appending {dirname(__file__)}")
sys.path.append(dirname(__file__))
from utilities import get_data, add_subplot_axes


def set_style(ax, title, xlabel, ylabel):
    ax.set_title(title)  #, fontsize=40)
    ax.set_ylabel(ylabel)  #, fontsize=30)
    ax.set_xlabel(xlabel)  #, fontsize=30)
    ax.legend()
    #ax.tick_params(axis='both', which='both', labelsize=40)
    #ax.legend(fontsize=35, loc='upper right')
    #ax.grid(True)


class Robot:
    """ Utility class to get the wrench from joint torques """
    def __init__(self, robot_urdf):
        self.model = pin.buildModelFromUrdf(robot_urdf)
        self.data = self.model.createData()
        self.q = np.zeros((self.model.nq, 1))
        self.tau = np.zeros((self.model.nv, 1))

    def wrench_from_torque(self, frame):
        frame_id = self.model.getFrameId(frame)
        pin.framesForwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.computeFrameJacobian(self.model, self.data, self.q, frame_id,
                                     pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        c, s = np.cos(self.q[2]), np.sin(self.q[2])
        J[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return np.linalg.pinv(J.T).dot(self.tau)


robot = Robot(os.path.join(dirname(__file__), "panda_mobile.urdf"))

REQUIRED_FIELDS = [
    "log/power_from_interaction", "log/tank_state", "log/power_channels",
    "log/state"
]

data_for_plot = {
    "wrench_norm": [],
    "object_position": [],
    "dissipated_power": [],
}

log_prefixes = ["no_filter_1", "filter_in_out_11", "filter_in_out_5"]
log_tags = ["no filter", "filter_in_out", "filter_out"]
experiment = {prefix: deepcopy(data_for_plot) for prefix in log_prefixes}

for prefix in log_prefixes:
    data = get_data("/home/giuseppe/logs_tests_26_10_21",
                    required_fields=REQUIRED_FIELDS,
                    log_prefix=prefix)

    first_field = list(data.values())[0]

    # data has for each key as many vectors as the experiments
    # with the same log prefix. As we specify the exact prefix, this has
    # length 1
    steps = len(first_field[0])
    print(f"The experiments last for {steps} steps.")
    for i in range(steps):
        for key in data.keys():
            val = data[key][0][i]
            if key == "state":
                for j in range(12):
                    robot.q[j] = val[j]
                    robot.tau[j] = val[-12 + j]
                wrench = robot.wrench_from_torque("panda_hand")

                experiment[prefix]["object_position"].append(val[24])
                experiment[prefix]["wrench_norm"].append(
                    np.linalg.norm(wrench))

            if key == "power_from_interaction":
                experiment[prefix]["dissipated_power"].append(np.sum(val))

# axis for object position
fig_pos, ax_pos = plt.subplots()

# axis for tank energy
fig_eng, ax_eng = plt.subplots()

# axis for wrench norm
fig_wnm, ax_wnm = plt.subplots()

# create sub axis for closeup
ax_eng_sub = add_subplot_axes(ax_eng, [0.1, 0.1, 0.6, 0.6])

# NOTE
# infer time from data length (recorded every 15 steps, each step being 1ms)
# also filter wrench from spikes

for prefix, log_tag in zip(log_prefixes, log_tags):
    t = np.arange(0, len(experiment[prefix]["dissipated_power"])) * 0.015

    wrench_filt = medfilt(experiment[prefix]["wrench_norm"], 7)
    ax_pos.plot(t, experiment[prefix]["object_position"], label=log_tag)

    initial_energy = 10
    tank_energy = initial_energy + np.cumsum(
        experiment[prefix]["dissipated_power"]) * 0.015
    ax_eng.plot(t, tank_energy, label=log_tag)
    ax_eng_sub.plot(t, tank_energy)

    ax_wnm.plot(t, wrench_filt, label=log_tag)

    print(
        f"Average interaction wrench for {log_tag}: {np.mean(wrench_filt[wrench_filt>2])} N"
    )

# plot min energy in subplot
min_energy = 2
ax_eng_sub.axhline(y=min_energy, ls="--", c='r', label="min energy")

# Set style for each plot
set_style(ax_pos, "Object Position", "t[s]", "deg")
set_style(ax_eng, "", "time [s]", "energy [J]")
set_style(ax_wnm, "", "time [s]", "wrench norm [N]")

# set style for subplot
ax_eng_sub.set_xlim(25, 60)
ax_eng_sub.set_ylim(-1, 11)
ax_eng_sub.legend()
ax_eng_sub.tick_params(axis='both', which='both', labelsize=30)

plt.show()
