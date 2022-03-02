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

# to get palette as in the notebooks plots
import seaborn as sns

from utilities import get_data, add_subplot_axes

plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times']
plt.rcParams['font.sans-serif'] = ['Helvetica']
plt.rcParams['font.size'] = 10
plt.rcParams['figure.figsize'] = [3.5, 2.625]  #[2.3, 1.5]
plt.rcParams['figure.dpi'] = 250
plt.rcParams['legend.frameon'] = False
plt.rcParams['savefig.bbox'] = 'tight'
plt.rcParams['savefig.pad_inches'] = 0.05
plt.rcParams['axes.grid'] = True
plt.rcParams[
    'text.latex.preamble'] = r'\usepackage{amsmath} \usepackage{amssymb}'


def set_style(ax, title, xlabel, ylabel):
    ax.set_title(title)  #, fontsize=40)
    ax.set_ylabel(ylabel)  #, fontsize=30)
    ax.set_xlabel(xlabel)  #, fontsize=30)
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

log_prefixes = ["no_filter_1", "filter_in_out_5"]  #, "filter_in_out_11", ]
log_tags = [r'$\Pi_{N}$', r'$\Pi_{IO}$']  #, r'$\Pi_{IO}$']
log_color = [sns.color_palette()[0], sns.color_palette()[3]]

experiment = {prefix: deepcopy(data_for_plot) for prefix in log_prefixes}

for idx, prefix in enumerate(log_prefixes):
    data = get_data("/home/giuseppe/logs_tests_26_10_21",
                    required_fields=REQUIRED_FIELDS,
                    log_prefix=prefix,
                    infer_time_from='state')

    # data has for each key as many vectors as the experiments
    # with the same log prefix. As we specify the exact prefix, this has
    # length 1
    steps = len(data['state'][0])
    print(f"The experiments last for {steps} steps.")
    for i in range(steps):
        for key in data.keys():
            if key == 'experiment_name':
                continue

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
ax_eng.set_xlim(15, 60)

# axis for wrench norm
fig_wnm, ax_wnm = plt.subplots()
ax_wnm.set_xlim(15, 60)
ax_wnm.set_ylim(-10, 110)

# create sub axis for closeup
ax_eng_sub = add_subplot_axes(ax_eng, [0.13, 0.14, 0.6, 0.6])

# NOTE
# infer time from data length (recorded every 15 steps, each step being 1ms)
# also filter wrench from spikes

# Also apply a dt for each time vector to make sure the experiments are aligned at release time
global_release_time = [47.8, 45.2, 40.75]
global_delta_time = [tr - global_release_time[0] for tr in global_release_time]

i = 0
for prefix, log_tag in zip(log_prefixes, log_tags):
    t = np.arange(0, len(
        experiment[prefix]["dissipated_power"])) * 0.015 - global_delta_time[i]

    wrench_filt = medfilt(experiment[prefix]["wrench_norm"], 7)
    ax_pos.plot(t, experiment[prefix]["object_position"], label=log_tag)

    initial_energy = 10
    tank_energy = initial_energy + np.cumsum(
        experiment[prefix]["dissipated_power"]) * 0.015
    ax_eng.plot(t, tank_energy, label=log_tag, color=log_color[i])
    ax_eng.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3)
    ax_eng_sub.plot(t, tank_energy, c=log_color[i])

    ax_wnm.plot(t, wrench_filt, label=log_tag, color=log_color[i])
    ax_wnm.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3)

    print(
        f"Average interaction wrench for {log_tag}: {np.mean(wrench_filt[wrench_filt>2])} N"
    )
    i += 1

# plot min energy in subplot
min_energy = 2
ax_eng_sub.axhline(y=min_energy, ls="--", c='k', label="min energy")

# Set style for each plot
set_style(ax_pos, "Object Position", "t[s]", "deg")
set_style(ax_eng, "", "time [s]", "energy [J]")
set_style(ax_wnm, "", "time [s]", "wrench norm [N]")

# set style for subplot
ax_eng_sub.set_xlim(25, 60)
ax_eng_sub.set_ylim(-1, 11)
ax_eng_sub.legend(loc='upper right', frameon=True, fontsize=6)

# add line to connect to the zoomed inset
from matplotlib.patches import ConnectionPatch

kw = dict(linestyle="--", color="gray")
cp1 = ConnectionPatch(((25 - 15.) / (60 - 15), (10 + 200.) / (20 + 200)),
                      (0, 1),
                      "axes fraction",
                      "axes fraction",
                      axesA=ax_eng,
                      axesB=ax_eng_sub,
                      **kw)
cp2 = ConnectionPatch((1, (0 + 200.) / (20 + 200)), (1, 1),
                      "axes fraction",
                      "axes fraction",
                      axesA=ax_eng,
                      axesB=ax_eng_sub,
                      **kw)
ax_eng.add_artist(cp1)
ax_eng.add_artist(cp2)

fig_eng.savefig("energy_tank.pdf", format='pdf')
fig_wnm.savefig("wrench_norm.pdf", format='pdf')

save_animation = True
if save_animation:
    import matplotlib.animation as animation

    # Set up formatting for the movie files
    # each data point is every 0.015 seconds, we subsample by a factor of 10
    # leading to 0.15 seconds per point = 1 / 0.15 ~ 6 fps
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=6, metadata=dict(artist='Me'), bitrate=1800)

    def subsample(a, factor):
        return a[::factor]

    def plot_energy(i, ax, x, y, label):
        ax.clear()
        ax.plot(x[:i], y[:i], label=label)

        min_energy = 2
        ax.axhline(y=min_energy, ls="--", c='k', label="min energy")
        ax.legend(frameon=True)
        # ax.legend(loc="lower left", frameon=True, bbox_to_anchor=(0.01, 0.15))
        ax.set_xlabel("time [s]")
        ax.set_ylabel("energy [J]")
        ax.set_xlim(15, 70)
        if y[i] > 0:
            ax.set_ylim(1, 11)

    def plot_wrench(i, ax, x, y, label):
        ax.clear()
        ax.plot(x[:i], y[:i], label=label, color="red")

        ax.legend(loc="upper left", frameon=True)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("wrench norm [N]")
        ax.set_xlim(15, 70)
        ax.set_ylim(-10, 130)

    for prefix, log_tag in zip(log_prefixes, log_tags):
        initial_energy = 10
        time = np.arange(0, len(
            experiment[prefix]["dissipated_power"])) * 0.015
        energy = initial_energy + np.cumsum(
            experiment[prefix]["dissipated_power"]) * 0.015
        wrench = medfilt(experiment[prefix]["wrench_norm"], 7)

        time = subsample(time, 10)
        energy = subsample(energy, 10)
        wrench = subsample(wrench, 10)

        fig, ax = plt.subplots()
        energy_ani = animation.FuncAnimation(fig,
                                             plot_energy,
                                             frames=len(energy),
                                             fargs=(ax, time, energy,
                                                    "tank energy"))

        fig.tight_layout(pad=2.0)
        energy_ani.save(f'{prefix}_energy_animated_lowres.mp4', writer=writer)

        fig, ax = plt.subplots()
        wrench_ani = animation.FuncAnimation(fig,
                                             plot_wrench,
                                             frames=len(wrench),
                                             fargs=(ax, time, wrench,
                                                    "external wrench"))

        fig.tight_layout(pad=2.0)
        wrench_ani.save(f'{prefix}_wrench_animated_lowres.mp4', writer=writer)

#plt.show()
