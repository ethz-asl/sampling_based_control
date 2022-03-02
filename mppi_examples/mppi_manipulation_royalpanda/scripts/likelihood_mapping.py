#!usr/env/bin python

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times']
plt.rcParams['font.sans-serif'] = ['Helvetica']
plt.rcParams['font.size'] = 8
plt.rcParams['figure.figsize'] = [3.5, 2.625]  #[2.3, 1.5]
plt.rcParams['figure.dpi'] = 600
plt.rcParams['legend.frameon'] = False
plt.rcParams['savefig.bbox'] = 'tight'
plt.rcParams['savefig.pad_inches'] = 0.05
plt.rcParams['axes.grid'] = True
plt.rcParams[
    'text.latex.preamble'] = r'\usepackage{amsmath} \usepackage{amssymb}'


def naive(costs, gamma):
    return np.exp(-gamma * costs)


def baseline(costs, gamma):
    return np.exp(-gamma * (costs - np.min(costs)))


def modified(costs, gamma):
    return np.exp(-gamma * (costs - np.min(costs)) /
                  (np.max(costs) - np.min(costs)))


gamma = 10
max_cost = [100, 10, 2, 1]
min_cost = [0.8 * c for c in max_cost]  # 20% maximum cost reduction
print(f"max cost: {max_cost}")
print(f"min cost: {min_cost}")

fig, axs = plt.subplots(2, 2)
ax = []
for axr in axs:
    ax.extend(axr)

for i in range(len(max_cost)):

    costs = np.sort(min_cost[i] + np.linspace(0, max_cost[i], 100))

    title = r"$J_{max}$=" + str(max_cost[i]) + ", "
    title += r"$J_{min}$="
    title += str(min_cost[i])
    ax[i].set_title(title)
    ax[i].plot(costs, naive(costs, gamma), label="naive")
    ax[i].plot(costs, baseline(costs, gamma), label="baseline")
    ax[i].plot(costs, modified(costs, gamma), label="invariant")

    if i == 0 or i == 2:
        ax[i].set_ylabel(r"$\omega$")

    if i == 2 or i == 3:
        ax[i].set_xlabel(r"$J$")

    # if i==0:
    # 	ax[i].legend(frameon=True, loc="upper center", bbox_to_anchor=(1.0, 1.7), ncol=3)
    ax[i].set_axisbelow(True)

handles, labels = ax[0].get_legend_handles_labels()
fig.legend(handles,
           labels,
           loc='upper center',
           ncol=3,
           bbox_to_anchor=(0.5, 1.05))

fig.tight_layout()
fig.savefig("likelihood_mapping.pdf", format='pdf')
#plt.show()