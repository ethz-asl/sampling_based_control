#!usr/env/bin python

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times']
plt.rcParams['font.sans-serif'] = ['Helvetica']
plt.rcParams['font.size'] = 10
plt.rcParams['figure.figsize'] = [3.5, 2.625]  #[2.3, 1.5]
plt.rcParams['figure.dpi'] = 600
plt.rcParams['legend.frameon'] = False
plt.rcParams['savefig.bbox'] = 'tight'
plt.rcParams['savefig.pad_inches'] = 0.05
plt.rcParams['axes.grid'] = True
plt.rcParams[
    'text.latex.preamble'] = r'\usepackage{amsmath} \usepackage{amssymb}'

fig, ax = plt.subplots()

t = np.linspace(0, 15, 1000)
for alpha in [1000, 100, 10, 1, 0.1]:
    ax.plot(t, 10 * np.exp(-alpha * t), label=r'$\alpha={}$'.format(alpha))

# show trend with arrow
import matplotlib.patches as patches

style = "Simple, tail_width=0.5, head_width=4, head_length=8"
kw = dict(arrowstyle=style, color="k", zorder=5)
arr = patches.FancyArrowPatch((0, 0), (7, 6),
                              connectionstyle="angle3,angleA=80,angleB=20",
                              **kw)
ax.add_patch(arr)

ax.legend(frameon=True)
ax.set_xlabel("time [s]")
ax.set_ylabel("energy [J]")
ax.set_axisbelow(True)

fig.savefig("energy_flow_plot.pdf", format='pdf')
