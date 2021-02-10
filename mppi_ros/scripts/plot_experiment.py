#! /usr/bin/env python
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from rospkg import RosPack

def plot_samples(ax, data):
    t = np.arange(len(data['weights']))
    weights = np.asarray(data['weights'])
    weights_min = weights.min(axis=1)
    weights_max = weights.max(axis=1)
    weights_squared = np.square(weights)
    effective_samples = 1.0 / (weights.shape[0] * weights_squared.sum(axis=1))

    ax.plot(t, effective_samples, lw=2, label='effective samples %', color='green')

    ax.plot(t, weights_max, lw=1, label='max weight', color='blue')
    ax.fill_between(t, weights_max, weights_min, facecolor='blue', alpha=0.5)
    ax.plot(t, weights_min, lw=1, label='min weight', color='red')
    ax.grid()
    ax.set_title("Samples")
    ax.legend()


def plot_cost(ax, data, prefix='', fill=True):
    t = np.arange(len(data['rollouts_cost']))
    costs = np.asarray(data['rollouts_cost'])
    samples = costs.shape[1]
    optimal_rollout_cost = np.asarray(data['optimal_rollout_cost'])
    mean_costs = costs.mean(axis=1)
    std_costs = costs.std(axis=1)
    ax.set_title("Cost")
    ax.plot(t, mean_costs, lw=2, label=prefix + 'mean cost, samples# = ' + str(samples))
    if fill:
        ax.fill_between(t, mean_costs + std_costs, mean_costs - std_costs, facecolor='orange', alpha=0.5,
                        label='$\sigma$ range')
    ax.grid()
    ax.legend()


if __name__ == "__main__":
    csv_file = os.path.join(RosPack().get_path("mppi_ros"), "log", "record.csv")
    df = pd.read_csv(csv_file, converters={'stage_cost': eval, 'step_count': eval, 'weights': eval})

    fig = plt.figure()
    ax = fig.add_subplot(111)

    for index, row in df.iterrows():
        if row['experiment_id'].startswith("mppi_panda"):
            print("mppi_panda experiment found")
            print(row['nr_rollouts'])
            if len(row['step_count']) > 0:
                postfix = ''
                if row['tree_search']:
                    postfix = 'tree'
                label = "mppi_panda_" + str(row['nr_rollouts']) + postfix
                ax.plot(row['step_count'], row['stage_cost'], label=label)

    ax.set_title('Stage cost')
    ax.grid()
    ax.legend()
    plt.show()
