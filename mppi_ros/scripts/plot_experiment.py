#! /usr/bin/env python
import argparse
import rosbag
import numpy as np
import matplotlib.pyplot as plt


# Hypothesis: a very large number of particles is ineffective in terms of quality solution
# and can even be detrimental in terms of performance:
# x: time
# y1: mean cost sample size 1
# y2: mean cost sample size 2
# ...
# yn: mean cost sample size n


def extract_data(data, bag):
    for topic, msg, t in bag.read_messages(topics=['/mppi_data']):
        if len(msg.weights.array) == 0:
            continue
        data['weights'].append(msg.weights.array)
        data['rollouts_cost'].append(msg.rollouts_cost.array)
        data['reset_time'].append(msg.reset_time)
        data['optimal_rollout_cost'].append(msg.optimal_rollout.total_cost)
    bag.close()


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
    parser = argparse.ArgumentParser(description='Plot data from experiment.')
    parser.add_argument('--bag-dir', type=str, help='path to the bag file containing experiment data.')
    args = parser.parse_args()

    import os
    import sys
    import glob

    bag_files = glob.glob(os.path.join(args.bag_dir, '*.bag'))
    print("Found {} bags in {}".format(len(bag_files), args.bag_dir))
    if len(bag_files) == 0:
        sys.exit(0)

    comparison_fig = plt.figure()
    comparison_ax = comparison_fig.add_subplot(111)

    for idx, bag_file in enumerate(bag_files):
        print("Opening: {}".format(bag_file))
        bag = rosbag.Bag(bag_file)

        d = {'reset_time': [],
             'rollouts_cost': [],
             'weights': [],
             'optimal_rollout_cost': []}

        extract_data(d, bag)

        fig = plt.figure()

        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)

        plot_samples(ax1, d)
        plot_cost(ax2, d)
        plot_cost(comparison_ax, d, fill=False, prefix=str(idx) + '_')

    plt.show()
