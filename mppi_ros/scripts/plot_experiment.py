#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from rospkg import RosPack
from matplotlib.colors import LogNorm
import math
sns.set_theme()
sns.set_context("paper")
sns.set(font_scale=2)

# Avoid Type3 fonts (RA-L submission)
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42


class Plotter:
    def __init__(self, experiment_id="mppi_panda"):
        self.df = self.get_data(experiment_id)

    def get_data(self, experiment_id):
        csv_file = os.path.join(RosPack().get_path("mppi_ros"), "log",
                                "record.csv")
        # df = pd.read_csv(csv_file, converters={'stage_cost': eval, 'step_count': eval, 'weights': eval, 'time': eval,
        #                                          'effective_samples': eval})
        df = pd.read_csv(csv_file)
        df['tree_search'] = df['tree_search'].apply(lambda x: "Tree"
                                                    if x else "Monte Carlo")
        df = df.rename(columns={'tree_search': 'Sampling Strategy'})

        if type(experiment_id) in [list, tuple]:
            df['experiment'] = ""
            print("Looking for experiments with ids: ")
            for exp in experiment_id:
                print("  - " + exp)
                df.loc[df['id'].str.contains(exp), 'experiment'] = exp

            df = df.loc[df['experiment'] != ""]
        else:
            print("Looking for experiments with id: {}".format(experiment_id))
            df = df[df['id'].str.contains(experiment_id)]
        print("Found {}".format(len(df)))
        return df

    def plot_average_cost_per_rollout(self, tree=False):
        # Mean and std
        strategy_name = "Tree" if tree else "Monte Carlo"
        average_cost = {}
        all_rollouts_nr = self.df.loc[self.df['Sampling Strategy'] ==
                                      strategy_name]['nr_rollouts'].unique()
        for rollouts_nr in all_rollouts_nr:
            df = self.df.loc[(self.df['nr_rollouts'] == rollouts_nr)
                             & (self.df['Sampling Strategy'] == strategy_name)]
            if rollouts_nr not in average_cost:
                average_cost[rollouts_nr] = [df['stage_cost'].mean()]
            else:
                average_cost[rollouts_nr].append(df['stage_cost'].mean())

        average_cost_mean = [
            np.mean(average_cost_vector)
            for average_cost_vector in average_cost.values()
        ]
        average_cost_std = [
            np.std(average_cost_vector)
            for average_cost_vector in average_cost.values()
        ]

        # Build the plot
        fig, ax = plt.subplots()
        ax.bar(all_rollouts_nr,
               average_cost_mean,
               yerr=average_cost_std,
               align='center',
               alpha=0.5,
               ecolor='black',
               capsize=10)
        ax.set_xlabel('Numbers of samples')
        ax.set_xticks(all_rollouts_nr)
        ax.set_title('Average cost')
        ax.yaxis.grid(True)

        # Save the figure and show
        plt.tight_layout()

    def plot_average_cost_per_substep(self, tree=False):
        # Mean and std
        strategy_name = "Tree" if tree else "Monte Carlo"
        average_cost = {}
        all_substeps_nr = self.df.loc[self.df['Sampling Strategy'] ==
                                      strategy_name]['substeps'].unique()
        for substeps_nr in all_substeps_nr:
            df = self.df.loc[(self.df['substeps'] == substeps_nr)
                             & (self.df['Sampling Strategy'] == strategy_name)]
            if substeps_nr not in average_cost:
                average_cost[substeps_nr] = [df['stage_cost'].mean()]
            else:
                average_cost[substeps_nr].append(df['stage_cost'].mean())

        average_cost_mean = [
            np.mean(average_cost_vector)
            for average_cost_vector in average_cost.values()
        ]
        average_cost_std = [
            np.std(average_cost_vector)
            for average_cost_vector in average_cost.values()
        ]

        # Build the plot
        fig, ax = plt.subplots()
        ax.bar(all_substeps_nr,
               average_cost_mean,
               yerr=average_cost_std,
               align='center',
               alpha=0.5,
               ecolor='black',
               capsize=10)
        ax.set_xlabel('Number of substeps')
        ax.set_xticks(all_substeps_nr)
        ax.set_title('Average cost')
        ax.yaxis.grid(True)

        # Save the figure and show
        plt.tight_layout()

    def plot_average_cost_per_rollout_tree(self):
        self.plot_average_cost_per_rollout(tree=True)

    def plot_average_cost(self, aggregator, hue=None, x_label=None):
        fig, ax = plt.subplots()
        sns.barplot(x=aggregator, y="stage_cost", hue=hue, data=self.df)
        ax.set_ylabel("Average stage cost")
        ax.set_xlabel(aggregator if x_label is None else x_label)

    def plot_effective_samples_per_rollout(self, tree=False):
        plt.figure()
        strategy_name = "Tree" if tree else "Monte Carlo"
        df = self.df.loc[self.df['Sampling Strategy'] == strategy_name]
        nr_samples = len(df["nr_rollouts"].unique())
        sns.lineplot(data=df,
                     x="index",
                     y="effective_samples",
                     hue="nr_rollouts",
                     ci="sd",
                     palette=sns.color_palette("tab10", n_colors=nr_samples))

    def plot_effective_samples(self, aggregator, style=None, x='index', x_label=None, col=None):
        fig, ax = plt.subplots()
        nr_samples = len(self.df[aggregator].unique())
        sns.relplot(data=self.df,
                     x=x,
                     y="effective_samples",
                     hue=aggregator,
                     col=col,
                     style=style,
                     ci="sd", kind='line',
                     palette=sns.color_palette("tab10", n_colors=nr_samples))
        ax.set_ylabel("Effective Samples")
        ax.set_xlabel(x if x_label is None else x_label)

    def plot_effective_samples_per_rollout_tree(self):
        self.plot_effective_samples_per_rollout(tree=True)

    def plot_cost_comparison(self):

        df_no_tree = self.df.loc[self.df['Sampling Strategy'] == "Monte Carlo"]
        sample_sizes_no_tree = df_no_tree["nr_rollouts"].unique()

        df_tree = self.df.loc[self.df['Sampling Strategy'] == "Tree"]
        sample_sizes_tree = df_tree["nr_rollouts"].unique()

        common_sample_sizes = set(sample_sizes_tree).intersection(
            set(sample_sizes_no_tree))
        time_padding = 0.1
        for samples_size in common_sample_sizes:
            fig, axs = plt.subplots(nrows=1, ncols=5, sharey=True)
            for i in range(5):
                df_ = self.df.loc[(self.df['nr_rollouts'] == samples_size)
                                  & (self.df['time'] < (i + 1) * 5) &
                                  (self.df['time'] > (i * 5 + time_padding))]
                df_['stage_cost'] = df_['stage_cost'].apply(
                    lambda x: np.exp(-x))
                sns.lineplot(data=df_,
                             x="time",
                             y="stage_cost",
                             hue="Sampling Strategy",
                             legend=True,
                             ci="sd",
                             palette=sns.color_palette("tab10", n_colors=2),
                             ax=axs[i])
                axs[i].set_title("Target {}".format(i + 1), fontsize=20)
                axs[i].set_xlabel("t [s]", fontsize=20)
                axs[i].set_ylabel("stage cost", fontsize=20)
                axs[i].set_ybound(lower=-0.01, upper=1.2)
                plt.legend(fontsize=18)

    def plot_cost(self, aggregator, x='index', style=None, x_label=None, y_log=False):
        fig, ax = plt.subplots()
        nr_samples = len(self.df[aggregator].unique())
        sns.lineplot(data=self.df,
                     x=x,
                     y="stage_cost",
                     hue=aggregator,
                     style=style,
                     legend=True,
                     ci="sd",
                     palette=sns.color_palette("tab10", n_colors=nr_samples))
        ax.set_ylabel("Stage Cost")
        ax.set_xlabel(x if x_label is None else x_label)
        if y_log:
            ax.set_yscale("log")

    def plot_average_cost_samples_comparison(self):
        different_samples = len(self.df["nr_rollouts"].unique())
        ax = sns.lineplot(data=self.df,
                          x="index",
                          y="stage_cost",
                          hue="nr_rollouts",
                          legend=True,
                          ci="sd",
                          palette=sns.color_palette(
                              "tab10", n_colors=different_samples))
        df = self.df.loc[self.df['index'] < 300]
        #ax = sns.barplot(x="nr_rollouts", y="stage_cost", data=df, capsize=.2)
        #ax.set_title("Whole Body Differential Base", fontsize=20)
        #ax.set_xticklabels([5, 10, 50, 100, 200, 500, 1000, 2000], size = 30)
        #ax.set
        ax.set_yticklabels(ax.get_yticks(), size=30)
        ax.set_xlabel("samples", fontsize=40)
        #ax.set_yscale("log")
        ax.set_ylabel("average stage cost", fontsize=40)
        # ax.set_ybound(lower=-0.01, upper=1.2)
        #plt.legend(fontsize=18)

    def plot_average_cost_with_without_momentum(self):
        fig, ax = plt.subplots()
        df = self.df.loc[(self.df['beta'] < 1)]
        ax = sns.barplot(x="id",
                         y="stage_cost",
                         hue="beta",
                         data=df,
                         capsize=.2)
        # ax = sns.barplot(x="nr_rollouts", y="stage_cost", data=df, capsize=.2)

        ax.legend_.remove()
        h, l = ax.get_legend_handles_labels()
        ax.legend(h, ["Without momentum", "With momentum"], fontsize=30)
        ax.set_xticklabels(
            ["shelf", "dishwasher", "microwave", "drawer", "target reaching"],
            size=30)
        ax.set_yticklabels(ax.get_yticks(), size=30)
        #ax.set_xticklabels([20, 50, 100, 200, 500, 1000], size = 30)
        ax.set_ylabel("average stage cost", fontsize=40)
        ax.set_xlabel("", fontsize=40)

    def plot_cost_momentum_comparison_tree(self):

        df_tree = self.df.loc[self.df['Sampling Strategy'] == "Tree"]
        sample_sizes_tree = df_tree["nr_rollouts"].unique()

        df_tree['beta'] = df_tree['beta'].apply(lambda x: "with"
                                                if x > 0 else "without")
        df_tree = df_tree.rename(columns={'beta': 'momentum'})
        time_padding = 0.15
        for samples_size in sample_sizes_tree:
            fig, axs = plt.subplots(nrows=1, ncols=5, sharey=True)
            for i in range(5):
                df_ = df_tree.loc[(self.df['nr_rollouts'] == samples_size) & (
                    df_tree['time'] < (i + 1) * 5 - time_padding) &
                                  (df_tree['time'] > (i * 5 + time_padding))]
                #df_['stage_cost'] = df_['stage_cost'].apply(lambda x : np.exp(-x))
                sns.lineplot(data=df_,
                             x="time",
                             y="stage_cost",
                             hue="momentum",
                             legend=False,
                             ci="sd",
                             palette=sns.color_palette("tab10", n_colors=2),
                             ax=axs[i])
                axs[i].set_title("Target {}".format(i + 1), fontsize=20)
                axs[i].set_xlabel("t [s]", fontsize=20)
                #axs[i].set_yscale('log')
                axs[i].set_ylabel("stage cost", fontsize=20)
                #axs[i].set_ybound(lower=-0.01, upper=1.2)

    def plot_momentum_comparison(self):
        plt.figure()

        df_no_tree = self.df.loc[self.df['Sampling Strategy'] == "Monte Carlo"]
        sample_sizes_no_tree = df_no_tree["nr_rollouts"].unique()

        for samples_size in sample_sizes_no_tree:
            plt.figure()
            df_ = df_no_tree.loc[df_no_tree['nr_rollouts'] == samples_size]
            ax = sns.lineplot(data=df_,
                              x="time",
                              y="effective_samples",
                              ci="sd",
                              hue="alpha",
                              size="beta")
            ax.set_title("Effective Samples ({} samples)".format(samples_size))

        df_tree = self.df.loc[self.df['Sampling Strategy'] == "Tree"]
        sample_sizes_tree = df_tree["nr_rollouts"].unique()

        for samples_size in sample_sizes_tree:
            plt.figure()
            df_ = df_tree.loc[df_tree['nr_rollouts'] == samples_size]
            ax = sns.lineplot(data=df_,
                              x="time",
                              y="effective_samples",
                              ci="sd",
                              hue="alpha",
                              size="beta")
            ax.set_title(
                "Effective Samples FD-MCTS ({} samples)".format(samples_size))

    @staticmethod
    def moving_average(x, w):
        return np.convolve(x, np.ones(w), 'valid') / w

    def plot_rate(self):
        colors = ['g', 'b', 'r']
        fig, ax = plt.subplots()
        for experiment_id, color in zip(self.df['id'].unique(), colors):
            df = self.df.loc[self.df['id'] == experiment_id]
            #ax.plot(df['time'], df['rate'], linestyle="--", c=color)

            window = 100
            average_rate = self.moving_average(df['rate'], window)
            time_average_rate = df['time'][int(window / 2):int(window / 2) +
                                           len(average_rate)]
            print("{id}, {min_rate}, {max_rate}, {average_rate}".format(
                id=experiment_id,
                min_rate=min(df['rate'][2:]),
                max_rate=max(df['rate']),
                average_rate=np.mean(df['rate'])))
            ax.plot(time_average_rate,
                    average_rate,
                    linewidth=2,
                    label=experiment_id.strip("manipulation"),
                    c=color)
            ax.set_yticklabels(ax.get_yticks(), size=30)
            ax.set_xticklabels(ax.get_xticks(), size=30)
            ax.set_xlabel("time [s]", fontsize=40)
            ax.set_ylabel("rate [Hz]", fontsize=40)

        plt.legend(fontsize=35)

    def plot_rollout_costs(self, experiment_id, logarithmic=True):
        strategy_name = "Monte Carlo"
        df = self.df.loc[self.df['id'].str.contains(experiment_id)
                         & (self.df['Sampling Strategy'] == strategy_name)]
        costs = []

        # convert dataframe of strings to a 2D np array
        costs_temp = df['cost_history']
        for index, row in costs_temp.iteritems():
            line = row[1:-1] # get rid of parentheisis first
            costs.append([float(s) for s in line.split(',')])
        costs = np.array(costs)

        plt.figure('costs')
        if logarithmic:
            log_norm = LogNorm(vmin=costs.min().min(), vmax=costs.max().max())
            cbar_ticks = [math.pow(10, i) for i in range(math.floor(math.log10(costs.min().min())), 1+math.ceil(math.log10(costs.max().max())))]
            sns.heatmap(costs, norm=log_norm, cbar_kws={"ticks": cbar_ticks})
        else:
            sns.heatmap(costs)
        plt.title("Rollout costs")
        plt.xlabel("Rollout index")
        plt.ylabel("Time")

    def plot_rollout_weights(self, experiment_id):
        strategy_name = "Monte Carlo"

        df = self.df.loc[self.df['id'].str.contains(experiment_id)
                         & (self.df['Sampling Strategy'] == strategy_name)]
        weights = []

        # convert dataframe of strings to a 2D np array
        weights_temp = df['weight_history']
        for index, row in weights_temp.iteritems():
            line = row[1:-1] # get rid of parentheisis first
            weights.append([float(s) for s in line.split(',')])
        weights = np.array(weights)

        plt.figure('weights')
        sns.heatmap(weights)
        plt.title("Rollout weights")
        plt.xlabel("Rollout index")
        plt.ylabel("Time")



if __name__ == "__main__":
    import sys
    import argparse

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('experiment_id',
                        type=str,
                        nargs='+',
                        help='the id of the experiment to plot')
    args = parser.parse_args(sys.argv[1:])

    plotter = Plotter(args.experiment_id)
    # plotter.plot_average_cost_per_substep()
    # plotter.plot_average_cost_per_rollout()
    # plotter.plot_average_cost_per_rollout_tree()
    # plotter.plot_effective_samples_per_rollout()
    # plotter.plot_effective_samples_per_rollout_tree()
    # plotter.plot_cost_comparison()
    # plotter.plot_cost_momentum_comparison_tree()
    # plotter.plot_average_cost_samples_comparison()
    # plotter.plot_momentum_comparison()
    # plotter.plot_average_cost_with_without_momentum()
    # plotter.plot_rate()
    # plotter.plot_cost('horizon')
    # plotter.plot_average_cost_with_without_momentum()
    # plotter.plot_stage_costs()
    # plotter.plot_average_cost('learning_factor', hue='experiment', x_label='Fraction of MPPI rollouts informed by learning')
    # plotter.plot_average_cost('learning_factor', x_label='Fraction of MPPI rollouts informed by learning')
    # plotter.plot_average_cost('horizon', x_label="Horizon [s]")
    plotter.plot_average_cost('controller_name', hue='learned_rollout_ratio', x_label='Controller Type')
    # plotter.plot_rollout_costs(args.experiment_id[0])
    # plotter.plot_rollout_weights(args.experiment_id[0])
    # plotter.plot_cost('learned_rollout_ratio')
    # plotter.plot_effective_samples('learned_rollout_ratio', col='experiment')

    plt.show()
