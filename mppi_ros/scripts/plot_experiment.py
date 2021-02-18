#! /usr/bin/env python
import os
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from rospkg import RosPack
sns.set_theme()
sns.set_context("paper")


class Plotter:
    def __init__(self, experiment_id="mppi_panda"):
        self.df = self.get_data(experiment_id)

    def get_data(self, experiment_id):
        csv_file = os.path.join(RosPack().get_path("mppi_ros"), "log", "record.csv")
        # df = pd.read_csv(csv_file, converters={'stage_cost': eval, 'step_count': eval, 'weights': eval, 'time': eval,
        #                                          'effective_samples': eval})
        df = pd.read_csv(csv_file)
        df['tree_search'] = df['tree_search'].apply(lambda x : "Tree" if x else "Monte Carlo")
        df = df.rename(columns={'tree_search': 'Sampling Strategy'})
        print(df['Sampling Strategy'])
        return df[df['id'].str.contains(experiment_id)]

    def plot_average_cost_per_rollout(self, tree=False):
        # Mean and std
        average_cost = {}

        all_rollouts_nr = self.df.loc[self.df['Sampling Strategy'] == tree]['nr_rollouts'].unique()
        for rollouts_nr in all_rollouts_nr:
            df = self.df.loc[(self.df['nr_rollouts'] == rollouts_nr) & (self.df['Sampling Strategy'] == tree)]
            if rollouts_nr not in average_cost:
                average_cost[rollouts_nr] = [df['stage_cost'].mean()]
            else:
                average_cost[rollouts_nr].append(df['stage_cost'].mean())

        average_cost_mean = [np.mean(average_cost_vector) for average_cost_vector in average_cost.values()]
        average_cost_std = [np.std(average_cost_vector) for average_cost_vector in average_cost.values()]

        # Build the plot
        fig, ax = plt.subplots()
        ax.bar(all_rollouts_nr, average_cost_mean, yerr=average_cost_std, align='center', alpha=0.5, ecolor='black',
               capsize=10)
        ax.set_ylabel('Numbers of samples')
        ax.set_xticks(all_rollouts_nr)
        ax.set_title('Average cost')
        ax.yaxis.grid(True)

        # Save the figure and show
        plt.tight_layout()

    def plot_average_cost_per_rollout_tree(self):
        self.plot_average_cost_per_rollout(tree=True)

    def plot_effective_samples_per_rollout(self, tree=False):
        plt.figure()
        df = self.df.loc[self.df['Sampling Strategy'] == tree]
        nr_samples = len(df["nr_rollouts"].unique())
        sns.lineplot(data=df, x="index", y="effective_samples", hue="nr_rollouts", ci="sd", palette=sns.color_palette("tab10", n_colors=nr_samples))

    def plot_effective_samples_per_rollout_tree(self):
        self.plot_effective_samples_per_rollout(tree=True)

    def plot_cost_comparison(self):

        df_no_tree = self.df.loc[self.df['Sampling Strategy'] == "Monte Carlo"]
        sample_sizes_no_tree = df_no_tree["nr_rollouts"].unique()

        df_tree = self.df.loc[self.df['Sampling Strategy'] == "Tree"]
        sample_sizes_tree = df_tree["nr_rollouts"].unique()

        common_sample_sizes = set(sample_sizes_tree).intersection(set(sample_sizes_no_tree))
        time_padding = 0.1
        for samples_size in common_sample_sizes:
            fig, axs = plt.subplots(nrows=1, ncols=5, sharey=True)
            for i in range(5):
                df_ = self.df.loc[(self.df['nr_rollouts'] == samples_size) & (self.df['time'] < (i+1) * 5) & (self.df['time'] > (i*5 + time_padding)) ]
                df_['stage_cost'] = df_['stage_cost'].apply(lambda x : np.exp(-x))
                sns.lineplot(data=df_, x="time", y="stage_cost", hue="Sampling Strategy", legend=True, ci="sd", palette=sns.color_palette("tab10", n_colors=2), ax=axs[i])
                axs[i].set_title("Target {}".format(i+1), fontsize=20)
                axs[i].set_xlabel("t [s]", fontsize=20)
                axs[i].set_ylabel("stage cost", fontsize=20)
                axs[i].set_ybound(lower=-0.01, upper=1.2)
                plt.legend(fontsize=18)

    def plot_cost_momentum_comparison_tree(self):

        df_tree = self.df.loc[self.df['Sampling Strategy'] == "Tree"]
        sample_sizes_tree = df_tree["nr_rollouts"].unique()

        df_tree['beta'] = df_tree['beta'].apply(lambda x : "with" if x > 0 else "without")
        df_tree = df_tree.rename(columns={'beta': 'momentum'})
        time_padding = 0.15
        for samples_size in sample_sizes_tree:
            fig, axs = plt.subplots(nrows=1, ncols=5, sharey=True)
            for i in range(5):
                df_ = df_tree.loc[(self.df['nr_rollouts'] == samples_size) & (df_tree['time'] < (i+1) * 5 - time_padding) & (df_tree['time'] > (i*5 + time_padding)) ]
                #df_['stage_cost'] = df_['stage_cost'].apply(lambda x : np.exp(-x))
                sns.lineplot(data=df_, x="time", y="stage_cost", hue="momentum", legend=False, ci="sd", palette=sns.color_palette("tab10", n_colors=2), ax=axs[i])
                axs[i].set_title("Target {}".format(i+1), fontsize=20)
                axs[i].set_xlabel("t [s]", fontsize=20)
                #axs[i].set_yscale('log')
                axs[i].set_ylabel("stage cost", fontsize=20)
                #axs[i].set_ybound(lower=-0.01, upper=1.2)

    def plot_momentum_comparison(self):
        plt.figure()

        df_no_tree = self.df.loc[self.df['Sampling Strategy'] == False]
        sample_sizes_no_tree = df_no_tree["nr_rollouts"].unique()

        for samples_size in sample_sizes_no_tree:
            plt.figure()
            df_ = df_no_tree.loc[df_no_tree['nr_rollouts'] == samples_size]
            ax = sns.lineplot(data=df_, x="time", y="effective_samples", ci="sd", hue="alpha", size="beta")
            ax.set_title("Effective Samples ({} samples)".format(samples_size))

        df_tree = self.df.loc[self.df['Sampling Strategy'] == True]
        sample_sizes_tree = df_tree["nr_rollouts"].unique()

        for samples_size in sample_sizes_tree:
            plt.figure()
            df_ = df_tree.loc[df_tree['nr_rollouts'] == samples_size]
            ax = sns.lineplot(data=df_, x="time", y="effective_samples", ci="sd", hue="alpha", size="beta")
            ax.set_title("Effective Samples FD-MCTS ({} samples)".format(samples_size))


if __name__ == "__main__":
    import sys
    import argparse

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('experiment_id', type=str, help='the id of the experiment to plot')
    args = parser.parse_args(sys.argv[1:])

    plotter = Plotter(args.experiment_id)
    #plotter.plot_average_cost_per_rollout()
    #plotter.plot_effective_samples_per_rollout()
    plotter.plot_cost_comparison()
    plotter.plot_cost_momentum_comparison_tree()
    # plotter.plot_momentum_comparison()
    plt.legend()
    plt.show()