#! /usr/bin/env python
import os
import numpy as np
import pandas as pd
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
        return df[df['id'].str.contains(experiment_id)]

    def plot_average_cost_per_rollout(self, tree=False):
        # Mean and std
        average_cost = {}

        all_rollouts_nr = self.df.loc[self.df['tree_search'] == tree]['nr_rollouts'].unique()
        for rollouts_nr in all_rollouts_nr:
            df = self.df.loc[(self.df['nr_rollouts'] == rollouts_nr) & (self.df['tree_search'] == tree)]
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
        df = self.df.loc[self.df['tree_search'] == tree]
        nr_samples = len(df["nr_rollouts"].unique())
        sns.lineplot(data=df, x="index", y="effective_samples", hue="nr_rollouts", ci="sd", palette=sns.color_palette("tab10", n_colors=nr_samples))

    def plot_effective_samples_per_rollout_tree(self):
        self.plot_effective_samples_per_rollout(tree=True)

    def plot_cost(self, tree=False):
        plt.figure()
        df = self.df.loc[self.df['tree_search'] == tree]
        nr_samples = len(df["nr_rollouts"].unique())
        sns.lineplot(data=df, x="index", y="stage_cost", hue="nr_rollouts", ci="sd", palette=sns.color_palette("tab10", n_colors=nr_samples))

    def plot_momentum_samples_comparison(self):
        plt.figure()
        df_alpha_1 = self.df.loc[(self.df['tree_search'] == False) & (self.df['alpha'] == 1.0)]
        nr_samples = len(df_alpha_1["nr_rollouts"].unique())
        ax_1 = sns.lineplot(data=df_alpha_1, x="index", y="effective_samples", hue="nr_rollouts", ci="sd", palette=sns.color_palette("tab10", n_colors=nr_samples))
        ax_1.set_title("Effective Samples (no momentum)")

        df_momentum = self.df.loc[(self.df['tree_search'] == False) & (self.df['alpha'] < 1.0)]
        nr_samples = len(df_momentum["nr_rollouts"].unique())
        ax_m = sns.lineplot(data=df_momentum, x="index", y="effective_samples", hue="nr_rollouts", ci="sd", palette=sns.color_palette("tab10", n_colors=nr_samples))
        ax_m.set_title("Effective Samples (with momentum)")


if __name__ == "__main__":
    plotter = Plotter("mppi_panda")
    # plotter.plot_experiment("mppi_panda")
    plotter.plot_average_cost_per_rollout()
    plotter.plot_effective_samples_per_rollout()
    plotter.plot_cost()
    plotter.plot_momentum_samples_comparison()
    plt.show()
