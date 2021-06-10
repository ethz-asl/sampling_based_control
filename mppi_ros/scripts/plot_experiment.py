#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from rospkg import RosPack
from matplotlib.colors import LogNorm
from scipy.interpolate import interp1d
import math
import uuid
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

        df['experiment'] = ""
        if type(experiment_id) in [list, tuple]:
            print("Looking for experiments with ids: ")
            for exp in experiment_id:
                print("  - " + exp)
                df.loc[df['id'].str.match(exp + "[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}"), 'experiment'] = exp
        else:
            print("Looking for experiments with id: {}".format(experiment_id))
            df.loc[df['id'].str.match(experiment_id + "[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}"), 'experiment'] = experiment_id
        df = df.loc[df['experiment'] != ""]
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

    def plot_average_cost(self, aggregator, hue=None, x_label=None, type='bar'):
        fig, ax = plt.subplots()
        ax.set_xlabel(aggregator if x_label is None else x_label)
        if type == 'bar':
            sns.barplot(x=aggregator, y="stage_cost", hue=hue, data=self.df)
            ax.set_ylabel("Average stage cost")
        elif type == 'box':
            sns.boxplot(x=aggregator, y="stage_cost", hue=hue, data=self.df)
            ax.set_yscale("log")
            ax.set_ylabel("Stage cost")    
        else:
            raise NotImplementedError(f"Unknown type '{type}'")
        
        
        

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
    
    def plot_avg_time_to_goal(self, aggregator, hue=None, x_label=None, type='bar'):
        fig, ax = plt.subplots()
        ax.set_xlabel(aggregator if x_label is None else x_label)
        if type == 'bar':
            sns.barplot(x=aggregator, y='time_to_goal', hue=hue, data=self.df)
            ax.set_ylabel("Average time to goal [s]")
        elif type == 'box':
            sns.boxplot(x=aggregator, y='time_to_goal', hue=hue, data=self.df)
            ax.set_yscale("log")
            ax.set_ylabel("Time to goal")    
        else:
            raise NotImplementedError(f"Unknown type '{type}'")

    def make_run_cost_video(self, save_dir):
        if not os.path.isdir(save_dir):
            os.makedirs(save_dir)
        fig, ax = plt.subplots(figsize=(8,6))

        # Those are the wall-times when the callbacks were received
        times = self.df['time'].to_numpy()
        dt = np.mean(times[1:] - times[:-1]) # dt to match screen capture
        print(f"dt: {dt}")

        # This is the sim-time used as x axis
        sim_dt = 0.01
        max_sim_time = 4
        sim_time = self.df['index'].to_numpy() * sim_dt
        goal_reached_time = self.df['time_to_goal'].to_numpy()[0]

        stage_costs = self.df['stage_cost'].to_numpy()
        stage_costs = stage_costs[sim_time <= max_sim_time]
        sim_time = sim_time[sim_time <= max_sim_time]


        sns.lineplot(x=sim_time, y=stage_costs)
        ax.set_ylabel("Stage cost")
        ax.set_xlabel("Sim time [s]")

        # Save layout
        plt.tight_layout()
        x_lim = plt.xlim()
        y_lim = plt.ylim()
        
        # Loop over frames
        for i in range(len(sim_time)):
            plt.clf()
            sns.lineplot(x=sim_time[:i+1], y=stage_costs[:i+1])
            plt.scatter(sim_time[i], stage_costs[i])
            if sim_time[i] >= goal_reached_time:
                plt.scatter(goal_reached_time, 0, marker='x', c='r')
            plt.xlim(x_lim)
            plt.ylim(y_lim)
            ax = plt.gca()
            ax.set_ylabel("Stage cost")
            ax.set_xlabel("Sim time [s]")
            plt.savefig(f"{save_dir}/img_{i:05}.png")
        plt.close()
        print("Convert to video with:")
        print(f"ffmpeg -framerate {1/dt:.2f} -i {save_dir}/img_%05d.png -c:v libx264 -profile:v high -crf 18 -pix_fmt yuv420p {save_dir}/output.mp4")

    def plot_avg_rate(self, aggregator, hue=None, x_label=None):
        experiments = self.df['id'].unique()
        df = self.df.copy()
        df['rate'] = 0.0
        for experiment in experiments:
            times = df.loc[df['id'] == experiment, 'time'].to_numpy()
            dts = times[1:] - times[:-1]
            dts = np.append(dts, dts[-1])
            df.at[df['id'] == experiment, 'rate'] = 1/dts
        
        fig, ax = plt.subplots()
        # ax.set_xlabel(aggregator if x_label is None else x_label)
        p = sns.barplot(x=aggregator, y='rate', hue=hue, data=df, dodge=False)
        p.set(xlabel=None)
        ax.set_ylabel("Average Rate [Hz]")
        ax.set_xticklabels(["MPPI", "L-MPPI\ndeep", "L-MPPI"])

    def plot_all_rollout_policy_weights(self, caching_factor, colorful_plot=False):
        cum_w_opt = []
        cum_w_pol = []
        time_to_goals = []
        timeouts = []
        #### Defect in data recording, needs to be adressed.
        #### Until then need to set timeout duration manually
        timeout = 7.5
        dt = 0.01
        if colorful_plot:
            fig, (ax1, ax2) = plt.subplots(2)
        else:
            fig, ax = plt.subplots()

        experiments = self.df['id'].unique()
        no_experiments = len(experiments)
        controller_name_old = None
        first = True
        if not colorful_plot:
            resolution = int(timeout / dt)
            normalised_weights_policy = np.zeros((no_experiments, resolution))
            i = 0
        for experiment_id in experiments:
            df = self.df.loc[self.df['id'] == experiment_id]
            controller_name = df['controller_name'].iloc[0]
            if first:
                controller_name_old = controller_name = df['controller_name'].iloc[0]
                first = False
            if controller_name != controller_name_old and not first:
                print(f'Mixing controllers in weights plot. This plot is producing {controller_name_old} controller plots.\n')
                print(f'It was asked to mix in {controller_name} controller run.\n')
                print('Skipping.')
                continue
            # assume MPPI settings are constant over one experiment
            lrr = df['learned_rollout_ratio'].iloc[0]
            nr = df['nr_rollouts'].iloc[0]
            cf = caching_factor
            opt_idx = math.ceil(cf*nr)
            if math.ceil(lrr*nr) == 0:
                print('Plot does not make sense if no policy was active. Skipping.\n')
                no_experiments -= 1
                continue

            policy_idx = math.ceil(lrr*nr) + opt_idx
            weights_opt = []
            weights_policy = []
            time_to_goals.append(df['time_to_goal'].iloc[0]) # time to goal also the same for one run
            timeouts.append(timeout)
            weights_temp = df['weight_history']
            for index, row in weights_temp.iteritems():
                line = row[1:-1] # get rid of parentheisis first
                w_array = [float(s) for s in line.split(',')]
                weights_opt.append(w_array[opt_idx])
                weights_policy.append(w_array[policy_idx])
            if colorful_plot:
                weights_opt_array = np.array(weights_opt)
                weights_policy_array = np.array(weights_policy)
                cum_w_opt = [sum(n) for n in zip(cum_w_opt + [0] * (len(weights_opt) - len(cum_w_opt)), weights_opt)]
                cum_w_pol = [sum(n) for n in zip(cum_w_pol + [0] * (len(weights_policy) - len(cum_w_pol)), weights_policy)]
                ax1.plot(weights_opt_array)
                ax1.set_ylabel('optimal weight [-]')
                ax2.plot(weights_policy_array)
                ax2.set_ylabel('policy weight [-]')
                ax2.set_xlabel('time step')
                # goal_reached_time_step = math.floor(time_to_goals[-1]/timeouts[-1] * len(weights_policy_array))-1
                # ax1.plot(goal_reached_time_step, weights_opt_array[goal_reached_time_step], 'or', markersize=20)
                # ax2.plot(goal_reached_time_step, weights_policy_array[goal_reached_time_step], 'or', markersize=20)
            else:
                goal_reached_time_step = math.floor(time_to_goals[-1]/timeouts[-1] * len(weights_policy))-1
                time_steps = np.linspace(0,len(weights_policy), num=len(weights_policy))
                weight_function = interp1d(time_steps, weights_policy)
                new_ts = np.linspace(0, goal_reached_time_step, num=resolution)
                normalised_weights_policy[i, :] = np.array(weight_function(new_ts))
                i += 1
        if colorful_plot:
            mean_w_opt = np.array(cum_w_opt)/no_experiments
            mean_w_pol = np.array(cum_w_pol)/no_experiments
            ax1.plot(mean_w_opt,'k', linewidth=4, label='mean')
            ax1.legend()
            ax1.set_title(f'Weights during {no_experiments} runs.\nController settings -> {controller_name} with learned rollout ratio: {lrr}')
            ax2.plot(mean_w_pol, 'k', linewidth=4, label='mean')
            ax2.legend()
        else:
            normalised_weights_policy = np.transpose(normalised_weights_policy)
            
            weights_min = np.min(normalised_weights_policy, axis=1)
            weights_max = np.max(normalised_weights_policy, axis=1)
            weights_avg = np.average(normalised_weights_policy, axis=1)
            x = np.linspace(0, 1, num=resolution)
            ax.fill_between(x, weights_max, weights_min, alpha=0.3)
            ax.plot(x, weights_avg, linewidth=2)
            ax.set_ylabel('policy weight')
            ax.set_xlabel('normalized time')

    def plot_dagger_progress(self):
        # Mean and std
        average_cost = {}

        iter_list = ['expert', 'iter_0', 'iter_4', 'iter_9', 'iter_14', 'iter_19',
        'iter_24', 'iter_29', 'iter_34']
        for iter in iter_list:
            df = self.df.loc[(self.df['id'].str.contains(iter))]
            if iter not in average_cost:
                average_cost[iter] = [df['stage_cost'].mean()]
            else:
                average_cost[iter].append(df['stage_cost'].mean())

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
        ax.bar(iter_list,
               average_cost_mean,
               yerr=average_cost_std,
               align='center',
               alpha=0.5,
               ecolor='black',
               capsize=10)
        ax.set_xlabel('Dagger iterations')
        ax.set_xticks(iter_list)
        ax.set_title('Average cost')
        ax.yaxis.grid(True)


    def plot_normalized_stage_cost(self, reference):
        # First find all reference runs
        reference_run_ids = self.df.loc[self.df['experiment'] == reference, 'id'].unique()
        print(f"Found {len(reference_run_ids)} runs that match the reference name '{reference}'")

        # Find all runs that correspond to one experiment
        experiments = self.df['experiment'].unique()
        experiments = experiments[experiments != reference] # we compare to the reference, so don't plot it
        exp_run_dict = {}
        n_runs = len(reference_run_ids)
        for exp in experiments:
            exp_run_dict[exp] = self.df.loc[self.df['experiment'] == exp, 'id'].unique()
            print(f"{len(exp_run_dict[exp])} runs for experiment '{exp}'")
            if len(exp_run_dict[exp]) < n_runs:
                print(f"Too few runs in experiment '{exp}' Reducing total number of runs!")
            n_runs = min(n_runs, len(exp_run_dict[exp]))
            # assert(len(exp_run_dict[exp]) == len(reference_run_ids))

        dt = 0.01
        n_support = 1000

        exp_norm_costs = np.zeros((len(experiments), n_support, n_runs))
        ref_norm_costs = np.zeros((n_support, n_runs))

        for i in range(n_runs): # Iterate over all runs as they have identical start and stop positions
            ref_run = self.df.loc[self.df['id'] == reference_run_ids[i]]
            ref_time_to_goal = ref_run.iloc[0].at['time_to_goal']
            ref_run_times = ref_run['index'].to_numpy() * dt
            ref_run_costs = ref_run['stage_cost'].to_numpy()

            # Make sure that last element is larger or equal to the ref_time_to_goal
            if np.max(ref_run_times) < ref_time_to_goal:
                ref_run_times = np.append(ref_run_times, ref_time_to_goal)
                ref_run_costs = np.append(ref_run_costs, ref_run_costs[-1])

            f_ref = interp1d(ref_run_times/ref_time_to_goal, ref_run_costs)
            norm_ref_run_costs = f_ref(np.linspace(0, 1, n_support))


            for e, exp in enumerate(experiments):
                exp_run = self.df.loc[self.df['id'] == exp_run_dict[exp][i]] # i'th run of experiment exp
                times = exp_run['index'].to_numpy() * dt
                costs = exp_run['stage_cost'].to_numpy()

                # Make sure that last element is larger or equal to the ref_time_to_goal
                if np.max(times) < ref_time_to_goal:
                    times = np.append(times, ref_time_to_goal)
                    costs = np.append(costs, ref_run_costs[-1])
                f = interp1d(times/ref_time_to_goal, costs)
                norm_costs = f(np.linspace(0, 1, n_support))

                exp_norm_costs[e, :, i] = norm_costs# - norm_ref_run_costs
            ref_norm_costs[:,i] = norm_ref_run_costs

        plt.figure("Normalized Stage Cost")
        norm_time = np.linspace(0, 1, n_support)
        plt.plot(norm_time, np.mean(ref_norm_costs,axis=1))
        plt.fill_between(norm_time, np.quantile(ref_norm_costs, 0.75, axis=1), np.quantile(ref_norm_costs, 0.25, axis=1), alpha=0.3)
        for e, exp in enumerate(experiments):
            c = exp_norm_costs[e, :,:]
            plt.plot(norm_time, np.mean(c, axis=1))
            plt.fill_between(norm_time, np.quantile(c, 0.75, axis=1), np.quantile(c, 0.25, axis=1), alpha=0.3)

        plt.xlabel("Normalized time")
        plt.ylabel("Stage cost")
        plt.legend(["default", *experiments])


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
    # plotter.plot_average_cost('controller_name', hue='learned_rollout_ratio', x_label='Controller Type')
    # plotter.plot_average_cost('controller_name', hue='learned_rollout_ratio', x_label='Controller Type', type='box')
    # plotter.plot_avg_time_to_goal('controller_name', hue='learned_rollout_ratio', x_label='Controller Type')
    # plotter.plot_avg_time_to_goal('controller_name', hue='learned_rollout_ratio', x_label='Controller Type', type='box')
    plotter.plot_all_rollout_policy_weights(0.3)
    # plotter.plot_rollout_costs(args.experiment_id[0])
    # plotter.plot_rollout_weights(args.experiment_id[0])
    # plotter.plot_cost('learned_rollout_ratio')
    # plotter.plot_effective_samples('learned_rollout_ratio', col='experiment')
    # plotter.make_run_cost_video("/home/andreas/video_tmp")
    #plotter.plot_dagger_progress()
    plotter.plot_normalized_stage_cost('default')

    plt.show()
