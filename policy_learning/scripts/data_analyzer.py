from dataset import StateActionDataset

import h5py
import os
import numpy as np
import torch
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
from scipy.ndimage import gaussian_filter1d
import matplotlib
import seaborn as sns
import pandas as pd
sns.set_theme()
sns.set_context("paper")
sns.set(font_scale=2)

# Avoid Type3 fonts (RA-L submission)
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42


class DataAnalyzer:
    def __init__(self):
        self.actions = None
        self.states  = None

    def check_data_loaded(self):
        if self.actions is None or self.states is None:
            raise RuntimeError('No data loaded.')

    def load_single_run(self, path):
        f = h5py.File(path, 'r')
        if not f:
            print(f"Could not load file '{path}'")
        if self.actions is None or self.states is None:
            self.states = np.array(f['states'])
            self.actions = np.array(f['actions'])
        else:
            self.states = np.append(self.states, f['states'], axis=0)
            self.actions = np.append(self.actions, f['actions'], axis=0)

    def load_dataset(self, path):
        dataset = StateActionDataset(root_dir=dir_path)
        self.actions = dataset.action_dataset
        self.states = dataset.state_dataset

    def plot_action_std_dev(self):
        self.check_data_loaded()

        bins_per_dim = 10
        n_state_dims = self.states.shape[1]
        n_states = self.states.shape[0]
        n_action_dims = self.actions.shape[1]
        n_bins = bins_per_dim**n_state_dims
        if n_bins > 1e6:
            print(f"Large states space results in {n_bins} bins! This might take a while")

        states_min = np.min(self.states, axis=0)
        states_max = np.max(self.states, axis=0)        
        bounds = np.linspace(states_min, states_max, bins_per_dim)
        bin_indices = np.column_stack([np.digitize(self.states[:,i], bounds[:,i]) for i in range(n_state_dims)])-1

        bin_indices_lin = np.zeros(n_states, dtype=np.int32)
        for d in range(n_state_dims):
            bin_indices_lin += (bins_per_dim**d)*bin_indices[:,d]


        std_devs = np.zeros((n_bins, n_action_dims))
        for i in range(n_bins):
            in_bin_mask = (bin_indices_lin==i)
            if np.count_nonzero(in_bin_mask)>0:
                std_devs[i,:] = np.std(self.actions[in_bin_mask,:], axis=0)

        plt.figure("Action Std Deviation")
        for a in range(n_action_dims):
            ax = plt.subplot(1, n_action_dims, a+1)
            non_zero_std_devs = std_devs[std_devs[:,a]>0,a]
            mean = np.mean(non_zero_std_devs)
            plt.bar(np.arange(non_zero_std_devs.size), non_zero_std_devs)
            plt.axline((0,mean), (1,mean), c='r')
            plt.xlabel("State bin (only non-empty)")
            plt.ylabel(f"Std dev of action[{a}] in bin")
            ax.set_title(f"Std devs. Mean: {mean:.3f}")

    def plot_states(self):
        self.check_data_loaded()

        n_state_dims = self.states.shape[1]
        plt.figure("State evolution")
        for s in range(n_state_dims):
            ax = plt.subplot(1, n_state_dims, s+1)
            plt.plot(self.states[:,s])
            plt.xlabel("Time step")
            ax.set_title(f"State[{s}]")

    def plot_actions(self, model_path=None, add_noise=False, plot_diff=False, low_pass=False):
        self.check_data_loaded()

        noise_variance = 0.01
        sigma = 3
        n_action_dims = self.actions.shape[1]

        model_actions = None
        noisy_model_actions = None
        if model_path:
            model = torch.jit.load(model_path)
            with torch.no_grad():
                model_actions = model(torch.as_tensor(self.states, dtype=torch.float32)).numpy()
            if add_noise:
                disturbed_states = torch.as_tensor(self.states, dtype=torch.float32) + torch.randn(*self.states.shape) * (noise_variance**0.5)
                with torch.no_grad():
                    noisy_model_actions = model(disturbed_states).numpy()          

        for a in range(n_action_dims):
            mean = gaussian_filter1d(self.actions[:,a], sigma)
            diff = np.abs(mean-self.actions[:,a])

            ax1 = plt.subplot(2 if plot_diff else 1, n_action_dims, a+1)
            plt.plot(self.actions[:,a])
            legend = ["GT-action"]
            if low_pass:
                plt.plot(mean)
                legend.append("GT-action + low pass")
            if noisy_model_actions is not None:
                plt.plot(noisy_model_actions[:,a])
                legend.append("noisy NN-action")
            if model_actions is not None:
                plt.plot(model_actions[:,a])
                legend.append("NN-action")
            plt.legend(legend)
            plt.xlabel("Time step")
            plt.ylabel("Action")
            # ax1.set_title(f"Action[{a}]")
            if plot_diff:
                ax2 = plt.subplot(2, n_action_dims, a+1 + n_action_dims)
                plt.plot(diff)
                ax2.set_title(f"Estimated std dev. Mean: {np.mean(diff):.3f}")
                plt.xlabel("Time step")

    def plot_state_PCA(self, n=2):
        self.check_data_loaded()
        assert(n in (2,3))

        pca = PCA(n_components=n)
        pca.fit(self.states) 
        states_nd = pca.transform(self.states)

        plt.figure("State PCA")
        plt.figure
        if n==2:
            plt.scatter(states_nd[:,0], states_nd[:,1])
        else:
            ax = plt.subplot(111, projection='3d')
            ax.scatter(states_nd[:,0], states_nd[:,1], states_nd[:,2])

    def plot_action_vs_state_PCA(self):
        self.check_data_loaded()
        
        pca = PCA(n_components=2)
        pca.fit(self.states) 
        states_2d = pca.transform(self.states)

        n_action_dims = self.actions.shape[1]

        plt.figure("Actions vs state PCs")
        plt.figure
        for a in range(n_action_dims):
            ax = plt.subplot(1, n_action_dims, a+1, projection='3d')
            ax.scatter(states_2d[:,0], states_2d[:,1], self.actions[:,a], c=self.actions[:,a])
        
    def plot_state_histogram(self):
        self.check_data_loaded()
        n_state_dims = self.states.shape[1]
        plt.figure("State histogram")
        for i in range(n_state_dims):
            ax = plt.subplot(1,n_state_dims,i+1)
            ax.set_title(f"State[{i}]")
            plt.hist(self.states[:,i], bins=100)

    def plot_action_histogram(self):
        self.check_data_loaded()
        n_action_dims = self.actions.shape[1]
        plt.figure("action histogram")
        for i in range(n_action_dims):
            ax = plt.subplot(1,n_action_dims,i+1)
            ax.set_title(f"Action[{i}]")
            plt.hist(self.actions[:,i], bins=100)

    def plot_multi_actions(self, model_paths, run_dirs, names):
        assert(len(model_paths) == len(run_dirs) == len(names))
        data_dict = {'Action': [],
                     'Index': [], 
                     'Type': [],
                     'Name': []}
        for i in range(len(model_paths)):
            self.actions = None
            self.states = None
            self.load_single_run(run_dirs[i])
            length = len(self.actions)
            model = torch.jit.load(model_paths[i])
            with torch.no_grad():
                model_actions = model(torch.as_tensor(self.states, dtype=torch.float32)).numpy()
            data_dict['Action'].extend(self.actions.flatten().tolist())
            data_dict['Index'].extend(np.arange(length)[:].tolist())
            data_dict['Type'].extend(["Ground Truth" for j in range(length)])
            data_dict['Name'].extend([names[i] for j in range(length)])

            data_dict['Action'].extend(model_actions.flatten().tolist())
            data_dict['Index'].extend(np.arange(length)[:].tolist())
            data_dict['Type'].extend(["Policy" for j in range(length)])
            data_dict['Name'].extend([names[i] for j in range(length)])

        df = pd.DataFrame(data_dict)
        g = sns.relplot(data=df,
                        x='Index',
                        y='Action',
                        style='Type',
                        size='Type', size_order=["Policy", "Ground Truth"],
                        hue='Type',
                        col='Name',
                        kind="line", legend=False, height=10)
        g.set_titles("{col_name}").tight_layout()


if __name__ == "__main__":
    task_path = os.path.dirname(os.path.realpath(__file__))
    dir_path = task_path + "/../data/Horizon_3_Test_0419153905"
    run_name = "/run_1.hdf5"
    run_path = dir_path + run_name

    

    model_path = None
    model_path = "/home/andreas/plr_ws/src/sampling_based_control/Horizon_3_Train_0419144557.pt"

    d = DataAnalyzer()
    d.load_single_run(run_path)

    model_paths = ("/home/andreas/plr_ws/src/sampling_based_control/Horizon_2_Train_0419133816.pt", 
                   "/home/andreas/plr_ws/src/sampling_based_control/Horizon_3_Train_0419144557.pt",
                   "/home/andreas/plr_ws/src/sampling_based_control/Horizon_4_Train_0419140234.pt")
    run_paths = (task_path+"/../data/Horizon_2_Test_0419135518"+run_name,
                 task_path+"/../data/Horizon_3_Test_0419153905"+run_name,
                 task_path+"/../data/Horizon_4_Test_0419143555"+run_name)
    names = ("Horizon 2s", "Horizon 3s", "Horizon 4s")
        
    d.plot_multi_actions(model_paths, run_paths, names)
    # d.load_dataset(dir_path)
    # d.plot_action_std_dev()
    # d.plot_actions(model_path=model_path, add_noise=False, plot_diff=False)
    # d.plot_actions(model_path=model_path, add_noise=False, plot_diff=False)
    # d.plot_actions(model_path=model_path, add_noise=False, plot_diff=False)
    # d.plot_state_PCA(2)
    # d.plot_action_vs_state_PCA()
    # d.plot_action_histogram()
    # d.plot_state_histogram()
    # d.plot_states()
    plt.show()