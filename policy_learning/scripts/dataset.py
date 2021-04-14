import os
import h5py
import csv
import torch
import numpy as np
from torch.utils.data import Dataset

class StateActionDataset(Dataset):

    def __init__(self, root_dir, transform=None):
        """
        Args:
            root_dir (string): Directory with datasets
            transform (callable, optional): Optional transform to be applied
        """
        self.root_dir = root_dir
        self.transform = transform

        # root_dir must have a folder with runs and a value log file as generated
        # by the data_collection bash script
        # TODO: (Kiran) adapt to final i/o of data collection

        # ascertain number of data files from IC log .csv file
        # --> values inside of csv do not have any use for learning currently
        log_file_path = os.path.join(root_dir, 'value_log.csv')
        with open(log_file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                line_count += 1

        n_runs = line_count - 1 # minus header
        print('Identified ', n_runs, ' runs.')

        # loop over all files and concatenate into one dataset for each attribute
        # !!Naive impelementation, needs to be revised if datasets become larger

        for i in range(0, n_runs):
            file_name = "run_" + str(i) + ".hdf5"
            file_path = os.path.join(root_dir, file_name)
            f = h5py.File(file_path, 'r')
            if not f:
                print('Could not load file at iteration: ', i)
            more_states = f['states']
            more_actions = f['actions']
            if i==0:
                self.state_dataset = np.array(more_states[:,:])
                self.action_dataset = np.array(more_actions[:,:])
            else:
                self.state_dataset = np.append(self.state_dataset, more_states[:,:], axis=0)
                self.action_dataset = np.append(self.action_dataset, more_actions[:,:], axis=0)

        self._n_states = self.state_dataset.shape[1]
        self._n_actions = self.action_dataset.shape[1]


    def __len__(self):
        """
        Define magic method.
        """
        n_data_points_states = len(self.state_dataset)
        n_data_points_actions = len(self.action_dataset)
        if n_data_points_states == n_data_points_actions:
            return n_data_points_states
        else:
            print("Number of state data points don't match number of action data points")
            return n_data_points_states

    def __getitem__(self, idx):
        """
        Define magic method.
        """
        state = self.state_dataset[idx, :]
        action = self.action_dataset[idx, :]
        sample = {'state': state, 'action': action}
        return sample

    def get_dimensions(self):
        """
        Out:
            int, int: dimensions of the state and action spaces
        """
        return self._n_states, self._n_actions