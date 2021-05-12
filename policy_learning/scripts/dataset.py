import glob

import numpy as np
from torch.utils.data import Dataset

import h5py

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

        # loop over all files and concatenate into one dataset for each attribute
        # !!Naive impelementation, needs to be revised if datasets become larger
        files = glob.glob(root_dir + '/*.hdf5')
        if len(files) < 1:
            raise IOError(f"No hdf5 files found in directory {root_dir}")

        self.state_dataset = None
        self.action_dataset = None

        for file in files:
            f = h5py.File(file, 'r')
            if not f:
                raise IOError(f'Could not load file {f}')
            more_states = f['states']
            more_actions = f['actions']
            if self.state_dataset is None or self.action_dataset is None:
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

    def append_dataset(self, path):
        files = glob.glob(path + '/*.hdf5')
        if len(files) < 1:
            raise IOError(f"No hdf5 files found in directory {path}")

        for file in files:
            f = h5py.File(file, 'r')
            if not f:
                raise IOError(f'Could not load file {f}')
            more_states = f['states']
            more_actions = f['actions']
            self.state_dataset = np.append(self.state_dataset, more_states[:,:], axis=0)
            self.action_dataset = np.append(self.action_dataset, more_actions[:,:], axis=0)
