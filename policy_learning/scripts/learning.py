#!/usr/bin/env python3

import os
import pandas as pd
import torch
from torch.utils.data import Dataset
import torch.nn as nn

class StateActionDataset(Dataset):

    def __init__(self, root_dir, transform=None):
        """
        Args:
            root_dir (string): Directory with datasets
            transform (callable, optional): Optional transform to be applied
        """
        self.root_dir = root_dir
        self.transform = transform

        # root_dir must have a data file
        # TODO: (Kiran) adapt to final i/o of data collection


        file_name = os.path.join(root_dir, 'data', 'out.csv')
        # code to read in following csv format:
        # n_states, n_actions, state_values, action_values
        n_states = pd.read_csv(file_name, header=None, usecols=[0])  # assusmes that n_states does not change in one csv file
        n_actions = pd.read_csv(file_name, header=None, usecols=[1]) # same as above for n_actions
        self._n_states = n_states.values[0][0]
        self._n_actions = n_actions.values[0][0]
        self.state = pd.read_csv(file_name, header=None,
            usecols=range(2, self._n_states+2), dtype=float)
        self.action = pd.read_csv(file_name, header=None,
            usecols=range(self._n_states+2, self._n_states+self._n_actions+1),
            dtype=float) # last entry should be +2 but csv is one column too short for some reason

    def __len__(self):
        """
        Define magic method.
        """
        n_data_points_states = len(self.state)
        n_data_points_actions = len(self.action)
        if n_data_points_states == n_data_points_actions:
            return n_data_points_states
        else:
            print("Number of state data points don't match number of action data points")
            return n_data_points_states

    def __getitem__(self, idx):
        """
        Define magic method.
        """
        state = self.state.values[idx]
        action = self.action.values[idx]
        sample = {'state': state, 'action': action}
        return sample

    def get_dimensions(self):
        """
        Out:
            int, int: dimensions of the state and action spaces
        """
        return self._n_states, self._n_actions

class MLPSampler(nn.Module):
    """
    A class for the informed sample generation.
    """
    def __init__(self, n_in, n_out):
        super(MLPSampler, self).__init__()
        # TODO (Kiran): Dummy structure of network, needs to be properly defined
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(n_in, 512),
            nn.ReLU(),
            nn.Linear(512,512),
            nn.ReLU(),
            nn.Linear(512,n_out)
        )
    def forward(self, x):
        return self.linear_relu_stack(x)


if __name__ == "__main__":
    from torch.utils.data import DataLoader

    task_path = os.path.dirname(os.path.realpath(__file__))
    file_path = task_path + "/../"
    dataset = StateActionDataset(root_dir=file_path)
    idx=0
    sample = dataset[idx]
    #print("Index: ", idx, "\nstate: " , sample['state'], "\naction: ", sample['action'])

    n_states, n_actions = dataset.get_dimensions()

    # set device for training
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = MLPSampler(n_states, n_actions).to(device)
    #print(model)

    # make data in correct format for torch
    train_dataloader = DataLoader(dataset, batch_size=1)

    for different_sample in train_dataloader:

        #print(different_sample)
        state = different_sample['state']
        a_pred = model(state.float())
        #print(a_pred)
