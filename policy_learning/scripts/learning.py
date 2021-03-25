#!/usr/bin/env python3

import os
import pandas as pd
import torch
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

        # root_dir must have a data file
        # TODO: (Kiran) adapt to final i/o of data collection


        file_name = os.path.join(root_dir, 'data', 'out.csv')
        # code to read in following csv format:
        # n_states, n_actions, state_values, action_values
        n_states = pd.read_csv(file_name, header=None, usecols=[0])  # assusmes that n_states does not change in one csv file
        n_actions = pd.read_csv(file_name, header=None, usecols=[1]) # same as above for n_actions
        n_states = n_states.values[0][0]
        n_actions = n_actions.values[0][0]
        self.state = pd.read_csv(file_name, header=None,
            usecols=range(2, n_states+2))
        self.action = pd.read_csv(file_name, header=None,
            usecols=range(n_states+2, n_states+n_actions+1)) # last entry should be +2 but csv is one column too short for some reason

    def __len__(self):
        """
        Override base class method.
        """
        n_states = len(self.state)
        n_actions = len(self.action)
        if n_states == n_actions:
            return n_states
        else:
            print("Number of states don't match number of actions")
            return n_states

    def __getitem__(self, idx):
        """
        Override base class method.
        """
        state = self.state.values[idx]
        action = self.action.values[idx]
        sample = {'state': state, 'action': action}
        return sample


if __name__ == "__main__":
    task_path = os.path.dirname(os.path.realpath(__file__))
    file_path = task_path + "/../"
    dataset = StateActionDataset(root_dir=file_path)
    idx=0
    sample = dataset[idx]
    print("Index: ", idx, "\nstate: " , sample['state'], "\naction: ", sample['action'])
    print(len(dataset))
