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

        # root_dir must have a state data file and an action data file
        # TODO: (Kiran) adapt to final i/o of data collection


        state_file = os.path.join(root_dir, 'states.csv')
        action_file = os.path.join(root_dir, 'actions.csv')
        self.state = pd.read_csv(state_file, header=None)
        self.action = pd.read_csv(action_file, header=None)

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
        state = self.state[idx]
        action = self.action[idx]
        sample = {'state': state, 'action': action}
        return sample


if __name__ == "__main__":
    #task_path = os.path.dirname(os.path.realpath(__file__))
    print("hello world")
