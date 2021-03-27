#!/usr/bin/env python3

import os
import pandas as pd
import torch
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
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
        """
        return: output of model
        """
        return self.linear_relu_stack(x)


class PolicyLearner:
    """
    Class to handle the learning loop and give back an action after net has trained.
    """

    def __init__(self, file_path, device):
        """
        Args:
            file_path (string): directory with data folder
            device (string): choose gpu or cpu
        """
        # load data set
        self.dataset = StateActionDataset(root_dir=file_path)
        self.n_states, self.n_actions = self.dataset.get_dimensions()
        # initialise MLP model
        self.model = MLPSampler(self.n_states, self.n_actions-1).to(device) #reduce action size for now because of csv n_action bug
        self._is_trained = False

    def train(self):
        """
        Main training loop.
        """

        # Training parameters
        epochs = 2
        batch_size = 2
        learning_rate = 1e-3

        # initialise loss function
        loss_fn = nn.MSELoss()
        optimizer = torch.optim.SGD(self.model.parameters(), lr=learning_rate)

        # initialise data loader
        train_dataloader = DataLoader(self.dataset, batch_size=batch_size)
        size = len(train_dataloader.dataset)

        for t in range(epochs):
            print(f"Epoch {t+1}\n-------------------------------")
            for batch, sample in enumerate(train_dataloader):
                pred = self.model(sample['state'].float())
                loss = loss_fn(pred, sample['action'].float())

                # Backpropagation
                optimizer.zero_grad() # set grad to zero firs
                loss.backward()
                optimizer.step()

                if batch % 10 == 0:
                    loss, current = loss.item(), batch * len(sample['state'])
                    print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")

        self._is_trained = True

    def get_action(self, state):
        """
        returns action from trained model
        """
        if self._is_trained:
            with torch.no_grad():
                return self.model(torch.as_tensor(state, dtype=torch.float32))
        else:
            print('Model has not been trained yet. Action is the output of random initialised network')
            with torch.no_grad():
                return self.model(torch.as_tensor(state, dtype=torch.float32))


if __name__ == "__main__":


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
    learner = PolicyLearner(file_path, device)
    learner.train()

    # # make data in correct format for torch
    # train_dataloader = DataLoader(dataset, batch_size=1)
    #
    # for different_sample in train_dataloader:
    #
    #     #print(different_sample)
    #     state = different_sample['state']
    #     a_pred = model(state.float())
    #     #print(a_pred)

    action = learner.get_action(sample['state'])
    print(action)
