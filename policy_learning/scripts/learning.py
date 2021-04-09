#!/usr/bin/env python3

import os
import h5py
import csv
import numpy as np
import torch
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
from torch.utils.data import RandomSampler
from torch.utils.data import random_split
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
        self.full_dataset = StateActionDataset(root_dir=file_path)
        self.n_states, self.n_actions = self.full_dataset.get_dimensions()
        # split the dataset into train and test sets
        n_data_points = len(self.full_dataset)
        n_train_set = int(0.8 * n_data_points)
        n_test_set = n_data_points - n_train_set
        self.train_dataset, self.test_dataset = random_split(self.full_dataset,
            [n_train_set, n_test_set], generator=torch.Generator().manual_seed(1))
        # initialise MLP model
        self.model = MLPSampler(self.n_states, self.n_actions).to(device)
        self._is_trained = False

    def train(self):
        """
        Main training loop.
        """

        # Training parameters
        epochs = 2
        batch_size = 20
        learning_rate = 1e-3

        # initialise loss function
        loss_fn = nn.MSELoss()
        optimizer = torch.optim.SGD(self.model.parameters(), lr=learning_rate)

        # initialise sampler and data loader
        train_sampler = RandomSampler(self.train_dataset)
        train_dataloader = DataLoader(self.train_dataset, batch_size=batch_size,
            sampler = train_sampler)
        test_dataloader = DataLoader(self.test_dataset, batch_size=batch_size)
        size = len(train_dataloader.dataset)

        for t in range(epochs):
            print(f"Epoch {t+1}\n-------------------------------")
            for batch, sample in enumerate(train_dataloader):
                pred = self.model(sample['state'])
                loss = loss_fn(pred, sample['action'])

                # Backpropagation
                optimizer.zero_grad() # set grad to zero first
                loss.backward()
                optimizer.step()

                if batch % 1000 == 0:
                    loss, current = loss.item(), batch * len(sample['state'])
                    print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")

            test_size = len(test_dataloader.dataset)
            test_loss = 0
            with torch.no_grad():
                for sample in test_dataloader:
                    pred = self.model(sample['state'])
                    test_loss += loss_fn(pred, sample['action']).item()

            test_loss /= batch_size
            print(f"Test Error: \n Avg loss: {test_loss:>8f} \n")

        self._is_trained = True
        print("Done!")

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
    dir_path = task_path + "/../data/0407141317"
    dataset = StateActionDataset(root_dir=dir_path)
    # fix a random seed
    torch.manual_seed(1)
    idx=1
    sample = dataset[idx]
    print("Index: ", idx, "\nstate: " , sample['state'], "\naction: ", sample['action'])
    dimensions = len(dataset)
    print('number of training samples: ', dimensions)
    # set device for training
    device = 'cuda' if torch.cuda.is_available() else 'cpu'


    learner = PolicyLearner(dir_path, device)
    learner.train()


    action = learner.get_action(sample['state'])
    print('predicted action: ', action)
    print('action from dataset: ', sample['action'])
