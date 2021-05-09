#!/usr/bin/env python3
import os
import numpy as np
import torch

from torch.utils.data import DataLoader
from torch.utils.data import RandomSampler
from torch.utils.data import random_split
import torch.nn as nn
from torch.optim.lr_scheduler import StepLR

from dataset import StateActionDataset
from models import LinReLu



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
        self.train_dataset, self.test_dataset = self.split_dataset()
        # initialise MLP model
        self.model = LinReLu(self.n_states, self.n_actions).to(device)
        self._is_trained = False

    def split_dataset(self):
        n_data_points = len(self.full_dataset)
        n_train_set = int(0.8 * n_data_points)
        n_test_set = n_data_points - n_train_set
        train_dataset, test_dataset = random_split(self.full_dataset,
            [n_train_set, n_test_set], generator=torch.Generator().manual_seed(1))
        return train_dataset, test_dataset

    def train(self):
        """
        Main training loop.
        """

        # Training parameters
        epochs = 2
        batch_size = 32
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
        scheduler = StepLR(optimizer, epochs/3, gamma=0.1)

        for t in range(epochs):
            print(f"Epoch {t+1}\n-------------------------------")
            for batch, sample in enumerate(train_dataloader):
                pred = self.model(sample['state'])
                loss = loss_fn(pred, sample['action'])

                # Backpropagation
                optimizer.zero_grad() # set grad to zero first
                loss.backward()
                optimizer.step()

                if batch % (len(train_dataloader)//5) == 0:
                    loss, current = loss.item(), batch * len(sample['state'])
                    print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")

            test_size = len(test_dataloader.dataset)
            test_loss = 0
            with torch.no_grad():
                for sample in test_dataloader:
                    pred = self.model(sample['state'])
                    test_loss += loss_fn(pred, sample['action']).item()


            test_loss /= len(test_dataloader)
            print(f"Test Error: \n Avg loss: {test_loss:>8f} \n")
            scheduler.step()

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

    def append_dataset(self, path):
        # update the dataset object with the datapoints stored in path
        self.full_dataset.append_dataset(path)
        # split the dataset into train and test sets again
        self.train_dataset, self.test_dataset = split_dataset()



    def save_model(self, state, name):
        """
        saves model if trained
        """
        # TODO (Kiran): change I/O to get rid of state, not required because
        # a sample state can also be generated from inside this class (see
        # sample_state below)
        if self._is_trained:
            with torch.no_grad():
                sample_state = self.full_dataset[0]
                # save the weights of the model to be used in python
                torch.save(self.model.state_dict(), f'{name}.pth')
                # save the model such that it can be called from cpp
                traced_script_module = torch.jit.trace(self.model,
                    torch.as_tensor(sample_state, dtype=torch.float32))
                traced_script_module.save(f'{name}.pt')
                print('Model saved.')

        else:
            print('Cannot save model because not trained yet.')

if __name__ == "__main__":
    task_path = os.path.dirname(os.path.realpath(__file__))
    dataset_name = "Horizon_4_Train_0419140234"
    dir_path = task_path + "/../data/" + dataset_name
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
    learner.save_model(sample['state'], dataset_name)

    action = learner.get_action(sample['state'])
    print('predicted action: ', action)
    print('action from dataset: ', sample['action'])
