#!/usr/bin/env python3
import os
import numpy as np
import torch

from torch.utils.data import DataLoader
from torch.utils.data import RandomSampler
from torch.utils.data import random_split
import torch.nn as nn

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
        n_data_points = len(self.full_dataset)
        n_train_set = int(0.8 * n_data_points)
        n_test_set = n_data_points - n_train_set
        self.train_dataset, self.test_dataset = random_split(self.full_dataset,
            [n_train_set, n_test_set], generator=torch.Generator().manual_seed(1))
        # initialise MLP model
        self.model = LinReLu(self.n_states, self.n_actions).to(device)
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


            test_loss /= len(test_dataloader)
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

    def save_model(self, state):
        """
        saves model if trained
        """
        if self._is_trained:
            with torch.no_grad():
                # save the weights of the model to be used in python
                torch.save(self.model.state_dict(), 'expert_model.pth')
                # save the model such that it can be called from cpp
                traced_script_module = torch.jit.trace(self.model,
                    torch.as_tensor(state, dtype=torch.float32))
                traced_script_module.save('expert_model.pt')
                print('Model saved.')

        else:
            print('Cannot save model because not trained yet.')

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
    learner.save_model(sample['state'])
    learner.train()
    learner.save_model(sample['state'])

    action = learner.get_action(sample['state'])
    print('predicted action: ', action)
    print('action from dataset: ', sample['action'])
