#!/usr/bin/env python3
import os
import numpy as np
from datetime import datetime

import torch
from torch.utils.data import DataLoader
from torch.utils.data import RandomSampler
from torch.utils.data import random_split
import torch.nn as nn
from torch.optim.lr_scheduler import StepLR
from torch.utils.tensorboard import SummaryWriter

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

        experiment_name = 'panda'
        run_name = datetime.now().strftime('_%y%m%d_%H%M%S') 
        self.writer = SummaryWriter(log_dir=os.path.join(os.path.dirname(os.path.realpath(__file__)), 'runs', experiment_name, run_name))

    def train(self):
        """
        Main training loop.
        """

        # Training parameters
        epochs = 8
        batch_size = 32
        learning_rate = 1e-4

        # initialise loss function
        loss_fn = nn.MSELoss()
        optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)

        # initialise sampler and data loader
        train_sampler = RandomSampler(self.train_dataset)
        train_dataloader = DataLoader(self.train_dataset, batch_size=batch_size,
            sampler = train_sampler)
        test_dataloader = DataLoader(self.test_dataset, batch_size=batch_size)
        size = len(train_dataloader.dataset)
        scheduler = StepLR(optimizer, epochs/2, gamma=0.1)

        global_step = 0
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
                    self.writer.add_scalar('loss/train', loss, global_step)
                global_step += 1

            test_size = len(test_dataloader.dataset)
            test_loss = 0
            with torch.no_grad():
                for sample in test_dataloader:
                    pred = self.model(sample['state'])
                    test_loss += loss_fn(pred, sample['action']).item()


            test_loss /= len(test_dataloader)
            print(f"Test Error: \n Avg loss: {test_loss:>8f} \n")
            self.writer.add_scalar('loss/val', test_loss, global_step)
            self.writer.add_scalar('learning_rate', scheduler.get_last_lr()[0], global_step)
            scheduler.step()


        self.writer.add_hparams({'lr': learning_rate, 
                                 'bsize': batch_size, 
                                 'optimizer': type(optimizer).__name__, 
                                 'loss': type(loss_fn).__name__},
                                {'hparam/loss': test_loss}, run_name='hparams')
        self.writer.flush()
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

    def save_model(self, state, name):
        """
        saves model if trained
        """
        if self._is_trained:
            with torch.no_grad():
                # save the weights of the model to be used in python
                torch.save(self.model.state_dict(), f'{name}.pth')
                # save the model such that it can be called from cpp
                traced_script_module = torch.jit.trace(self.model,
                    torch.as_tensor(state, dtype=torch.float32))
                traced_script_module.save(f'{name}.pt')
                print('Model saved.')

        else:
            print('Cannot save model because not trained yet.')

if __name__ == "__main__":
    task_path = os.path.dirname(os.path.realpath(__file__))
    dataset_name = "PandaR100_0501213400"
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
