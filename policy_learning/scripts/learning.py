#!/usr/bin/env python3
import os
from datetime import datetime

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, RandomSampler
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
        self.device = device
        self.file_path = file_path
        self.train_dataset = StateActionDataset(root_dir=os.path.join(file_path, 'train'))
        self.test_dataset = StateActionDataset(root_dir=os.path.join(file_path, 'test'))
        self.n_states, self.n_actions = self.train_dataset.get_dimensions()
        # initialise MLP model
        self.model = None

    def train(self):
        """
        Main training loop.
        """
        # Model parameters
        n_nodes = 32
        n_hidden_layers = 1
        self.model = LinReLu(self.n_states, self.n_actions,
                             n_nodes=n_nodes, n_hidden_layers=n_hidden_layers).to(self.device)

        # Training parameters
        epochs = 16
        batch_size = 32
        learning_rate = 1e-4
        # initialise loss function
        loss_fn = nn.MSELoss()
        optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
        scheduler = None
        # scheduler = StepLR(optimizer, epochs/2, gamma=0.1)

        # Initialize TensorBoard logger
        experiment_name = os.path.join(os.path.basename(self.file_path),
                                       type(loss_fn).__name__)
        run_name = datetime.now().strftime('_%y%m%d_%H%M%S')
        writer = SummaryWriter(log_dir=os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            'runs', experiment_name, run_name))

        # initialise sampler and data loader
        train_sampler = RandomSampler(self.train_dataset)
        train_dataloader = DataLoader(self.train_dataset, batch_size=batch_size,
            sampler = train_sampler)
        test_dataloader = DataLoader(self.test_dataset, batch_size=batch_size)
        size = len(train_dataloader.dataset)

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
                    writer.add_scalar('loss/train', loss, global_step)
                global_step += 1

            test_loss = 0
            with torch.no_grad():
                for sample in test_dataloader:
                    pred = self.model(sample['state'])
                    test_loss += loss_fn(pred, sample['action']).item()

            test_loss /= len(test_dataloader)
            print(f"Test Error: \n Avg loss: {test_loss:>8f} \n")
            writer.add_scalar('loss/val', test_loss, global_step)
            writer.add_scalar('learning_rate',
                              scheduler.get_last_lr()[0] if scheduler is not None else learning_rate,
                              global_step)
            if scheduler is not None:
                scheduler.step()

        writer.add_hparams({'lr': learning_rate,
                            'bsize': batch_size,
                            'optimizer': type(optimizer).__name__,
                            'loss': type(loss_fn).__name__,
                            'n_nodes': n_nodes,
                            'n_hidden_layers': n_hidden_layers,
                            'model_name': type(self.model).__name__},
                            {'hparam/loss': test_loss}, run_name='hparams')
        writer.flush()

    def get_action(self, state):
        """
        returns action from trained model
        """
        if self.model is not None:
            with torch.no_grad():
                return self.model(torch.as_tensor(state, dtype=torch.float32))
        else:
            print('Model has not been trained yet.')
            return None

    def save_model(self, path, name):
        """
        saves model if trained
        """
        if self.model is not None:
            with torch.no_grad():
                state = self.train_dataset[0]['state']
                # save the weights of the model to be used in python
                torch.save(self.model.state_dict(), f'{path}/{name}.pth')
                # save the model such that it can be called from cpp
                traced_script_module = torch.jit.trace(self.model,
                    torch.as_tensor(state, dtype=torch.float32))
                traced_script_module.save(f'{path}/{name}.pt')
                print('Model saved.')

        else:
            print('Cannot save model because not trained yet.')

if __name__ == "__main__":
    DATASET_NAME = "PandaPrivileged250_210507_172626"

    task_path = os.path.dirname(os.path.realpath(__file__))
    dir_path = task_path + "/../data/" + DATASET_NAME

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    learner = PolicyLearner(dir_path, device)
    learner.train()
    learner.save_model(task_path + "/../models/", DATASET_NAME)
