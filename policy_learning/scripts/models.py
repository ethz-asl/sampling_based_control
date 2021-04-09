import torch
import torch.nn as nn

class LinReLu(nn.Module):
    """
    A class for the informed sample generation.
    """
    def __init__(self, n_in, n_out, n_nodes=512, n_hidden_layers=1):
        super(LinReLu, self).__init__()

        layers = [
            nn.Linear(n_in, n_nodes),
            nn.ReLU()
        ]
        for i in range(n_hidden_layers):
            layers.append(nn.Linear(n_nodes,n_nodes))
            layers.append(nn.ReLU())
        layers.append(nn.Linear(n_nodes,n_out))
        self.linear_relu_stack = nn.Sequential(*layers)


    def forward(self, x):
        """
        return: output of model
        """
        return self.linear_relu_stack(x)