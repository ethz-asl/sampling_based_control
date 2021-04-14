import torch
import torch.nn as nn
import math

class CartPoleNet(nn.Module):
    """
    A policy for the cart pole example. Theta (x(1)) will be normalized to +- pi
    """
    def __init__(self, n_in, n_out, n_nodes=512, n_hidden_layers=1):
        super(CartPoleNet, self).__init__()

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
        x = torch.atleast_2d(x)
        x[:,1] = torch.remainder(x[:,1]+math.pi, 2*math.pi) - math.pi
        return self.linear_relu_stack(x)