import time

import random
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from velodyne_env import GazeboEnv


class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()

        self.layer_1 = nn.Linear(state_dim, 256)
        self.layer_2 = nn.Linear(256, 256)
        self.layer_3 = nn.Linear(256, action_dim)

    def forward(self, x):
        x = F.relu(self.layer_1(x))
        x = F.relu(self.layer_2(x))
        x = torch.tanh(self.layer_3(x))
        return x


# TD3 network
class TD3(object):
    def __init__(self, state_dim, action_dim):
        # Initialize the Actor network
        self.actor = Actor(state_dim, action_dim).to(device)

    def get_action(self, state):
        # Function to get the action from the actor
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def load(self, filename, directory):
        # Function to load network parameters
        self.actor.load_state_dict(
            torch.load("%s/%s_actor.pth" % (directory, filename))
        )


# Set the parameters for the implementation
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # cuda or cpu
seed = 0  # Random seed number
max_ep = 250  # maximum number of steps per episode
file_name = "TD3_velodyne"  # name of the file to load the policy from


# Create the testing environment
environment_dim = 20
robot_dim = 4
env = GazeboEnv("multi_robot_scenario.launch", environment_dim)
time.sleep(5)

random.seed(seed)
np.random.seed(seed)
torch.manual_seed(seed)

state_dim = environment_dim + robot_dim
action_dim = 2

# Create the network
network = TD3(state_dim, action_dim)
try:
    network.load(file_name, "./pytorch_models")
except:
    raise ValueError("Could not load the stored model parameters")

done = False
episode_timesteps = 0
state = env.reset()

# Begin the testing loop
while True:
    action = network.get_action(np.array(state))

    # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
    a_in = [(action[0] + 1) / 2, action[1]]
    next_state, reward, done, target = env.step(a_in)
    done = 1 if episode_timesteps + 1 == max_ep else int(done)

    # On termination of episode
    if done:
        state = env.reset()
        done = False
        episode_timesteps = 0
    else:
        state = next_state
        episode_timesteps += 1