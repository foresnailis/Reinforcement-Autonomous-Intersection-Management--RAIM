import numpy as np
import random
import copy
import os
from collections import namedtuple, deque

import torch
import torch.nn.functional as F
import torch.optim as optim

from TD3PER.model import Actor, Critic
from TD3PER.PER import PER

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print('Device on DDPG:', device)

BUFFER_SIZE = 2**17
BATCH_SIZE = 64
GAMMA = 0.99
TAU = 1e-3
LR_ACTOR = 1e-4
LR_CRITIC = 1e-3
WEIGHT_DECAY = 0
LEARN_BATCH = 1

class DDPGAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.Qloss = 0
        self.Ploss = 0
        self.Q = 0

        self.actor_local = Actor(state_size, action_size).to(device)
        self.actor_target = Actor(state_size, action_size).to(device)
        self.actor_optimizer = optim.Adam(self.actor_local.parameters(), lr=LR_ACTOR)

        self.critic_local = Critic(state_size, action_size).to(device)
        self.critic_target = Critic(state_size, action_size).to(device)
        self.critic_optimizer = optim.Adam(self.critic_local.parameters(), lr=LR_CRITIC, weight_decay=WEIGHT_DECAY)

        self.memory = PER(BUFFER_SIZE)

    def step(self, state, action, reward, next_state, done):
        self.memory.add((state, action, reward, next_state, done))

    def select_action(self, state):
        self.actor_local.eval()
        with torch.no_grad():
            action = self.actor_local(state).cpu().data.numpy()
        self.actor_local.train()
        return np.clip(action, -1., 1.)

    def learn(self):
        for _ in range(LEARN_BATCH):
            idxs, experiences, is_weights = self.memory.sample(BATCH_SIZE)
            states = torch.from_numpy(np.vstack([e[0] for e in experiences if e is not None])).float().to(device)
            actions = torch.from_numpy(np.vstack([e[1] for e in experiences if e is not None])).float().to(device)
            rewards = torch.from_numpy(np.vstack([e[2] for e in experiences if e is not None])).float().to(device)
            next_states = torch.from_numpy(np.vstack([e[3] for e in experiences if e is not None])).float().to(device)
            dones = torch.from_numpy(np.vstack([e[4] for e in experiences if e is not None]).astype(np.uint8)).float().to(device)
            is_weights = torch.from_numpy(is_weights).float().to(device)

            next_actions = self.actor_target(next_states)
            Q_targets_next = self.critic_target(next_states, next_actions)
            Q_targets = rewards + (GAMMA * Q_targets_next * (1 - dones))

            Q_expected = self.critic_local(states, actions)
            critic_loss = F.mse_loss(Q_expected, Q_targets)
            self.Qloss += critic_loss.item()

            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            self.critic_optimizer.step()

            actions_pred = self.actor_local(states)
            actor_loss = -self.critic_local(states, actions_pred).mean()
            self.Ploss += actor_loss.item()

            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            self.soft_update(self.critic_local, self.critic_target, TAU)
            self.soft_update(self.actor_local, self.actor_target, TAU)

        self.Qloss /= LEARN_BATCH
        self.Ploss /= LEARN_BATCH

    def soft_update(self, local_model, target_model, tau):
        for target_param, local_param in zip(target_model.parameters(), local_model.parameters()):
            target_param.data.copy_(tau * local_param.data + (1.0 - tau) * target_param.data)

    def save_weights(self, path):
        actor_weights = os.path.join(path, 'weights_actor.pt')
        critic_weights = os.path.join(path, 'weights_critic.pt')
        torch.save(self.actor_local.state_dict(), actor_weights)
        torch.save(self.critic_local.state_dict(), critic_weights)

    def load_weights(self, path):
        actor_weights = os.path.join(path, 'weights_actor.pt')
        critic_weights = os.path.join(path, 'weights_critic.pt')
        self.actor_local.load_state_dict(torch.load(actor_weights))
        self.actor_target.load_state_dict(torch.load(actor_weights))
        self.critic_local.load_state_dict(torch.load(critic_weights))
        self.critic_target.load_state_dict(torch.load(critic_weights))
