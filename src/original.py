#!/usr/bin/env python
# Authors: Junior Costa de Jesus #

import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32
import gym
import gym_mobilerobot
import torch
import torch.nn.functional as F
import gc
import torch.nn as nn
import math
from collections import deque
import pandas as pd

torch.manual_seed(0)
np.random.seed(0)

#---Directory Path---#
dirPath = os.path.dirname(os.path.realpath(__file__))
#---Functions to make network updates---#

def soft_update(target, source, tau):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(target_param.data*(1.0 - tau)+ param.data*tau)

def hard_update(target,source):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(param.data)

#---Actor---#

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, action_limit_v, action_limit_w):
        super(Actor, self).__init__()
        self.state_dim = state_dim = state_dim
        self.action_dim = action_dim
        self.action_limit_v = action_limit_v
        self.action_limit_w = action_limit_w

        self.fa1 = nn.Linear(state_dim, 512)
        self.fa1.weight.data = fanin_init(self.fa1.weight.data.size())

        self.fa2 = nn.Linear(512, 512)
        self.fa2.weight.data = fanin_init(self.fa2.weight.data.size())

        self.fa3 = nn.Linear(512, action_dim)
        self.fa3.weight.data.uniform_(-EPS,EPS)

    def forward(self, state):
        x = torch.relu(self.fa1(state))
        x = torch.relu(self.fa2(x))
        action = self.fa3(x)
        if state.shape == torch.Size([14]):
            action[0] = torch.sigmoid(action[0])*self.action_limit_v
            action[1] = torch.tanh(action[1])*self.action_limit_w
        else:
            action[:,0] = torch.sigmoid(action[:,0])*self.action_limit_v
            action[:,1] = torch.tanh(action[:,1])*self.action_limit_w
        return action

#---Critic--#

EPS = 0.003
def fanin_init(size, fanin=None):
    fanin = fanin or size[0]
    v = 1./np.sqrt(fanin)
    return torch.Tensor(size).uniform_(-v,v)

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()

        self.state_dim = state_dim = state_dim
        self.action_dim = action_dim

        self.fc1 = nn.Linear(state_dim, 256)
        self.fc1.weight.data = fanin_init(self.fc1.weight.data.size())

        self.fa1 = nn.Linear(action_dim, 256)
        self.fa1.weight.data = fanin_init(self.fa1.weight.data.size())

        self.fca1 = nn.Linear(512, 512)
        self.fca1.weight.data = fanin_init(self.fca1.weight.data.size())

        self.fca2 = nn.Linear(512, 1)
        self.fca2.weight.data.uniform_(-EPS, EPS)

    def forward(self, state, action):
        xs = torch.relu(self.fc1(state))
        xa = torch.relu(self.fa1(action))
        x = torch.cat((xs,xa), dim=1)
        x = torch.relu(self.fca1(x))
        vs = self.fca2(x)
        return vs

#---Memory Buffer---#

class MemoryBuffer:
    def __init__(self, size):
        self.buffer = deque(maxlen=size)
        self.maxSize = size
        self.len = 0

    def sample(self, count):
        batch = []
        count = min(count, self.len)
        batch = random.sample(self.buffer, count)

        s_array = np.float32([array[0] for array in batch])
        a_array = np.float32([array[1] for array in batch])
        r_array = np.float32([array[2] for array in batch])
        new_s_array = np.float32([array[3] for array in batch])

        return s_array, a_array, r_array, new_s_array

    def len(self):
        return self.len

    def add(self, s, a, r, new_s):
        transition = (s, a, r, new_s)
        self.len += 1
        if self.len > self.maxSize:
            self.len = self.maxSize
        self.buffer.append(transition)

#---Where the train is made---#

BATCH_SIZE = 128
LEARNING_RATE = 0.001
GAMMA = 0.99
TAU = 0.001

class Trainer:

    def __init__(self, state_dim, action_dim, action_limit_v, action_limit_w, ram):

        self.state_dim = state_dim
        self.action_dim = action_dim
        self.action_limit_v = action_limit_v
        self.action_limit_w = action_limit_w
        #print('w',self.action_limit_w)
        self.ram = ram

        self.actor = Actor(self.state_dim, self.action_dim, self.action_limit_v, self.action_limit_w)
        self.target_actor = Actor(self.state_dim, self.action_dim, self.action_limit_v, self.action_limit_w)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), LEARNING_RATE)

        self.critic = Critic(self.state_dim, self.action_dim)
        self.target_critic = Critic(self.state_dim, self.action_dim)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), LEARNING_RATE)
        self.pub_qvalue = rospy.Publisher('qvalue', Float32, queue_size=5)
        self.qvalue = Float32()

        hard_update(self.target_actor, self.actor)
        hard_update(self.target_critic, self.critic)

    def get_exploitation_action(self,state):
        state = torch.from_numpy(state)
        action = self.target_actor.forward(state).detach()
        return action.data.numpy()

    def get_exploration_action(self, state):
        state = torch.from_numpy(state)
        action = self.actor.forward(state).detach()
        new_action = action.data.numpy()
        return new_action

    def get_exploration_noise(self,goal_count, var_v, var_w, noise_decay='timestep_based'):
        if noise_decay=='timestep_based':
            var_v = max([var_v*0.99999, 0.10*ACTION_V_MAX])
            var_w = max([var_w*0.99999, 0.10*ACTION_W_MAX])

        elif noise_decay=='goal_count_based':
            var_v = max([var_v*0.99999, 0.10*ACTION_V_MAX])
            var_w = max([var_w*0.99999, 0.10*ACTION_W_MAX])

        return var_v,var_w


    def optimizer(self):
        s_sample, a_sample, r_sample, new_s_sample = ram.sample(BATCH_SIZE)

        s_sample = torch.from_numpy(s_sample)
        a_sample = torch.from_numpy(a_sample)
        r_sample = torch.from_numpy(r_sample)
        new_s_sample = torch.from_numpy(new_s_sample)

        #-------------- optimize critic

        a_target = self.target_actor.forward(new_s_sample).detach()
        next_value = torch.squeeze(self.target_critic.forward(new_s_sample, a_target).detach())
        # y_exp = r _ gamma*Q'(s', P'(s'))
        y_expected = r_sample + GAMMA*next_value
        # y_pred = Q(s,a)
        y_predicted = torch.squeeze(self.critic.forward(s_sample, a_sample))
        #-------Publisher of Vs------
        self.qvalue = y_predicted.detach()
        self.pub_qvalue.publish(torch.max(self.qvalue))
        #print(self.qvalue, torch.max(self.qvalue))
        #----------------------------
        loss_critic = F.smooth_l1_loss(y_predicted, y_expected)

        self.critic_optimizer.zero_grad()
        loss_critic.backward()
        self.critic_optimizer.step()

        #------------ optimize actor
        pred_a_sample = self.actor.forward(s_sample)
        loss_actor = -1*torch.sum(self.critic.forward(s_sample, pred_a_sample))

        self.actor_optimizer.zero_grad()
        loss_actor.backward()
        self.actor_optimizer.step()

        soft_update(self.target_actor, self.actor, TAU)
        soft_update(self.target_critic, self.critic, TAU)

    def save_models(self, episode_count):
        torch.save(self.target_actor.state_dict(), dirPath +'/Models/'+str(episode_count)+ '_actor.pt')
        torch.save(self.target_critic.state_dict(), dirPath + '/Models/'+str(episode_count)+ '_critic.pt')
        print('****Models saved***')

    def load_models(self, episode):
        self.actor.load_state_dict(torch.load(dirPath + '/Models/'+str(episode)+ '_actor.pt'))
        self.critic.load_state_dict(torch.load(dirPath + '/Models/'+str(episode)+ '_critic.pt'))
        hard_update(self.target_actor, self.actor)
        hard_update(self.target_critic, self.critic)
        print('***Models load***')

#---Run agent---#

is_training = True

if is_training:
    exploration_rate = 1
    max_exploration_rate = 1
    min_exploration_rate = 0.05
else:
    exploration_rate = 0.05
    max_exploration_rate = 0.05
    min_exploration_rate = 0.05

exploration_decay_rate = 0.00

MAX_EPISODES = 10001
MAX_STEPS = 500
MAX_BUFFER = 100000
rewards_all_episodes = []

STATE_DIMENSION = 14
ACTION_DIMENSION = 2
ACTION_V_MAX = 0.22 # m/s
ACTION_W_MAX = 2 # rad/s

if is_training:
    var_v = ACTION_V_MAX*0.20
    var_w = ACTION_W_MAX*2*0.20
else:
    var_v = ACTION_V_MAX*0.10
    var_w = ACTION_W_MAX*0.10

print('State Dimensions: ' + str(STATE_DIMENSION))
print('Action Dimensions: ' + str(ACTION_DIMENSION))
print('Action Max: ' + str(ACTION_V_MAX) + ' m/s and ' + str(ACTION_W_MAX) + ' rad/s')
ram = MemoryBuffer(MAX_BUFFER)
trainer = Trainer(STATE_DIMENSION, ACTION_DIMENSION, ACTION_V_MAX, ACTION_W_MAX, ram)
episode_load = 0
# trainer.load_models(episode_load)


if __name__ == '__main__':
    rospy.init_node('ddpg_stage_1')
    pub_result = rospy.Publisher('result', Float32, queue_size=5)
    result = Float32()
    env = gym.make("Mobilerobot-v0")
    df1 = pd.DataFrame({"collision_n":[0], "overtime_n":[0],"goal_n":[0]})
    df2 = pd.DataFrame({"episode":[0],"timestep":[0],"linear":[0],"angular":[0],"reward":[0]})

    start_time = time.time()
    past_action = np.array([0.,0.])
    overtime_n=0
    goal_n=0
    timestep= 0

    for ep in range(MAX_EPISODES):
        done = False
        state = env.reset()
        print('Episode: ' + str(ep+episode_load))

        rewards_current_episode = 0
        for step in range(MAX_STEPS):
            state = np.float32(state)
            timestep+=1
            if is_training:
                action = trainer.get_exploration_action(state)
                action[0] = np.clip(np.random.normal(action[0], var_v), 0., ACTION_V_MAX)
                action[1] = np.clip(np.random.normal(action[1], var_w), -ACTION_W_MAX, ACTION_W_MAX)
            if not is_training:
                action = trainer.get_exploitation_action(state)
            next_state, reward, done, _ = env.step(action)
            # Load detais to csv file
            df_temp1 = pd.DataFrame({"episode":[ep], "timestep":[timestep],"linear":[action.item(0)],"angular":[action.item(1)], "reward":[reward]})
            df1 = df1.append(df_temp1, ignore_index = True,sort=False)
            # print str(ep+episode_load),'linear_vel:',action.item(0),'angular_vel:',action.item(1),'reward',reward
            past_action = action

            rewards_current_episode += reward
            next_state = np.float32(next_state)
            ram.add(state, action, reward, next_state)
            state = next_state

            if ram.len >= 2*MAX_STEPS and is_training:
                # var_v = max([var_v*0.99999, 0.10*ACTION_V_MAX])
                # var_w = max([var_w*0.99999, 0.10*ACTION_W_MAX])
                var_v,var_w = trainer.get_exploration_noise(goal_n, var_v, var_w)
                trainer.optimizer()

            if done or step == MAX_STEPS-1:
                print('reward per ep: ' + str(rewards_current_episode))
                print('explore_v: ' + str(var_v) + ' and explore_w: ' + str(var_w))
                if step == MAX_STEPS-1: overtime_n+=1

                rewards_all_episodes.append(rewards_current_episode)
                result = rewards_current_episode
                pub_result.publish(result)
                m, s = divmod(int(time.time() - start_time), 60)
                h, m = divmod(m, 60)
                collision_n, goal_n = env.stats()

                df_temp2 = pd.DataFrame({"episode":[ep],"ep_reward":[rewards_current_episode],"collision_n":[collision_n],
                                        "overtime_n":[overtime_n],"goal_n":[goal_n]})
                df2 = df2.append(df_temp2, ignore_index = True,sort=False)

                break
        exploration_rate = (min_exploration_rate +
                (max_exploration_rate - min_exploration_rate)* np.exp(-exploration_decay_rate*ep))
        gc.collect()

        if ep%10 == 0:
            trainer.save_models(ep+episode_load)
            print('Saved files')
            df1.to_csv(dirPath + '/log_files/'+ 'outcome_data.csv')
            df2.to_csv(dirPath + '/log_files/'+ 'step_data.csv')

print('Completed Training')
