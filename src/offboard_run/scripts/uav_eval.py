##! /usr/bin/python
## /home/ubuntu/miniconda3/envs/px4/bin/python

import gymnasium


import os
import sys
# print(os.path.abspath("."))
# 由于使用roscore执行本脚本时的当前工作路径为工作空间路径(~/UAV_DEMO/)，所以需要将UAVGymEnv的路径手动添加到PATH中
path = os.path.abspath(".")
sys.path.insert(0, path + "/uav_demo/src/offboard_run/scripts")

# print(sys.path)
import UAVGymEnv
from dqn import *
import random
import numpy as np
import collections
from tqdm import tqdm
import torch
import torch.nn.functional as F
import matplotlib.pyplot as plt
from datetime import datetime
import time
import json
import rl_utils as rl_utils

test_time = "0521-1039"
checkpoints_path = './checkpoints/'+test_time


if __name__ == "__main__":

    restore_from = 800

    env_name = 'UAVGymEnv/UAVLandingEnv-v0'
    env = gymnasium.make(env_name)

    with open(f"{checkpoints_path}/{restore_from}_hyperparameter.json") as f:
        d = json.load(f)
        tmp_episode = d.get('episode')
        print(f"test with mode from episode: {tmp_episode}")
        
        lr = d.get('lr')
        num_episodes = d.get('num_episodes')            
        gamma = d.get('gamma')
        epsilon = 0 #d.get('epsilon')
        target_update = d.get('target_update')
        buffer_size = d.get('buffer_size')
        minimal_size = d.get('minimal_size')
        batch_size = d.get('batch_size')
        state_dim = d.get('state_dim')
        action_dim = d.get('action_dim')
        hidden_dim = d.get('hidden_dim')

    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    agent = DQN(state_dim, hidden_dim, action_dim, lr, gamma, epsilon, target_update, device)
    agent.load(checkpoints_path, restore_from)
    agent.eval()

    with torch.no_grad():

        strAction = ["x+", "x-", "y+", "y-"]

        state, info = env.reset()
        done = False
        time.sleep(5)

        i_step = 0
        episode_return = 0
        while not done:                    
            action = agent.take_action(state)

            i_step += 1
            print(f'{i_step:-^50}')
            print(f'action is {strAction[action]}')

            next_state, reward, done, _ = env.step(action)                    
            
            state = next_state
            episode_return += reward
            
        print(f'return: {episode_return}')



    env.close()  

