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
from ddpg import *
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
import argparse

test_time = "0113-2330"
checkpoints_path = './checkpoints/'+test_time


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--motion-type', type=str, 
                        choices=['static', 'linear', 'circular', 'random'], 
                        default='linear', 
                        help="移动平台运动类型: 静止/匀速直线/匀速圆周/随机")
    parser.add_argument('--speed', type=float, default=0.2, 
                        help="移动平台速度 (最大0.8 m/s)")
    args = parser.parse_args()

    # 确保速度不超过0.8
    args.speed = min(args.speed, 0.8)

    restore_from = 7000

    env_name = 'UAVGymEnv/UAVLandingEnv-v0'
    env = gymnasium.make(env_name, motion_type=args.motion_type, speed=args.speed)

    with open(f"{checkpoints_path}/{restore_from}_hyperparameter.json") as f:
        d = json.load(f)
        tmp_episode = d.get('episode')
        print(f"start from episode: {tmp_episode}")

        actor_lr = d.get('actor_lr')
        critic_lr = d.get('critic_lr')
        hidden_dim = d.get('hidden_dim')
        gamma = d.get('gamma')
        tau = d.get('tau')
        buffer_size = d.get('buffer_size')
        minimal_size = d.get('minimal_size')
        batch_size = d.get('batch_size')
        sigma = d.get('sigma')
        total_iterated = d.get('total_iterated')

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    action_bound = env.action_space.high[0]  # 动作最大值

    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    agent = DDPG(state_dim, hidden_dim, action_dim, action_bound, sigma, actor_lr, critic_lr, tau, gamma, device)
    agent.load(checkpoints_path, restore_from)
    agent.eval()

    with torch.no_grad():
        finish = 0
        out = 0
        timeout = 0

        for i_episode in range(0, 10):
            state, info = env.reset()
            done = False

            while not done:                    
                action = agent.take_action(state)
                next_state, reward, done, _ = env.step(action)                                    
                state = next_state
                
            if _['done_reason'] == 'finish':
                finish += 1
            elif _['done_reason'] == 'out of map':
                out += 1
            elif _['done_reason'] == 'timeout':
                timeout += 1
            
            print(f'finish/total: {(finish, out, timeout, round(finish/(i_episode+1),2))}/{i_episode+1}')

    env.close()  

