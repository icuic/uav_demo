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

checkpoints_path = './checkpoints'

def create_checkpoints_folder():
    folder_name = 'checkpoints'
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
        print(f"Folder '{folder_name}' created successfully.")
    else:
        print(f"Folder '{folder_name}' already exists.")

def save_return_list(i, path, return_list):
    with open(f"{path}/{i}_return_list.pkl", 'wb') as f:
        pickle.dump(return_list, f)

def load_return_list(i, path):
    with open(f"{path}/{i}_return_list.pkl", 'rb') as f:
        return pickle.load(f)

if __name__ == "__main__":

    create_checkpoints_folder()

    restore_from_checkpoint = True
    restore_from = 8

    env_name = 'UAVGymEnv/UAVLandingEnv-v0'
    env = gymnasium.make(env_name)

    if restore_from_checkpoint:
        with open(f"{checkpoints_path}/{restore_from}_hyperparameter.json") as f:
            d = json.load(f)
            tmp_episode = d.get('episode')
            print(f"start from episode: {tmp_episode}")
            
            lr = d.get('lr')
            num_episodes = d.get('num_episodes')            
            gamma = d.get('gamma')
            epsilon = d.get('epsilon')
            target_update = d.get('target_update')
            buffer_size = d.get('buffer_size')
            minimal_size = d.get('minimal_size')
            batch_size = d.get('batch_size')
            state_dim = d.get('state_dim')
            action_dim = d.get('action_dim')
            hidden_dim = d.get('hidden_dim')
    else:
        lr = 2e-3
        num_episodes = 500
        gamma = 0.98
        epsilon = 0.9
        target_update = 10
        buffer_size = 10000
        minimal_size = 500
        batch_size = 128
        state_dim = env.observation_space.shape[0]
        action_dim = env.action_space.n
        hidden_dim = 128

    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    random.seed(0)
    np.random.seed(0)
    # env.seed(0)
    torch.manual_seed(0)    

    replay_buffer = ReplayBuffer(buffer_size)
    if restore_from_checkpoint:
        replay_buffer.load(f"{checkpoints_path}/{restore_from}_buffer.pth")
        print(f"buffer size: {replay_buffer.size()}")

    agent = DQN(state_dim, hidden_dim, action_dim, lr, gamma, epsilon, target_update, device)
    if restore_from_checkpoint:
        agent.load(checkpoints_path, restore_from)

    return_list = []
    if restore_from_checkpoint:
        return_list = load_return_list(restore_from, checkpoints_path)

    strAction = ["x+", "x-", "y+", "y-"]
    total_iterated = 0

    for i_episode in range(restore_from+1, 500):
        # print("--------------type s---------------")
        # print(f"i_episode: {type(i_episode)}")
        # print(f"num_episodes: {type(num_episodes)}")
        # print(f"total_iterated: {type(total_iterated)}")
        # print(f"target_update: {type(target_update)}")
        # print(f"epsilon: {type(epsilon)}")
        # print(f"batch_size: {type(batch_size)}")
        # print(f"lr: {type(lr)}")
        # print(f"gamma: {type(gamma)}")
        # print(f"buffer_size: {type(buffer_size)}")
        # print(f"minimal_size: {type(minimal_size)}")
        # print(f"state_dim: {type(state_dim)}")
        # print(f"action_dim: {type(action_dim)}")
        # print(f"hidden_dim: {type(hidden_dim)}")
        # print("--------------type e---------------")

        episode_return = 0
        state = env.reset()
        done = False
        time.sleep(60)

        # for i in range(500):
        #     env.step(0)
        #     time.sleep(1)


        i_step = 0
        while not done:                    
            time.sleep(1)
            action = agent.take_action(state)
            # action = 0

            i_step += 1
            print(f'{i_step:-^50}')
            print(f'action is {strAction[action]}')

            next_state, reward, done, _ = env.step(action)                    
            replay_buffer.add(state, action, reward, next_state, done)
            
            state = next_state
            episode_return += reward
            
            # 当buffer数据的数量超过一定值后,才进行Q网络训练
            if replay_buffer.size() > minimal_size:
                b_s, b_a, b_r, b_ns, b_d = replay_buffer.sample(batch_size)
                transition_dict = {
                    'states': b_s,
                    'actions': b_a,
                    'next_states': b_ns,
                    'rewards': b_r,
                    'dones': b_d
                }
                agent.update(transition_dict)
                total_iterated  += 1

        epsilon *= 0.98
        agent.set_epsilon(epsilon)
        return_list.append(episode_return)

        print(f'episode: {i_episode}, return: {episode_return}')

        agent.save(checkpoints_path, i_episode)
        replay_buffer.save(f"{checkpoints_path}/{i_episode}_buffer.pth")
        save_return_list(i_episode, checkpoints_path, return_list)

        parameter_keys = ['episode', 'num_episodes', 'total_iterated', 'target_update', 'epsilon', 
                          'batch_size', 'lr', 'gamma', 'buffer_size', 'minimal_size', 'state_dim', 'action_dim', 'hidden_dim']
        parameter_values = [i_episode, num_episodes, total_iterated, target_update, epsilon, 
                            batch_size, lr, gamma, buffer_size, minimal_size, state_dim, int(action_dim), hidden_dim]
        
        parameter_dictionary = dict(zip(parameter_keys, parameter_values))
        with open(f'{checkpoints_path}/{i_episode}_hyperparameter' + '.json', 'w') as outfile:
            json.dump(parameter_dictionary, outfile)

    env.close()


    episodes_list = list(range(len(return_list)))
    plt.plot(episodes_list, return_list)
    plt.xlabel('Episodes')
    plt.ylabel('Returns')
    plt.title('DQN on {}'.format(env_name))
    plt.show()

    mv_return = rl_utils.moving_average(return_list, 9)
    plt.plot(episodes_list, mv_return)
    plt.xlabel('Episodes')
    plt.ylabel('Returns')
    plt.title('DQN on {}'.format(env_name))
    plt.show()      

