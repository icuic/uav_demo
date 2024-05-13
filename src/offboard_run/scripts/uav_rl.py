##! /usr/bin/python
## /home/ubuntu/miniconda3/envs/px4/bin/python

import gymnasium


import os
import sys
# print(os.path.abspath("."))
# 由于使用roscore执行本脚本时的当前工作路径为工作空间路径(~/UAV_DEMO/)，所以需要将UAVGymEnv的路径手动添加到PATH中
path = os.path.abspath(".")
sys.path.insert(0, path + "/uav_demo/src/offboard_run/scripts")

# /home/ubuntu/ws/uav_demo/src/offboard_run/scripts/UAVGymEnv
# /home/ubuntu/ws/uav_demo/src/offboard_run/scripts/UAVGymEnv/envs/uav_landing.py
# /home/ubuntu/ws/src/offboard_run/scripts/UAVGymEnv

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

# env = gymnasium.make('UAVGymEnv/UAVLandingEnv-v0')
# observation = env.reset()


if __name__ == "__main__":

    # for _ in range(10):
    #     action = env.action_space.sample()
    #     observation, reward, terminated, truncated, info = env.step(action)

    #     if terminated or truncated:
    #         observation, info = env.reset()

    # env.close()

#####################
    lr = 2e-3
    num_episodes = 500
    hidden_dim = 128
    gamma = 0.98
    epsilon = 0.9
    target_update = 10
    buffer_size = 10000
    minimal_size = 500
    batch_size = 128
    device = torch.device("cuda") if torch.cuda.is_available() else torch.device(
        "cpu")

    env_name = 'UAVGymEnv/UAVLandingEnv-v0'
    env = gymnasium.make(env_name)
    random.seed(0)
    np.random.seed(0)
    # env.seed(0)
    torch.manual_seed(0)
    replay_buffer = ReplayBuffer(buffer_size)
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.n

    agent = DQN(state_dim, hidden_dim, action_dim, lr, gamma, epsilon,
                target_update, device)

    return_list = []
    strAction = ["x+", "x-", "y+", "y-"]

    for i in range(10):
        with tqdm(total=int(num_episodes / 10), desc='Iteration %d' % i) as pbar:
            for i_episode in range(int(num_episodes / 10)):
                episode_return = 0
                state = env.reset()
                done = False

                i_step = 0
                while not done:                    
                    action = agent.take_action(state)
                    
                    print(f'{i_step:-^50}')
                    print(f'action is {strAction[action]}')

                    next_state, reward, done, _ = env.step(action)
                    i_step += 1

                    replay_buffer.add(state, action, reward, next_state, done)
                    state = next_state
                    episode_return += reward
                    # 当buffer数据的数量超过一定值后,才进行Q网络训练
                    if replay_buffer.size() > minimal_size:
                        epsilon = 0.01
                        b_s, b_a, b_r, b_ns, b_d = replay_buffer.sample(batch_size)
                        transition_dict = {
                            'states': b_s,
                            'actions': b_a,
                            'next_states': b_ns,
                            'rewards': b_r,
                            'dones': b_d
                        }
                        agent.update(transition_dict)
                return_list.append(episode_return)
                if (i_episode + 1) % 10 == 0:
                    pbar.set_postfix({
                        'episode':
                        '%d' % (num_episodes / 10 * i + i_episode + 1),
                        'return':
                        '%.3f' % np.mean(return_list[-10:])
                    })
                pbar.update(1)       


    date_str = datetime.now().strftime('%m%d')
    # np.save(f'return_list_{date_str}.npy', np.array(return_list))
    with open(f"return_list_{date_str}.json", 'w') as f:
        json.dump(return_list, f)

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
######################    

    # cnt = 0
    # action = 0
    # strAction = ["x+", "x-", "y+", "y-", "z+", "z-", "keep"]

    # while(1):
    #     print(f"{cnt:-^50}")
    #     cnt += 1
    #     # print("current action: " + strAction[action])
        
    #     state, reward, done, info = env.step(action)
    #     print("State:", state)
    #     print("Reward:", reward)
    #     print("Done:", done)
    #     # print("Info:", info)  
        
    #     if done:
    #         print("DONE")
    #         while 1:
    #             pass

    #     time.sleep(10)
    #     pass

