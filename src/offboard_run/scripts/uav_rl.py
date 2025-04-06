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

test_time = "0327-0940"
checkpoints_path = './checkpoints/'+test_time

def create_checkpoints_folder():
    folder_name = 'checkpoints/'+test_time
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

def save_success_rate_list(i, path, success_rate_list):
    with open(f"{path}/{i}_success_rate_list.pkl", 'wb') as f:
        pickle.dump(success_rate_list, f)

def load_success_rate_list(i, path):
    with open(f"{path}/{i}_success_rate_list.pkl", 'rb') as f:
        return pickle.load(f)

def save_reason_list(i, path, reason_list):
    with open(f"{path}/{i}_reason_list.pkl", 'wb') as f:
        pickle.dump(reason_list, f)

def load_reason_list(i, path):
    with open(f"{path}/{i}_reason_list.pkl", 'rb') as f:
        return pickle.load(f)

def plot_realtime(ax1, ax2, ax3, ax4, returns, critic_losses, actor_losses, success_rates, window_size=100):
    """实时更新四幅图表"""
    try:
        # 第一幅：回报曲线
        ax1.cla()
        ax1.plot(list(returns)[-window_size:], 'b-')
        ax1.set_title(f"Recent {window_size} Episodes Return (Current: {returns[-1] if returns else 0:.2f})")
        ax1.set_ylabel("Return")

        # 第二幅：Critic Loss
        ax2.cla()
        ax2.plot(list(critic_losses), 'r-')
        ax2.set_title(f"Critic Loss (Latest: {critic_losses[-1] if critic_losses else 0:.4f})")
        ax2.set_ylabel("Loss")

        # 第三幅：成功率指标
        ax3.cla()
        if success_rates:
            episodes = [x[0] for x in success_rates]
            ax3.plot(episodes, [x[1] for x in success_rates], 'g-', label='Success Rate')
            ax3.plot(episodes, [x[2] for x in success_rates], 'y-', label='Timeout Rate')
            ax3.plot(episodes, [x[3] for x in success_rates], 'r-', label='Crash Rate')
            ax3.plot(episodes, [x[4] for x in success_rates], 'm-', label='OutMap Rate')
            ax3.legend(loc='upper right')
            ax3.set_title("Termination Reasons Rate")
            ax3.set_xlabel("Episodes")
            ax3.set_ylabel("Rate")
            ax3.set_ylim(0, 1)  # 固定Y轴范围

        # 第四幅：Actor Loss
        ax4.cla()
        ax4.plot(list(actor_losses), 'g-')
        ax4.set_title(f"Actor Loss (Latest: {actor_losses[-1] if actor_losses else 0:.4f})")
        ax4.set_ylabel("Loss")

        plt.tight_layout()
        plt.pause(0.001)
    except Exception as e:
        print(f"Plotting error: {str(e)}")

# def curriculum_learning(episode):
#     if episode < 3000:  # 延长静态目标训练
#         env.motion_type = 'static'
#     elif episode < 10000:  # 延长线性运动训练
#         env.motion_type = 'linear'
#         env.speed = 0.2  # 降低初始速度
#     else:  # 渐进式增加难度
#         env.motion_type = 'circular'
#         env.speed = min(0.2 + (episode-10000)//1000*0.02, 0.4)  # 逐步加速

if __name__ == "__main__":

    create_checkpoints_folder()

    algorithm = 'ddpg'
    restore_from_checkpoint = True
    restore_from = 30146
    episode_from = 0

    env_name = 'UAVGymEnv/UAVLandingEnv-v0'
    env = gymnasium.make(env_name)

    if restore_from_checkpoint:
        episode_from = restore_from + 1
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
            # sigma = 0.15
            total_iterated = d.get('total_iterated')
    else:
        actor_lr = 1e-4
        critic_lr = 1e-4
        hidden_dim = 64*2
        gamma = 0.98
        tau = 0.005  # 软更新参数
        buffer_size = 20000
        minimal_size = 1000
        batch_size = 128
        sigma = 0.3  # 高斯噪声标准差
        total_iterated = 0

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    action_bound = env.action_space.high[0]  # 动作最大值


    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    random.seed(0)
    np.random.seed(0)
    # env.seed(0)
    torch.manual_seed(0)    

    replay_buffer = rl_utils.ReplayBuffer(buffer_size)
    agent = DDPG(state_dim, hidden_dim, action_dim, action_bound, sigma, actor_lr, critic_lr, tau, gamma, device)

    return_list = []
    reason_list = []
    success_rate_list = []


    if restore_from_checkpoint:
        replay_buffer.load(f"{checkpoints_path}/{restore_from}_buffer.pth")       
        agent.load(checkpoints_path, restore_from)        
        return_list = load_return_list(restore_from, checkpoints_path)
        reason_list = load_reason_list(restore_from, checkpoints_path)
        success_rate_list = load_success_rate_list(restore_from, checkpoints_path)
        # success_rate_list = []

    early_stop = False
    continue_times = 0
    reason_fifo_list = collections.deque(maxlen=100)
    critic_loss_list = collections.deque(maxlen=1000)   # 保留最近1000个训练步的Critic损失
    actor_loss_list = collections.deque(maxlen=1000)    # 保留最近1000个训练步的Actor损失

    # 创建绘图窗口
    plt.ion()  # 启用交互模式
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(8, 6))  # 2x2布局

    for i_episode in range(episode_from, 50000):
        # curriculum_learning(i_episode)

        episode_return = 0
        distance = 0
        state, info = env.reset()
        distance = info.get('distance')
        destination = info.get('dest')
        done = False

        print(f"{'='*20} episode: {i_episode} {'='*20}")
        i_step = 0
        while not done:                    
            # time.sleep(0.05)
            action = agent.take_action(state, i_episode)
            action = np.round(action, 2)

            # print(f"state: {state} action: {action}")

            # action = np.array([0, 0], dtype=float)

            # print("action type: ", type(action))
            # print("action size: ", action.size)   
            # print(f'action: {action}')      
            # print(f'shape: {action.shape}')   

            i_step += 1
            print(f'{f"{i_episode}/{i_step}-{destination}":-^50}')
            # print(f'action is {action[0], action[1], action[2]}')

            next_state, reward, done, _ = env.step(action)     
            # print(f"--000--- exp: state: {state}, action: {action}, reward: {reward}, next_state: {next_state}, done: {done}")
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
                actor_loss, critic_loss =  agent.update(transition_dict)

                critic_loss_list.append(critic_loss)  # 自动限制长度
                actor_loss_list.append(actor_loss)    # 自动限制长度

                total_iterated  += 1


        return_list.append(episode_return)
        reason_list.append(_['done_reason'])        
        reason_fifo_list.append(_['done_reason'])

        # len_reason_fifo_list = len(reason_fifo_list)
        len_reason_fifo_list = 100
        rate_success = reason_fifo_list.count('finish') / len_reason_fifo_list
        rate_timeout = reason_fifo_list.count('timeout') / len_reason_fifo_list
        rate_crash = reason_fifo_list.count('crash') / len_reason_fifo_list
        rate_outmap = reason_fifo_list.count('out of map') / len_reason_fifo_list
        print(f"success: {rate_success:.2f}, timeout: {rate_timeout:.2f}, crash: {rate_crash:.2f}, outmap: {rate_outmap:.2f}, learning: {replay_buffer.size() > minimal_size}")      
        print(f'episode: {i_episode}, return: {episode_return:.2f}')

        success_rate_list.append((i_episode, round(rate_success, 2), round(rate_timeout, 2), round(rate_crash, 2), round(rate_outmap, 2)))

        # 动态更新图表（每个episode更新一次）
        if replay_buffer.size() > minimal_size:
            plot_realtime(ax1, ax2, ax3, ax4, return_list, critic_loss_list, actor_loss_list, success_rate_list, window_size=100)

        if rate_success >= 0.90:
            continue_times += 1
            if continue_times > 100:
                early_stop = True
        else:
            continue_times = 0

        if i_episode % 50 == 0 or early_stop:
            agent.save(checkpoints_path, i_episode)
            replay_buffer.save(f"{checkpoints_path}/{i_episode}_buffer.pth")
            save_return_list(i_episode, checkpoints_path, return_list)
            save_reason_list(i_episode, checkpoints_path, reason_list)
            save_success_rate_list(i_episode, checkpoints_path, success_rate_list)

            parameter_keys = ['episode', 'total_iterated', 'actor_lr', 'critic_lr', 
                            'batch_size', 'tau', 'gamma', 'buffer_size', 'minimal_size', 'sigma', 'hidden_dim']
            parameter_values = [i_episode, total_iterated, actor_lr, critic_lr, 
                                batch_size, tau, gamma, buffer_size, minimal_size, sigma, hidden_dim]
            
            parameter_dictionary = dict(zip(parameter_keys, parameter_values))
            with open(f'{checkpoints_path}/{i_episode}_hyperparameter' + '.json', 'w') as outfile:
                json.dump(parameter_dictionary, outfile)

        if early_stop:
            break

    env.close()
