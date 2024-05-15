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

if __name__ == "__main__":

    env_name = 'UAVGymEnv/UAVLandingEnv-v0'
    env = gymnasium.make(env_name)
    state = env.reset()
    time.sleep(10)

    strAction = ["x+", "x-", "y+", "y-"]

    for i_episode in range(10):
        action = 0
        # next_state, reward, done, _ = env.step(action)
        print(f"{i_episode:-^50}")

        # for i in range(10):
        #     env.step(0)
        #     time.sleep(0.5)

        # for i in range(10):
        #     env.step(2)
        #     time.sleep(0.5)

        time.sleep(1)

    env.close()

