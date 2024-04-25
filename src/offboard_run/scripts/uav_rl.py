#! /usr/bin/python
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

print(sys.path)
import UAVGymEnv

import time
import numpy as np

env = gymnasium.make('UAVGymEnv/UAVLandingEnv-v0')
observation = env.reset()


if __name__ == "__main__":

    # for _ in range(10):
    #     action = env.action_space.sample()
    #     observation, reward, terminated, truncated, info = env.step(action)

    #     if terminated or truncated:
    #         observation, info = env.reset()

    # env.close()

    action = 0
    strAction = ["x+", "x-", "y+", "y-", "z+", "z-", "keep"]

    while(1):
        print("=========================================")
        print("current action: " + strAction[action])
        
        state, reward, done, info = env.step(action)
        print("State:", state)
        print("Reward:", reward)
        print("Done:", done)
        # print("Info:", info)  
        
        

        time.sleep(10)
        pass

