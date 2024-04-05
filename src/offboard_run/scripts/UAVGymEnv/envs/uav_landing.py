#! /usr/bin/python
## /home/ubuntu/miniconda3/envs/px4/bin/python

import gymnasium
from gymnasium import spaces
import numpy as np

import rospy
import math

# 
from geometry_msgs.msg import PoseStamped, Quaternion

from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest
from std_msgs.msg import Header

from threading import Thread
from pymavlink import mavutil

from tf.transformations import quaternion_from_euler




import time


import subprocess
import os
import pickle
import types

from gymnasium import utils, spaces
# from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gymnasium.utils import seeding

class simulationHandler():
    def __init__(self):

        self.ready = False
        self.sub_topics_ready = {
            key: False
            for key in [
                'local_pos', 'state', 'ext_state'
            ]
        }

        # 发布的话题 topic published
        self.pos = PoseStamped()
        
        # 订阅的话题 topic subscribed
        self.state = State()
        self.extended_state = ExtendedState()
        self.local_position = PoseStamped()

        # 本节点订阅的话题
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state', ExtendedState, self.extended_state_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)

        # 本节点发布的话题
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # 创建服务客户端
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        # 创建新线程，专门用于发布话题
        self.pos_thread = Thread(target=self.send_pos, args=(), name="pos_thread")
        self.pos_thread.daemon = True
        self.pos_thread.start()        

        # Target offset radius
        self.radius = 0.25

        # Gazebo 提供的服务
        # self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        # self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)        

    def setup(self):
        service_timeout = 60

        try:
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
        except rospy.ROSException:
            rospy.loginfo("simulationHandler failed to connect to services")

        rospy.loginfo("simulation handler setup.")


    def getReady(self):
        # rospy.loginfo('@ctrl_server@ getReady and takeoff')

        try:
            rospy.wait_for_service('mavros/cmd/arming', 60)
            rospy.wait_for_service('mavros/set_mode', 60)
        except rospy.ROSException:
            rospy.loginfo("simulationHandler failed to connect to services")

        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        self.reach_position(0, 0, 5, 10)

        self.ready = True


    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                # rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                #               format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.loginfo(e)

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                # rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                #               format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

    def operate(self, command):
        try:
            data = command.split('#')
            # print('@ctrl_server@ get cmd ' + str(data))
            # get cmd content
            cmd = data[0]
            margin = 1
            if 1 < len(data) < 3:
                margin = int(data[1])

            r_msg = ''

            # print('@ctrl_server@ executing cmd: ' + cmd)
            if cmd == 'reset':
                self.reach_position(0, 0, 0, 2)
                self.land()
                r_msg = self.getState()

            #   the env is killed
            elif cmd == 'takeoff':
                self.getReady()
                r_msg = self.getState()

            elif cmd == 'shutdown':
                # self.shutDown()
                # over = True
                # r_msg = 'recv shutdown'
                pass

            elif cmd == 'move':
                self.move(data[1], data[2], data[3])
                r_msg = self.getState()                
            else:
                self.moveOnce(cmd, margin)
                r_msg = self.getState()
            # print('@ctrl_server@ executing' + cmd + 'over, return msg ' + str(r_msg))
            return r_msg
        
        except BaseException as e:
            print(e)
            time.sleep(3)  

    def reset(self):
        self.reach_position(0, 0, 0, 2)
        self.land()
        return self.getState()

    def takeoff(self):
        self.getReady()
        return self.getState()

    def land(self):
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)    

    def getState(self):
        data = np.array([self.local_position.pose.position.x,
                         self.local_position.pose.position.y,
                         self.local_position.pose.position.z])
        # data = np.append(data, self.scan.ranges)
        # a string state date
        return data

    def send_pos(self):
        rate = rospy.Rate(30)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    # services 
    def step(self, action):
        margin = 1
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        old_position = [self.pos[0],
                        self.pos[1],
                        self.pos[2]]

        done_reason = ''

        cmd = ''
        if type(action) == np.numarray:
            cmd = 'move#{0}#{1}#{2}'.format(action[0], action[1], action[2])
        elif action == 0:  # xPlus
            cmd = 'moveXPlus' + '#' + str(margin)
        elif action == 1:  # xMin
            cmd = 'moveXMin' + '#' + str(margin)
        elif action == 2:  # yPlus
            cmd = 'moveYPlus' + '#' + str(margin)
        elif action == 3:  # yMin
            cmd = 'moveYMin' + '#' + str(margin)
        elif action == 4:  # up
            cmd = 'moveUp' + '#' + str(margin)
        elif action == 5:  # down
            cmd = 'moveDown' + '#' + str(margin)
        elif action == 6:  # stay
            cmd = 'stay' + '#' + str(margin)

        data = self.send_msg_get_return(cmd)
        self.pos = [data[0], data[1], data[2]]
        lidar_ranges = data[3:]
        for idx in range(0, len(lidar_ranges)):
            if lidar_ranges[idx] > 10 or lidar_ranges[idx] == np.inf:
                lidar_ranges[idx] = 10

        # print('@env@ data' + str(data))

        reward = 0
        done = False  # done check

        # finish reward
        if self.is_at_position(self.des[0], self.des[1], self.des[2],
                               self.pos[0], self.pos[1], self.pos[2],
                               self.radius):
            done = True
            done_reason = 'finish'
            reward = reward + 10
        # move reward
        reward = reward + 2 * self.cal_distence(old_position, self.pos, self.des)

        # danger reward
        for i in lidar_ranges:
            if i < 1.5:
                reward = -5
                done = True
                if done and done_reason == '':
                    done_reason = 'laser_danger'
            elif i <= 6:
                reward = reward - 1 / (i - 1)

        # fail reward
        if (self.pos[0] < -50 or
                self.pos[0] > 50 or
                np.abs(self.pos[1]) > 50 or
                self.pos[2] > 40 or
                self.pos[2] < 1):
            reward = reward - 5
            done = True
            if done and done_reason == '':
                done_reason = 'out of map'

        # trans relative position
        data[0] = data[0] - self.des[0]
        data[1] = data[1] - self.des[1]
        data[2] = data[2] - self.des[2]

        for idx in range(len(data)):
            if idx < 3:
                data[idx] = (data[idx] + 50) / 100
            else:
                if data[idx] > 10 or data[idx] == np.inf:
                    data[idx] = 10
                data[idx] = (data[idx] - 0.2) / 9.8

        state = data

        if 'nan' in str(data):
            state = np.zeros([len(data)])
            done = True
            reward = 0

        # print('@env@ observation:' + str(state))
        # print('@env@ reward:' + str(reward))
        # print('@env@ done:' + str(done))
        return state, reward, done, {'done_reason': done_reason}            

    def moveOnce(self, cmd, margin):
        if cmd == 'moveUp':
            self.moveUp(margin)
        elif cmd == 'moveDown':
            self.moveDown(margin)
        elif cmd == 'moveXPlus':
            self.moveXPlus(margin)
        elif cmd == 'moveXMin':
            self.moveXMin(margin)
        elif cmd == 'moveYPlus':
            self.moveYPlus(margin)
        elif cmd == 'moveYMin':
            self.moveYMin(margin)
        elif cmd == 'stay':
            pass            

    def moveUp(self, margin=1):
        if not self.ready:
            self.getReady()
        self.reach_position(self.local_position.pose.position.x,
                            self.local_position.pose.position.y,
                            self.local_position.pose.position.z + margin,
                            5)

    def moveDown(self, margin=1):
        if not self.ready:
            self.getReady()
        self.reach_position(self.local_position.pose.position.x,
                            self.local_position.pose.position.y,
                            self.local_position.pose.position.z - margin,
                            5)

    def moveXPlus(self, margin=1):
        if not self.ready:
            self.getReady()
        self.reach_position(self.local_position.pose.position.x + margin,
                            self.local_position.pose.position.y,
                            self.local_position.pose.position.z,
                            5)

    def moveXMin(self, margin=1):
        if not self.ready:
            self.getReady()
        self.reach_position(self.local_position.pose.position.x - margin,
                            self.local_position.pose.position.y,
                            self.local_position.pose.position.z,
                            5)

    def moveYPlus(self, margin=1):
        if not self.ready:
            self.getReady()
        self.reach_position(self.local_position.pose.position.x,
                            self.local_position.pose.position.y + margin,
                            self.local_position.pose.position.z,
                            5)

    def moveYMin(self, margin=1):
        if not self.ready:
            self.getReady()
        self.reach_position(self.local_position.pose.position.x,
                            self.local_position.pose.position.y - margin,
                            self.local_position.pose.position.z,
                            5)

    def move(self, x, y, z, timeout=5):
        t_x = self.pos.pose.position.x + float(x)
        t_y = self.pos.pose.position.y + float(y)
        t_z = self.pos.pose.position.z + float(z)
        self.reach_position(t_x, t_y, t_z, timeout)            

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # dose it reach the position in 'time' seconds?
        loop_freq = 100  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                break
            try:
                rate.sleep()
            except rospy.ROSException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset:meters"""
        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    # call service
    def set_mode(self, mode, timeout):

        """mode: PX4 mode string, timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if self.state.mode == mode:
                rospy.logerr('no need to set mode')
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

    def set_arm(self, arm, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if self.state.armed == arm:
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

    # topic callback
    def local_position_callback(self, data):
        self.local_position = data
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def state_callback(self, data):
        self.state = data
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True 

    def extended_state_callback(self, data):
        self.extended_state = data
        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True               


class UAVLandingEnv(gymnasium.Env):
    def __init__(self):

        rospy.init_node("offb_test")

        rospy.wait_for_service('/gazebo/unpause_physics', 30)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        self.simHandler = simulationHandler()

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
        self.action_space = spaces.Discrete(7)  # U, D, F, B, L, R
        self.reward_range = (-np.inf, np.inf)

        self._seed()
        self.radius = 3

        self.pos = np.array([0, 0, 0])        
        self.des = [170, 0, 15]

        rospy.loginfo("Environment is ready.")

        time.sleep(5)
    
    def step(self, action):
        margin = 1
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        old_position = [self.pos[0],
                        self.pos[1],
                        self.pos[2]]

        done_reason = ''

        cmd = ''
        if type(action) == np.numarray:
            cmd = 'move#{0}#{1}#{2}'.format(action[0], action[1], action[2])
        elif action == 0:  # xPlus
            cmd = 'moveXPlus' + '#' + str(margin)
        elif action == 1:  # xMin
            cmd = 'moveXMin' + '#' + str(margin)
        elif action == 2:  # yPlus
            cmd = 'moveYPlus' + '#' + str(margin)
        elif action == 3:  # yMin
            cmd = 'moveYMin' + '#' + str(margin)
        elif action == 4:  # up
            cmd = 'moveUp' + '#' + str(margin)
        elif action == 5:  # down
            cmd = 'moveDown' + '#' + str(margin)
        elif action == 6:  # stay
            cmd = 'stay' + '#' + str(margin)

        # data = self.send_msg_get_return(cmd)
        data = self.simHandler.operate(cmd)
        self.pos = [data[0], data[1], data[2]]
        lidar_ranges = data[3:]
        for idx in range(0, len(lidar_ranges)):
            if lidar_ranges[idx] > 10 or lidar_ranges[idx] == np.inf:
                lidar_ranges[idx] = 10

        # print('@env@ data' + str(data))

        reward = 0
        done = False  # done check

        # finish reward
        if self.is_at_position(self.des[0], self.des[1], self.des[2],
                               self.pos[0], self.pos[1], self.pos[2],
                               self.radius):
            done = True
            done_reason = 'finish'
            reward = reward + 10
        # move reward
        reward = reward + 2 * self.cal_distence(old_position, self.pos, self.des)

        # danger reward
        for i in lidar_ranges:
            if i < 1.5:
                reward = -5
                done = True
                if done and done_reason == '':
                    done_reason = 'laser_danger'
            elif i <= 6:
                reward = reward - 1 / (i - 1)

        # fail reward
        if (self.pos[0] < -50 or
                self.pos[0] > 50 or
                np.abs(self.pos[1]) > 50 or
                self.pos[2] > 40 or
                self.pos[2] < 1):
            reward = reward - 5
            done = True
            if done and done_reason == '':
                done_reason = 'out of map'

        # trans relative position
        data[0] = data[0] - self.des[0]
        data[1] = data[1] - self.des[1]
        data[2] = data[2] - self.des[2]

        for idx in range(len(data)):
            if idx < 3:
                data[idx] = (data[idx] + 50) / 100
            else:
                if data[idx] > 10 or data[idx] == np.inf:
                    data[idx] = 10
                data[idx] = (data[idx] - 0.2) / 9.8

        state = data

        if 'nan' in str(data):
            state = np.zeros([len(data)])
            done = True
            reward = 0

        # print('@env@ observation:' + str(state))
        # print('@env@ reward:' + str(reward))
        # print('@env@ done:' + str(done))
        return state, reward, done, {'done_reason': done_reason}
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except rospy.ServiceException as e:
            print ("@env@ /gazebo/reset_world service call failed")

        data = self.simHandler.takeoff()

        data[0] = data[0] - self.des[0]
        data[1] = data[1] - self.des[1]
        data[2] = data[2] - self.des[2]

        self.pos = np.array([0, 0, 0])

        for idx in range(len(data)):
            if idx < 3:
                data[idx] = (data[idx] + 50) / 100
            else:
                if data[idx] > 10 or data[idx] == np.inf:
                    data[idx] = 10
                data[idx] = (data[idx] - 0.2) / 9.8

        state = data

        if 'nan' in str(state):
            state = np.zeros([len(state)])

        rospy.loginfo("Env is reset.")

        return state

    def set_des(self, destination):
        self.des = destination

    def is_at_position(self, tx, ty, tz, x, y, z, offset):
        desired = np.array((tx, ty, tz))
        pos = np.array((x, y, z))
        return np.linalg.norm(desired - pos) < offset

    def cal_distence(self, old_position, new_position, destination):
        old_distance = np.sqrt(
            np.square(destination[0] - old_position[0]) + np.square(destination[1] - old_position[1]) + np.square(
                destination[2] - old_position[2]))

        new_distance = np.sqrt(
            np.square(destination[0] - new_position[0]) + np.square(destination[1] - new_position[1]) + np.square(
                destination[2] - new_position[2]))

        return old_distance - new_distance

    def close(self):
        pass

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]    