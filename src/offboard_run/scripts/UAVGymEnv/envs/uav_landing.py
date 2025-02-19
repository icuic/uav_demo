#! /usr/bin/python
## /home/ubuntu/miniconda3/envs/px4/bin/python

import gymnasium
from gymnasium import spaces
import numpy as np

import rospy
import math

#
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, Twist

from mavros_msgs.msg import State, ExtendedState, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest
from std_msgs.msg import Header

from gazebo_msgs.msg import ModelStates, ModelState

from threading import Thread, Event as ThreadingEvent

from pymavlink import mavutil

from tf.transformations import quaternion_from_euler




import time
import random

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

# 起飞点
g_start_point_x = 1
g_start_point_y = 1
g_start_point_z = 5

# 目的地
g_destination_x = 3
g_destination_y = 3
g_destination_z = g_start_point_z

# 地理围栏
g_max_x = 5
g_max_y = 5
g_max_z = 100
g_min_z = 4

class simulationHandler():

    def __init__(self):
        self.ready = False
        self.sub_topics_ready = {key: False for key in ['local_pos', 'state', 'ext_state']}

        # 发布的话题 topic published
        self.pos = PoseStamped()        
        self.vel = TwistStamped()
        self.raw = PositionTarget()

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
        self.vel_setpoint_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.raw_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # 创建服务客户端
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        # 创建新线程，专门用于发布话题
        # self.pos_thread = Thread(target=self.send_pos, args=(), name="pos_thread")
        # self.pos_thread = Thread(target=self.send_vel, args=(), name="vel_thread")
        self.pos_thread = Thread(target=self.send_raw, args=(), name="raw_thread")
        self.pos_thread.daemon = True
        self.pos_thread.start()

        # Target offset radius
        self.radius = 0.2

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
        try:
            rospy.wait_for_service('mavros/cmd/arming', 60)
            rospy.wait_for_service('mavros/set_mode', 60)
        except rospy.ROSException:
            rospy.loginfo("simulationHandler failed to connect to services")

        self.wait_for_topics(60)
        # self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        #                            10, -1)

        self.state
        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD", 5)

        if not self.state.armed:
            self.set_arm(True, 5)

        self.reach_position(g_start_point_x, g_start_point_y, g_start_point_z, 10)
        print(f"reached start point: {g_start_point_x}, {g_start_point_y}, {g_start_point_z}")

        self.ready = True
        # time.sleep(5)

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
            margin = 0.5
            if 1 < len(data) < 3:
                margin = float(data[1])

            r_msg = ''

            # print('@ctrl_server@ executing cmd: ' + cmd)
            if cmd == 'reset':
                self.reach_position(g_des_x, g_des_y, g_des_z, 5)
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

            elif cmd == 'setVelocity':
                # self.setVelocity(data[1], data[2])
                self.setRaw(4039, g_start_point_x, g_start_point_y, g_start_point_z, data[1], data[2], data[3])
                r_msg = self.getState()

            elif cmd == 'move':
                self.move(data[1], data[2], data[3])
                r_msg = self.getState()
            else:
                self.moveOnce(cmd, margin)
                r_msg = self.getState()
            # print('@ctrl_server@ executing' + cmd + 'over, return msg ' + str(r_msg))
            # print(f'r_msg: ', r_msg)
            return r_msg

        except BaseException as e:
            print(e)
            time.sleep(3)

    def reset(self):
        self.reach_position(1, 1, g_start_point_z, 20)
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
        self.set_pos()

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_vel(self):
        rate = rospy.Rate(30)
        self.vel.header = Header()
        self.vel.header.frame_id = "manual_vel"

        while not rospy.is_shutdown():
            self.vel.header.stamp = rospy.Time.now()
            self.vel_setpoint_pub.publish(self.vel)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_raw(self):
        rate = rospy.Rate(30)
        self.raw.header = Header()
        self.raw.header.frame_id = "manual_raw"

        while not rospy.is_shutdown():
            self.raw.header.stamp = rospy.Time.now()
            self.raw_setpoint_pub.publish(self.raw)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass            

    def moveOnce(self, cmd, margin):
        self.local_position.pose.position.z = g_start_point_z
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

        # set a position setpoint
        # self.vel.twist.linear.x = 0
        # self.vel.twist.linear.y = 0
        # self.vel.twist.linear.z = 1

        # For demo purposes we will lock yaw/heading to north.
        # yaw_degrees = 0  # North
        # yaw = math.radians(yaw_degrees)
        # quaternion = quaternion_from_euler(0, 0, yaw)
        # self.pos.pose.orientation = Quaternion(*quaternion)

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

    def set_pos(self):
        self.pos.pose.position.x = g_start_point_x
        self.pos.pose.position.y = g_start_point_y
        self.pos.pose.position.z = g_start_point_z

    def resetVelocity(self, vx, vy):
        self.vel.twist.linear.x = float(0)
        self.vel.twist.linear.y = float(0)
        self.vel.twist.linear.z = float(0.2)

    def setVelocity(self, vx, vy):
        self.vel.twist.linear.x = float(vx)
        self.vel.twist.linear.y = float(vy)
    
        vz = 0
        if self.local_position.pose.position.z < 4:
            vz = 0.3
        elif self.local_position.pose.position.z > 5:
            vz = -0.2
    
        self.vel.twist.linear.z = vz

    def setRaw(self, mask, px, py, pz, vx, vy, vz):
        self.raw.coordinate_frame = 1
        self.raw.type_mask = mask   # 0: px/py/pz take effect; 3: pz/vx/vy take effect, 4039: vx/vy/vz take effect

        self.raw.position.x = px
        self.raw.position.y = py
        self.raw.position.z = pz

        self.raw.velocity.x = float(vx)
        self.raw.velocity.y = float(vy)
        self.raw.velocity.z = float(vz)



class UAVLandingEnv(gymnasium.Env):
    def __init__(self):

        rospy.init_node("offb_test")

        rospy.wait_for_service('/gazebo/unpause_physics', 30)
        rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=30)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        # rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # 移动 landing_area 模型
        self.landing_area_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
        self.landing_area_msg = ModelState()    
        self.stop_event = ThreadingEvent()

        self.simHandler = simulationHandler()

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
        # self.action_space = spaces.Discrete(4)  # U, D, F, B, L, R
        self.action_space = spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)
        self.reward_range = (-np.inf, np.inf)

        self._seed()

        self.radius = 0.1
        self.position = np.array([g_start_point_x, g_start_point_y, g_start_point_z])
        self.des = [3, 3, 1]
        self.cnt = 0

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        # self.first_time_after_reset = True
        
        # self.last_position = np.zeros(3, dtype=float) # poistion in last step
        # self.last_speed = np.zeros(3, dtype=float)
        # self.last_shaping = 0


        rospy.loginfo("Environment is ready.")

        time.sleep(5)

    def model_states_callback(self, msg):
        try:
            # 查找 landing_area 模型的索引
            model_index = msg.name.index("landing_area")
            
            # 提取 landing_area 的位置信息
            position = msg.pose[model_index].position
            
            # 将位置信息添加到列表中
            self.des.clear()
            self.des.extend((position.x, position.y, 5))
            
            # 打印位置信息
            # rospy.loginfo(f"Position of landing_area: x={landing_area_position.x}, y={landing_area_position.y}, z={landing_area_position.z}")

        except ValueError:
            # 如果找不到 landing_area 模型
            rospy.logwarn("Model 'landing_area' not found in the list of models.")        


    def step(self, action):        
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        done_reason = ''
        cmd = ''
        margin = 0.3

        ttt = []
        ttt.append(self.des[0] - self.position[0])
        ttt.append(self.des[1] - self.position[1])
        ttt.append(1 - self.position[2])

        # 启发式更新Z轴速度
        vz, _height, _dist = self.calculate_velocity(ttt)

        if type(action) == np.ndarray:
            if action.size == 3:
                cmd = f'move#{action[0]}#{action[1]}#{action[2]}'
            elif action.size == 2:
                cmd = f'setVelocity#{action[0]}#{action[1]}#{vz}'
        elif action == 0:  # xPlus
            cmd = 'moveXPlus' + '#' + str(margin)
        elif action == 1:  # xMin
            cmd = 'moveXMin' + '#' + str(margin)
        elif action == 2:  # yPlus
            cmd = 'moveYPlus' + '#' + str(margin)
        elif action == 3:  # yMin
            cmd = 'moveYMin' + '#' + str(margin)
        # elif action == 4:  # up
        #     cmd = 'moveUp' + '#' + str(margin)
        # elif action == 5:  # down
        #     cmd = 'moveDown' + '#' + str(margin)
        # elif action == 4:  # stay
        #     cmd = 'stay' + '#' + str(margin)


        
        reward = 0
        reward = self.cal_reward(ttt[:2], action[:2])   
        # reward -= 0.1

        old_position = np.array([self.position[0], self.position[1], self.position[2]])

        data = self.simHandler.operate(cmd)
        time.sleep(0.05)
        data = self.simHandler.getState()
        self.position = [round(data[i], 2) for i in range(3)]

        print("self.position: ", ' '.join(f"{pos:.2f}" for pos in self.position))
        print("landing area: ", " ".join(f"{num:.2f}" for num in self.des[:3]))
        print(f"height: {_height:.2f}, dist: {_dist:.2f}")
        print("action: ", ' '.join(f"{v:.2f}" for v in (action[0], action[1], vz)))
        print("---")
        
        done = False

        distance = self.cal_distence(self.position, self.des)
        # if distance < self.radius and abs(_height) < 0.5:
        if distance < 0.5 and abs(_height) < 0.5:
            done = True
            done_reason = 'finish'
            reward += 10        

# ---        
        # fail reward
        if (np.abs(self.position[0]) > g_max_x+1 or
                np.abs(self.position[1]) > g_max_y+1 or
                self.position[2] > g_max_z):
            reward -= 50
            done = True
            if done and done_reason == '':
                done_reason = 'out of map'

        self.cnt += 1
        if self.cnt > 600:
            done = True
            done_reason = 'timeout'

        # 如果降落任务完成或超时，就杀掉子线程，停止移动降落平台
        if done:
            self.stop_event.set()
            self.landing_area_thread.join()


        
        print(f"done: {done}-({done_reason}), reward: {reward:.2f}, ")


        # trans relative position
        data[0] = self.des[0] - data[0] 
        data[1] = self.des[1] - data[1] 
        data[2] = self.des[2] - data[2]

        # for idx in range(len(data)):
        #     if idx < 3:
        #         data[idx] = (data[idx] + 5) / 10
        #     else:
        #         if data[idx] > 10 or data[idx] == np.inf:
        #             data[idx] = 10
        #         data[idx] = (data[idx] - 0.2) / 9.8

        state = data

        if 'nan' in str(data):
            state = np.zeros([len(data)])
            done = True
            reward = 0

        # print('@env@ observation:' + str(state))
        # print('@env@ reward:' + str(reward))
        # print('@env@ done:' + str(done))
        # print("---------------- end ------------------")

        return state, reward, done, {'done_reason': done_reason}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # 随机生成起飞点和目的地
        global g_start_point_x, g_start_point_y, g_start_point_z, g_destination_x, g_destination_y, g_destination_z
        
        g_start_point_x = round(random.uniform(-1*g_max_x, g_max_x), 1)
        g_start_point_y = round(random.uniform(-1*g_max_y, g_max_y), 1)
        g_start_point_z = g_start_point_z
        
        # g_destination_x = round(random.uniform(-1*g_max_x, g_max_x), 1)
        # g_destination_y = round(random.uniform(-1*g_max_y, g_max_y), 1)
        # g_destination_z = g_start_point_z

        # g_start_point_x = 2
        # g_start_point_y = 3
        # g_start_point_z = g_start_point_z
        
        g_destination_x = 3
        g_destination_y = 3
        g_destination_z = 1

        # g_start_point_x, g_start_point_y, g_start_point_z = np.random.randint([[-1*g_max_x, -1*g_max_y, 10]], [[g_max_x, g_max_y, 10+1]], size=3).tolist()
        # g_destination_x, g_destination_y, g_destination_z = np.random.randint([[-1*g_max_x, -1*g_max_y, 10]], [[g_max_x, g_max_y, 10+1]], size=3).tolist()
        
        self.position = [g_start_point_x, g_start_point_y, g_start_point_z]
        self.des = [g_destination_x, g_destination_y, g_destination_z]

        print("start point: ", ' '.join(f"{pos}" for pos in self.position))
        print("destination: ", ' '.join(f"{pos}" for pos in self.des))

        # self.position = np.array([g_start_point_x, g_start_point_y, g_start_point_z])
        # self.simHandler.set_pos()
        # self.simHandler.resetVelocity(0, 0)
        self.simHandler.setRaw(0, g_start_point_x, g_start_point_y, g_start_point_z, 0, 0, 0)

        # 移动降落平台至目的地
        # 改变模型pose
    
        # 创建新线程，专门用于发布话题
        self.landing_area_thread = Thread(target=self.move_landing_area, args=(), name="move_landing_area_thread")
        self.landing_area_thread.daemon = True
        self.landing_area_thread.start()     
        self.direction = -1  # 运动方向，1 表示从 (0, 0) 到 (3, 3)，-1 表示从 (3, 3) 到 (0, 0)
        self.stop_event.clear()

        # rospy.wait_for_service('/gazebo/reset_world')
        # try:
        #     self.reset_proxy()
        # except rospy.ServiceException as e:
        #     print ("@env@ /gazebo/reset_world service call failed")

        # self.unpause()

        data = self.simHandler.takeoff()

        data[0] = self.des[0] - data[0] 
        data[1] = self.des[1] - data[1] 
        # data[2] = self.des[2] - data[2]
        # 为方便仿真，假设降落平台的高度为1米
        data[2] = self.des[2] - data[2]      

        state = data

        # if 'nan' in str(state):
        #     state = np.zeros([len(state)])

        self.cnt = 0
        # self.first_time_after_reset = True
        rospy.loginfo("Env is reset.")

        return np.array(state, dtype=np.float32), {'distance':abs(g_start_point_x-g_destination_x)+abs(g_start_point_y-g_destination_y), 'dest':(g_destination_x, g_destination_y)}

    def move_landing_area(self, options=None):
        self.landing_area_msg.model_name = 'landing_area'
        frq = 30
        rate = rospy.Rate(frq)

        print("rate.to_sec()=", frq)

        speed = 0.2  # 运动速度
        direction = -1
        self.landing_area_msg.pose.position.x = 3
        self.landing_area_msg.pose.position.y = 3
        self.landing_area_msg.pose.position.z = 0

        if options == None:
            while not self.stop_event.is_set():
                if direction == 1:  # 从 (0, 0) 到 (3, 3) 运动
                    if self.landing_area_msg.pose.position.x < 3:
                        self.landing_area_msg.pose.position.x += speed / frq
                        self.landing_area_msg.pose.position.y += speed / frq
                    else:
                        direction = -1  # 到达 (3, 3)，改变运动方向
                else:  # 从 (3, 3) 到 (0, 0) 运动
                    if self.landing_area_msg.pose.position.x > 0:
                        self.landing_area_msg.pose.position.x -= speed / frq
                        self.landing_area_msg.pose.position.y -= speed / frq
                    else:
                        direction = 1  # 到达 (0, 0)，改变运动方向

                self.landing_area_pub.publish(self.landing_area_msg)
                rate.sleep()

    def set_des(self, destination):
        self.des = destination


    def cmp_distence(self, old_position, new_position, destination):
        old_distance = np.sqrt(
            np.square(destination[0] - old_position[0]) + np.square(destination[1] - old_position[1]) + np.square(
                destination[2] - old_position[2]))

        new_distance = np.sqrt(
            np.square(destination[0] - new_position[0]) + np.square(destination[1] - new_position[1]) + np.square(
                destination[2] - new_position[2]))

        return old_distance - new_distance

    # 计算当前位置与目标位置的距离
    def cal_distence(self, new_position, destination):
        new_distance = np.sqrt(
            # np.square(destination[0] - new_position[0]) + np.square(destination[1] - new_position[1]) + np.square(destination[2] - new_position[2]))
            np.square(destination[0] - new_position[0]) + np.square(destination[1] - new_position[1]))

        return new_distance

    # 计算奖励
    def cal_reward(self, v1, v2):
        # 将输入的列表转换为 numpy 数组
        v1 = np.array(v1)
        v2 = np.array(v2)
        # 计算向量 v1 的模
        magnitude_v1 = np.linalg.norm(v1)
        # 计算向量 v2 的模
        magnitude_v2 = np.linalg.norm(v2)
        # 计算向量 v1 和 v2 的点积
        dot_product = np.dot(v1, v2)
        # 计算夹角的余弦值
        if magnitude_v1 * magnitude_v2 != 0:
            cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
        else:
            cos_theta = 0
        
        # reward_ = cos_theta * (1 / (1 + abs(magnitude_v1 - magnitude_v2)))
        reward_ = cos_theta * (1 + magnitude_v2)
        return reward_

    def close(self):
        # self.simHandler.reset()
        self.simHandler.land()
        pass

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def calculate_velocity(self, state):
        x_distance = state[0]
        y_distance = state[1]
        height = state[2]
        ret = 0
        # 计算 xy 平面的距离
        dist = math.sqrt(x_distance ** 2 + y_distance ** 2)
        if 0 < dist < 0.8:
            if 0 <= abs(height) <= 0.1:
                ret = 0
            elif 0.1 < abs(height) <= 3.5:
                ret = 0.5 * height
            elif abs(height) > 3.5:
                ret = 0.5 * height
        elif 0.8 <= dist <= 4:
            if 0 <= abs(height) <= 0.1:
                ret = 0.5 * height
            elif 0.1 < abs(height) <= 3.5:
                ret = 0.5 * height
            elif abs(height) > 3.5:
                ret = 0.5 * height
        elif dist > 4:
            # if 0 <= abs(height) <= 0.1:
            #     ret = -1
            # elif 0.1 < abs(height) <= 3.5:
            #     ret = -1
            # elif abs(height) > 3.5:
                ret = 0

        return ret, height, dist