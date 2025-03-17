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

from threading import Thread, Event as ThreadingEvent, Lock

from pymavlink import mavutil

from tf.transformations import quaternion_from_euler
from uav_trajectory_recorder import DroneAndPlatformTrajectoryRecorder



import time
from datetime import datetime
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
g_start_point_z = 8

# 目的地
g_destination_x = 3
g_destination_y = 3
g_destination_z = 0.7

# 地理围栏
g_max_x = 5
g_max_y = 5
g_max_z = 10

# 定义成功降落
g_landing_tolerance = 1
g_crash_shreshold = 0.5

# train or eval
g_eval = False
g_uav_true_value = True

last_time = 0

def normalize_state(actual_diff):
    """将实际坐标差值转换为归一化值
    Args:
        actual_diff: numpy数组，形状(3,) 表示 [dx, dy, dz]
    Returns:
        归一化后的numpy数组，形状(3,)
    """
    return np.array([
        actual_diff[0] / (2 * g_max_x),
        actual_diff[1] / (2 * g_max_y),
        actual_diff[2] / g_max_z
    ], dtype=np.float32)

def denormalize_state(normalized_diff):
    """将归一化值转换回实际坐标差值
    Args:
        normalized_diff: numpy数组，形状(3,) 表示归一化后的差值
    Returns:
        实际坐标差值numpy数组，形状(3,)
    """
    return np.array([
        normalized_diff[0] * 2 * g_max_x,
        normalized_diff[1] * 2 * g_max_y,
        normalized_diff[2] * g_max_z
    ], dtype=np.float32)

def normalize_xy(value):
    """将单个X/Y实际值转换为归一化值
    Args:
        value: 标量或numpy数组，X/Y方向的实际差值
    Returns:
        归一化后的值（范围[-1,1]）
    """
    return np.array(value / (2 * g_max_x), dtype=np.float32)

def denormalize_xy(norm_value):
    """将归一化的X/Y值转换回实际值
    Args:
        norm_value: 标量或numpy数组
    Returns:
        实际差值
    """
    return np.array(norm_value * 2 * g_max_x, dtype=np.float32)

def normalize_z(value):
    """将Z实际值转换为归一化值
    Args:
        value: 标量或numpy数组，Z方向的实际差值
    Returns:
        归一化后的值（范围[-1,1]）
    """
    return np.array(value / g_max_z, dtype=np.float32)

def denormalize_z(norm_value):
    """将归一化的Z值转换回实际值
    Args:
        norm_value: 标量或numpy数组
    Returns:
        实际差值
    """
    return np.array(norm_value * g_max_z, dtype=np.float32)

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
            # print("333", *data)

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
                # data[0]: 'setVelocity'
                # data[1]: vx
                # data[2]: vy
                # data[3]: vz

                # print("444", data[4])
                # mask = 0    # 4088: px,py,pz only, 4067: pz,vx,vy only, 4039: vx,vy,vz only
                # if data[3] == "True":
                #     mask = 4067
                #     data[4] = 5
                # else:
                #     mask = 4039

                # print(type(data[3]))
                # print(data[3])
                # print("555", data[4])

                # print(f"mask: {mask}, pos: {data[3]}, data[4]: {data[4]}")
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
    def __init__(self, motion_type='static', speed=0.2, model=None):
        # 新增运动参数
        self.motion_type = motion_type
        self.speed = speed
        self.direction = -1  # 直线运动方向
        self.angle = 0      # 圆周运动角度
        self.landing_area_radius = 3     # 圆周运动半径
        self.model = model
        
        # 统计相关变量
        self.episode_count = 0
        self.success_count = 0
        self.total_error = 0.0
        self.total_time = 0.0
        self.landing_results = []

        self.lock = Lock()

        rospy.init_node("offb_test")

        rospy.wait_for_service('/gazebo/unpause_physics', 30)
        rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=30)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
        # 移动 landing_area 模型
        self.landing_area_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
        self.landing_area_msg = ModelState()    
        self.stop_event = ThreadingEvent()

        self.simHandler = simulationHandler()

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
        # self.action_space = spaces.Discrete(4)  # U, D, F, B, L, R
        self.action_space = spaces.Box(low=np.array([-1, -1, -1]), high=np.array([1, 1, 1]), dtype=np.float32)
        self.reward_range = (-np.inf, np.inf)

        self._seed()

        self.radius = 0.1
        # 无人机当前位置
        if not g_uav_true_value:
            self.position = np.array([g_start_point_x, g_start_point_y, g_start_point_z])
        else:
            self.position = [g_start_point_x, g_start_point_y, g_start_point_z]

        # 降落平台当前位置
        self.des = [g_destination_x, g_destination_y, g_destination_z]
        self.cnt = 0

        if g_uav_true_value:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        if g_eval:
            self.recorder = DroneAndPlatformTrajectoryRecorder()

        rospy.loginfo("Environment is ready.")

        time.sleep(5)

    # 更新降落平台位置
    def model_states_callback(self, msg):
        try:
            # 查找 landing_area 模型的索引
            model_index = msg.name.index("landing_area")
            
            # 提取 landing_area 的位置信息
            position = msg.pose[model_index].position
            
            # 将位置信息添加到列表中
            self.des.clear()
            self.des.extend((position.x, position.y, g_destination_z))  # 假装降落平台的高度是2米
            
            if g_uav_true_value:
                # 查找 iris 模型的索引
                iris_index = msg.name.index("iris")
                
                # 提取 iris 的位置信息
                position = msg.pose[iris_index].position
                
                # 将位置信息添加到列表中
                self.position.clear()
                self.position.extend((position.x, position.y, position.z)) 


            # 打印位置信息
            # rospy.loginfo(f"Position of landing_area: x={landing_area_position.x}, y={landing_area_position.y}, z={landing_area_position.z}")

        except ValueError:
            # 如果找不到 landing_area 模型
            rospy.logwarn("Model 'landing_area' not found in the list of models.")        


    def step(self, action):   
        global last_time
        elapsed_time = time.time() - last_time
        last_time = time.time()
        # print(f"time exhaust: {elapsed_time:.3f}")

        # 记录执行动作之前的状态
        # old_position = np.array([self.position[0], self.position[1], self.position[2]])

        cmd = ''
        margin = 0.3
        done_reason = ''

        reward = 0
        done = False


        # 计算奖励
        _state = np.subtract(normalize_state(self.des), normalize_state(self.position))
        _state_to_print = np.subtract(self.des, self.position)

        reward = self.cal_reward(action[:3], _state[:3])

        # 打印

        print("a_t: ", ', '.join(f"{a:.2f}" for a in action))       # a_t
        print("s_t: ", ', '.join(f"{pos:.2f}" for pos in _state_to_print))      # s_t

        print("uav: ", ', '.join(f"{pos:.2f}" for pos in self.position))    # 无人机当前位置
        print("tgt: ", ", ".join(f"{num:.2f}" for num in self.des))         # 目标位置

        height = abs(self.des[2] - self.position[2])
        distance = self.cal_distence(self.position, self.des)
        print(f"hight_t: {height:.2f}, dist_t: {distance:.2f}")


        # 恢复仿真     
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
            # print("unpause physics in step")
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed in step")


        # 执行动作
        if type(action) == np.ndarray  and action.size == 3:
            cmd = f'setVelocity#{action[0]}#{action[1]}#{action[2]}'

        data = self.simHandler.operate(cmd)
        time.sleep(0.05)
        data = self.simHandler.getState()

        if not g_uav_true_value:
            self.position = [round(data[i], 2) for i in range(3)]
        else:
            self.position = [round(self.position[i], 2) for i in range(3)]


        # 计算高度和距离
        print("---")
        print("uav_t+1: ", ', '.join(f"{pos:.2f}" for pos in self.position))    # 无人机新的位置
        print("tgt_t+1: ", ", ".join(f"{num:.2f}" for num in self.des))         # 目标新的位置
        
        height = abs(self.des[2] - self.position[2])
        distance = self.cal_distence(self.position, self.des)
        print(f"hight_t+1: {height:.2f}, dist_t+1: {distance:.2f}")
        print("---")

        # 判断是否降落成功
        if distance < g_landing_tolerance and height < g_landing_tolerance:
            done = True
            done_reason = 'finish'
            reward += 100
   

        # 超出范围
        if (np.abs(self.position[0]) > g_max_x+1 or
                np.abs(self.position[1]) > g_max_y+1 or
                self.position[2] > g_max_z):
            reward -= 150
            done = True
            if done and done_reason == '':
                done_reason = 'out of map'

        # 时间效率惩罚
        # reward -= 0.2  # 每步微小惩罚鼓励快速决策

        # 超时
        self.cnt += 1
        if self.cnt > 200:
            done = True
            done_reason = 'timeout'
            reward -= 100

        # 过低
        if self.position[2] < g_crash_shreshold:
            done = True
            done_reason = 'crash'
            reward -= 150

        # 如果降落任务完成或超时，就杀掉子线程，停止移动降落平台
        if done:
            # 记录降落结果
            self.episode_count += 1         

            if g_eval:
                if done_reason == 'finish':
                    self.success_count += 1
                    x_error, y_error = self.des[0] - self.position[0], self.des[1] - self.position[1]
                    error = np.linalg.norm([x_error, y_error])  # 平面误差    
                    self.total_error += error
                    episode_time = self.recorder.drone_current_trajectory[-1][0] - self.recorder.drone_current_trajectory[0][0]
                    self.total_time += episode_time
                    self.landing_results.append({
                        "success": True,
                        "landing_point": [self.position[0], self.position[1]],
                        "platform_position": [self.des[0], self.des[1]],
                        "time": episode_time,
                        # "trajectory": self.recorder.drone_current_trajectory
                    })
                else:
                    self.landing_results.append({
                        "success": False,
                        "landing_point": [self.position[0], self.position[1]],
                        "platform_position": [self.des[0], self.des[1]],
                        # "time": episode_time,
                        # "trajectory": self.recorder.drone_current_trajectory
                    })

            self.stop_event.set()
            self.landing_area_thread.join()

            if g_eval:
                self.recorder.stop_new_trajectory()

        # 暂停仿真
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
            # print("pause physics in step")
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed in step")

        print(f"done: {done}-({done_reason}), reward: {reward:.2f}")


        # 返回下一状态
        state = np.array([
            (self.des[0] - self.position[0]) / (2 * g_max_x),   # X轴归一化到[-1,1]
            (self.des[1] - self.position[1]) / (2 * g_max_y),   # Y轴归一化到[-1,1]
            (self.des[2] - self.position[2]) / (g_max_z)        # Z轴归一化到[-1,1]（因为g_max_z=10）
        ], dtype=np.float32)

        return state, reward, done, {'done_reason': done_reason}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # 随机生成起飞点和目的地
        global g_start_point_x, g_start_point_y, g_start_point_z, g_destination_x, g_destination_y, g_destination_z
        
        g_start_point_x = round(random.uniform(-1*g_max_x, g_max_x), 1)
        g_start_point_y = round(random.uniform(-1*g_max_y, g_max_y), 1)
        g_start_point_z = g_start_point_z
        
        self.position = [g_start_point_x, g_start_point_y, g_start_point_z]
       

        print("start point: ", ' '.join(f"{pos:.2f}" for pos in self.position))
        # print("destination: ", ' '.join(f"{pos:.2f}" for pos in self.des))

        self.simHandler.setRaw(0, g_start_point_x, g_start_point_y, g_start_point_z, 0, 0, 0)

        # 移动降落平台至目的地
        # 改变模型pose
    
        # 创建新线程，专门用于发布话题
        # 初始化运动参数
        with self.lock:
            x, y = 0.0, 0.0
            if self.motion_type == 'static':
                # 随机生成[-5,5]范围内的初始位置
                x = round(random.uniform(-5, 5), 2)
                y = round(random.uniform(-5, 5), 2)
            elif self.motion_type == 'linear':
                # 初始化在起点(4,4)
                x, y = 4.0, 4.0
                self.direction = -1
            elif self.motion_type == 'circular':
                # 圆周运动初始角度
                self.angle = 0
                self.landing_area_radius = 4  # 半径设为4米

            g_destination_x , g_destination_y = x, y
            self.des = [g_destination_x, g_destination_y, g_destination_z]

            self.landing_area_thread = Thread(target=self.move_landing_area, args=(x, y), name="move_landing_area_thread")
            self.landing_area_thread.daemon = True
            self.landing_area_thread.start()     
            self.direction = -1  # 运动方向，1 表示从 (0, 0) 到 (3, 3)，-1 表示从 (3, 3) 到 (0, 0)
            self.stop_event.clear()

        # rospy.wait_for_service('/gazebo/reset_world')
        # try:
        #     self.reset_proxy()
        # except rospy.ServiceException as e:
        #     print ("@env@ /gazebo/reset_world service call failed")


        # 恢复仿真     
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
            # print("unpause physics in reset")
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed on reset")
        
        self.simHandler.takeoff()

        # 返回下一状态
        state = np.array([
            (self.des[0] - self.position[0]) / (2 * g_max_x),   # X轴归一化到[-1,1]
            (self.des[1] - self.position[1]) / (2 * g_max_y),   # Y轴归一化到[-1,1]
            (self.des[2] - self.position[2]) / (g_max_z)        # Z轴归一化到[-1,1]（因为g_max_z=10）
        ], dtype=np.float32)


        # 重置计数器
        self.cnt = 0

        if g_eval:
            self.recorder.start_new_trajectory()


        # 暂停仿真     
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
            # print("pause physics in reset")
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed on reset")


        rospy.loginfo("Env is reset.")

        return np.array(state, dtype=np.float32), {'distance':abs(g_start_point_x-g_destination_x)+abs(g_start_point_y-g_destination_y), 'dest':(g_destination_x, g_destination_y)}


    def move_landing_area(self, x, y):
        self.landing_area_msg.model_name = 'landing_area'
        frq = 30
        rate = rospy.Rate(frq)
        
        self.stop_event.clear()

        while not self.stop_event.is_set():
            with self.lock:
                if self.motion_type == 'static':
                    pass  # 保持静止
                
                elif self.motion_type == 'linear':
                    # 在(4,4)和(-4,-4)之间往返
                    if self.direction == 1:  # 向(4,4)移动
                        if x < 4.0:
                            x += self.speed / frq
                            y += self.speed / frq
                        else:
                            self.direction = -1
                    else:  # 向(-4,-4)移动
                        if x > -4.0:
                            x -= self.speed / frq
                            y -= self.speed / frq
                        else:
                            self.direction = 1
                
                elif self.motion_type == 'circular':
                    # 以(0,0)为圆心做圆周运动
                    self.angle += self.speed / frq
                    x = self.landing_area_radius * math.cos(self.angle)
                    y = self.landing_area_radius * math.sin(self.angle)
                
                elif self.motion_type == 'random':
                    # 随机运动（保持原有逻辑）
                    x += random.uniform(-self.speed/frq, self.speed/frq)
                    y += random.uniform(-self.speed/frq, self.speed/frq)
                    x = np.clip(x, -5.0, 5.0)
                    y = np.clip(y, -5.0, 5.0)

                self.landing_area_msg.pose.position.x = x
                self.landing_area_msg.pose.position.y = y
                self.landing_area_msg.pose.position.z = 0.0
                self.landing_area_pub.publish(self.landing_area_msg)
                rate.sleep()


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
    def cal_reward(self, v_action, v_distance, dt=0.05, alpha=0.8, beta=10.0, w_angle=0.7, w_speed=0.3):
        """
        v_action: 三维速度向量 [vx, vy, vz]
        v_distance: 三维相对位置 [dx, dy, dz]

        dt : float - 时间步长(默认0.05s)
        alpha : float - 动态速度比例系数(默认0.5)
        beta : float - 速度奖励衰减系数(默认10.0)
        w_angle : float - 方向奖励权重(默认0.7)
        w_speed : float - 速度奖励权重(默认0.3)        
        """

        # 计算物理约束范围
        act_max = np.sqrt(3)                   # 速度模长最大值：√3 ≈ 1.732
        dis_max = np.sqrt(3)  # 动作空间范围假设为 [-1, 1]，则速度模长最大为 √3
        
        # 归一化处理（防止除以零）
        norm_dis = np.linalg.norm(v_distance) / (dis_max + 1e-8)  # 目标距离归一化到[0,1]
        norm_act = np.linalg.norm(v_action) / (act_max + 1e-8)  # 速度模长归一化到[0,1]

        # 方向奖励（余弦相似度）
        dot_product = np.dot(v_action, v_distance)
        norm_act_denominator = np.linalg.norm(v_action) + 1e-8  # 防止除零
        norm_dis_denominator = np.linalg.norm(v_distance) + 1e-8
        cos_sim = dot_product / (norm_act_denominator * norm_dis_denominator)

        # 动态理想速度模型（方案一）
        # v_ideal = min(alpha * norm_dis, 1.0)  # 限制归一化后的理想速度不超过1
        v_ideal = alpha * norm_dis
        speed_diff = norm_act - v_ideal
        speed_reward = np.exp(-beta * (speed_diff ** 2))  # 高斯型速度奖励

        # 综合奖励
        return w_angle * cos_sim + w_speed * speed_reward * cos_sim


    def close(self):
        self.simHandler.reset()

        if g_eval:
            if self.episode_count > 0:
                success_rate = self.success_count / self.episode_count
                avg_error = self.total_error / self.success_count if self.success_count > 0 else 0
                avg_time = self.total_time / self.success_count if self.success_count > 0 else 0
            else:
                success_rate = 0.0
                avg_error = 0.0
                avg_time = 0.0

            # 动态生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"traj_{self.motion_type}_speed{self.speed:.1f}_success{success_rate:.2f}_error{avg_error:.2f}_time{avg_time:.1f}_{timestamp}_model{self.model}.json"
            folder_path = os.path.expanduser('~/ws/uav_demo/trajectory/')
            self.recorder.file_path = os.path.join(folder_path, filename)

            # 添加元数据到轨迹记录器
            self.recorder.metadata = {
                "motion_type": self.motion_type,
                "speed": self.speed,
                "success_rate": success_rate,
                "avg_planar_error": avg_error,
                "avg_time": avg_time,
                "landing_results": self.landing_results
            }

            self.recorder.save_trajectories_to_file()


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


        if 0 < dist < 1:
            if 0 <= abs(height) <= 0.1:
                ret = 0
            elif 0.1 < abs(height) <= 3.5:
                ret = 0.5 * height
            elif abs(height) > 3.5:
                ret = 0.5 * height

        return ret, height, dist