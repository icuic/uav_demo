#! /usr/bin/python
## /home/ubuntu/miniconda3/envs/px4/bin/python

import gymnasium
from gymnasium import spaces
import numpy as np

import rospy

# 
from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest
from std_msgs.msg import Header

from threading import Thread
from pymavlink import mavutil

class simulationHandler():
    def __init__(self):
        rospy.init_node("offb_test")

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

        # 订阅话题 subscribe topic
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state', ExtendedState, self.extended_state_callback)

        # 发布话题 publish topic
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # 创建服务客户端 service client
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # send setpoints in seperate thread
        self.pos_thread = Thread(target=self.send_pos, args=(), name="pos_thread")
        self.pos_thread.daemon = True
        self.pos_thread.start()        


    def setup(self):
        

        service_timeout = 60
        rospy.loginfo("waiting for ROS services")

        try:
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.loginfo("simulationHandler failed to connect to services")

        rospy.loginfo("simulation handler setup.")

    def getReady(self):
        self.wait_for_topics(60)
        # self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        #                            10, -1)

        rospy.loginfo("all topics are ready to get")

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.state.connected):
            rate.sleep()

        rospy.loginfo("connected")

        # 设置初始位置
        self.pos.pose.position.x = 0
        self.pos.pose.position.y = 0
        self.pos.pose.position.z = 2

        # 切换至 offboard 模式
        while self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD", 5)
        rospy.loginfo("switched to offboard mode")

        # 解锁
        while not self.state.armed:
            self.set_arm(True, 5)
        rospy.loginfo("swithed to armed")

        while True:
            rate.sleep()

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

    def reset(self):
        pass

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
        # 与无人机(MAVROS)、仿真环境(Gazebo)交互的句柄
        simHandler = simulationHandler()
        simHandler.setup()
        simHandler.getReady()

        # 定义动作空间：无人机xyz三个轴上的速度
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # 定义状态空间：无人机与指定降落地点的xyz三个轴上的相对距离
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)

        # 设置指定降落地点的坐标
        self.target_position = np.array([0.0, 0.0, 0.0])


    def reset(self):
        print("UAVLandingEnv reset.")

        # 重置无人机的位置，可以随机选择一个初始位置
        self.drone_position = np.random.uniform(-10, 10, size=(3,))
        return self._get_observation()
        
    
    def step(self, action):
        print(f"Execute times step.")

        # 更新无人机的位置

        # 计算奖励

        # 检查是否达到目标位置

        # 返回状态、奖励、是否结束以及其他信息
        return (0,0,0), 0, False, False, {}
    
    def _get_observation(self):
        # 计算无人机与指定降落地点的xyz三个轴上的相对距离
        return (0, 0, 0)
    
