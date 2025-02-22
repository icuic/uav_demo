#! /usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point
from gazebo_msgs.msg import ModelState
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
import os
from datetime import datetime

class DroneAndPlatformTrajectoryRecorder:
    def __init__(self):
        # 初始化 ROS 节点
        # rospy.init_node('drone_and_platform_trajectory_recorder', anonymous=True)
        # 初始化无人机和移动降落平台的轨迹存储列表
        self.drone_all_trajectories = []
        self.drone_current_trajectory = []
        self.platform_all_trajectories = []
        self.platform_current_trajectory = []

        # 增加轨迹保存标志，初始为 0
        self.save_trajectory_flag = 0

        # 用于临时存储无人机的速度
        self.drone_velocity = None
        # 订阅无人机位置话题
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
        # 订阅无人机速度话题
        rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, self.drone_velocity_callback)
        # 订阅 gazebo/set_model_state 话题获取移动平台轨迹
        rospy.Subscriber('gazebo/set_model_state', ModelState, self.platform_pose_callback)

        # 获取当前日期和时间
        now = datetime.now()
        # 格式化日期和时间
        timestamp = now.strftime("%Y%m%d_%H%M%S")
        # 目标文件夹路径
        folder_path = os.path.expanduser('~/ws/uav_demo/trajectory/')
        # 确保目标文件夹存在，如果不存在则创建
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        # 构造包含日期和时间的完整文件名
        self.file_path = os.path.join(folder_path, f"drone_and_platform_trajectories_{timestamp}.json")

    def drone_pose_callback(self, msg):
        if self.save_trajectory_flag == 1:
            # 提取无人机位置信息
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            # 获取当前时间戳
            timestamp = rospy.Time.now().to_sec()

            if self.drone_velocity is not None:
                vx = self.drone_velocity.twist.linear.x
                vy = self.drone_velocity.twist.linear.y
                vz = self.drone_velocity.twist.linear.z
            else:
                vx, vy, vz = 0, 0, 0

            # 将 (timestamp, x, y, z, vx, vy, vz) 元组添加到当前无人机飞行轨迹列表中
            self.drone_current_trajectory.append((timestamp, x, y, z, vx, vy, vz))

    def platform_pose_callback(self, msg):
        if self.save_trajectory_flag == 1:
            # 提取移动平台位置信息
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            # 获取当前时间戳
            timestamp = rospy.Time.now().to_sec()
            # 将 (timestamp, x, y, z) 元组添加到当前移动平台轨迹列表中
            self.platform_current_trajectory.append((timestamp, x, y, z))

    def drone_velocity_callback(self, msg):
        self.drone_velocity = msg

    def start_new_trajectory(self):
        # 清空当前无人机和移动平台的轨迹列表，开始记录新的轨迹
        self.drone_current_trajectory = []
        self.platform_current_trajectory = []
        # 置为 1 表示开始保存轨迹
        self.save_trajectory_flag = 1
        print("Started recording new trajectory.")

    def stop_new_trajectory(self):
        if self.drone_current_trajectory:
            # 如果当前无人机轨迹不为空，将其添加到所有无人机轨迹列表中
            self.drone_all_trajectories.append(self.drone_current_trajectory)
        if self.platform_current_trajectory:
            # 如果当前移动平台轨迹不为空，将其添加到所有移动平台轨迹列表中
            self.platform_all_trajectories.append(self.platform_current_trajectory)
        # 清空当前轨迹列表
        self.drone_current_trajectory = []
        self.platform_current_trajectory = []
        # 置为 0 表示停止保存轨迹
        self.save_trajectory_flag = 0
        print("Stopped recording new trajectory and saved current trajectories.")

    def clear_current_trajectories(self):
        # 清除当前无人机和移动平台的轨迹
        self.drone_current_trajectory = []
        self.platform_current_trajectory = []
        print("Current drone and platform trajectories have been cleared.")

    def clear_save_flag(self):
        # 清除保存标志
        self.save_trajectory_flag = 0
        print("Trajectory save flag has been cleared.")

    def save_trajectories_to_file(self):
        if self.drone_current_trajectory:
            # 如果当前无人机轨迹不为空，将其添加到所有无人机轨迹列表中
            self.drone_all_trajectories.append(self.drone_current_trajectory)
        if self.platform_current_trajectory:
            # 如果当前移动平台轨迹不为空，将其添加到所有移动平台轨迹列表中
            self.platform_all_trajectories.append(self.platform_current_trajectory)

        # 把元组转换为列表，因为 JSON 不支持直接存储元组
        json_drone_trajectories = [[list(point) for point in trajectory] for trajectory in self.drone_all_trajectories]
        json_platform_trajectories = [[list(point) for point in trajectory] for trajectory in self.platform_all_trajectories]

        # 合并元数据
        data = {
            "metadata": getattr(self, 'metadata', {}),  # 从外部传入的元数据
            "drone_trajectories": json_drone_trajectories,
            "platform_trajectories": json_platform_trajectories
        }

        with open(self.file_path, 'w') as file:
            json.dump(data, file, indent=4)

    def save_and_plot_trajectories(self):
        self.save_trajectories_to_file()

        num_drone_trajectories = len(self.drone_all_trajectories)
        num_platform_trajectories = len(self.platform_all_trajectories)

        if num_drone_trajectories == 0 and num_platform_trajectories == 0:
            print("No trajectories to plot.")
            return

        while True:
            try:
                input_str = input(f"Enter the drone trajectory numbers to plot (1 - {num_drone_trajectories}, comma-separated, or 'all' for all drone trajectories): ")
                if input_str.lower() == 'all':
                    drone_selected_indices = range(num_drone_trajectories)
                else:
                    drone_selected_indices = [int(i) - 1 for i in input_str.split(',') if 1 <= int(i) <= num_drone_trajectories]

                input_str = input(f"Enter the platform trajectory numbers to plot (1 - {num_platform_trajectories}, comma-separated, or 'all' for all platform trajectories): ")
                if input_str.lower() == 'all':
                    platform_selected_indices = range(num_platform_trajectories)
                else:
                    platform_selected_indices = [int(i) - 1 for i in input_str.split(',') if 1 <= int(i) <= num_platform_trajectories]

                if not drone_selected_indices and not platform_selected_indices:
                    print("Invalid input. Please try again.")
                    continue
                break
            except ValueError:
                print("Invalid input. Please enter valid numbers.")

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for index in drone_selected_indices:
            trajectory = self.drone_all_trajectories[index]
            if trajectory:
                # 分离出 x, y, z 坐标用于绘图
                x_coords = [point[1] for point in trajectory]
                y_coords = [point[2] for point in trajectory]
                z_coords = [point[3] for point in trajectory]

                # 绘制每一段无人机飞行轨迹
                ax.plot(x_coords, y_coords, z_coords, marker='o', linestyle='-', color='b', label=f'Drone Trajectory {index + 1}')

        for index in platform_selected_indices:
            trajectory = self.platform_all_trajectories[index]
            if trajectory:
                # 分离出 x, y, z 坐标用于绘图
                x_coords = [point[1] for point in trajectory]
                y_coords = [point[2] for point in trajectory]
                z_coords = [point[3] for point in trajectory]

                # 绘制每一段移动平台轨迹
                ax.plot(x_coords, y_coords, z_coords, marker='s', linestyle='--', color='r', label=f'Platform Trajectory {index + 1}')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Selected Drone and Platform Flight Trajectories')
        ax.legend()
        plt.show()

    def plot_specified_trajectories(self, file_path, combine_plots=True):
        try:
            with open(file_path, 'r') as file:
                data = json.load(file)
            drone_trajectories = data.get("drone_trajectories", [])
            platform_trajectories = data.get("platform_trajectories", [])

            num_drone_trajectories = len(drone_trajectories)
            num_platform_trajectories = len(platform_trajectories)

            if num_drone_trajectories == 0 and num_platform_trajectories == 0:
                print("No trajectories to plot.")
                return

            while True:
                try:
                    input_str = input(f"Enter the drone trajectory numbers to plot (1 - {num_drone_trajectories}, comma-separated, or 'all' for all drone trajectories): ")
                    if input_str.lower() == 'all':
                        drone_selected_indices = range(num_drone_trajectories)
                    else:
                        drone_selected_indices = [int(i) - 1 for i in input_str.split(',') if 1 <= int(i) <= num_drone_trajectories]

                    input_str = input(f"Enter the platform trajectory numbers to plot (1 - {num_platform_trajectories}, comma-separated, or 'all' for all platform trajectories): ")
                    if input_str.lower() == 'all':
                        platform_selected_indices = range(num_platform_trajectories)
                    else:
                        platform_selected_indices = [int(i) - 1 for i in input_str.split(',') if 1 <= int(i) <= num_platform_trajectories]

                    if not drone_selected_indices and not platform_selected_indices:
                        print("Invalid input. Please try again.")
                        continue
                    break
                except ValueError:
                    print("Invalid input. Please enter valid numbers.")

            if combine_plots:
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')

                for index in drone_selected_indices:
                    trajectory = drone_trajectories[index]
                    if trajectory:
                        x_coords = [point[1] for point in trajectory]
                        y_coords = [point[2] for point in trajectory]
                        z_coords = [point[3] for point in trajectory]
                        ax.plot(x_coords, y_coords, z_coords, linestyle='-', color='b', label=f'Drone Trajectory {index + 1}')

                for index in platform_selected_indices:
                    trajectory = platform_trajectories[index]
                    if trajectory:
                        x_coords = [point[1] for point in trajectory]
                        y_coords = [point[2] for point in trajectory]
                        z_coords = [point[3] for point in trajectory]
                        ax.plot(x_coords, y_coords, z_coords, linestyle='-', color='r', label=f'Platform Trajectory {index + 1}')

                ax.set_xlabel('X (m)')
                ax.set_ylabel('Y (m)')
                ax.set_zlabel('Z (m)')
                ax.set_title('Selected Drone and Platform Flight Trajectories')
                ax.legend()
                plt.show()
            else:
                if drone_selected_indices:
                    fig_drone = plt.figure()
                    ax_drone = fig_drone.add_subplot(111, projection='3d')
                    for index in drone_selected_indices:
                        trajectory = drone_trajectories[index]
                        if trajectory:
                            x_coords = [point[1] for point in trajectory]
                            y_coords = [point[2] for point in trajectory]
                            z_coords = [point[3] for point in trajectory]
                            ax_drone.plot(x_coords, y_coords, z_coords, marker='o', linestyle='-', color='b', label=f'Drone Trajectory {index + 1}')
                    ax_drone.set_xlabel('X (m)')
                    ax_drone.set_ylabel('Y (m)')
                    ax_drone.set_zlabel('Z (m)')
                    ax_drone.set_title('Selected Drone Flight Trajectories')
                    ax_drone.legend()
                    plt.show()

                if platform_selected_indices:
                    fig_platform = plt.figure()
                    ax_platform = fig_platform.add_subplot(111, projection='3d')
                    for index in platform_selected_indices:
                        trajectory = platform_trajectories[index]
                        if trajectory:
                            x_coords = [point[1] for point in trajectory]
                            y_coords = [point[2] for point in trajectory]
                            z_coords = [point[3] for point in trajectory]
                            ax_platform.plot(x_coords, y_coords, z_coords, marker='s', linestyle='--', color='r', label=f'Platform Trajectory {index + 1}')
                    ax_platform.set_xlabel('X (m)')
                    ax_platform.set_ylabel('Y (m)')
                    ax_platform.set_zlabel('Z (m)')
                    ax_platform.set_title('Selected Platform Flight Trajectories')
                    ax_platform.legend()
                    plt.show()

        except FileNotFoundError:
            print(f"Error: The specified file {file_path} was not found.")
        except json.JSONDecodeError:
            print(f"Error: The file {file_path} does not contain valid JSON data.")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
    
    def run(self):
        try:
            while True:
                command = input("Enter 's' to start a new trajectory, 'e' to end and save current trajectory, 'c' to clear current trajectories, 'f' to clear save flag, 'p' to plot specified trajectories, or 'q' to quit, save to file and plot: ")
                if command.lower() == 's':
                    self.start_new_trajectory()
                elif command.lower() == 'e':
                    self.stop_new_trajectory()
                elif command.lower() == 'c':
                    self.clear_current_trajectories()
                elif command.lower() == 'f':
                    self.clear_save_flag()
                elif command.lower() == 'p':
                    file_path = input("Enter the path of the trajectory file: ")
                    combine_input = input("Do you want to combine the plots? (y/n): ").lower()
                    combine_plots = combine_input == 'y'
                    self.plot_specified_trajectories(file_path, combine_plots)
                elif command.lower() == 'q':
                    self.save_and_plot_trajectories()
                    break
        except KeyboardInterrupt:
            print("Shutting down")
            # self.save_and_plot_trajectories()


if __name__ == '__main__':
    recorder = DroneAndPlatformTrajectoryRecorder()
    recorder.run()