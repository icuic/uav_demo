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

        # 存储绘图配置
        self.plot_file_path = None
        self.combine_plots_setting = True

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
        
        num_drone = len(self.drone_all_trajectories)
        num_platform = len(self.platform_all_trajectories)
        max_paired_idx = min(num_drone, num_platform)  # 取最小长度确保配对

        if max_paired_idx == 0:
            print("No trajectories to plot.")
            return

        while True:
            try:
                input_str = input(f"Enter paired trajectory numbers (1-{max_paired_idx}), comma-sep, or 'all': ")
                if input_str.lower() == 'all':
                    selected_indices = list(range(max_paired_idx))
                else:
                    selected_indices = [int(i)-1 for i in input_str.split(',') if 1 <= int(i) <= max_paired_idx]
                
                if not selected_indices:
                    print("Invalid input. Please try again.")
                    continue
                break
            except ValueError:
                print("Invalid input. Please enter valid numbers.")

        figures = []
        for idx in selected_indices:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            
            # 绘制无人机轨迹
            drone_traj = self.drone_all_trajectories[idx]
            if drone_traj:
                x = [p[1] for p in drone_traj]
                y = [p[2] for p in drone_traj]
                z = [p[3] for p in drone_traj]
                ax.plot(x, y, z, marker='o', linestyle='-', color='b', label=f'Drone {idx+1}')
            
            # 绘制平台轨迹
            platform_traj = self.platform_all_trajectories[idx]
            if platform_traj:
                x = [p[1] for p in platform_traj]
                y = [p[2] for p in platform_traj]
                z = [p[3] for p in platform_traj]
                ax.plot(x, y, z, marker='s', linestyle='--', color='r', label=f'Platform {idx+1}')
            
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f'Paired Trajectories {idx+1}')
            ax.legend()
            figures.append(fig)

        plt.show(block=False)  # 非阻塞模式显示所有窗口
        input("Press Enter to close all plots...")  # 保持窗口打开直到用户确认
        plt.close('all')

    def get_trajectory_files(self, directory):
        """获取指定目录下所有.json轨迹文件"""
        if not os.path.exists(directory):
            print(f"Error: Directory {directory} does not exist.")
            return []
        files = [f for f in os.listdir(directory) if f.endswith('.json')]
        return sorted(files)

    def select_trajectory_file(self):
        """交互式选择轨迹文件"""
        # 固定目标目录（根据实际需求修改路径）
        target_dir = "/home/ubuntu/ws/uav_demo/trajectory/0225-2315-20000"  
        files = self.get_trajectory_files(target_dir)
        
        if not files:
            print("No trajectory files found in directory.")
            return None
        
        print("\nAvailable trajectory files:")
        for idx, f in enumerate(files, 1):
            print(f"  [{idx}] {f}")
        
        while True:
            try:
                choice = input(f"Select file (1-{len(files)}) or 'q' to cancel: ")
                if choice.lower() == 'q':
                    return None
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(files):
                    return os.path.join(target_dir, files[choice_idx])
                print("Invalid selection.")
            except ValueError:
                print("Please enter a valid number.")

    def plot_specified_trajectories(self):
        """可视化指定轨迹文件（支持文件列表选择/配对绘制/多窗口同时显示）"""
        try:
            # 首次调用时需要选择文件
            if self.plot_file_path is None:
                target_dir = "/home/ubuntu/ws/uav_demo/trajectory/0225-2315-20000"
                files = self.get_trajectory_files(target_dir)
                
                if not files:
                    print(f"No JSON files found in {target_dir}")
                    return
                    
                print("\nAvailable trajectory files:")
                for idx, fname in enumerate(files, 1):
                    print(f"  [{idx}] {fname}")
                    
                while True:
                    choice = input(f"Select file (1-{len(files)}) or 'q' to cancel: ")
                    if choice.lower() == 'q':
                        return
                    try:
                        choice_idx = int(choice) - 1
                        if 0 <= choice_idx < len(files):
                            self.plot_file_path = os.path.join(target_dir, files[choice_idx])
                            break
                        print("Invalid selection.")
                    except ValueError:
                        print("Please enter a valid number.")
                
                # 设置配对绘图模式
                combine = input("Show drone and platform in same plot? (y/n): ").lower()
                self.combine_plots_setting = (combine == 'y')

            # 加载轨迹数据
            with open(self.plot_file_path, 'r') as f:
                data = json.load(f)
                
            drone_trajs = data.get("drone_trajectories", [])
            platform_trajs = data.get("platform_trajectories", [])
            max_pairs = min(len(drone_trajs), len(platform_trajs))

            if max_pairs == 0:
                print("No valid trajectory pairs to plot.")
                return

            # 获取用户输入的配对索引
            while True:
                input_str = input(f"Enter pair numbers (1-{max_pairs}), comma-sep, or 'all': ")
                if input_str.lower() == 'all':
                    selected = list(range(max_pairs))
                    break
                else:
                    try:
                        selected = [int(i)-1 for i in input_str.split(',') if i.isdigit()]
                        selected = [idx for idx in selected if 0 <= idx < max_pairs]
                        if selected:
                            break
                        print("No valid indices selected.")
                    except:
                        print("Invalid input format.")

            # 生成所有图表对象
            figures = []
            for pair_idx in selected:
                fig = plt.figure(figsize=(10, 6), num=f"Trajectory Pair {pair_idx+1}")
                ax = fig.add_subplot(111, projection='3d')
                
                # 绘制无人机轨迹
                drone_data = drone_trajs[pair_idx]
                if drone_data:
                    x = [p[1] for p in drone_data]
                    y = [p[2] for p in drone_data]
                    z = [p[3] for p in drone_data]
                    ax.plot(x, y, z, 
                        # marker='o', 
                        markersize=4,
                        linestyle='-',
                        linewidth=1.5,
                        color='#1f77b4',
                        label=f'Drone {pair_idx+1}')
                
                # 绘制平台轨迹
                platform_data = platform_trajs[pair_idx]
                if platform_data:
                    x = [p[1] for p in platform_data]
                    y = [p[2] for p in platform_data]
                    z = [p[3] for p in platform_data]
                    ax.plot(x, y, z,
                        # marker='s',
                        markersize=4,
                        linestyle='--',
                        linewidth=1.5,
                        color='#ff7f0e',
                        label=f'Platform {pair_idx+1}')
                
                # 图表装饰
                ax.set_xlabel('X (m)', labelpad=12)
                ax.set_ylabel('Y (m)', labelpad=12)
                ax.set_zlabel('Z (m)', labelpad=12)
                ax.xaxis.pane.fill = False
                ax.yaxis.pane.fill = False
                ax.zaxis.pane.fill = False
                ax.grid(True, linestyle=':', alpha=0.6)
                ax.legend(loc='upper right')
                ax.set_title(f"Trajectory Pair {pair_idx+1}\n{os.path.basename(self.plot_file_path)}",
                            fontsize=10, pad=20)
                
                figures.append(fig)

            # 非阻塞显示所有窗口
            plt.show(block=False)
            
            # 保持窗口直到用户确认
            input("\nAll plots displayed. Press Enter to close windows...")
            plt.close('all')
            print("All plot windows closed.")

        except FileNotFoundError:
            print(f"Error: File {self.plot_file_path} not found. Resetting file path.")
            self.plot_file_path = None
        except json.JSONDecodeError:
            print(f"Invalid JSON format in {self.plot_file_path}. Resetting file path.")
            self.plot_file_path = None
        except Exception as e:
            print(f"Unexpected error: {str(e)}")
            self.plot_file_path = None

    # 修改 run 方法中的命令处理
    def run(self):
        try:
            while True:
                # cmd = input("Enter 's' start, 'e' end, 'p' plot, 'q' quit: ")
                # if cmd == 'p':
                self.plot_specified_trajectories()
                # ... 其他命令处理 ...
        except KeyboardInterrupt:
            print("Shutdown")


if __name__ == '__main__':
    recorder = DroneAndPlatformTrajectoryRecorder()
    recorder.run()