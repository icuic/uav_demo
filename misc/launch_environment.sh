SIM_ID=$1
# UAV_NAME=$2
# ROS_PORT=$3
# GAZ_PORT=$4
ROSIP=$(hostname -I | cut -d' ' -f1)

# export ROS_MASTER_URI=http://$ROSIP:$ROS_PORT;
# export GAZ_MASTER_URI=http://$ROSIP:$GAZ_PORT;
export ROS_IP=$ROSIP

# echo "ROS_PORT = $ROS_PORT"
# echo "GAZEBO_PORT = $GAZ_PORT"
echo "ROS_IP = $ROS_IP"
# echo "ROS_MASTER_URI = $ROS_MASTER_URI"
# echo "GAZEBO_MASTER_URI = $GAXEBO_MASTER_URI"

# cd ~/ws/uav_demo
# catkin_make

# px4 and gazebo
# screen -d -m -S "$(echo $SIM_ID)_landing_simulation" bash -i -c "cd ~/project/PX4-Autopilot; make px4_sitl gazebo-classic"&
# echo "Launch landing_simulation. Waiting 10 seconds to ensure proper start up..."
# sleep 5

# screen -d -m -S "$(echo $SIM_ID)_landing_simulation" bash -i -c "cd ~/project/PX4-Autopilot; 
# source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default；
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)；
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic；
# roslaunch px4 mavros_posix_sitl.launch"&

# echo "Launch px4, gazebo and mavros. Waiting 10 seconds to ensure proper start up..."
# sleep 5

cd ~/project/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 mavros_posix_sitl.launch



# cd ~/project/PX4-Autopilot
# # DONT_RUN=1 make px4_sitl_default gazebo
# # source ~/catkin_ws/devel/setup.bash    # (optional)
# source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
# roslaunch px4 mavros_posix_sitl.launch


echo "Launch px4, gazebo and mavros. Waiting 10 seconds to ensure proper start up..."
sleep 10


# roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"


# mavros
# screen -d -m -S "$(echo $SIM_ID)_landing_mavros" bash -i -c "cd ~/ws/uav_demo; roslaunch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14557\""&
# echo "Launch mavros. Waiting 10 seconds to ensure proper start up..."
# sleep 5

# # offboard
# screen -d -m -S "$(echo $SIM_ID)_landing_offboard" bash -i -c "source ~/ws/uav_demo/devel/setup.bash; cd ~/ws; rosrun offboard_run offboard_run_node"&
# echo "Launch offboard. Waiting 10 seconds to ensure proper start up..."
# sleep 10

# offboard
# screen -d -m -S "$(echo $SIM_ID)_landing_offboard" bash -i -c "source ~/ws/uav_demo/devel/setup.bash; cd ~/ws; rosrun offboard_run uav_rl.py" 
# echo "Launch offboard. Waiting 10 seconds to ensure proper start up..."
# sleep 5

# offboard
# source ~/ws/uav_demo/devel/setup.bash; cd ~/ws; rosrun offboard_run uav_rl.py
# echo "Launch offboard. Waiting 10 seconds to ensure proper start up..."
# sleep 5