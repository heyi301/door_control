#! /bin/bash

# Set memory limits
ulimit -v 8000000  # 8GB virtual memory limit

# Source ROS and workspace
source  /opt/ros/foxy/setup.bash;
source ~/cyclonedds_ws/install/setup.bash;
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml;
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH;
source /home/unitree/door_ws/door_control/install/setup.bash

# Wait for ROS core to be ready
sleep 2

# Launch with error handling
echo "Starting door control node..."
ros2 launch door_control door_control_launch.py

# If the launch fails, exit with error code
if [ $? -ne 0 ]; then
    echo "Failed to start door control node"
    exit 1
fi