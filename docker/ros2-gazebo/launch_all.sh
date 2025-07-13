#!/usr/bin/env bash
set -e
source /opt/ros/rolling/setup.bash

# Determine if we should run gazebo headless
if [ -z "$DISPLAY" ]; then
  echo "[launch_all] DISPLAY not set; running Gazebo headless"
  GUI_FLAG="gui:=false"
else
  GUI_FLAG=""
fi

# Start TurtleBot3 world in Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py $GUI_FLAG &
GZ_PID=$!

# Start rosbridge websocket
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
BRIDGE_PID=$!

# Wait for either process to exit to keep container running
wait -n $GZ_PID $BRIDGE_PID
