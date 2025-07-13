# ROS-MCP-Server Quick-Start Guide

This checklist gets you from a blank Ubuntu ↔ Gazebo ↔ rosbridge ↔ FastMCP pipeline to a running TurtleBot3 simulation you can drive with AI.

---
## 1. Install ROS 2 Rolling
Follow the official binary installation instructions:
https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html

*Reboot or open a fresh shell after finishing.*

---
## 2. Install rosbridge for ROS 2
```
sudo apt install ros-rolling-rosbridge-server
```
Reference: https://github.com/RobotWebTools/rosbridge_suite

---
## 3. Install Gazebo (Ignition / "gz-sim")
ROS 2 Rolling already pulls the matching Gazebo packages, but to install manually:
```
sudo apt install ros-rolling-gz-sim ros-rolling-ros-gz-sim
```
Upstream docs: https://gazebosim.org/docs

---
## 4. Install TurtleBot3 packages
```
sudo apt install ros-rolling-turtlebot3-gazebo ros-rolling-turtlebot3-msgs
```
Reference: https://github.com/ROBOTIS-GIT/turtlebot3

---
## 5. Run rosbridge server
```bash
source /opt/ros/rolling/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# websockets now listening on ws://<HOST>:9090
```

---
## 6. Launch Gazebo with a TurtleBot3 world
```bash
source /opt/ros/rolling/setup.bash
export TURTLEBOT3_MODEL=burger   # or waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Gazebo GUI opens; the robot publishes `/cmd_vel`, `/camera/image_raw`, `/joint_states`, etc.

---
## 7. Run the ROS-MCP Server
```bash
cd ~/SynologyDrive/1_projects/41_ros-mcp-server/ros-mcp-server
uv run python server.py
```
You should see:
```
[FastMCP] WebSocketManager connected to rosbridge at <IP> : 9090
[FastMCP] Starting ros-mcp-server ... waiting for tool calls
```
The server now exposes MCP tools such as `get_topics`, `pub_twist`, `sub_image`, … via JSON-RPC (stdio). Windsurf’s MCP panel can talk to it directly.

---
## Smoke-Test
(See step 8 for a Docker-Compose alternative.)

1. From an MCP client, call `pub_twist` with `{ "linear": [0.2,0,0], "angular": [0,0,0] }`.
2. The TurtleBot3 should drive forward in Gazebo.

---
## 8. One-command Docker Compose setup
If you prefer an all-in-one container solution (useful for teammates):

1. Ensure Docker and docker-compose v2 are installed on the host.
2. From the project root:
   ```bash
   cd docker
   ```
   ### Enabling the Gazebo GUI on Linux desktops
   If you want to see the Gazebo window, you must allow the container’s X-client to talk to your host X-server:

   ```bash
   xhost +local:root       # run once per terminal session
   ```
   Then launch the stack as usual:

   ```bash
   docker compose up --build
   ```
   When you’re done you can revoke the permission:

   ```bash
   xhost -local:root
   ```
   The `sim` service runs Gazebo + TurtleBot3 + rosbridge (websocket exposed on port 9090).

3. Optional: remove the `DISPLAY` mapping in `docker/docker-compose.yml` to run Gazebo headless on servers without GUI.

### Connecting your MCP server (Windsurf)
Run the MCP server on your host (or any machine that can reach `localhost:9090`).

If using Windsurf’s `mcp_config.json`, wrap the command with environment variables so `server.py` points to Docker rosbridge:
```json
"ros-mcp-server": {
  "command": "env",
  "args": [
    "ROSBRIDGE_IP=127.0.0.1",
    "LOCAL_IP=127.0.0.1",
    "uv",
    "--directory",
    "/path/to/ros-mcp-server",
    "run",
    "server.py"
  ]
}
```
(Alternatively hard-code `LOCAL_IP` / `ROSBRIDGE_IP` inside `server.py` to `"127.0.0.1"`.)

The `docker/` directory contains:
* `docker-compose.yml`
* `ros2-gazebo/`  – Dockerfile + launch script
* `mcp-server/`   – Dockerfile for FastMCP server

After `docker compose up` finishes, visit Gazebo GUI (if mapped) and test the same MCP `pub_twist` call.

1. From an MCP client, call `pub_twist` with `{ "linear": [0.2,0,0], "angular": [0,0,0] }`.
2. The TurtleBot3 should drive forward in Gazebo.

If you see movement, the full stack is working!
