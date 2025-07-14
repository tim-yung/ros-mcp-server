# ROS-MCP-Server Quick-Start Guide

This checklist gets you from a blank Ubuntu ↔ Gazebo ↔ rosbridge ↔ FastMCP pipeline to a running TurtleBot3 simulation you can drive with AI.

## (A) Simulation Setup

### (A1) Docker-based Simulation Setup
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

---

### (A2) Manual Simulation Setup

---
#### 1. Install ROS 2 Rolling
Follow the official binary installation instructions:
https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html

*Reboot or open a fresh shell after finishing.*

---
#### 2. Install rosbridge for ROS 2
```
sudo apt install ros-rolling-rosbridge-server
```
Reference: https://github.com/RobotWebTools/rosbridge_suite

---
#### 3. Install Gazebo (Ignition / "gz-sim")
ROS 2 Rolling already pulls the matching Gazebo packages, but to install manually:
```
sudo apt install ros-rolling-gz-sim ros-rolling-ros-gz-sim
```
Upstream docs: https://gazebosim.org/docs

---
#### 4. Install TurtleBot3 packages
```
sudo apt install ros-rolling-turtlebot3-gazebo ros-rolling-turtlebot3-msgs
```
Reference: https://github.com/ROBOTIS-GIT/turtlebot3

---
#### 5. Run rosbridge server
```bash
source /opt/ros/rolling/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# websockets now listening on ws://<HOST>:9090
```

---
#### 6. Launch Gazebo with a TurtleBot3 world
```bash
source /opt/ros/rolling/setup.bash
export TURTLEBOT3_MODEL=burger   # or waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Gazebo GUI opens; the robot publishes `/cmd_vel`, `/camera/image_raw`, `/joint_states`, etc.

---


## (B) AI Agent Setup

### Connecting your MCP server (LangGraph Agent)

Once `docker compose up` is running, open a **new terminal** and launch the LangGraph agent:

```bash
cd agent
uv run langgraph dev
```

The terminal will print a Studio URL similar to:

```
https://smith.langchain.com/studio/?baseUrl=http://127.0.0.1:2024
```

Open this link in your browser to chat with the agent and issue MCP tool calls.

#### Prerequisites
1. **LangSmith account (free)** – required to access LangGraph Studio.
2. **Environment variables** – copy `agent/.env.template` to `agent/.env` and fill in:
   * `OPENAI_API_KEY` (required)
   * `LANGCHAIN_API_KEY`, `LANGCHAIN_ENDPOINT` (optional for LangSmith tracing).

#### Initial configuration
Edit `agent/src/react_agent/mcp_config.json` so that the `directories` field points to the **absolute path** of your local `ros-mcp-server` directory, e.g.

```json
{
  "directories": ["<ABSOLUTE_PATH_TO_ROS_MCP_SERVER>"]
}
```

Make sure this path is correct before starting the agent.


After `docker compose up` finishes, visit Gazebo GUI (if mapped) and test the AI agent..

1. From Langgraph Studio, ask the robot to move forward.
2. The TurtleBot3 should drive forward in Gazebo.

If you see movement, the full stack is working!
