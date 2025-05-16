# RoboRacer Sim Racing League @ ICRA 2025

This repository contains code for the **RoboRacer Sim Racing League @ ICRA 2025** competition.

🔗 [Competition Website](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-icra-2025/#resources)

---

## 🧩 Dependencies

- **OS**: Ubuntu 22.04
- **Python**: 3.10.12
- **ROS 2**: Humble Hawksbill
- **TensorFlow**: 2.16.1
- **CUDA**: 12.4

> 💡 **Check NVIDIA Driver**:  
> Run `nvidia-smi` to check your GPU status.  
> If nothing shows up:
> ```bash
> sudo ubuntu-drivers autoinstall
> ```

---

## 🚀 Setup Instructions

### 1. Pull and Run AutoDRIVE Simulator

**1. Pull the container**:
```bash
docker pull autodriveecosystem/autodrive_roboracer_sim:2025-icra-practice
```

**2. Enable Display Forwarding:**
``` bash
xhost +local:root
```

**3. Run the Simulator Container:**
``` bash
docker run --name autodrive_roboracer_sim -it \
  --entrypoint /bin/bash \
  --network=host \
  --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --env DISPLAY \
  --privileged \
  --gpus all \
  autodriveecosystem/autodrive_roboracer_sim:2025-icra-practice
```

### 2. Pull and Run AutoDRIVE Devkit Container
Repeat the above steps, replacing `autodrive_roboracer_sim` with `autodrive_roboracer_api` and using the appropriate image tag for the Devkit.

## Running the System
### 1. Launch WebSocket-ROS2 Bridge
In the **Devkit Container:**
``` bash
cd /home/autodrive_devkit
ros2 launch autodrive_roboracer bringup_graphics.launch.py
```
This bridge facilitates bidirectional communication between WebSocket and ROS 2, enabling real-time control and monitoring of AutoDrive Simulator vehicles by transmitting and receiving sensor and vehicle state data.

### 2. Start the AutoDRIVE Simulator
In the **Simulator Container:**
``` bash
cd /home/autodrive_simulator
./AutoDRIVE\ Simulator.x86_64
```
In the GUI:
- Click the **"Disconnected"** button to connect (status changes to **"Connected"**).
- The simulator supports two modes:
  - **Autonomous Mode**: Controlled via ROS 2 nodes publishing to `/steering_command` and `/throttle_command`.
  - **Manual Mode**: Controlled manually using the keyboard within the simulator GUI.

## Autonomous Mode: Running Control Nodes
In the **Devkit Container**, you can run various nodes to control the vehicle:
  - **Keyboard Teleoperation**:
  ```bash
  ros2 run autodrive_roboracer teleop_keyboard
  ```
  - **Gap Follower Algorithm**:
  ```bash
  ros2 run autodrive_roboracer gapfollower
  ```
  - **Joystick Control**:
  ```bash
  ros2 run autodrive_roboracer joystick
  ```
  - **TinyLidarNet (Neural Network Model)**:
  ```bash
  ros2 run autodrive_roboracer tinylidar
  ```
  Or due to GUI dependencies, run using xvfb-run:
  ```bash
  cd /home/autodrive_devkit/src/autodrive_devkit/autodrive_roboracer/TinyLidarNet
  xvfb-run -a python3 tinylidar.py
  ```
  > Note: `xvfb-run` is used to simulate an X server environment, allowing GUI applications to run in headless environments without a physical display.

  Install required Python packages before running:
  ```bash
  pip install -r requirements.txt
  ```
## Manual Mode
In Manual Mode, control the vehicle directly using the keyboard within the simulator GUI. No additional ROS 2 nodes are required.

##

```bash

```
