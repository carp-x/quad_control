# quad_control

A quadrupedal robot control framework which supports ROS 2 and Gazebo simulation.

<figure align="left">
  <figcaption align="left"><b>Figure 1:</b> Quadruped control with MPC and WBC</figcaption><br>
  <img src="https://github.com/user-attachments/assets/1201ddc4-4299-4c91-a457-5b08774ac8e9" width="600"><br>
</figure>

<figure align="left">
  <figcaption align="left"><b>Figure 2:</b> Quadruped control with Reinforcement Learning</figcaption><br>
  <img src="https://github.com/user-attachments/assets/57eb852d-a7cf-45e6-b842-f83487c8bbc6" width="600"><br>
</figure>

## Key Features
* **Modern Stack**: Native support for **Ubuntu 24.04** & **ROS 2 Jazzy**.
* **Modular Control**: **`ros2_control`** powered integration for seamless algorithm switching and hardware abstraction.
* **Simulation**: High-fidelity physics validation in **Gazebo Harmonic**.
* **MPC**: High-performance optimal control based on **OCS2**.
* **WBC**: Whole Body Control implementation ported from **[legged_control](https://github.com/qiayuanliao/legged_control)**.
* **ONNX**: Support deployment of learning models in ONNX format (e.g., exported from **Isaac Lab**).

```text
quad_control/
├── gait_panel_plugin      # rviz plugin for gait command
├── quad_control           # model, config and launch
├── quad_control_ct        # controller interface with mpc and wbc
├── quad_control_ct_rl     # controller interface with rl onnx model
├── quad_control_gz        # simulated hardware via gazebo
├── quad_control_mpc       # ocs2 mpc interface
├── quad_control_ros       # ros node for gait and goal command
├── quad_control_se        # base pose state estimation
└── gait_panel_wbc         # wbc implementation
```

## Environments
* **OS**: Ubuntu 24.04 LTS
* **ROS 2**: Jazzy Jalisco (LTS)
* **Middleware**: **`ros2_control`**
* **Simulator**: Gazebo Harmonic

## Dependencies
Most dependencies are managed automatically via `rosdep`. The following core libraries require manual installation.
* **OCS2 & Pinocchio**: Follow the official installation guide:
  * [OCS2 ROS 2 Installation Guide](https://github.com/leggedrobotics/ocs2/blob/ros2/installation.md)
* **qpOASES**: Install from source with the official repository:
  * [qpOASES](https://github.com/coin-or/qpOASES)
* **ONNX Runtime**: Install with the official release version:
  * [ONNX Runtime](https://github.com/microsoft/onnxruntime/releases)
  ```bash
  tar -zxvf onnxruntime-linux-x64-1.24.4.tgz
  cd onnxruntime-linux-x64-1.24.4
  sudo mkdir -p /usr/local/include/onnxruntime
  sudo cp -r include/* /usr/local/include/onnxruntime/
  sudo mkdir -p /usr/local/lib
  sudo cp -d lib/libonnxruntime.so* /usr/local/lib/
  sudo mkdir -p /usr/local/lib/cmake/onnxruntime
  sudo cp lib/cmake/onnxruntime/*.cmake /usr/local/lib/cmake/onnxruntime/
  sudo ldconfig
  ```

## Build
  ```bash
  cd ${your_ws}/src
  git clone git@github.com:carp-x/quad_control.git
  cd ..
  colcon build --symlink-install
  ```

## Run
  ```bash
  source install/setup.bash
  ros2 launch quad_control quad_control.launch.py        # for control with mpc and wbc
  ros2 launch quad_control quad_control_rl.launch.py     # for control with rl onnx model
  ```
  Note: 
  * For control with mpc and wbc: click one gait in rviz firstly, then set a goal with 2D Goal Pose in rviz.  
  * For control with rl onnx model: send cmd_vel with rqt Robot Steering tool.

## 🤝 Acknowledgments
* **[legged_control](https://github.com/qiayuanliao/legged_control)**: The primary reference for this implementation.
* **[OCS2](https://leggedrobotics.github.io/ocs2/)**: The core dependency for the control algorithms.
* **[ros2_control](https://control.ros.org/jazzy/index.html)**: The underlying architecture of this project.

