# quad_control

A quadrupedal robot control framework which supports ROS 2 and Gazebo simulation.

<p align="left">
  <img src="https://github.com/user-attachments/assets/1201ddc4-4299-4c91-a457-5b08774ac8e9" width="600">
</p>

## Key Features
* **Modern Stack**: Native support for **Ubuntu 24.04** & **ROS 2 Jazzy**.
* **Modular Control**: **`ros2_control`** powered integration for seamless algorithm switching and hardware abstraction.
* **Simulation**: High-fidelity physics validation in **Gazebo Harmonic**.
* **MPC**: High-performance optimal control based on **OCS2**.
* **WBC**: Whole Body Control implementation ported from **[legged_control](https://github.com/qiayuanliao/legged_control)**.

```text
quad_control/
├── gait_panel_plugin      # rviz plugin for gait command
├── quad_control           # model, config and launch
├── quad_control_ct        # controller interface
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
* **qpOASES**: Install from source from the official repository:
  * [qpOASES](https://github.com/coin-or/qpOASES)

## Build
   ```bash
   cd {your_ws}/src
   git clone git@github.com:carp-x/quad_control.git
   cd ..
   colcon build --symlink-install
   ```

## Run
   ```bash
   source install/setup.bash
   ros2 launch quad_control quad_control.launch.py
   ```

## 🤝 Acknowledgments
* **[legged_control](https://github.com/qiayuanliao/legged_control)**: The primary reference for this implementation.
* **[OCS2](https://leggedrobotics.github.io/ocs2/)**: The core dependency for the control algorithms.
* **[ros2_control](https://control.ros.org/jazzy/index.html)**: The underlying architecture of this project.

