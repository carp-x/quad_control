# quad_control

A quadrupedal robot control framework which supports ROS 2 and Gazebo simulation.

<p align="center">
  <img src="https://github.com/user-attachments/assets/c36abc95-2b6b-4673-bfcf-e463c0c82207" width="48%" />
  <img src="https://github.com/user-attachments/assets/0933aa6a-89d8-42bf-b0cf-5c7da1568391" width="48%" />
</p>

<p align="center">
  <video src="https://github.com/user-attachments/assets/bcc9eaa4-4e32-43f7-9fca-ad2bcf7ec76f" width="66%" autoplay muted loop></video>
</p>

## Environment
- **OS**: Ubuntu 24.04
- **ROS 2**: Jazzy Jalisco
- **Simulator**: Gazebo Harmonic

## Dependencies
The framework relies on the following core libraries:
- Eigen3  
- Pinocchio  
- OCS2
- qpOASES
- ros2_control, ros_gz_sim  

## Build
   ```bash
   git clone git@github.com:carp-x/quad_control.git
   cd quad_control
   colcon build --symlink-install
   ```

 # Run
   ```bash
   source install/setup.bash
   ros2 launch quad_control quad_control.launch.py
   ```
