# Robocon Quadruped ROS2 Simulation

## 1. Start

#### 1.1 Set Environment

- Ubuntu 22.04

- ROS: Humble

- Ignition Gazebo [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/):

#### 1.2 Create Workspace

```bash
mkdir -p quad_sim_ws/src
cd quad_sim_ws
```

#### 1.3 Build

- rosdep

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

- Compile the package

```bash
colcon build --symlink-install
```

#### 1.4 Run

- 启动仿真环境

```bash
source install/setup.bash
ros2 launch rc_quadruped_ros2_simulation_bringup quad_sim_bringup.launch.py world_file:=warehouse
```

- 启动键盘控制

```bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

## Thanks

### 1. legubiao/quadruped_ros2_control

[https://github.com/legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control)
