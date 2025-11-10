# Keyboard Input

This node will read the keyboard input and publish a control_input_msgs/Input message.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy
* Ubuntu 22.04
  * ROS2 Humble

### Build Command
```bash
cd ~/ros2_ws
colcon build --packages-up-to keyboard_input
```

### Launch Command
```bash
source ~/ros2_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```

## 1. Use Instructions for Unitree Guide
### 1.1 Control Mode
* Passive Mode: Keyboard 1    被动站
* Fixed Stand: Keyboard 2     固定站
    * Free Stand: Keyboard 3  自由站
    * Trot: Keyboard 4    小跑
    * SwingTest: Keyboard 5 摆动测试
    * Balance: Keyboard 6  平衡测试
### 1.2 Control Input
* WASD IJKL: Move robot
* Space: Reset Speed Input  重置速度输入