# OpenArm v1 ROS 2 servo teleoperation

Package: `openarm_servo_teleop`

The node reads two 8-channel serial leaders, sends `PULK`, samples a startup
zero pose, applies `joint_mapping_v1.json`, and publishes standard ROS 2
trajectory commands at 20 Hz.

Run from Windows:

```powershell
D:\openarm\servo_teleop\run_ros2_teleop_v1.ps1
```

Published controller topics:

```text
/left_joint_trajectory_controller/joint_trajectory
/right_joint_trajectory_controller/joint_trajectory
/left_gripper_controller/joint_trajectory
/right_gripper_controller/joint_trajectory
```

Additional monitoring topics:

```text
/servo_teleop/target_joint_states  sensor_msgs/msg/JointState
/servo_teleop/status               std_msgs/msg/String
```

Build manually in WSL:

```bash
source /opt/ros/humble/setup.bash
cd /mnt/d/openarm/ros2_ws
colcon build --symlink-install --packages-select openarm_servo_teleop
source install/setup.bash
ros2 run openarm_servo_teleop servo_teleop_node
```

The node uses the shared mapping file:

```text
/mnt/d/openarm/servo_teleop/joint_mapping_v1.json
```
