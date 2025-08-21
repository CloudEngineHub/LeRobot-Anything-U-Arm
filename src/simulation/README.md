# 🤖 LeRobot Everything - Simulation Environment User Guide

This directory contains simulation environment code for the LeRobot Everything project, supporting simulation control and teleoperation for various robot arms, which is developed based on [ManiSkill](https://github.com/haosulab/ManiSkill).

## 📁 File Structure

```
simulation/
├── mani_skill/              # ManiSkill simulation environment core code
├── teleop_sim.py      # Servo teleoperation simulation system
├── static_robot_viewer.py   # Static robot viewer
├── sin_xarm.py             # XArm simple motion demo
└── README.md               # This user guide
```

## 🚀 Quick Start

### Notes

You can skip this part if you've already installed the requirement with the following command. 
```bash
pip install -r overall_requirement.txt
```

### Requirements

- **Python**: 3.9+
- **Operating System**: Ubuntu 20.04 / 22.04

### Install Dependencies

```bash
# Navigate to simulation directory
cd src/simulation

# Install ManiSkill and related dependencies
pip install --upgrade mani_skill

pip install torch
```

Finally you should need to set up Vulkan with [following instructions](https://maniskill.readthedocs.io/en/latest/user_guide/getting_started/installation.html#vulkan).

For more details about installation, you can refer to the original ManiSkill [documentation](https://maniskill.readthedocs.io/en/latest/user_guide/getting_started/installation.html).

## 📋 Script Descriptions

### 1. `static_robot_viewer.py` - Static Robot Viewer

**Function**: Display specified robot types in simulation environment with various pose settings.

**Supported Robots**:

- `arx-x5`: ARX-X5 6-axis robot arm
- `so100`: SO100 5-axis robot arm
- `xarm6_robotiq`: XArm6 robot arm + Robotiq gripper
- `panda`: Panda 7-axis robot arm
- `x_fetch`: XLeRobot
- `unitree_h1`: Unitree H1 humanoid robot

**Usage**:

```bash
# Basic usage
python static_robot_viewer.py

# Specify robot type
python static_robot_viewer.py --robot panda

# Set specific pose
python static_robot_viewer.py --robot unitree_h1 --pose standing

# Set runtime duration
python static_robot_viewer.py --robot so100 --pose home --duration 10
```

**Parameters**:

- `--robot, -r`: Robot type
- `--scene, -s`: Simulation scene (default: Empty-v1)
- `--pose, -p`: Pose name (default, home, standing, t_pose)
- `--duration, -d`: Runtime duration (seconds)

### 2. `teleop_sim.py` - Servo Teleoperation Simulation System

**Function**: Read servo angles through serial port and control simulated robot arms in real-time.

**Supported Robots**:

- `arx-x5`: ARX-X5 6-axis robot arm
- `so100`: SO-100 5-axis robot arm
- `xarm6_robotiq`: XArm6 robot arm with Robotiq gripper
- `panda`: Panda 7-axis robot arm
- `x_fetch`: XLeRobot
- `unitree_h1`: Unitree H1 humanoid robot(Coming Soon)

**Usage**:

```bash
# Basic usage (SO100 robot)
python teleop_sim.py

# Specify robot type
python teleop_sim.py --robot panda

# Specify serial port
python teleop_sim.py --robot xarm6_robotiq --serial-port /dev/ttyUSB1

# Set control frequency
python teleop_sim.py --robot unitree_h1 --rate 30.0
```

**Parameters**:

- `--robot, -r`: Robot type
- `--scene, -s`: Simulation scene
- `--rate`: Control frequency (Hz)
- `--serial-port`: Serial device path

**Workflow**:

1. Initialize serial connection
2. Calibrate servo zero angles
3. Start angle reading thread
4. Start simulation control thread
5. Real-time mapping of servo angles to robot actions

## 🎮 Usage Examples

### Example 1: View Different Robots

```bash
# View Panda robot arm
python static_robot_viewer.py --robot panda --pose home

# View H1 humanoid robot standing pose
python static_robot_viewer.py --robot unitree_h1 --pose standing

# View Fetch dual-arm robot
python static_robot_viewer.py --robot x_fetch --pose home
```

### Example 2: Teleoperation Control

```bash
# Use SO100 robot for teleoperation
python teleop_sim.py --robot so100 --rate 50.0

# Use Panda robot for teleoperation
python teleop_sim.py --robot panda --serial-port /dev/ttyUSB0
```

### Example 3: Motion Demo

```bash
# Run XArm motion demo
python sin_xarm.py
```

## 🔧 Advanced Configuration

### Custom Robot Mapping

If you meet some problems in joint angle mapping, you can customize the mapping from servo angles to robot actions, in the `convert_pose_to_action` method of `teleop_sim.py`:

```python
def convert_pose_to_action(self, pose: list) -> np.ndarray:
    # Map angles based on robot type
    if self.robot_uids == "your_robot":
        # Custom mapping logic
        action = np.array(pose)
        # Add joint direction adjustments
        action[0] = -action[0]  # Reverse joint 1
        return action
```

## 📚 Related Resources

- [ManiSkill Official Documentation](https://github.com/haosulab/ManiSkill)
- [SAPIEN Simulation Engine](https://sapien.ucsd.edu/)
- [Gymnasium Environment Interface](https://gymnasium.farama.org/)
