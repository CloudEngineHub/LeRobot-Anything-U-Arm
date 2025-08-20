# 🔧 System Setup

## Prerequisites

- **Ubuntu 20.04**
- **ROS Noetic**
- **Python 3.9+**

---

## Step-by-Step Setup

1. **Install Python Dependencies**

   ```sh
   pip install -r overall_requirements.txt
   ```

2. **Build Catkin Workspace**

   ```sh
   catkin_make
   source devel/setup.bash
   ```

3. **Verify Installation**

   ```sh
   # Test if ROS can find the package
   rospack find uarm
   ```

---

# 🤖 Plug-and-Play with Real Robot

## 1. Start ROS Core

Open a terminal and run:

```sh
roscore
```

## 2. Verify Teleop Arm Output

In a new terminal, check servo readings:

```sh
rosrun uarm servo_zero.py
```

This will display real-time angles from all servos. You should check whether `SERIAL_PORT` is available on your device and modify the variable if necessary. 

## 3. Publish Teleop Data

Still in the second terminal, start the teleop publisher:

```sh
rosrun uarm servo_reader.py
```

Your teleop arm now publishes to the `/servo_angles` topic.

## 4. Control the Follower Arm

Choose your robot and run the corresponding script:

- **For Dobot CR5:**
  ```sh
  rosrun uarm scripts/Follower_Arm/Dobot/servo2Dobot.py
  ```

- **For xArm:**
  ```sh
  rosrun uarm scripts/Follower_Arm/xarm/servo2xarm.py
  ```

---

# 🖥️ Try It Out in Simulation

If you do not have robot hardware, you can try teleoperation in simulation.  
See detailed guidance [here](https://github.com/MINT-SJTU/Lerobot-Everything-Cross-Embodiment-Teleoperation/blob/feat/simulation/src/simulation/README.md).