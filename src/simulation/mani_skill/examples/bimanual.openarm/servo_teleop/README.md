# OpenArm v1.0 双臂 Leader 遥操

本项目在 Windows 11 的 WSL2 Ubuntu 22.04 中运行 ROS 2 Humble，将左右两套 8 通道
leader 舵机的相对零位变化转换为 OpenArm v1.0 标准
`joint_trajectory_controller` 命令。

默认使用 ROS 2 fake hardware 和 RViz，不需要真实 OpenArm 从臂，也不需要 MoveIt。

## 1. 系统结构

```text
左/右 leader（每侧 8 个舵机）
        |
        | CH340 串口，1,000,000 baud
        v
openarm_servo_teleop（20 Hz）
        |
        | joint_trajectory_controller 话题
        v
ROS 2 fake hardware（750 Hz）
        |
        v
joint_states -> robot_state_publisher -> RViz
```

每侧 leader 使用：

- 舵机 `0-6`：机械臂关节 J1-J7
- 舵机 `7`：夹爪
- 启动时先采集 2 秒释放姿态，再自动采集两个扳机的按到底端点
- 左右扳机独立识别按下时角度增大或减小，不需要手工配置夹爪方向
- 默认发送 `PULK`，释放 16 个 leader 舵机的力矩

## 2. 目录结构

请保留完整的 `D:\openarm` 目录，至少需要以下内容：

```text
D:\openarm\
|-- servo_zero.py
|-- servo_teleop\
|   |-- README.md
|   |-- joint_mapping_v1.json
|   `-- run_ros2_teleop_v1.ps1
|-- ros2_ws\
|   `-- src\openarm_servo_teleop\
|-- openarm_ros2\
|   `-- openarm_bringup\
`-- openarm_description\
```

主要文件：

| 文件 | 用途 |
|---|---|
| `servo_teleop/run_ros2_teleop_v1.ps1` | Windows 一键启动 leader 遥操节点 |
| `servo_teleop/joint_mapping_v1.json` | 舵机 ID、正负方向、Home 和最大变化量 |
| `ros2_ws/src/openarm_servo_teleop` | ROS 2 遥操包 |
| `openarm_description` | 官方 v1.0 URDF、网格和 RViz 配置 |
| `openarm_ros2/openarm_bringup` | fake hardware 和控制器 launch |

## 3. 硬件与软件要求

- Windows 11
- WSL2 Ubuntu 22.04，支持 WSLg
- ROS 2 Humble
- `usbipd-win`
- 两个 USB 转串口设备，本项目使用 CH340
- 左右 leader 各 8 个舵机并已上电

测试电脑当前使用：

```text
左 leader：BUSID 3-1，Windows COM14
右 leader：BUSID 3-2，Windows COM15
```

BUSID 在不同电脑或 USB 接口上可能变化，必须以 `usbipd list` 为准。

## 4. 首次安装

已经完成安装的电脑可直接跳到“日常启动”。

### 4.1 安装 WSL2 和 usbipd-win

在管理员 PowerShell 中执行：

```powershell
wsl --install -d Ubuntu-22.04
wsl --update
winget install --exact --id dorssel.usbipd-win
```

重新打开管理员 PowerShell，查找两个 CH340 的 BUSID：

```powershell
usbipd list
```

首次使用需要共享设备：

```powershell
usbipd bind --busid 3-1
usbipd bind --busid 3-2
```

如果出现 `Unknown USB filter 'nxusbf'`，可强制重新绑定：

```powershell
usbipd unbind --busid 3-1
usbipd unbind --busid 3-2
usbipd bind --busid 3-1 --force
usbipd bind --busid 3-2 --force
```

### 4.2 安装 ROS 2 Humble

按 ROS 2 官方文档在 Ubuntu 22.04 中安装 Humble。建议安装 desktop 版本：

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-serial
```

安装 fake hardware、控制器和 RViz 所需依赖：

```bash
sudo apt install -y \
  ros-humble-controller-manager \
  ros-humble-gripper-controllers \
  ros-humble-hardware-interface \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-robot-state-publisher \
  ros-humble-ros2-controllers \
  ros-humble-rviz2 \
  ros-humble-xacro
```

### 4.3 获取官方模型和 bringup

如果收到的是完整的 `D:\openarm` 文件夹，并且这两个目录已存在，则不要重复克隆。

```powershell
git clone https://github.com/enactic/openarm_description.git D:\openarm\openarm_description
git clone https://github.com/enactic/openarm_ros2.git D:\openarm\openarm_ros2
```

fake hardware 不需要构建 `openarm_hardware` 或安装 CAN 依赖。

### 4.4 构建工作区

打开 PowerShell：

```powershell
wsl -d Ubuntu-22.04 -u root
```

进入 WSL 后执行：

```bash
source /opt/ros/humble/setup.bash
cd /mnt/d/openarm/ros2_ws

# Leader 遥操节点
colcon build --symlink-install \
  --base-paths src \
  --packages-select openarm_servo_teleop

# 官方 URDF 和 RViz 资源
colcon build --symlink-install \
  --base-paths /mnt/d/openarm/openarm_description \
  --packages-select openarm_description

source install/setup.bash

# 只构建 fake hardware 所需的 bringup，避免引入 MoveIt 和真机插件
colcon build --symlink-install \
  --base-paths /mnt/d/openarm/openarm_ros2/openarm_bringup \
  --packages-select openarm_bringup

source install/setup.bash
```

验证三个包：

```bash
ros2 pkg prefix openarm_servo_teleop
ros2 pkg prefix openarm_description
ros2 pkg prefix openarm_bringup
```

三条命令都应返回 `/mnt/d/openarm/ros2_ws/install/...`。

## 5. 日常启动

需要两个 PowerShell 窗口。不要启动 MuJoCo 直连遥操。

### 5.1 窗口 A：启动 fake hardware 和 RViz

在 PowerShell 中执行：

```powershell
wsl -d Ubuntu-22.04 -u root -- bash -lc "source /opt/ros/humble/setup.bash && source /mnt/d/openarm/ros2_ws/install/setup.bash && export DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir PULSE_SERVER=unix:/mnt/wslg/PulseServer && ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v1.0 use_fake_hardware:=true robot_controller:=joint_trajectory_controller"
```

正常情况下会弹出 RViz，并启动：

- `controller_manager`
- `joint_state_broadcaster`
- 左右臂 `joint_trajectory_controller`
- 左右夹爪控制器
- `robot_state_publisher`
- RViz2

### 5.2 窗口 B：启动 leader 遥操

1. 连接并上电左右 leader。
2. 将两只 leader 臂放在希望作为遥操零位的姿态。
3. 保持两只 leader 臂静止，并让两个扳机处于完全释放状态。

在 PowerShell 中执行：

```powershell
D:\openarm\servo_teleop\run_ros2_teleop_v1.ps1
```

如果 PowerShell 禁止脚本执行：

```powershell
powershell -ExecutionPolicy Bypass -File D:\openarm\servo_teleop\run_ros2_teleop_v1.ps1
```

如果 BUSID 不是默认的 `3-1` 和 `3-2`：

```powershell
D:\openarm\servo_teleop\run_ros2_teleop_v1.ps1 -LeftBusId 4-1 -RightBusId 4-2
```

正常启动会看到：

```text
Keep both leader arms still and both triggers fully released during zero calibration
Press both triggers fully and hold them still for calibration (请将左右扳机按到底并保持不动，进行校准)
Left trigger calibrated: angle increases/decreases by ... deg
Right trigger calibrated: angle increases/decreases by ... deg
Trigger calibration complete; release the triggers to open the grippers (扳机校准完成，请松开扳机以打开夹爪)
Calibration complete; publishing OpenArm v1 targets (校准完成，正在发布 OpenArm v1 目标)
```

看到第二行提示后，将两个扳机都按到底并稳定保持约 1 秒。左右扳机可以先后达到端点，
程序会分别完成校准。看到 `Trigger calibration complete` 后松开扳机，再缓慢移动 leader；
RViz 中的 OpenArm 双臂和夹爪应同步运动。

## 6. 状态检查

另开一个 PowerShell：

```powershell
wsl -d Ubuntu-22.04 -u root
```

进入 WSL 后：

```bash
source /opt/ros/humble/setup.bash
source /mnt/d/openarm/ros2_ws/install/setup.bash
```

检查遥操状态：

```bash
ros2 topic echo --once /servo_teleop/status
```

正常输出：

```yaml
data: tracking
```

启动过程中可能看到：

```yaml
data: calibrating_released
data: calibrating_pressed
```

检查四条控制链路是否都有发布者和订阅者：

```bash
ros2 topic info /left_joint_trajectory_controller/joint_trajectory
ros2 topic info /right_joint_trajectory_controller/joint_trajectory
ros2 topic info /left_gripper_controller/joint_trajectory
ros2 topic info /right_gripper_controller/joint_trajectory
```

每条都应显示：

```text
Publisher count: 1
Subscription count: 1
```

检查控制器状态：

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
```

以下五个控制器都应为 `active`：

```text
joint_state_broadcaster
left_joint_trajectory_controller
right_joint_trajectory_controller
left_gripper_controller
right_gripper_controller
```

查看一次 fake hardware 关节状态：

```bash
ros2 topic echo --once /joint_states
```

## 7. ROS 2 话题

遥操节点发布：

```text
/left_joint_trajectory_controller/joint_trajectory
/right_joint_trajectory_controller/joint_trajectory
/left_gripper_controller/joint_trajectory
/right_gripper_controller/joint_trajectory
/servo_teleop/target_joint_states
/servo_teleop/status
```

fake hardware 和显示链路额外发布：

```text
/joint_states
/dynamic_joint_states
/robot_description
/tf
/tf_static
```

## 8. 关节映射和正负方向

唯一需要修改的遥操映射文件：

```text
D:\openarm\servo_teleop\joint_mapping_v1.json
```

机械臂关节换算关系：

```text
ROS 目标 = Home + sign * scale * (leader 当前角度 - 启动零位)
```

夹爪使用自动测得的释放端和按到底端：

```text
按压比例 = clamp((当前角度 - 释放角度) / (按到底角度 - 释放角度), 0, 1)
夹爪目标 = open_m + 按压比例 * (closed_m - open_m)
```

分母可以为正数或负数，所以程序会自动识别按下时舵机角度是增大还是减小。

当前默认映射：

| 关节 | 舵机 ID | 左/右 sign | 左/右 URDF 正轴 | 左臂限位 | 右臂限位 | Home | 最大变化 |
|---|---:|---|---|---:|---:|---:|---:|
| J1 | 0 | +1 / +1 | +Z / +Z | -200~80 deg | -80~200 deg | 0 deg | +/-30 deg |
| J2 | 1 | +1 / +1 | -X / -X | -190~10 deg | -10~190 deg | 0 deg | +/-30 deg |
| J3 | 2 | +1 / +1 | +Z / +Z | -90~90 deg | -90~90 deg | 0 deg | +/-30 deg |
| J4 | 3 | +1 / +1 | +Y / +Y | 0~140 deg | 0~140 deg | 90 deg | +/-30 deg |
| J5 | 4 | +1 / +1 | +Z / +Z | -90~90 deg | -90~90 deg | 0 deg | +/-30 deg |
| J6 | 5 | +1 / +1 | +X / +X | -45~45 deg | -45~45 deg | 0 deg | +/-25 deg |
| J7 | 6 | +1 / +1 | -Y / +Y | -90~90 deg | -90~90 deg | 0 deg | +/-30 deg |
| 夹爪 | 7 | 自动 / 自动 | 直线开合 | 0~44 mm | 0~44 mm | 释放 44 mm | 按到底 0 mm |

字段说明：

| 字段 | 含义 |
|---|---|
| `servo_id` | 该 ROS 关节读取的 leader 舵机编号，范围 `0-7` |
| `sign` | `1` 保持角度增量方向，`-1` 反转方向 |
| `scale` | leader 角度变化到 ROS 目标变化的比例 |
| `home_deg` | leader 零位对应的 OpenArm 目标角度 |
| `max_delta_deg` | 相对 Home 允许的最大角度变化 |
| `open_m` | 扳机完全释放时的夹爪开度，默认 `0.044 m` |
| `closed_m` | 扳机按到底时的夹爪开度，默认 `0.0 m` |
| `deadband_ratio` | 释放端死区占实际扳机行程的比例，默认 `0.05` |
| `minimum_travel_deg` | 接受按到底端点所需的最小扳机行程，默认 `10 deg` |
| `hold_seconds` | 按到底后需要稳定保持的时间，默认 `0.75 s` |
| `stability_deg` | 保持期间允许的最大角度波动，默认 `1 deg` |

夹爪中的 `sign`、`m_per_deg`、`home_m` 和 `max_delta_m` 是 MuJoCo v1 直连模式使用的
兼容字段；ROS 2 自动扳机校准不使用这些字段。

映射验证方法：

1. 每次只移动一只 leader 的一个关节。
2. 如果 RViz 中运动了错误关节，交换对应的 `servo_id`。
3. 如果关节正确但方向相反，将该项 `sign` 改为 `-1`。
4. 夹爪方向不需要修改 `sign`，重新执行释放端和按到底端校准即可。
5. 每侧 `servo_id 0-7` 必须各使用一次，不能重复。
6. 修改后停止并重新启动遥操节点，使配置重新加载并重新校零。

URDF 正轴遵循右手定则，并位于各关节的局部坐标系。左右臂在模型中是镜像安装，
因此视觉上的镜像运动不能仅根据轴名称判断，最终以逐关节 RViz 验证为准。

## 9. 停止程序

- 在窗口 B 按 `Ctrl+C`：停止 leader 遥操并释放串口。
- 在窗口 A 按 `Ctrl+C`：停止 fake hardware、控制器和 RViz。
- 必须先停止遥操节点，才能运行其他会读取相同串口的程序。

如果窗口丢失，可在 PowerShell 中停止整个 WSL：

```powershell
wsl --shutdown
```

此命令会停止所有 WSL 发行版中的程序，不要在有其他 WSL 工作时使用。

## 10. 常见问题

### 10.1 `/dev/ttyUSB0` 或 `/dev/ttyUSB1` 不存在

```powershell
usbipd list
```

确认两个设备为 `Shared` 或 `Attached`，并确认传给脚本的 BUSID 正确。必要时拔插设备后重试。

### 10.2 左右 leader 互换

停止遥操后交换 BUSID：

```powershell
D:\openarm\servo_teleop\run_ros2_teleop_v1.ps1 -LeftBusId 3-2 -RightBusId 3-1
```

### 10.3 状态为 `serial_timeout`

表示任一串口超过 `0.5 s` 没有收到完整数据。检查：

- 两个 leader 是否上电
- USB 是否松动
- 是否有其他程序占用 `/dev/ttyUSB0` 或 `/dev/ttyUSB1`
- 是否误启动了 `run_teleop_v1.ps1`

### 10.4 话题只有发布者，没有订阅者

说明遥操节点已经发布，但 fake hardware/controller 没有启动或没有激活。重新执行窗口 A 的 launch，
再检查 `/controller_manager/list_controllers`。

### 10.5 扳机校准超时

如果出现 `Timed out calibrating fully pressed triggers`：

- 等待 `Press both triggers fully` 提示后再按扳机
- 确认左右扳机都按到底，而不是只按一侧
- 按到底后保持约 1 秒，不要立即松开
- 检查实际扳机行程是否大于 `minimum_travel_deg`
- 如果机械行程确实小于 10 度，适当降低对应侧的 `minimum_travel_deg`

### 10.6 RViz 没有弹出

先更新 WSL：

```powershell
wsl --update
wsl --shutdown
```

重新启动时保留窗口 A 命令中的 `DISPLAY`、`WAYLAND_DISPLAY` 和 `XDG_RUNTIME_DIR` 环境变量。

### 10.7 RViz 报手指惯量不合理

官方 v1.0 模型可能输出 finger link inertia 警告。该警告只影响惯量可视化，不影响机器人模型、
fake hardware 或遥操运动。

### 10.8 PowerShell 禁止运行脚本

```powershell
powershell -ExecutionPolicy Bypass -File D:\openarm\servo_teleop\run_ros2_teleop_v1.ps1
```

## 11. 重要限制和安全说明

- `PULK` 会释放 leader 舵机力矩，启动前应扶稳设备，避免因重力突然下落。
- 当前流程只驱动 ROS 2 fake hardware，不会控制真实 OpenArm 从臂。
- MoveIt 不是本流程的必需组件，也没有在默认命令中启动。
- 不要同时运行 `run_teleop_v1.ps1` 和 `run_ros2_teleop_v1.ps1`，两者会争用同一对串口。
- 不要同时启动两套 `openarm.bimanual.launch.py`，否则会重复创建 `controller_manager` 和 RViz。
- 首次映射时使用小角度、逐关节验证；确认 `servo_id` 和 `sign` 后再扩大运动范围。

## 12. 可选的 MuJoCo 直连模式

仓库仍保留 `run_teleop_v1.ps1`，它绕过 ROS 2，直接控制 MuJoCo v1.0 模型。当前推荐使用
ROS 2 fake hardware + RViz 流程。两种模式不能同时运行，因为它们会争用同一对 leader 串口。
