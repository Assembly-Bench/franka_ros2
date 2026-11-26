# GELLO 遥操作 FR3 与 Franka Hand

GELLO 以 **30 Hz** 发布七轴目标角到 `/gello/joint_states`。`gello_software` 的 `joint_impedance_controller` 在控制电脑上以 **1000 Hz** 读取实测关节角和速度，计算七轴力矩，经 `franka_hardware` 发给 FR3。

## 首次校准（只连接 GELLO）

首次使用、更换舵机或重新装配关节后，先校准这台 GELLO。已有配置中的偏置和扳机端点来自原设备。采样时保持 FR3 控制器和夹爪客户端停止，并退出占用 GELLO 串口的发布器。单臂舵机 ID 1–7 对应 J1–J7，ID 8 对应扳机。

**1. 确认串口和姿态。** 在同一终端执行以下命令，从列表中粘贴 GELLO 的完整路径。`gello_baud` 填舵机**当前**波特率：出厂通常为 `57600`，已切换过的设备填 `115200`。校准工具只尝试指定速率。

```bash
cd ~/assemble_ws
source gello_software/ros2/env_fr3_gazebo.bash
ls -1 /dev/serial/by-id/*
gello_baud=57600  # 出厂值；已切换到 115200 的设备在此填 115200
read -r -p '粘贴上面列出的 GELLO 串口完整路径：' gello_port
```

按[单臂参考姿态图](https://github.com/Assembly-Bench/gello_software/blob/main/imgs/franka_gello_single_v05_annotated.jpg)摆放并托住 GELLO。J1–J7 参考角为 **[0°, 0°, 0°, −90°, 0°, +90°, 0°]**。下方七个方向符号对应本工作区当前装配，换装后须逐轴核对。

**2. 实测两次。** 保持七轴姿态，先**完全松开扳机**，执行：

```bash
read_gello_pose() {
  python gello_software/ros2/src/franka_gello_state_publisher/scripts/get_offsets.py \
    --port "$gello_port" --baudrate "$gello_baud" \
    --start-joints 0 0 0 -1.571 0 1.571 0 --joint-signs 1 1 1 1 1 -1 1
}
read_gello_pose
```

保留这次的 `assembly_offsets`，记下 `gripper_range_rad` 的**第二个数**作为全开角。然后**完全压下扳机**，在同一终端再次执行 `read_gello_pose`，取第二个数作为全闭角。工具打印的第一个数是按固定行程估算的闭合角。

**3. 写入并重建。** 在 `~/assemble_ws` 下编辑 `gello_software/ros2/src/franka_gello_state_publisher/config/gello_gripper_calibrated.yaml` 的 `SINGLE` 段。[配置文件](https://github.com/Assembly-Bench/gello_software/blob/main/ros2/src/franka_gello_state_publisher/config/gello_gripper_calibrated.yaml)

| 字段 | 填写要求 |
|---|---|
| `com_port` | 填串口文件名，省略 `/dev/serial/by-id/` 前缀。 |
| `joint_signs` | 填本次 `--joint-signs` 使用的 7 个符号，按 J1–J7 排列。 |
| `assembly_offsets` | 填全开采样得到的 7 个关节偏置（rad）。符号为 `+1` 的轴原样填写；符号为 `-1` 的轴（当前为 J6），工具输出 `1.571` 时填 `4.712`，输出 `4.712` 时填 `1.571`，`0` 和 `3.142` 原样填写。 |
| `gripper_range_rad` | 按 **[实测全闭角, 实测全开角]** 填两个不同的弧度值，顺序按动作。 |
| 运行配置 | `num_arm_joints: 7`、`gripper: true`、`baudrate: 115200`、`publishing_rate: 30`，`dynamixel_torque_enable` 八项均为 `0`。 |

```bash
(cd gello_software/ros2 && python -m colcon build --packages-select franka_gello_state_publisher)
```

**4. 验收后再接管真机。** 先只启动下文的**终端 1（GELLO 发布器）**。发布器首次把整条舵机总线切换到 `115200` 后，后续重测偏置也要在校准终端设置 `gello_baud=115200`。另开一个已加载相同环境的终端，逐条运行以下命令，按 `Ctrl+C` 结束当前观察：

```bash
ros2 topic echo /gello/joint_states --field position
ros2 topic echo /gripper/gripper_client/target_gripper_width_percent --field data
ros2 topic hz /gello/joint_states
```

- **机械臂**：参考姿态应读到接近 `[0, 0, 0, -1.571, 0, 1.571, 0]` rad。逐轴沿 FR3 对应关节的正方向小幅转动，确认 J1–J7 中对应的那一项增大。若方向相反，停止发布器，修正该轴的 `--joint-signs`，按当前串口波特率重做步骤 2–4。
- **扳机与频率**：扳机从全闭到全开，读数应从接近 `0` 连续增大至接近 `1`，覆盖抓取阈值 `0.10` 和释放阈值 `0.25`。关节话题频率应约为 `30 Hz`。

全部通过后，保持终端 1 的发布器运行，在 Franka Desk 启用 FCI，再按下节启动终端 2、3。

## 启动真机（3 个终端）

连接 GELLO，在 Franka Desk 启用 FCI。关闭同一 ROS 域里的 Gazebo，确保只有一个节点发布 `/gello/joint_states`。**每个终端先执行**：

```bash
cd ~/assemble_ws
source gello_software/ros2/env_fr3_gazebo.bash
```

**终端 1 · 启动 GELLO。** 等日志显示 `30 Hz` 和波特率 `115200`。首次连接波特率不同的舵机时，驱动会更新其 EEPROM 设置。

```bash
ros2 launch franka_gello_state_publisher main.launch.py config_file:=gello_gripper_calibrated.yaml
```

**终端 2 · 启动 FR3 和夹爪驱动。**

```bash
ros2 launch franka_bringup franka.launch.py robot_type:=fr3 robot_ip:=172.16.0.2 load_gripper:=true
```

**终端 3 · 激活跟随和夹爪。** 先将 GELLO 摆到接近 FR3 当前的姿态，保持手臂不动、扳机松开，留出机械臂和手指的活动空间。执行后，FR3 会主动向第一帧目标移动，夹爪会自动回零。

```bash
cfg="$(ros2 pkg prefix --share franka_fr3_arm_controllers)/config"
ros2 topic echo /gello/joint_states --once --field position && \
ros2 run controller_manager spawner joint_impedance_controller \
  -t franka_fr3_arm_controllers/JointImpedanceController \
  -p "$cfg/gello_hand_controller.yaml" -p "$cfg/controllers.yaml" \
  --controller-manager-timeout 60 && \
ros2 run franka_gripper_manager franka_gripper_client
```

夹爪客户端订阅 `/gripper/gripper_client/target_gripper_width_percent`。扳机开度 **≤10%** 时抓取，**≥25%** 时全开，中间区间保持上一指令。

## 停机与状态查看

**先停控制器，再停 GELLO。** 正常跟随时，约 `0.5 s` 收不到有效目标就会触发超时退出。先在终端 3 按 `Ctrl+C` 退出夹爪客户端，再执行：

```bash
ros2 control switch_controllers --deactivate joint_impedance_controller --strict
```

停用成功后，在终端 2、终端 1 依次按 `Ctrl+C`。若已经发生断流退出，恢复 GELLO 发布后重新启动驱动和控制器。

查看跟随效果时，对比 GELLO 目标 `/gello/joint_states` 与真机实测 `/franka_robot_state_broadcaster/measured_joint_states` 的 `position`（rad）。同一状态节点的 `desired_joint_states` 是 Franka 自报的期望关节状态；`current_pose` 是基座坐标系下的当前末端位姿。

## 改用 Gazebo

沿用上述终端环境和终端 1。**终端 2** 启动仿真场景：

```bash
ros2 launch franka_gazebo_bringup fr3_hand_sim.launch.py
```

看到 `gello_finger_controller` 激活后，在**终端 3** 从重力补偿切换到 GELLO 关节阻抗控制。场景自带夹爪桥，两指按扳机开度连续开合。

```bash
cfg="$(ros2 pkg prefix --share franka_fr3_arm_controllers)/config"
ros2 run controller_manager spawner joint_impedance_controller --inactive \
  -t franka_fr3_arm_controllers/JointImpedanceController \
  -p "$cfg/gello_hand_controller.yaml" --controller-manager-timeout 60 && \
ros2 control switch_controllers --deactivate gravity_compensation_example_controller \
  --activate joint_impedance_controller --strict
```
