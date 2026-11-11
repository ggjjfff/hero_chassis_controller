# hero_chassis_controller

麦克纳姆轮底盘控制器，用于在 Gazebo 中控制英雄机器人底盘。

对应「第三课控制组作业」的选做需求：

| 需求 | 状态 |
|------|------|
| 1. 创建新的 ros_package | ✅ |
| 2. 完善的版本管理 | 进行中 |
| 3. 正确的代码规范（ROS C++ Style + roslint + clang-format） | ✅ |
| 4. 使用 PID 控制轮子的速度（四个 PID，control_toolbox） | ✅ |
| 5. 使用逆运动学计算各轮期望速度（订阅 `/cmd_vel`） | ✅ |
| 6. 使用正运动学实现里程计（`/odom` + tf） | ✅ |
| 7. 使用 tf 实现世界坐标下的速度控制 | ✅ |
| 8. 其他特色功能 | 待定 |

---

## 1. 它做什么

```
/cmd_vel ──► 坐标变换（可选）──► 逆运动学 ──► 四个速度 PID ──► 力矩
   │            tf: world→base      │              │
   │                                │              ▼
   │                                │        setCommand()
   │                                │              │
   │                                │        Gazebo 轮子
   │                                │              │
   │                                └── 正运动学 ◄─┴── /joint_states
   │                                        │
   └────────────────────────────────────► 里程计积分 ──► /odom + tf(odom→base_link)
```

一个控制周期（`update()`）做四件事：

1. 从 `RealtimeBuffer` 取最新的 `/cmd_vel`
2. 逆运动学：`(vx, vy, wz)` → 四个轮子的期望角速度
3. 每个轮子一个 PID：速度误差 → 力矩
4. 用实测轮速做正运动学，积分出位姿，发布 `/odom`

---

## 2. 目录结构

```
hero_chassis_controller/
├── CMakeLists.txt
├── package.xml
├── hero_chassis_controller_plugins.xml   # pluginlib 描述
├── .clang-format                          # ROS C++ Style，列宽 120
├── config/
│   └── hero_chassis_controller.yaml       # 运动学参数、PID、模式开关
├── include/hero_chassis_controller/
│   ├── hero_chassis_controller.h          # 控制器主体
│   ├── mecanum_kinematics.h               # 纯数学，不依赖 ROS
│   └── chassis_odometry.h                 # 里程计
├── src/
│   ├── hero_chassis_controller.cpp
│   ├── mecanum_kinematics.cpp
│   └── chassis_odometry.cpp
├── launch/
│   ├── run_simulation_and_controller.launch   # Gazebo + 控制器
│   └── load_controller.launch                 # 只加载控制器
├── test/
│   └── test_kinematics.cpp                # 运动学单元测试（gtest）
└── doc/
    └── derivation.md                      # 运动学推导，答辩用
```

**为什么把运动学单独拆成一个库**：公式错和符号错在 Gazebo 里表现完全一样
（车乱走），只能靠反复试。拆出来之后单元测试 1 秒就能判断公式对不对，
剩下的问题就只可能出在 PID 或符号上，排查范围立刻小一半。

---

## 3. 依赖

```bash
sudo apt install ros-noetic-control-toolbox ros-noetic-tf2-ros \
                 ros-noetic-tf2-geometry-msgs ros-noetic-roslint
```

其余（`roscpp` / `controller_interface` / `hardware_interface` / `pluginlib` /
`realtime_tools` / `geometry_msgs` / `nav_msgs` / `urdf`）随 `ros-noetic-ros-control`
和 `ros-noetic-ros-base` 一起装。

---

## 4. 编译

在工作区根目录（**用 `catkin build`，不要用 `catkin_make`**，作业要求）：

```bash
cd ~/hero_ws
catkin build hero_chassis_controller
source devel/setup.bash
```

只编这一个包，不带上别人：

```bash
catkin build hero_chassis_controller --no-deps
```

---

## 5. 运行

### 一次起全套（Gazebo + 机器人 + 控制器）

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch
```

### 发速度指令

新开一个终端：

```bash
# 前进 0.3 m/s
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

或者用键盘遥控（更直观，也是选做 8 的一个选项）：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 横移和自转

```bash
# 向左横移
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.2}}"
# 原地逆时针
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"
```

`Ctrl+C` 之后控制器会保持最后一次指令。想让它停下，发一条全零：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"
```

### 看里程计

```bash
rostopic echo /odom
```

`pose.pose.position` 是积分出来的位置，`twist.twist` 是正运动学算出的当前速度。
注意 `/odom` 会随轮子打滑而漂移，这是轮式里程计的固有缺陷，不是 bug。

### 只重装控制器（Gazebo 不用重启）

改完代码重新编译后：

```bash
roslaunch hero_chassis_controller load_controller.launch
```

如果提示控制器已存在，先停再起：

```bash
rosservice call /controller_manager/switch_controller \
  "{start_controllers: [], stop_controllers: ['hero_chassis_controller'], strictness: 2}"
```

---

## 6. 参数

全部在 `config/hero_chassis_controller.yaml`，参数树挂在 `/hero_chassis_controller/` 下。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `wheel_radius` | 0.07625 | 轮半径，m |
| `wheel_base` | 0.4 | 前后轴距，m |
| `wheel_track` | 0.4 | 左右轮距，m |
| `max_linear_velocity` | 2.0 | 线速度上限，m/s |
| `max_angular_velocity` | 3.0 | 角速度上限，rad/s |
| `effort_limit` | 5.0 | 力矩上限，N·m，与 urdf 的 `max_effort` 一致 |
| `velocity_mode` | `chassis` | `chassis` 或 `world`，见下 |
| `publish_tf` | `true` | 是否发布 `odom→base_link` |
| `<轮子>_pid` | — | 每个轮子独立的 `p/i/d/i_clamp_min/i_clamp_max` |

`K = (wheel_base + wheel_track) / 2` 由代码算，不用手填。

### velocity_mode 两种模式

- **`chassis`**（默认）：`/cmd_vel` 直接当车体系速度。你发 `y: 0.2`，
  车就永远朝自己的左侧走，不管车头转到哪。
- **`world`**：`/cmd_vel` 当世界系（`odom`）速度。你发 `x: 0.3`，
  车会朝世界坐标的 +x 方向走，哪怕车头正对着 +y。
  内部用 `tf` 查 `odom→base_link`，把速度向量转到车体系再执行。

切换只需要改 yaml 重启控制器，不用重编译。

---

## 7. 调 PID

四个轮子各有独立 PID，参数在 yaml 里。调参**不要**靠改文件重启，用 `rqt_reconfigure`：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

左侧会看到四组 `/hero_chassis_controller/<轮子>_pid/pid_parameters`，
拖动 `p` / `i` / `d` 滑块，实时生效。

配合 `rqt_plot` 看曲线：

```bash
# 期望速度 vs 实测速度。两条线贴合 = 跟得上
rosrun rqt_plot rqt_plot /joint_states/velocity[0]
```

更好的做法是用 `rqt_plot` 画自定义表达式，但 `joint_states` 数组下标在
`rqt_plot` 里不太好用。简化方案是直接看 `/odom.twist.twist.linear.x`
和你发出的指令是否一致。

### 建议的调整顺序

1. **只留 P**（`i = 0`, `d = 0`），从小到大加，直到响应变快但还没抖
2. **加一点 I**，消掉稳态误差（速度跟不上的那一截）
3. **D 一般不加**：速度环的测量噪声会被放大，除非明显超调

一组可用的起点（yaml 里的默认值）：

```yaml
p: 0.5
i: 1.0
d: 0.0
i_clamp_min: -2.0
i_clamp_max: 2.0
```

调出满意的值之后，**写回 yaml**，否则下次重启就丢了。

---

## 8. 单元测试

```bash
cd ~/hero_ws
catkin build hero_chassis_controller --no-deps
./devel/lib/hero_chassis_controller/test_mecanum_kinematics
```

覆盖的内容：

- `K = (wheel_base + wheel_track) / 2`
- 零指令 → 零转速
- 纯前进 / 纯后退 / 横移 / 自转的轮子方向
- **正逆运动学严格互逆**（这是里程计准不准的前提）
- 单位 sanity check（1 m/s → 125 rpm）
- 空指针不崩

这些断言的价值在于「定位」：如果车在仿真里乱走，而单元测试全绿，
那就一定不是公式问题，直接去看 PID 和符号。反过来，如果测试红了，
就不用怀疑 PID 了。

---

## 9. 已知限制

- **实时性**：`update()` 里发布 `nav_msgs/Odometry` 会产生内存分配，
  严格来说违反实时约束。正经做法是用 `realtime_tools::RealtimePublisher`
  在非实时线程发。学生的仿真作业里这点开销可以接受，但要知道这是个妥协。
- **打滑**：轮式里程计无法感知打滑，`/odom` 会漂。仿真里麦轮横移时打滑尤其明显。
- **tf**：`odom→base_link` 的 `z` 固定为 0。轮子里程计推不出高度，
  这和绝大多数轮式机器人的做法一致。要看真实高度得用 Gazebo 的 `/gazebo/model_states`。
- **`velocity_mode: world` 依赖 tf 可用性**：启动最初几帧可能查不到变换，
  此时控制器会「停在原地」并打印一条限频警告，而不是用错误方向乱冲。

---

## 10. 参考

- [ros_control wiki](https://github.com/ros-controls/ros_control/wiki)
- [control_toolbox](http://wiki.ros.org/control_toolbox)
- [effort_controllers/joint_velocity_controller](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/effort_controllers/src/joint_velocity_controller.cpp)
- [Mecanum wheel (Wikipedia)](https://en.wikipedia.org/wiki/Mecanum_wheel)
- 运动学推导细节见 [`doc/derivation.md`](doc/derivation.md)
