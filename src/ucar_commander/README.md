# UCar Commander - 智能小车指挥系统

## 概述

基于状态机的智能小车任务控制系统，支持PID和TEB双模式导航，实现快递配送任务的自动化执行。

## 系统架构

### 模块化设计

系统采用模块化设计，分为以下几个核心模块：

```
src/ucar_commander/scripts/
├── ucar_commander.py          # 主控制节点
├── state_machine.py           # 状态机模块
├── navigation_controller.py   # 导航控制模块
├── qr_controller.py           # QR码识别模块
├── voice_controller.py        # 语音播报模块
└── waypoint_manager.py        # 路径点管理模块
```

### 核心模块说明

#### 1. state_machine.py - 状态机模块

管理任务状态转换，包含以下状态：

- `IDLE` - 等待开始
- `NAV_TO_INTERSECTION_1` - 导航到第一个路口（直线）
- `NAV_TO_PICKUP` - 导航到取货点（直线）
- `QR_SCAN_CARGO` - 扫描货物QR码
- `VOICE_CARGO_PICKED` - 播报已取货
- `NAV_TO_INTERSECTION_2` - 导航到第二个路口（直线）
- `ROTATE_90` - 旋转90度
- `NAV_TO_QR_UP` - 导航到上方QR识别区（障碍物段）
- `QR_SCAN_STATION` - 扫描快递站地址QR码
- `NAV_TO_INTERSECTION_3` - 导航到第三个路口（障碍物段）
- `NAV_TO_QR_DOWN` - 导航到下方QR识别区（直线）
- `QR_SCAN_LOCKER` - 扫描快递柜号码QR码
- `CALCULATE_TARGET` - 计算目标快递柜
- `NAV_TO_INTERSECTION_4` - 导航到第四个路口（直线）
- `NAV_TO_CABINET` - 导航到目标快递柜（直线）
- `VOICE_DELIVERED` - 播报已送达
- `NAV_TO_END` - 导航到终点（直线）
- `VOICE_COMPLETED` - 播报任务完成
- `FINISHED` - 任务完成

#### 2. navigation_controller.py - 导航控制模块

支持两种导航模式：

**PID控制模式（直线路段）**
- 基于位置的PID速度控制
- 适用于无障碍物的直线路段
- 更精确的位置控制

**TEB Planner模式（障碍物路段）**
- 使用move_base的TEB局部规划器
- 适用于有障碍物的复杂路段
- 自动避障和路径优化

#### 3. qr_controller.py - QR码识别模块

负责QR码的扫描和识别：
- 货物QR码识别（映射到1-3）
- 快递站地址QR码识别
- 快递柜号码QR码识别
- 支持自动重试机制

#### 4. voice_controller.py - 语音播报模块

提供语音播报功能：
- 播报已取货："我已取到XX"
- 播报已送达："货物已送达至X号柜 请注意查收"
- 播报任务完成："我已完成快递运输 请给个五星好评欧"
- 等待播报完成后才继续任务

#### 5. waypoint_manager.py - 路径点管理模块

管理所有路径点及其导航模式：
- 从参数服务器加载路径点
- 为每个路径点指定导航模式（PID/TEB）
- 支持动态添加和更新路径点

## 任务流程

1. **启动计时器** - 开始任务计时
2. **移动至第一个路口** - 使用PID控制（直线）
3. **移动到取货点** - 使用PID控制（直线）
4. **QR识别货物** - 识别货物类型
5. **语音播报** - "我已取到XX"
6. **移动到第二个路口** - 使用PID控制（直线）
7. **旋转90度** - 调整方向
8. **移动到上方QR识别区** - 使用TEB控制（障碍物段）
9. **QR识别快递站** - 获取Q1值
10. **移动到第三个路口** - 使用TEB控制（障碍物段）
11. **移动到下方QR识别区** - 使用PID控制（直线）
12. **QR识别快递柜** - 获取Q2值
13. **计算目标柜** - number = (Q1 + Q2) % 3 + 1
14. **移动到第四个路口** - 使用PID控制（直线）
15. **移动到目标快递柜** - 使用PID控制（直线）
16. **语音播报** - "货物已送达至X号柜 请注意查收"
17. **移动到终点** - 使用PID控制（直线）
18. **停止计时器** - 结束计时
19. **语音播报** - "我已完成快递运输 请给个五星好评欧"

## 使用方法

### 1. 配置路径点

在launch文件中配置路径点参数（可选，使用默认值）：

```xml
<node name="ucar_commander" pkg="ucar_commander" type="ucar_commander.py" output="screen">
    <param name="intersection_1" value="[0.5, -1.0, 0.0]"/>
    <param name="pickup" value="[1.64, -1.69, -1.32]"/>
    <param name="intersection_2" value="[2.0, -1.5, 0.0]"/>
    <param name="qr_station_up" value="[3.33866, -1.78492, 0.223]"/>
    <param name="intersection_3" value="[3.0, -0.5, 1.57]"/>
    <param name="qr_station_down" value="[3.14618, 0.52428, 1.808]"/>
    <param name="intersection_4" value="[2.5, 0.5, 3.14]"/>
    <param name="station_A" value="[2.55107, 0.60859, 0.25]"/>
    <param name="station_B" value="[2.14873, 0.51149, 0.25]"/>
    <param name="station_C" value="[1.63901, 0.38807, 0.25]"/>
    <param name="end" value="[0.1, 0.15, 0.25]"/>
</node>
```

### 2. 启动节点

```bash
roslaunch ucar_commander ucar_commander.launch
```

或直接运行：

```bash
rosrun ucar_commander ucar_commander.py
```

### 3. 启动任务

```bash
rosservice call /ucar_commander/start_mission true
```

## 导航模式配置

路径点的导航模式在 `waypoint_manager.py` 中配置：

```python
# 直线路段使用PID控制
'pickup': Waypoint('pickup', *pickup_data, use_pid=True)

# 障碍物路段使用TEB控制
'qr_station_up': Waypoint('qr_station_up', *qr_station_up_data, use_pid=False)
```

## 参数调整

### PID控制器参数

在 `navigation_controller.py` 中调整：

```python
# 线速度PID参数
self.linear_pid = PIDController(kp=0.5, ki=0.05, kd=0.1, 
                               max_output=0.3, min_output=0.0)

# 角速度PID参数
self.angular_pid = PIDController(kp=1.0, ki=0.0, kd=0.2,
                                max_output=0.5, min_output=0.0)

# 位置和角度容差
self.pid_position_tolerance = 0.05  # 米
self.pid_angle_tolerance = 0.05     # 弧度
```

### QR识别重试参数

在 `qr_controller.py` 中调整：

```python
def scan_qr(self, max_retries=3, retry_delay=1.0):
    # max_retries: 最大重试次数
    # retry_delay: 重试延迟（秒）
```

## 依赖项

- ROS Melodic/Noetic
- move_base
- actionlib
- tf.transformations
- qr_msgs (QR识别服务)
- std_srvs (计时器服务)
- geometry_msgs
- nav_msgs

## 服务接口

### 输入服务

- `/ucar_commander/start_mission` (SetBool) - 启动任务
- `/qr/scan_qr` (qr) - QR码识别
- `/start_stop_service` (SetBool) - 计时器控制

### 话题订阅

- `/odom` (Odometry) - 里程计数据

### 话题发布

- `/cmd_vel` (Twist) - 速度控制命令

## 语音文件

语音文件应放置在 `voice_library/` 目录下：

- `我已取到学习用品.wav`
- `我已取到娱乐用品.wav`
- `我已取到生活用品.wav`
- `货物已送达至A号柜 请注意查收.wav`
- `货物已送达至B号柜 请注意查收.wav`
- `货物已送达至C号柜 请注意查收.wav`
- `我已完成快递运输 请给个五星好评欧.wav`

需要设置 `/voice_root` 参数指向语音文件根目录。

## 故障排除

### 1. 导航失败

- 检查move_base是否正常运行
- 检查地图和定位是否准确
- 调整PID参数或切换导航模式

### 2. QR识别失败

- 检查QR识别服务是否启动
- 调整重试次数和延迟
- 确保小车位置正确

### 3. 语音播报失败

- 检查 `/voice_root` 参数是否设置
- 确认语音文件存在
- 检查 `aplay` 命令是否可用

## 开发者信息

- 版本：2.0
- 日期：2025/11/07
- 特性：模块化设计、状态机控制、双模式导航

## 许可证

本项目遵循原有项目的许可证。
