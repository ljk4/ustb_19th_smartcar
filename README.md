# ustb_19th_smartcar - 智能小车项目

## 项目介绍

本项目是北京科技大学第十九届智能车竞赛ROS组的完整解决方案，赛题内容见19thROS组决赛赛题.pdf，基于ROS Noetic开发。项目实现了以下核心功能：

- **自主导航**：基于Cartographer建图和move_base的自主导航
- **目标检测**：集成YOLOv5进行实时物体识别
- **二维码识别**：自动识别和处理二维码信息
- **语音播报**：支持语音反馈
- **任务调度**：完整的状态机控制逻辑，实现多任务协调执行
- **仿真环境**：基于Gazebo的高保真仿真环境

项目包含多个ROS包：
- `detection_msgs`：YOLOv5目标与主任务通讯消息定义
- `final_task`：主任务控制包，协调各个子系统
- `gazebo_pkg`：Gazebo仿真环境和模型
- `image_task`：第二次分站赛图像实现包，规则见第二次分站赛图像细则.pdf
- `mecanum_sim`：小车模型定义包
- `qr`：二维码识别模块
- `qr_msgs`：二维码通讯消息定义
- `ucar_accumtimer`：计时器包
- `ucar_bringup`：建图及测试包（待完善）
- `ucar_navigation`：导航相关配置和参数
- `usb_cam`：usb相机调用包，第二次分站赛使用
- `yolov5_ros`：YOLOv5目标检测集成

其他文件：`.gazebo`（项目所需模型文件）、`cartographer_ros`（建图包，也可使用gmapping等建图）

## 前提要求

- **操作系统**：Ubuntu 20.04（推荐）或 docker部署
- **ROS版本**：ROS Noetic
- 已配置深度学习环境

## 使用方法

### 克隆项目
```
https://github.com/ljk4/ustb_19th_smartcar.git
```

### 本地部署

#### 构建项目
```bash
cd ustb_19th_smartcar/

# 构建项目
cd catkin_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make

# 设置环境变量
source devel/setup.bash
```
也可将其写入.bashrc文件。

#### 运行项目
```bash
# 启动完整任务
roslaunch final_task fin.launch
```

新建终端，添加随机的二维码和图像：
```bash
source devel/setup.bash
cd src/gazebo_pkg/scripts/
python3 random_board.py
```

新建终端，启动任务：
```bash
source devel/setup.bash
rosservice call /ucar_commander/start_mission true
```

小车开始自动导航，并依照决策树进行各自任务。

### Docker一键部署

注意：本run_docker.sh仅在WSL2上进行过测试运行，需提前在本地配好音频播放、显卡环境和NVIDIA Container Toolkit等，可以参考这篇文章：https://ljk4.github.io/posts/安装并配置wsl与docker/

其他环境运行docker需进行相应的配置和修改，无法直接运行。

#### 构建Docker镜像
```bash
# 在项目根目录下构建镜像
docker build -t smartcar:noetic .
# 运行启动脚本
chmod +x run_docker.sh
./run_docker.sh
```

该命令会挂载项目文件并进入docker终端，随后需手动构建项目：
```bash
catkin_make
source devel/setup.bash
roslaunch final_task fin.launch
```

新开终端需要：运行
```bash
docker exec -it smartcar_dev /bin/bash
```

再运行：
```bash
source devel/setup.bash
cd src/gazebo_pkg/scripts/
python3 random_board.py
```
或
```bash
source devel/setup.bash
rosservice call /ucar_commander/start_mission true
```

## Cartographer ROS安装说明

本项目可以使用Cartographer ROS进行SLAM建图，但非必须。如需安装Cartographer ROS，请参考官方文档：

[Cartographer ROS Compilation Guide](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)
