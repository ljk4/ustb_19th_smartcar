# ustb_19th_smartcar - 智能小车项目

## 项目介绍

本项目是北京科技大学第十九届智能车竞赛的完整解决方案，基于ROS（Robot Operating System）开发。项目实现了以下核心功能：

- **自主导航**：基于Cartographer SLAM和move_base的室内自主导航
- **目标检测**：集成YOLOv5进行实时物体识别
- **二维码识别**：自动识别和处理二维码信息
- **语音交互**：支持语音指令和语音反馈
- **任务调度**：完整的状态机控制逻辑，实现多任务协调执行
- **仿真环境**：基于Gazebo的高保真仿真环境

项目包含多个ROS包：
- `final_task`：主任务控制包，协调各个子系统
- `ucar_commander`：小车控制和路径规划
- `yolov5_ros`：YOLOv5目标检测集成
- `qr`：二维码识别模块
- `ucar_navigation`：导航相关配置和参数
- `gazebo_pkg`：Gazebo仿真环境和模型
- `ucar_bringup`：硬件启动和配置

## 系统要求

- **操作系统**：Ubuntu 20.04 (推荐) 或 Ubuntu 22.04
- **ROS版本**：ROS Noetic
- **硬件要求**：建议8GB以上内存，支持GPU加速（可选但推荐）

## 使用方法

### 1. 本地部署

#### 安装依赖
```bash
# 安装ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full

# 初始化rosdep
sudo rosdep init
rosdep update

# 安装其他依赖
sudo apt install python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# 安装Python依赖
pip3 install torch torchvision numpy opencv-python
```

#### 构建项目
```bash
cd /path/to/ustb_19th_smartcar
# 初始化catkin工作空间（如果还没有）
mkdir -p catkin_ws/src
cp -r catkin_ws/src/* catkin_ws/src/

# 构建项目
cd catkin_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make

# 设置环境变量
echo "source ~/smartCAR/ustb_19th_smartcar/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 运行项目
```bash
# 启动完整任务
roslaunch final_task fin.launch

# 或者分别启动各个模块进行调试
roslaunch final_task gaz.launch      # Gazebo仿真
roslaunch final_task nav.launch      # 导航系统
roslaunch yolov5_ros yolov5.launch   # YOLOv5检测
roslaunch qr qr.launch               # 二维码识别
```

### 2. Docker一键部署

#### 构建Docker镜像
```bash
# 在项目根目录下构建镜像
docker build -t smartcar:noetic .

# 或者从Docker Hub拉取预构建镜像（如果有）
# docker pull your-username/smartcar:noetic
source devel/setup.bash
roslaunch final_task fin.launch
rosservice call /ucar_commander/start_mission true
docker exec -it smartcar_dev /bin/bash
cd src/gazebo_pkg/scripts/
python3 random_board.py
```

#### 运行完整项目
```bash
# 一键启动完整项目（包含GUI）
docker run -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  smartcar:noetic \
  roslaunch final_task fin.launch
```

#### 注意事项
- 需要允许Docker容器访问X11显示：`xhost +local:docker`
- 使用`--network=host`以便ROS节点间正常通信
- 如果不需要GUI，可以去掉X11相关的环境变量和挂载

### 3. Docker逐模块调试

#### 启动交互式容器
```bash
# 启动带有完整环境的交互式容器
docker run -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  smartcar:noetic \
  bash
```

#### 在容器内逐个启动模块
```bash
# 在容器内执行以下命令进行模块化调试

# 1. 启动ROS核心（如果需要）
roscore &

# 2. 启动Gazebo仿真环境
roslaunch final_task gaz.launch

# 3. 启动导航系统
roslaunch final_task nav.launch

# 4. 启动YOLOv5目标检测
roslaunch yolov5_ros yolov5.launch

# 5. 启动二维码识别
roslaunch qr qr.launch

# 6. 启动主控制器
rosrun final_task ucar_commander.py
```

#### 调试技巧
- 可以在不同的终端中分别运行不同模块，便于观察日志
- 使用`rostopic list`和`rostopic echo`查看话题信息
- 使用`rosnode list`查看运行的节点
- 使用`rqt_graph`可视化节点和话题关系

## Cartographer ROS安装说明

本项目依赖Cartographer ROS进行SLAM建图和定位。如需安装Cartographer ROS，请参考官方文档：

[Cartographer ROS Compilation Guide](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)

## 项目结构说明

```
ustb_19th_smartcar/
├── catkin_ws/                 # ROS工作空间
│   └── src/                   # 源代码目录
│       ├── final_task/        # 主任务控制包
│       ├── ucar_commander/    # 小车控制包
│       ├── yolov5_ros/        # YOLOv5集成包
│       ├── qr/                # 二维码识别包
│       ├── ucar_navigation/   # 导航配置包
│       ├── gazebo_pkg/        # Gazebo仿真包
│       └── ...                # 其他功能包
├── Dockerfile                 # Docker构建文件
└── README.md                  # 项目说明文档
```

## 常见问题

### Q: 运行时出现权限错误
A: 确保所有文件有正确的读写权限。在Docker中已经处理了权限问题，本地部署时可以运行：
```bash
chmod -R 755 catkin_ws/
```

### Q: YOLOv5模型加载失败
A: 确保weights目录存在且包含正确的模型文件。模型文件通常需要单独下载。

### Q: Gazebo无法启动或显示异常
A: 确保已正确安装显卡驱动，并且X11转发配置正确。在Docker中运行时需要正确设置DISPLAY环境变量。

## 许可证

本项目仅供学习和研究使用。各子模块的许可证请参考对应包中的LICENSE文件。