# 1. 基础镜像：ROS Noetic (Ubuntu 20.04)
FROM osrf/ros:noetic-desktop-full

# 2. 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
# 设置 Python 环境变量，优先使用 python3
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
ENV CATKIN_WS=/root/catkin_ws

# 3. 安装系统依赖
# 安装必要的工具和库，包括 Gazebo 运行时库和 Python 包管理工具
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-dev \
    git \
    vim \
    wget \
    curl \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxrender1 \
    libxext6 \
    # 语音播放依赖 (aplay)
    alsa-utils \
    # ROS 依赖包
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    && rm -rf /var/lib/apt/lists/*

# 4. 配置 Python 环境
# 升级 pip 并安装基础库
RUN pip3 install --upgrade pip
RUN pip3 install matplotlib opencv-python requests tqdm pyyaml tensorboard


# 5. 安装 PyTorch (GPU 版) - 针对 CUDA 11.8 (适配 Noetic)
# 如果构建太慢，可以注释掉这行，改用下面的预装命令，或者提前把 pip 源换成国内源
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# 其他包，计时用
RUN apt-get update
RUN apt-get install python3-tk libzbar0 alsa-utils pulseaudio -y
RUN pip install tk pyzbar pandas seaborn
# 6. 复制并安装 YOLOv5_ROS 包
# 我们假设你的 yolov5_ros 包在项目根目录下
# 如果你的项目里已经有 yolov5_ros，这一步主要是为了安装它的依赖
# 如果没有，你需要把 ultralytics/yolov5 的代码放进来，或者确保它在 catkin_ws/src 里
# 这里我们直接在 catkin build 时处理

# 7. 准备 ROS 工作空间
WORKDIR $CATKIN_WS

# 8. 复制你的源代码
# 注意：我们稍后在 docker run 时会挂载代码，但这里为了预装依赖，先复制一个骨架（如果需要）
# 实际上，我们主要依赖挂载卷，所以这里可以留空，或者在构建时传入
# 为了通用性，我们只创建目录
RUN mkdir -p src

# 9. 复制你的 catkin_ws/src 目录下的所有内容到容器
# 如果你把 src 目录放在了和 Dockerfile 同一级的 catkin_ws/src 下，请使用以下命令：
# COPY ./catkin_ws/src ./src/
# 但是为了灵活性（允许你在外面修改代码），我们通常不 COPY，而是 RUN 时挂载。
# 所以这里我们只做环境配置。

# 10. 处理 YOLOv5 权重文件 (可选)
# 如果你有自定义的 best.pt，可以放在项目根目录的 weights 文件夹下
# RUN mkdir -p /opt/yolov5/weights
# COPY ./weights/best.pt /opt/yolov5/weights/best.pt

# 11. 安装 yolov5_ros 的 Python 依赖 (如果它在你的 src 里)
# 假设你的 yolov5_ros 包里有 requirements.txt
# RUN if [ -f "src/yolov5_ros/requirements.txt" ]; then pip3 install -r src/yolov5_ros/requirements.txt; fi

# 12. 编译 ROS 工作空间
# 注意：这里我们不运行 catkin_make，因为代码是挂载进来的，编译产物在容器内，退出即消失。
# 我们只做环境配置，编译留给手动运行，或者在运行容器时执行脚本。
# 但是为了 PATH 和环境变量生效，我们 source 一下
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; echo 'ROS Setup Sourced'"

# 13. 设置环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
# 这里不自动 source devel/setup.bash，因为第一次没有 devel 目录
# 我们会在启动脚本里处理

# 14. 创建启动脚本 (推荐方式)
# 创建一个脚本来启动所有服务，避免手动输入命令
RUN echo '#!/bin/bash \n\
source /opt/ros/noetic/setup.bash \n\
if [ -f "$CATKIN_WS/devel/setup.bash" ]; then \n\
  source $CATKIN_WS/devel/setup.bash \n\
else \n\
  echo "请先编译: cd $CATKIN_WS && catkin_make" \n\
fi \n\
exec "$@" \n\
' > /entrypoint.sh \
&& chmod +x /entrypoint.sh

# 15. 使用自定义入口点
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]