# 基础镜像：ROS Noetic (Ubuntu 20.04)
FROM osrf/ros:noetic-desktop-full

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH \
    CATKIN_WS=/root/catkin_ws

# 安装系统依赖和工具
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-dev \
    python3-tk \
    git \
    vim \
    wget \
    curl \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxrender1 \
    libxext6 \
    libzbar0 \
    alsa-utils \
    pulseaudio \
    # ROS 依赖包
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-rospy \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# 升级 pip 并安装基础 Python 包
RUN pip3 install --upgrade pip setuptools wheel && \
    pip3 install tk pyzbar

# 创建 ROS 工作空间目录
WORKDIR $CATKIN_WS
RUN mkdir -p src

# 复制源代码到容器（用于预装依赖）
COPY catkin_ws/src ./src/

# 安装 yolov5_ros 的 Python 依赖
RUN if [ -f "src/yolov5_ros/src/yolov5/requirements.txt" ]; then \
        pip3 install -r src/yolov5_ros/src/yolov5/requirements.txt; \
    fi

# 设置 ROS 环境
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && echo 'ROS Setup Sourced'"

# 配置 bash 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# 创建启动脚本
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/noetic/setup.bash\n\
if [ -f "$CATKIN_WS/devel/setup.bash" ]; then\n\
    source $CATKIN_WS/devel/setup.bash\n\
else\n\
    echo "警告: $CATKIN_WS/devel/setup.bash 不存在，请先编译工作空间"\n\
    echo "运行: cd $CATKIN_WS && catkin_make"\n\
fi\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# 设置工作目录
WORKDIR $CATKIN_WS

# 入口点和默认命令
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
