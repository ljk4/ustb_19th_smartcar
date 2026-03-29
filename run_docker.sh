#!/bin/bash

IMAGE_NAME="smartcar-nav:latest"
CONTAINER_NAME="smartcar_dev"

echo "正在构建 Docker 镜像..."
docker build -t $IMAGE_NAME .

echo "清理旧容器..."
docker stop $CONTAINER_NAME 2>/dev/null || true
docker rm $CONTAINER_NAME 2>/dev/null || true

echo "启动容器..."

docker run -it --rm \
  --name $CONTAINER_NAME \
  --gpus all \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e "PULSE_SERVER=${PULSE_SERVER}" \
  -v /mnt/wslg/:/mnt/wslg/ \
  --device /dev/snd:/dev/snd \
  -v $(pwd)/catkin_ws:/root/catkin_ws \
  -v $(pwd)/.gazebo:/root/.gazebo \
  $IMAGE_NAME