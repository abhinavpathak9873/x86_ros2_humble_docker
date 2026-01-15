#!/bin/bash

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Starting ROS2 Humble + RealSense Container...${NC}"

# Allow X11 forwarding
xhost +local:docker > /dev/null 2>&1

docker run -it --rm \
    --privileged \
    --network=host \
    --name ros2-realsense \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /etc/udev/rules.d:/etc/udev/rules.d \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/ros2_ws:/ros2_ws \
    --device-cgroup-rule='c 81:* rmw' \
    --device-cgroup-rule='c 189:* rmw' \
    ros2-humble-realsense:latest

echo -e "${GREEN}Container stopped${NC}"