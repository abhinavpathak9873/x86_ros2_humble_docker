#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}ROS2 Humble Docker Setup Script${NC}"
echo -e "${GREEN}========================================${NC}"

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Docker is not installed. Please install Docker first.${NC}"
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo -e "${RED}Docker Compose is not installed. Please install Docker Compose first.${NC}"
    exit 1
fi

# Detect if NVIDIA GPU is available
echo -e "${YELLOW}Checking for NVIDIA GPU...${NC}"
if command -v nvidia-smi &> /dev/null; then
    echo -e "${GREEN}NVIDIA GPU detected!${NC}"
    nvidia-smi
    
    # Check if nvidia-docker is installed
    if ! docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi &> /dev/null; then
        echo -e "${YELLOW}NVIDIA Docker runtime not properly configured.${NC}"
        echo -e "${YELLOW}Please install nvidia-docker2:${NC}"
        echo "  sudo apt-get install -y nvidia-docker2"
        echo "  sudo systemctl restart docker"
    else
        echo -e "${GREEN}NVIDIA Docker runtime is configured!${NC}"
    fi
else
    echo -e "${YELLOW}No NVIDIA GPU detected. GPU features will be disabled.${NC}"
fi

# Create workspace directory
echo -e "${YELLOW}Creating workspace directory...${NC}"
mkdir -p ros2_ws/src
mkdir -p config

# Create custom .bashrc for container
echo -e "${YELLOW}Creating custom .bashrc...${NC}"
cat > config/.bashrc << 'EOF'
# Source ROS2
source /opt/ros/humble/setup.bash   

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Aliases
alias cb='cd /ros2_ws && colcon build --symlink-install'
alias cs='source /ros2_ws/install/setup.bash'
alias ws='cd /ros2_ws'

# ROS2 helpful aliases
alias rt='ros2 topic list'
alias rn='ros2 node list'
alias rs='ros2 service list'

# Color prompt
PS1='\[\033[01;32m\][ROS2-Container]\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '

echo "ROS2 Humble Environment Ready!"
echo "Workspace: /ros2_ws"
echo ""
echo "Quick commands:"
echo "  cb  - Build workspace (colcon build)"
echo "  cs  - Source workspace"
echo "  ws  - Go to workspace"
echo ""
EOF

# Allow X server connections
echo -e "${YELLOW}Configuring X11 display access...${NC}"
xhost +local:docker > /dev/null 2>&1 || echo -e "${YELLOW}Note: xhost command not available or X server not running${NC}"

# Build Docker image
echo -e "${YELLOW}Building Docker image...${NC}"
docker-compose build

if [ $? -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Docker image built successfully!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${GREEN}To start the container, run:${NC}"
    echo "  docker-compose up -d"
    echo ""
    echo -e "${GREEN}To enter the container, run:${NC}"
    echo "  docker-compose exec ros2_humble bash"
    echo ""
    echo -e "${GREEN}Or use the provided start.sh script${NC}"
else
    echo -e "${RED}Failed to build Docker image${NC}"
    exit 1
fi