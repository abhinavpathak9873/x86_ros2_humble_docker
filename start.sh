#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}Starting ROS2 Humble Container...${NC}"

# Allow X server connections
xhost +local:docker > /dev/null 2>&1

# Start the container
docker-compose up -d

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Container started successfully!${NC}"
    echo ""
    echo -e "${YELLOW}To enter the container, run:${NC}"
    echo "  docker-compose exec ros2_humble bash"
    echo ""
    echo -e "${YELLOW}Or simply run:${NC}"
    echo "  ./shell.sh"
else
    echo -e "${RED}Failed to start container${NC}"
    exit 1
fi