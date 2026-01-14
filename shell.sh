#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

# Check if container is running
if ! docker ps | grep -q ros2_humble_container; then
    echo -e "${RED}Container is not running. Starting it now...${NC}"
    ./start.sh
    sleep 2
fi

echo -e "${GREEN}Entering ROS2 Humble Container...${NC}"
docker-compose exec ros2_humble bash