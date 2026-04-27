#!/bin/bash
set -e

read -p "Container name: " CONTAINER_NAME
read -p "Workspace name: " WORKSPACE_NAME

export CONTAINER_NAME WORKSPACE_NAME

mkdir -p "$WORKSPACE_NAME"

echo "Building image: $CONTAINER_NAME:latest"
docker compose build --build-arg WS_NAME="$WORKSPACE_NAME" --build-arg CONTAINER_NAME="$CONTAINER_NAME"

echo "Starting container..."
docker compose up -d

echo "Done! Container: $CONTAINER_NAME-dev"
echo "Run: docker exec -it $CONTAINER_NAME-dev bash"