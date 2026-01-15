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
