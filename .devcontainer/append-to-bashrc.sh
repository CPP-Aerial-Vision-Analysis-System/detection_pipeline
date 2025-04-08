#!/bin/bash

# Check if it's already appended to avoid duplication
if ! grep -q "# >>> Devcontainer ROS setup >>>" ~/.bashrc; then
  cat << 'EOF' >> ~/.bashrc

# >>> Devcontainer ROS setup >>>

if [ -f "$PWD/catkin_ws/devel/setup.bash" ]; then
  source "$PWD/catkin_ws/devel/setup.bash"
fi

export GAZEBO_MODEL_PATH="$HOME/ardupilot_gazebo/models:$PWD/catkin_ws/src/sitl-gazebo/models"
# <<< Devcontainer ROS setup <<<
EOF
fi