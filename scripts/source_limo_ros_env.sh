#!/usr/bin/env bash
# Source this file before running Limo Gazebo/Nav2/VLFM online checks.

set -e

source /opt/ros/jazzy/setup.bash
source /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws/install/setup.bash
source /home/asong/venvs/vlfm_ros312/bin/activate

export PYTHONPATH="/home/asong/vlfm:${PYTHONPATH:-}"
