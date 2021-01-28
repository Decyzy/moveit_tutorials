#!/bin/sh

# Install rosdoc_lite if it isn't there yet
test -x `which rosdoc_lite` || sudo apt install ros-$ROS_DISTRO-rosdoc-lite

# Setup Environment
rm -rf docs

# Build
rosdoc_lite -o docs .

# Run
# xdg-open ./build/html/index.html
