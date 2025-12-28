#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Set the ROS distribution if not already set
ROS_DISTRO=${ROS_DISTRO:-"humble"} # Replace "humble" with your default ROS distro if needed

# Navigate to the ProbablyBartender/src directory
echo "Navigating to ProbablyBartender/src directory..."
cd ..
cd src

# Clone the xarm_ros2 repository
echo "Cloning xarm_ros2 repository..."
#git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO

# Navigate into the xarm_ros2 directory
cd xarm_ros2

# Pull the latest changes
echo "Pulling the latest changes..."
git pull

# Synchronize and update git submodules
echo "Syncing and updating submodules..."
git submodule sync
git submodule update --init --remote

# Navigate back to the parent directory
cd ..

# Update rosdep
echo "Updating rosdep..."
rosdep update

# Install dependencies
echo "Installing dependencies..."
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

echo "Script execution completed successfully."

