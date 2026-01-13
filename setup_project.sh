#!/bin/bash
# Setup script for Terrain-Aware Locomotion Project
# This script sets up the complete ROS2 workspace and installs all dependencies

set -e  # Exit on error

echo "=========================================="
echo "Terrain-Aware Locomotion Setup Script"
echo "=========================================="
echo ""

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "22.04" ]; then
        echo "Warning: This project is designed for Ubuntu 22.04. You are running Ubuntu $VERSION_ID"
        echo "Continue anyway? (y/n)"
        read -r response
        if [ "$response" != "y" ]; then
            exit 1
        fi
    fi
fi

# Check if ROS2 Humble is installed
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS2 Humble not found. Installing..."
    
    # Set up sources
    sudo apt update && sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe -y
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS2 Humble
    sudo apt update
    sudo apt install -y ros-humble-desktop
    
    echo "ROS2 Humble installed successfully!"
else
    echo "✓ ROS2 Humble is already installed"
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Install ROS2 dependencies
echo ""
echo "Installing ROS2 dependencies..."
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip

echo "✓ ROS2 dependencies installed"

# Initialize rosdep if not already done
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo ""
    echo "Initializing rosdep..."
    sudo rosdep init
fi
rosdep update
echo "✓ rosdep initialized"

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip3 install --user numpy opencv-python scipy PyYAML transforms3d

# Optional: PyTorch (for terrain classification CNN)
echo ""
echo "Do you want to install PyTorch? (Required for terrain classification) (y/n)"
read -r install_pytorch
if [ "$install_pytorch" = "y" ]; then
    pip3 install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu
    echo "✓ PyTorch installed"
else
    echo "⊘ Skipping PyTorch installation"
fi

echo "✓ Python dependencies installed"

# Set up workspace
WORKSPACE_DIR="$HOME/terrain_locomotion_ws"
echo ""
echo "Setting up workspace at $WORKSPACE_DIR..."

if [ -d "$WORKSPACE_DIR" ]; then
    echo "Workspace directory already exists. Do you want to remove it and start fresh? (y/n)"
    read -r response
    if [ "$response" = "y" ]; then
        rm -rf "$WORKSPACE_DIR"
        echo "Removed existing workspace"
    fi
fi

# Create workspace structure
mkdir -p "$WORKSPACE_DIR/src"

# Copy project files to workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Copying project files from $SCRIPT_DIR to $WORKSPACE_DIR/src..."

# Copy source directories
cp -r "$SCRIPT_DIR/src/terrain_description" "$WORKSPACE_DIR/src/"
cp -r "$SCRIPT_DIR/src/terrain_locomotion" "$WORKSPACE_DIR/src/"

echo "✓ Project files copied"

# Navigate to workspace
cd "$WORKSPACE_DIR"

# Install workspace dependencies
echo ""
echo "Installing workspace dependencies..."
rosdep install --from-paths src --ignore-src -r -y || true
echo "✓ Workspace dependencies installed"

# Build workspace
echo ""
echo "Building workspace..."
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo "✓ Workspace built successfully!"
else
    echo "✗ Build failed. Check the output above for errors."
    exit 1
fi

# Set up environment
echo ""
echo "Setting up environment..."

# Add source command to bashrc if not already present
if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# Terrain-Aware Locomotion Workspace" >> "$HOME/.bashrc"
    echo "source $WORKSPACE_DIR/install/setup.bash" >> "$HOME/.bashrc"
    echo "✓ Added workspace to .bashrc"
else
    echo "✓ Workspace already in .bashrc"
fi

# Create convenience aliases
if ! grep -q "alias terrain_sim=" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# Terrain-Aware Locomotion Aliases" >> "$HOME/.bashrc"
    echo "alias terrain_sim='ros2 launch terrain_locomotion simulation.launch.py'" >> "$HOME/.bashrc"
    echo "alias terrain_walk='ros2 run terrain_locomotion simple_walk_demo'" >> "$HOME/.bashrc"
    echo "✓ Added convenience aliases"
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Workspace location: $WORKSPACE_DIR"
echo ""
echo "To start using the project:"
echo "  1. Open a new terminal (or run: source ~/.bashrc)"
echo "  2. Launch the simulation:"
echo "     ros2 launch terrain_locomotion simulation.launch.py"
echo ""
echo "Or use the convenience alias:"
echo "     terrain_sim"
echo ""
echo "For more information, see SETUP_AND_RUN.md"
echo ""
echo "=========================================="
