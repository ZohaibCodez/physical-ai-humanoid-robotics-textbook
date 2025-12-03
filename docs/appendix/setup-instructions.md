---
sidebar_position: 1
title: Software Installation Guide
---

# Software Installation Guide

Complete step-by-step instructions for setting up your development environment for Physical AI and Humanoid Robotics.

## Prerequisites

Before starting, ensure you have:
- Ubuntu 22.04 LTS installed (dual-boot, VM, or native)
- Stable internet connection
- At least 30GB free disk space
- Admin/sudo access

:::warning System Requirements
ROS 2 Humble requires Ubuntu 22.04 LTS (Jammy Jellyfish). Other Linux distributions may work but are not officially supported.
:::

---

## Step 1: Install Ubuntu 22.04 LTS

### Option A: Native Installation (Recommended)

1. Download Ubuntu 22.04 LTS Desktop: [ubuntu.com/download/desktop](https://ubuntu.com/download/desktop)
2. Create bootable USB with [Rufus](https://rufus.ie/) (Windows) or [Etcher](https://www.balena.io/etcher/) (Mac/Linux)
3. Boot from USB and follow installation wizard
4. Allocate at least 50GB for root partition

### Option B: Virtual Machine

**VirtualBox Setup:**
```bash
# Download VirtualBox from virtualbox.org
# Create new VM with:
# - Type: Linux
# - Version: Ubuntu (64-bit)
# - RAM: 8GB minimum
# - Disk: 50GB dynamically allocated
# - Enable 3D acceleration in Display settings
```

### Option C: Windows WSL2 (Limited Support)

```powershell
# Windows PowerShell (Admin)
wsl --install -d Ubuntu-22.04
wsl --set-version Ubuntu-22.04 2

# Launch Ubuntu and create user account
ubuntu2204.exe
```

:::tip WSL2 Limitations
WSL2 has limited GPU and GUI support. Use for ROS 2 command-line tools only. Not suitable for Gazebo/RViz visualization.
:::

---

## Step 2: System Update

```bash
# Update package lists and upgrade all packages
sudo apt update && sudo apt upgrade -y

# Install essential build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    software-properties-common \
    gnupg2 \
    lsb-release \
    ca-certificates

# Reboot if kernel was updated
sudo reboot
```

---

## Step 3: Install ROS 2 Humble

### Add ROS 2 Repository

```bash
# Set locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index
sudo apt update
```

### Install ROS 2 Desktop (Full)

```bash
# Install ROS 2 Humble Desktop (includes RViz, demos, tutorials)
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Configure Environment

```bash
# Source ROS 2 setup script (add to ~/.bashrc for persistence)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Expected output: ros2 cli version: humble
```

---

## Step 4: Install Gazebo Simulation

### Option A: Gazebo Fortress (Recommended for ROS 2)

```bash
# Add Gazebo repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Add GPG key
wget https://packages.osrfoundation.org/gazebo.key -O - | \
    sudo apt-key add -

# Update and install
sudo apt update
sudo apt install -y gazebo gz-fortress

# Install ROS 2 Gazebo bridge
sudo apt install -y ros-humble-ros-gz
```

### Option B: Gazebo Classic 11 (Legacy Support)

```bash
# Install Gazebo Classic
sudo apt install -y gazebo11 libgazebo11-dev

# Install ROS 2 Gazebo Classic plugins
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

### Test Gazebo Installation

```bash
# Launch Gazebo (should open empty world)
gazebo

# Test with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

---

## Step 5: Install NVIDIA Isaac Sim (Optional - GPU Required)

:::danger Requirements
Isaac Sim requires:
- NVIDIA GPU (RTX series recommended)
- Ubuntu 22.04
- 32GB RAM minimum
- 50GB disk space
:::

### Install NVIDIA Drivers

```bash
# Check if GPU is detected
lspci | grep -i nvidia

# Install recommended driver
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot

# Verify driver installation
nvidia-smi
# Should show GPU info and driver version
```

### Install CUDA Toolkit

```bash
# Download CUDA 12.x from NVIDIA website
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-3

# Add CUDA to PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA installation
nvcc --version
```

### Install Omniverse and Isaac Sim

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable and run
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# In Omniverse Launcher:
# 1. Sign in with NVIDIA account
# 2. Navigate to "Exchange" tab
# 3. Install "Isaac Sim" (latest version)
# 4. Launch Isaac Sim from "Library" tab
```

---

## Step 6: Install Python Dependencies

```bash
# Install Python 3.10 (should be default on Ubuntu 22.04)
python3 --version
# Expected: Python 3.10.x

# Install pip and virtualenv
sudo apt install -y python3-pip python3-venv

# Install common robotics packages
pip3 install --user \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    transforms3d \
    pyyaml

# Install PyTorch with CUDA support (if GPU available)
pip3 install --user torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Verify PyTorch GPU support
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

---

## Step 7: Install Additional ROS 2 Packages

```bash
# Navigation stack (Nav2)
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# Perception packages
sudo apt install -y \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-cv-bridge

# Manipulation (MoveIt2)
sudo apt install -y ros-humble-moveit

# SLAM toolbox
sudo apt install -y ros-humble-slam-toolbox

# TF2 tools
sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations

# ROS 2 Control
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

# Diagnostic tools
sudo apt install -y \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rviz2
```

---

## Step 8: Create ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone example packages (optional)
git clone https://github.com/ros/ros_tutorials.git -b humble

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 9: Test Installation with "Hello ROS 2"

Create a simple test node to verify everything works:

### Create Test Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python hello_ros2

cd hello_ros2/hello_ros2

# Create test node
cat > hello_node.py << 'EOF'
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Hello ROS 2! Count: {self.counter}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x hello_node.py
```

### Update setup.py

```bash
cd ~/ros2_ws/src/hello_ros2

# Edit setup.py to add entry point
cat > setup.py << 'EOF'
from setuptools import setup

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'hello_node = hello_ros2.hello_node:main'
        ],
    },
)
EOF
```

### Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select hello_ros2
source install/setup.bash

# Run the test node
ros2 run hello_ros2 hello_node

# Expected output (every second):
# [INFO] [hello_node]: Hello ROS 2! Count: 0
# [INFO] [hello_node]: Hello ROS 2! Count: 1
# [INFO] [hello_node]: Hello ROS 2! Count: 2

# Press Ctrl+C to stop
```

---

## Step 10: Environment Verification Checklist

Run these commands to verify your setup:

```bash
# Check ROS 2 installation
ros2 doctor

# List available packages
ros2 pkg list | head -20

# Check for common tools
which rviz2 && echo "✓ RViz2 installed" || echo "✗ RViz2 missing"
which gazebo && echo "✓ Gazebo installed" || echo "✗ Gazebo missing"
which colcon && echo "✓ Colcon installed" || echo "✗ Colcon missing"

# Test RViz2
rviz2 &
# Should open RViz window

# Test Gazebo
gazebo &
# Should open Gazebo simulator

# Check Python packages
python3 -c "import cv2; import numpy; import torch; print('✓ Python packages OK')"
```

---

## Common Installation Errors

See the [Troubleshooting Guide](./troubleshooting.md) for solutions to common issues:

- GPG key errors during apt update
- ROS 2 packages not found
- CUDA/NVIDIA driver conflicts
- Gazebo crashes on startup
- Permission denied errors
- WSL2 display issues

---

## Next Steps

1. Complete the "Hello ROS 2" test to verify installation
2. Read the [ROS 2 Cheatsheet](./ros2-cheatsheet.md) for quick reference
3. Start with [Week 1: Introduction to Physical AI](../week-01-02/01-introduction-to-physical-ai.md)
4. Join the community on [ROS Discourse](https://discourse.ros.org/)

---

## Quick Reference: Essential Commands

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build

# Source workspace
source install/setup.bash

# Run a node
ros2 run <package> <executable>

# Launch file
ros2 launch <package> <launch_file>

# List nodes
ros2 node list

# List topics
ros2 topic list

# Echo topic
ros2 topic echo /topic_name

# Show node info
ros2 node info /node_name
```

---

## Further Reading

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
