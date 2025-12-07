---
sidebar_position: 2
title: Troubleshooting Guide
---

# Troubleshooting Guide

Common errors and solutions organized by installation phase and operating system.

---

## ROS 2 Installation Issues

### Error: GPG Key Not Found

**Symptom:**
```
W: GPG error: http://packages.ros.org/ros2/ubuntu jammy InRelease: 
The following signatures couldn't be verified because the public key is not available
```

**Solution:**
```bash
# Remove old key if exists
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg

# Re-download key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Update package lists
sudo apt update
```

---

### Error: Package ros-humble-desktop Not Found

**Symptom:**
```
E: Unable to locate package ros-humble-desktop
```

**Solution:**
```bash
# Verify Ubuntu version (must be 22.04)
lsb_release -a

# Check if ROS 2 repository is added correctly
cat /etc/apt/sources.list.d/ros2.list

# Should show:
# deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] 
# http://packages.ros.org/ros2/ubuntu jammy main

# If missing, re-add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

---

### Error: rosdep Command Not Found

**Symptom:**
```bash
rosdep update
bash: rosdep: command not found
```

**Solution:**
```bash
# Install python3-rosdep
sudo apt install -y python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

---

### Error: Permission Denied When Running rosdep init

**Symptom:**
```
ERROR: cannot download default sources list from:
https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
```

**Solution:**
```bash
# Check if rosdep is already initialized
ls /etc/ros/rosdep/sources.list.d/

# If directory exists, skip init and just update
rosdep update

# If you need to reinitialize
sudo rm -rf /etc/ros/rosdep
sudo rosdep init
rosdep update
```

---

## Gazebo Issues

### Error: Gazebo Crashes on Startup (Black Screen)

**Symptom:**
```
gazebo: symbol lookup error: /usr/lib/x86_64-linux-gnu/libgazebo_common.so.9: 
undefined symbol: _ZN8ignition10fuel_tools12ClientConfig12SetUserAgentERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE
```

**Solution (Ubuntu 22.04):**
```bash
# Uninstall conflicting Gazebo versions
sudo apt remove --purge gazebo* libgazebo*

# Install Gazebo Fortress
sudo apt update
sudo apt install -y gz-fortress

# Test installation
gz sim
```

---

### Error: Gazebo "VMware: vmw_ioctl_command error..."

**Symptom:**
Running Gazebo in VMware shows:
```
VMware: vmw_ioctl_command error Invalid argument.
```

**Solution:**
```bash
# Add VMware graphics acceleration workaround
echo "export SVGA_VGPU10=0" >> ~/.bashrc
source ~/.bashrc

# Launch Gazebo
gazebo
```

---

### Error: Gazebo Fails to Load Models

**Symptom:**
```
[Err] [REST.cc:205] Error in REST request
```

**Solution:**
```bash
# Set Gazebo model path
echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
source ~/.bashrc

# Download models manually
cd ~/.gazebo/models
wget -r -np -nH --cut-dirs=2 -R "index.html*" http://models.gazebosim.org/

# Or use gz model download
gz model --download
```

---

## NVIDIA Driver and CUDA Issues

### Error: NVIDIA Driver Installation Failed

**Symptom:**
```
ERROR: An NVIDIA kernel module 'nvidia-drm' appears to already be loaded
```

**Solution:**
```bash
# Boot into recovery mode or disable GUI
sudo systemctl set-default multi-user.target
sudo reboot

# After reboot (text mode)
sudo apt purge nvidia*
sudo apt autoremove
sudo ubuntu-drivers autoinstall
sudo reboot

# Re-enable GUI
sudo systemctl set-default graphical.target
```

---

### Error: CUDA Version Mismatch

**Symptom:**
```python
>>> import torch
>>> torch.cuda.is_available()
False
```

**Solution:**
```bash
# Check installed CUDA version
nvcc --version

# Check PyTorch CUDA version
python3 -c "import torch; print(torch.version.cuda)"

# Reinstall PyTorch matching your CUDA version
# For CUDA 11.8:
pip3 uninstall torch torchvision torchaudio
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# For CUDA 12.1:
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

---

### Error: nvidia-smi Not Found

**Symptom:**
```bash
nvidia-smi
Command 'nvidia-smi' not found
```

**Solution:**
```bash
# Check if NVIDIA GPU is detected
lspci | grep -i nvidia

# If detected, install drivers
sudo ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
sudo reboot

# After reboot
nvidia-smi
```

---

## Colcon Build Errors

### Error: Package Not Found During Build

**Symptom:**
```
--- stderr: my_package
CMake Error at CMakeLists.txt:10 (find_package):
  Could not find a package configuration file provided by "rclcpp"
```

**Solution:**
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build
```

---

### Error: Colcon Build Permission Denied

**Symptom:**
```
PermissionError: [Errno 13] Permission denied: '/home/user/ros2_ws/build/COLCON_IGNORE'
```

**Solution:**
```bash
# Fix ownership
cd ~/ros2_ws
sudo chown -R $USER:$USER .

# Clean build artifacts
rm -rf build install log

# Rebuild
colcon build
```

---

### Error: Colcon Build Runs Out of Memory

**Symptom:**
```
c++: fatal error: Killed signal terminated program cc1plus
```

**Solution:**
```bash
# Build with single core
colcon build --executor sequential

# Or limit parallel jobs
colcon build --parallel-workers 2

# Increase swap space (if RAM < 8GB)
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

---

## RViz2 Issues

### Error: RViz2 Crashes with Segmentation Fault

**Symptom:**
```
Segmentation fault (core dumped)
```

**Solution:**
```bash
# Check graphics drivers
glxinfo | grep "OpenGL version"

# If using virtual machine, enable 3D acceleration in VM settings

# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
rviz2

# If that works, add to ~/.bashrc:
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
```

---

### Error: RViz2 "No displays found"

**Symptom:**
RViz2 opens but shows no display plugins in the left panel.

**Solution:**
```bash
# Reinstall RViz2
sudo apt remove ros-humble-rviz2
sudo apt install ros-humble-rviz2

# Clear RViz config
rm -rf ~/.rviz2

# Launch RViz2
rviz2
```

---

## Python Package Issues

### Error: ModuleNotFoundError: No module named 'cv2'

**Symptom:**
```python
>>> import cv2
ModuleNotFoundError: No module named 'cv2'
```

**Solution:**
```bash
# Install OpenCV
pip3 install opencv-python

# If still not working, check Python path
python3 -c "import sys; print(sys.path)"

# Install system-wide (not recommended but works)
sudo apt install python3-opencv
```

---

### Error: Conflicting NumPy Versions

**Symptom:**
```
RuntimeError: The current Numpy installation fails to pass a sanity check
```

**Solution:**
```bash
# Uninstall all NumPy versions
pip3 uninstall numpy -y
sudo apt remove python3-numpy

# Reinstall compatible version
pip3 install numpy==1.24.3

# Verify
python3 -c "import numpy; print(numpy.__version__)"
```

---

## WSL2 Specific Issues

### Error: WSL2 Cannot Access GPU

**Symptom:**
```bash
nvidia-smi
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver
```

**Solution:**
```powershell
# Windows PowerShell (Admin)
# Update WSL2 kernel
wsl --update

# Install CUDA toolkit in WSL2 (no driver needed)
# Follow NVIDIA WSL2 CUDA guide:
# https://docs.nvidia.com/cuda/wsl-user-guide/index.html
```

---

### Error: WSL2 GUI Applications Won't Start

**Symptom:**
```
Error: Can't open display
```

**Solution:**
```bash
# Install WSLg (Windows 11)
wsl --update

# For Windows 10, use VcXsrv:
# 1. Download and install VcXsrv
# 2. Launch XLaunch with "Disable access control"
# 3. Add to ~/.bashrc:
echo 'export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk "{print \$2}"):0' >> ~/.bashrc
source ~/.bashrc
```

---

## Network and Connectivity Issues

### Error: ROS 2 Nodes Can't Discover Each Other

**Symptom:**
```bash
ros2 node list
# Shows empty or missing nodes
```

**Solution:**
```bash
# Check ROS_DOMAIN_ID (must be same across all nodes)
echo $ROS_DOMAIN_ID

# Set domain ID (0-101)
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# Check firewall
sudo ufw status

# Allow ROS 2 DDS ports
sudo ufw allow 7400:7500/tcp
sudo ufw allow 7400:7500/udp

# Disable multicast filtering if on VM
sudo sysctl -w net.core.rmem_max=2147483647
```

---

### Error: GitHub Clone Fails (SSH Key Issues)

**Symptom:**
```
Permission denied (publickey)
```

**Solution:**
```bash
# Generate SSH key
ssh-keygen -t ed25519 -C "your_email@example.com"

# Add to ssh-agent
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

# Copy public key
cat ~/.ssh/id_ed25519.pub

# Add to GitHub: Settings → SSH and GPG keys → New SSH key

# Test connection
ssh -T git@github.com
```

---

## Performance Issues

### Issue: Gazebo Runs Slowly

**Symptoms:**
- Low FPS (less than 10)
- Simulation lag
- High CPU usage

**Solutions:**

**1. Reduce physics update rate - Edit world file:**
```xml
<physics type="ode">
  <max_step_size>0.01</max_step_size>  <!-- Increase from 0.001 -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- Decrease from 1000 -->
</physics>
```

**2. Disable shadows and reflections in Gazebo GUI**

**3. Use headless mode (no GUI):**
```bash
gzserver world.sdf
```

**4. Limit sensor update rates in URDF/SDF**

---

### Issue: Compilation Takes Too Long

**Symptoms:**
- `colcon build` takes >30 minutes
- System freezes during build

**Solutions:**
```bash
# 1. Build specific packages only
colcon build --packages-select my_package

# 2. Use symlink-install for Python packages (faster rebuilds)
colcon build --symlink-install

# 3. Enable compiler cache
sudo apt install ccache
echo 'export CC="ccache gcc"' >> ~/.bashrc
echo 'export CXX="ccache g++"' >> ~/.bashrc
source ~/.bashrc

# 4. Reduce parallel jobs
colcon build --parallel-workers 2
```

---

## Getting More Help

### Check System Logs

```bash
# ROS 2 logs
ros2 doctor --report

# System logs
journalctl -xe

# Kernel logs
dmesg | tail -50
```

### Useful Diagnostic Commands

```bash
# Check ROS 2 environment
printenv | grep ROS

# List installed ROS 2 packages
dpkg -l | grep ros-humble

# Check disk space
df -h

# Check memory usage
free -h

# Check CPU/GPU usage
htop
nvidia-smi
```

### Community Resources

- [ROS Discourse](https://discourse.ros.org/) - Ask questions
- [ROS Answers](https://answers.ros.org/) - Search solutions
- [GitHub Issues](https://github.com/ros2) - Report bugs
- [Gazebo Answers](http://answers.gazebosim.org/) - Simulation help

---

## Still Having Issues?

If your problem isn't listed here:

1. **Check the [ROS 2 Documentation](https://docs.ros.org/en/humble/)**
2. **Search [ROS Discourse](https://discourse.ros.org/)**
3. **Ask on [ROS Answers](https://answers.ros.org/)**
4. **Check package-specific GitHub Issues**
5. **Join the ROS community on Discord/Slack**

When asking for help, include:
- Ubuntu version (`lsb_release -a`)
- ROS 2 version (`ros2 --version`)
- Full error message
- Steps to reproduce
- Output of `ros2 doctor --report`
