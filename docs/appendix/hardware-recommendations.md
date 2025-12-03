---
sidebar_position: 4
title: Hardware Requirements and Options
---

# Hardware Requirements and Options

## Overview

This course supports multiple hardware configurations, from budget-friendly simulation setups to complete robot labs. Choose the option that fits your budget and learning goals.

## Hardware Tiers Comparison

| Feature | Economy Kit | Digital Twin | Robot Lab | Cloud Alternative |
|---------|------------|--------------|-----------|-------------------|
| **Cost** | $200-300 | $800-1,200 | $3,000-5,000 | $50-200/month |
| **Physical Robot** | ❌ | ❌ | ✅ | ❌ |
| **GPU Computing** | ❌ | ✅ | ✅ | ✅ |
| **ROS 2 Support** | ✅ | ✅ | ✅ | ✅ |
| **Simulation** | Basic | Advanced | Advanced | Advanced |
| **AI Training** | Limited | Good | Excellent | Good |
| **Best For** | Students | Developers | Research | Teams |

---

## Option 1: Economy Jetson Kit ($200-300)

### What You Get
A minimal setup for learning ROS 2 and running basic simulations without GPU acceleration.

### Hardware Components

| Component | Specification | Price (USD) |
|-----------|--------------|-------------|
| **NVIDIA Jetson Nano** | 4GB RAM, Quad-core ARM CPU | $99 |
| **MicroSD Card** | 128GB Class 10 | $20 |
| **Power Supply** | 5V 4A barrel jack | $15 |
| **Cooling Fan** | 40mm PWM fan + heatsink | $12 |
| **Camera Module** | IMX219 8MP (optional) | $25 |
| **USB Peripherals** | Keyboard, mouse, WiFi dongle | $30-50 |
| **Total** | | **$201-241** |

### What You Can Do
- Learn ROS 2 basics (nodes, topics, services)
- Run Gazebo simulations (simple environments)
- Basic computer vision with camera
- Program robot algorithms in Python/C++

### Limitations
- No advanced AI model training
- Limited simulation complexity
- No GPU-accelerated perception
- Slower compile times

### Setup Guide
1. Flash Ubuntu 20.04 or 22.04 to microSD card
2. Install ROS 2 Humble (see Setup Instructions)
3. Install Gazebo for basic simulation
4. Connect camera module for vision experiments

---

## Option 2: Digital Twin Workstation ($800-1,200)

### What You Get
A powerful development machine for advanced simulation, AI training, and virtual robotics.

### Hardware Components

| Component | Specification | Price (USD) |
|-----------|--------------|-------------|
| **CPU** | AMD Ryzen 7 5800X or Intel i7-12700 | $250-350 |
| **GPU** | NVIDIA RTX 3060 Ti (8GB) or RTX 3070 | $400-500 |
| **RAM** | 32GB DDR4 3200MHz | $100-120 |
| **Storage** | 1TB NVMe SSD | $80-100 |
| **Motherboard** | ATX with PCIe 4.0 | $120-150 |
| **Power Supply** | 750W 80+ Gold | $80-100 |
| **Case** | ATX with good airflow | $70-90 |
| **Total** | | **$1,100-1,410** |

### What You Can Do
- Full Gazebo Classic + Gazebo Fortress simulation
- NVIDIA Isaac Sim for photorealistic physics
- Train YOLO, PointNet, and other AI models
- Run multiple ROS 2 nodes simultaneously
- Virtual humanoid robot development
- Real-time sensor fusion simulation

### Recommended Software Stack
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress or NVIDIA Isaac Sim
- PyTorch with CUDA support
- Docker for containerized development

### Setup Guide
1. Install Ubuntu 22.04 LTS
2. Install NVIDIA drivers (525+)
3. Install ROS 2 Humble Desktop
4. Install NVIDIA Isaac Sim or Gazebo Fortress
5. Set up Docker with GPU support

---

## Option 3: Robot Lab ($3,000-5,000)

### What You Get
Complete hardware setup with real robot, computing, and perception equipment.

### Core Computing

| Component | Specification | Price (USD) |
|-----------|--------------|-------------|
| **Workstation** | Same as Digital Twin tier | $1,100-1,400 |
| **Jetson AGX Orin** | 64GB, 2048-core GPU | $1,999 |
| **Network Switch** | Gigabit Ethernet 8-port | $50 |

### Robot Hardware

| Component | Specification | Price (USD) |
|-----------|--------------|-------------|
| **Humanoid Robot** | TurtleBot4 or equivalent mobile platform | $1,200-1,500 |
| **Lidar** | RPLidar A2 or Hokuyo | $300-400 |
| **RGB-D Camera** | Intel RealSense D435i | $400 |
| **IMU** | 9-DOF MPU-9250 | $15 |
| **Servos** | Dynamixel MX-28 or XM430 (optional) | $150 each |

### Total Investment: $3,000-5,000+

### What You Can Do
- Real-world ROS 2 deployment
- SLAM and autonomous navigation
- Vision-based manipulation
- Hardware-software integration testing
- Humanoid locomotion experiments
- Multi-robot coordination

### Lab Setup Requirements
- Dedicated workspace (10ft x 10ft minimum)
- Stable power supply (UPS recommended)
- Safety barriers for robot movement area
- Good lighting for vision systems

---

## Option 4: Cloud Alternatives ($50-200/month)

For students without access to hardware or those wanting to experiment before committing.

### Google Colab Pro+
- **Cost**: $50/month
- **GPU**: NVIDIA V100 or A100
- **RAM**: 52GB
- **Good For**: AI model training, Jupyter notebooks, quick experiments
- **Limitations**: No ROS 2 GUI, session timeouts

### AWS EC2 with ROS 2
- **Cost**: $100-200/month (p3.2xlarge)
- **GPU**: NVIDIA V100
- **Storage**: 100GB EBS
- **Good For**: Full ROS 2 stack, persistent environments, team collaboration
- **Setup**: Use AWS RoboMaker or custom AMI

### Azure ML with GPU
- **Cost**: $150-250/month
- **GPU**: NVIDIA T4 or V100
- **Good For**: AI/ML experiments, cloud simulation, scalable training
- **Integration**: Azure IoT Hub for real robot deployment

### GitHub Codespaces
- **Cost**: Free tier + $0.18/hour for GPU
- **Good For**: ROS 2 development, code sharing, CI/CD
- **Limitations**: Limited GPU options, requires Dockerfile

---

## Minimum Software Requirements (All Tiers)

| Software | Version | Purpose |
|----------|---------|---------|
| **Ubuntu** | 22.04 LTS | ROS 2 compatibility |
| **ROS 2** | Humble Hawksbill | Robot framework |
| **Python** | 3.10+ | Programming |
| **CMake** | 3.16+ | Building C++ nodes |
| **Gazebo** | Fortress or Classic 11 | Simulation |
| **RViz** | ROS 2 version | Visualization |

---

## Hardware Selection Guide

### Choose Economy Kit If:
- Budget under $300
- Learning ROS 2 fundamentals only
- No immediate need for AI training
- Comfortable with simulation-only learning

### Choose Digital Twin If:
- Budget $800-1,200
- Want to train AI models locally
- Need advanced simulation (Isaac Sim)
- Plan to develop complex algorithms
- No space for physical robots

### Choose Robot Lab If:
- Budget $3,000+
- Need real hardware testing
- Research or commercial development
- Teaching robotics courses
- Want complete learning experience

### Choose Cloud If:
- No upfront budget for hardware
- Need flexible computing resources
- Team collaboration required
- Short-term project (< 6 months)

---

## Cost Optimization Tips

1. **Start Small**: Begin with Economy Kit, upgrade as needed
2. **Used Hardware**: eBay/Craigslist for Jetson Nano or older GPUs
3. **University Resources**: Use campus computer labs with GPUs
4. **Cloud Free Tiers**: Google Colab free version for basic experiments
5. **Open Source**: Use free simulators (Gazebo) before paying for Isaac Sim
6. **Community**: Join ROS Discourse for hardware recommendations and deals

---

## Recommended Accessories (All Tiers)

| Accessory | Purpose | Cost |
|-----------|---------|------|
| **External Monitor** | Development efficiency | $100-200 |
| **Mechanical Keyboard** | Coding comfort | $50-150 |
| **USB Hub** | Connect multiple devices | $20-40 |
| **Cable Management** | Organized workspace | $15-30 |
| **Anti-Static Mat** | Hardware safety | $20-40 |
| **Backup Drive** | Data protection | $60-100 |

---

## Next Steps

1. Choose your hardware tier based on budget and goals
2. Proceed to [Setup Instructions](./setup-instructions.md) for installation
3. Join the [ROS Discourse](https://discourse.ros.org/) community
4. Check the [Troubleshooting Guide](./troubleshooting.md) if you encounter issues

## Further Reading

- [NVIDIA Jetson Documentation](https://developer.nvidia.com/embedded/jetson)
- [ROS 2 Hardware Requirements](https://docs.ros.org/en/humble/Installation.html)
- [NVIDIA Isaac Sim System Requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html)
- [PCPartPicker](https://pcpartpicker.com/) for building custom workstations
