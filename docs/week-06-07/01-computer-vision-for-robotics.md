---
sidebar_position: 1
title: Computer Vision for Robotics
---

# Computer Vision for Robotics

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand image formation and camera models
- Apply object detection in robotic applications
- Implement depth estimation from stereo cameras
- Process real-time video feeds for robot perception

## Introduction

Computer vision is the eyes of Physical AI systems. This chapter covers how robots see and interpret their environment—from basic image processing to modern deep learning approaches for object detection and scene understanding.

## Camera Models and Calibration

**Pinhole Camera Model**: The fundamental model relating 3D world to 2D image

**Key Concepts**:
- Intrinsic parameters (focal length, optical center)
- Extrinsic parameters (rotation, translation)
- Lens distortion correction

## Object Detection for Robots

**YOLO (You Only Look Once)**: Real-time object detection

**Example Use Cases**:
- Detecting people and obstacles for navigation
- Identifying objects for manipulation
- Reading signs and markers

## Depth Estimation

**Approaches**:
1. **Stereo Vision**: Two cameras, triangulation
2. **Structured Light**: Project pattern, measure distortion
3. **Time-of-Flight**: Measure light return time (LIDAR)

## Simulation Platform Options

Choose your preferred simulation platform for computer vision development:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
  <TabItem value="gazebo" label="Gazebo" default>

### Gazebo Simulation

Gazebo is the standard ROS 2 simulation environment with excellent camera and sensor support.

**Setup:**
```bash
# Install Gazebo Fortress
sudo apt-get update
sudo apt-get install ros-humble-gazebo-ros-pkgs

# Launch with camera
ros2 launch gazebo_ros gazebo.launch.py
```

**Camera Plugin Example:**
```xml
<sensor name="camera" type="camera">
  <pose>0 0 0 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <argument>~/image_raw:=image</argument>
    </ros>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

**Pros:** Free and open-source • Native ROS 2 integration • Large community support • Physics-accurate sensors

**Cons:** Higher learning curve • Less photorealistic rendering • Requires more manual setup

  </TabItem>
  <TabItem value="unity" label="Unity + ROS-TCP">

### Unity Simulation

Unity provides high-quality graphics and Unity Robotics Hub for ROS 2 integration.

**Setup**:
```bash
# Install Unity Robotics Hub
# 1. Download Unity Hub from unity.com
# 2. Install Unity 2021.3 LTS or newer
# 3. Create new project with Universal Render Pipeline

# Install ROS-TCP-Connector package
# In Unity: Window > Package Manager > Add package from git URL
# https://github.com/Unity-Technologies/ROS-TCP-Connector.git
```

**Camera Setup in Unity**:
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/camera/image_raw";
    public Camera cameraComponent;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
    }
    
    void Update()
    {
        // Capture and publish camera frame
        PublishImage();
    }
}
```

**Pros:** Photorealistic rendering • Excellent visual quality • Rich asset ecosystem • Easier scene creation

**Cons:** Proprietary (though free tier available) • Additional ROS-TCP bridge required • Physics less accurate than Gazebo • More resource-intensive

  </TabItem>
  <TabItem value="isaac" label="NVIDIA Isaac Sim">

### NVIDIA Isaac Sim

NVIDIA's Isaac Sim provides RTX-ray traced graphics and advanced AI capabilities.

**Setup**:
```bash
# Download Isaac Sim from NVIDIA Omniverse
# Requirements: NVIDIA RTX GPU, Ubuntu 20.04/22.04

# Install Isaac Sim ROS 2 Bridge
cd ~/.local/share/ov/pkg/isaac_sim-*
./python.sh -m pip install pyyaml rclpy
```

**Camera Example**:
```python
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import Camera

# Create camera
camera = Camera(
    prim_path="/World/camera",
    resolution=(1280, 720),
    frequency=30,
)

# Enable ROS 2 publishing
from omni.isaac.ros2_bridge import ROS2CameraBridge
camera_bridge = ROS2CameraBridge(
    camera_prim_path="/World/camera",
    topic_name="/camera/image_raw"
)
```

**Pros:** Best graphics quality (RTX ray tracing) • Advanced physics simulation • Built-in domain randomization • Native deep learning integration

**Cons:** Requires high-end NVIDIA GPU • Large download size (~50GB) • Steeper learning curve • More complex setup

  </TabItem>
</Tabs>

## Recommendation

- **Learning/Teaching**: Gazebo (best ROS 2 integration, free)
- **Visual Quality**: Unity (easiest for beautiful demos)
- **Research/AI**: Isaac Sim (advanced AI/ML features)

## Summary

Computer vision transforms raw pixels into semantic understanding. Modern deep learning approaches combined with classical geometry enable robust robot perception.

**Key Takeaways**:
- Camera models relate 3D world to 2D images
- Object detection identifies what's in the scene
- Depth estimation determines where objects are
- Real-time processing is critical for robotics

## Further Reading

- [Multiple View Geometry](https://www.robots.ox.ac.uk/~vgg/hzbook/) by Hartley & Zisserman
- [Stanford CS231n](http://cs231n.stanford.edu/) - CNNs for Visual Recognition

---

**Next Chapter**: [Depth Cameras and LIDAR](./02-depth-cameras-and-lidar.md) →
