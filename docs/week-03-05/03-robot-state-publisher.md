---
sidebar_position: 3
title: Robot State Publisher and URDF
---

# Robot State Publisher and URDF

## Learning Objectives

- Understand URDF (Unified Robot Description Format)
- Learn to describe robot geometry and kinematics
- Use robot_state_publisher to broadcast transforms
- Visualize robot models in RViz

## Introduction

URDF is an XML format for describing robot geometry, kinematics, and visual properties. The `robot_state_publisher` node reads URDF and publishes the robot's state to the TF2 transform tree.

## Simple Robot URDF Example

Here's a complete URDF for a simple two-link robot arm:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- First Link -->
  <link name="link_1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0"
               iyy="0.004" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Second Link -->
  <link name="link_2">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.002" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 1: Base to Link 1 -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Joint 2: Link 1 to Link 2 -->
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

### URDF Components Explained

- **`<link>`**: Defines a rigid body with visual, collision, and inertial properties
- **`<joint>`**: Connects two links, defines motion constraints and limits
- **`<visual>`**: Appearance in simulation/visualization (geometry, color, texture)
- **`<collision>`**: Shape for physics collision detection
- **`<inertial>`**: Mass and inertia matrix for dynamics simulation

### Joint Types

- **`revolute`**: Rotates around axis with limits
- **`continuous`**: Rotates around axis without limits
- **`prismatic`**: Slides along axis (linear motion)
- **`fixed`**: No motion, rigidly attached
- **`planar`**: Moves in a plane
- **`floating`**: 6 DOF unconstrained motion

## Using robot_state_publisher

### Launch File Example

```xml
<launch>
  <!-- Load URDF into parameter server -->
  <param name="robot_description"
         command="$(find xacro)/xacro '$(find my_robot_description)/urdf/simple_arm.urdf'" />

  <!-- Start robot_state_publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
    <param name="publish_frequency" value="50.0"/>
  </node>

  <!-- Start joint_state_publisher_gui for manual control -->
  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui">
    <param name="use_gui" value="true"/>
  </node>

  <!-- Launch RViz for visualization -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_robot_description)/rviz/config.rviz"/>
</launch>
```

### Python Node to Publish Joint States

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        self.angle = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names must match URDF
        msg.name = ['joint_1', 'joint_2']
        
        # Simple sinusoidal motion
        msg.position = [
            math.sin(self.angle),
            math.cos(self.angle * 0.5)
        ]
        msg.velocity = []
        msg.effort = []
        
        self.publisher_.publish(msg)
        self.angle += 0.05

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    
    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Visualizing in RViz

```bash
# Terminal 1: Start robot_state_publisher with URDF
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_arm.urdf)"

# Terminal 2: Publish joint states
python3 joint_state_publisher.py

# Terminal 3: Launch RViz
ros2 run rviz2 rviz2

# In RViz:
# 1. Add > RobotModel
# 2. Fixed Frame: base_link
# 3. Robot Description: robot_description
```

## Summary

- URDF defines robot structure in XML format
- robot_state_publisher broadcasts TF transforms based on joint states
- Proper URDF includes visual, collision, and inertial properties
- Use RViz to visualize and debug robot models

## Hands-on Exercise

1. Create a URDF for a mobile robot with wheels
2. Add sensors (camera, lidar) to the URDF
3. Publish joint states and visualize in RViz
4. Experiment with different joint types and limits

## Further Reading

- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [URDF XML Specification](http://wiki.ros.org/urdf/XML)
- [robot_state_publisher Package](http://wiki.ros.org/robot_state_publisher)
