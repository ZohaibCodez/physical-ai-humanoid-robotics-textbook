---
sidebar_position: 4
title: TF2 and Coordinate Transformations
---

# TF2 and Coordinate Transformations

## Learning Objectives

- Understand coordinate frames and transformations in robotics
- Master TF2 library for managing transform trees
- Broadcast and listen to transforms
- Perform coordinate transformations for sensor data

## Introduction

TF2 is ROS 2's transform library that maintains relationships between coordinate frames over time. Every robot has multiple coordinate frames (base, sensors, end-effector), and TF2 tracks their relationships.

## Launch File with TF2 Parameters

Here's a complete ROS 2 launch file in Python format:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'robot_name',
            default_value='my_robot',
            description='Robot namespace'
        ),
        
        # Static transform: base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'laser_frame'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        # Static transform: base_link to camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0.2', '0', '0.5', '0', '0', '0', 'base_link', 'camera_frame'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_frequency': 50.0
            }]
        ),
    ])
```

### Running the Launch File

```bash
# Save as robot_launch.py in your package's launch directory
ros2 launch my_robot_pkg robot_launch.py

# With parameters
ros2 launch my_robot_pkg robot_launch.py use_sim_time:=true robot_name:=my_custom_robot
```

## Broadcasting Transforms

### Static Transform Broadcaster

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_euler

class StaticFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('static_frame_broadcaster')
        
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Broadcast static transform
        self.broadcast_static_transform()

    def broadcast_static_transform(self):
        static_transform = TransformStamped()
        
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'
        static_transform.child_frame_id = 'base_link'
        
        # Position
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        
        # Orientation (quaternion from roll, pitch, yaw)
        quat = quaternion_from_euler(0, 0, 0)
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]
        
        self.tf_static_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Static transform published: world -> base_link')

def main(args=None):
    rclpy.init(args=args)
    node = StaticFrameBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Dynamic Transform Broadcaster

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_euler
import math

class DynamicFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_frame_broadcaster')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Broadcast at 50 Hz
        self.timer = self.create_timer(0.02, self.broadcast_timer_callback)
        self.angle = 0.0

    def broadcast_timer_callback(self):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'moving_frame'
        
        # Circular motion
        t.transform.translation.x = 2.0 * math.cos(self.angle)
        t.transform.translation.y = 2.0 * math.sin(self.angle)
        t.transform.translation.z = 0.0
        
        # Rotating orientation
        quat = quaternion_from_euler(0, 0, self.angle)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        self.angle += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = DynamicFrameBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Listening to Transforms

```python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped
import math

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        
        # Create TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer to look up transforms
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        from_frame = 'base_link'
        to_frame = 'camera_frame'
        
        try:
            # Look up transform (latest available)
            now = Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.get_logger().info(
                f'Transform from {from_frame} to {to_frame}:\n'
                f'  Position: ({trans.transform.translation.x:.2f}, '
                f'{trans.transform.translation.y:.2f}, '
                f'{trans.transform.translation.z:.2f})\n'
                f'  Orientation: ({trans.transform.rotation.x:.2f}, '
                f'{trans.transform.rotation.y:.2f}, '
                f'{trans.transform.rotation.z:.2f}, '
                f'{trans.transform.rotation.w:.2f})'
            )
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')

    def transform_point(self, point_in, target_frame):
        """Transform a point to a different frame"""
        try:
            # Transform point
            point_out = self.tf_buffer.transform(point_in, target_frame)
            return point_out
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform point: {ex}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## TF2 Command Line Tools

```bash
# View current transform tree
ros2 run tf2_tools view_frames

# Echo transform between two frames
ros2 run tf2_ros tf2_echo base_link camera_frame

# Monitor TF tree in real-time
ros2 run tf2_tools echo_all_frames

# Static transform publisher (command line)
ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 base_link laser_frame
```

## Practical Example: Transforming Laser Scans

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_ros import TransformException
import math

class LaserToBaseLink(Node):
    def __init__(self):
        super().__init__('laser_to_base_link')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, scan_msg):
        # Transform laser scan points to base_link frame
        try:
            # Get transform from laser frame to base_link
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                scan_msg.header.frame_id,
                scan_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Process first point as example
            angle = scan_msg.angle_min
            range_val = scan_msg.ranges[0]
            
            if range_val > scan_msg.range_min and range_val < scan_msg.range_max:
                # Convert polar to Cartesian in laser frame
                point = PointStamped()
                point.header = scan_msg.header
                point.point.x = range_val * math.cos(angle)
                point.point.y = range_val * math.sin(angle)
                point.point.z = 0.0
                
                # Transform to base_link
                point_in_base = self.tf_buffer.transform(point, 'base_link')
                
                self.get_logger().info(
                    f'Point in base_link: '
                    f'({point_in_base.point.x:.2f}, '
                    f'{point_in_base.point.y:.2f}, '
                    f'{point_in_base.point.z:.2f})'
                )
                
        except TransformException as ex:
            self.get_logger().warn(f'Transform failed: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = LaserToBaseLink()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

- TF2 manages coordinate frame relationships in a tree structure
- Static transforms for fixed relationships (sensors mounted on robot)
- Dynamic transforms for moving parts (joints, mobile base odometry)
- Use `tf_buffer.lookup_transform()` to query transforms
- Use `tf_buffer.transform()` to transform points/poses between frames

## Hands-on Exercise

1. Create a robot with multiple sensor frames
2. Broadcast static transforms for all sensors
3. Write a node that transforms sensor data to base_link
4. Visualize the TF tree in RViz

## Further Reading

- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Understanding TF2](http://wiki.ros.org/tf2/Tutorials)
- [TF2 Migration Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Vs-Tf.html)
