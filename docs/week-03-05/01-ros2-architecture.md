---
sidebar_position: 1
title: ROS 2 Architecture
---

# ROS 2 Architecture

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain the core architecture of ROS 2
- Understand nodes, topics, services, and actions
- Describe the DDS middleware layer
- Compare ROS 1 and ROS 2 design decisions

## Introduction

**ROS 2** (Robot Operating System 2) is the de facto standard middleware for building robot software. Unlike ROS 1, ROS 2 was designed from the ground up for production robotics with real-time performance, security, and multi-robot support.

## Why ROS 2?

**ROS 1 Limitations** (addressed in ROS 2):
- Single point of failure (roscore)
- No real-time support
- Limited security
- TCP-only communication (no UDP for real-time)

**ROS 2 Improvements**:
- ✅ Distributed architecture (no master node)
- ✅ DDS middleware for real-time communication
- ✅ Built-in security (SROS2)
- ✅ Better Windows/macOS support
- ✅ Lifecycle management for nodes

## Core Concepts

### Nodes

**Definition**: Independent processes that perform computation

**Example**: A robot might have:
- `camera_node` - Publishes images
- `detector_node` - Detects objects in images
- `controller_node` - Plans robot motion

### Topics

**Definition**: Named buses for asynchronous message passing

**Pattern**: Publish-Subscribe (many-to-many)

**Example**:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [simple_publisher]: Publishing: "Hello ROS 2: 0"
[INFO] [simple_publisher]: Publishing: "Hello ROS 2: 1"
[INFO] [simple_publisher]: Publishing: "Hello ROS 2: 2"
...
```

### Services

**Definition**: Synchronous request-response communication

**Pattern**: Client-Server (one-to-one)

**Use Case**: Triggering actions that return a result (e.g., "capture image now")

### Actions

**Definition**: Asynchronous goal-based tasks with feedback

**Pattern**: Goal-Result-Feedback

**Use Case**: Long-running tasks like "navigate to waypoint" with progress updates

## DDS Middleware

ROS 2 uses **Data Distribution Service (DDS)** for communication:

- **Discovery**: Nodes automatically find each other (no master)
- **QoS**: Quality of Service policies (reliability, durability, etc.)
- **Real-Time**: UDP support for low-latency communication

## Summary

ROS 2 provides a robust, distributed middleware for building robot software. Its node-based architecture, flexible communication patterns, and DDS foundation make it ideal for Physical AI systems.

**Key Takeaways**:
- Nodes are independent processes that communicate via topics/services/actions
- DDS provides decentralized discovery and real-time performance
- ROS 2 is designed for production robotics, not just research

## Hands-On Exercise

1. Install ROS 2 Humble following [Setup Instructions](../../appendix/setup-instructions)
2. Run the example publisher above
3. In another terminal, run: `ros2 topic echo /chatter`
4. Observe the messages flowing

## Further Reading

- [ROS 2 Design Decisions](https://design.ros2.org/)
- [DDS Introduction](https://www.dds-foundation.org/what-is-dds-3/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

---

**Next Chapter**: [Nodes, Topics, and Services](./02-nodes-topics-services) →
