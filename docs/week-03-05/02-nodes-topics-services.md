---
sidebar_position: 2
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services

## Learning Objectives

- Understand ROS 2 nodes as independent processes
- Master publish/subscribe communication with topics
- Implement request/response patterns with services
- Build multi-node systems with proper communication patterns

## Introduction

ROS 2 nodes are the building blocks of robot systems. Each node is an independent process that communicates with other nodes through topics (publish/subscribe) or services (request/response).

## ROS 2 Publisher and Subscriber Example

Here's a complete example showing how to create a publisher and subscriber in Python:

### Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Example

```bash
# Terminal 1 - Start the publisher
python3 publisher_node.py

# Terminal 2 - Start the subscriber
python3 subscriber_node.py
```

**Expected Output:**
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
```

## ROS 2 Service Example

Services provide synchronous request/response communication:

### Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Server Ready')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: a={request.a} b={request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()
    
    try:
        rclpy.spin(add_two_ints_server)
    except KeyboardInterrupt:
        pass
    
    add_two_ints_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    
    add_two_ints_client = AddTwoIntsClient()
    future = add_two_ints_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    
    rclpy.spin_until_future_complete(add_two_ints_client, future)
    response = future.result()
    
    add_two_ints_client.get_logger().info(
        f'Result: {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    
    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Service Example

```bash
# Terminal 1 - Start the server
python3 service_server.py

# Terminal 2 - Call the service
python3 service_client.py 5 7

# Output: Result: 5 + 7 = 12
```

## Summary

- **Nodes**: Independent processes that perform specific tasks
- **Topics**: Asynchronous publish/subscribe communication for continuous data streams
- **Services**: Synchronous request/response for occasional operations
- Use topics for sensor data, use services for configuration/commands

## Hands-on Exercise

1. Modify the publisher to send custom robot position data
2. Create a subscriber that processes and logs the position
3. Create a service that calculates distance between two positions

## Further Reading

- [ROS 2 Creating a Node](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
