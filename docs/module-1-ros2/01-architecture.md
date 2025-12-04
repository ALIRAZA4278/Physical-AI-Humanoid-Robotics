---
sidebar_position: 1
title: "1.1 ROS 2 Architecture Essentials"
description: Understanding the core concepts of Robot Operating System 2 - nodes, topics, services, and actions
---

# ROS 2 Architecture Essentials

Welcome to Module 1! In this chapter, you'll learn the fundamental building blocks of ROS 2 that power modern robotic systems.

## Learning Objectives

By the end of this chapter, you will:
- Understand the ROS 2 computation graph
- Know the difference between nodes, topics, services, and actions
- Be able to inspect running ROS 2 systems
- Understand why ROS 2 uses DDS middleware

---

## What is ROS 2?

**Robot Operating System 2 (ROS 2)** is not actually an operating system—it's a set of software libraries and tools for building robot applications. Think of it as a communication framework that helps different parts of a robot talk to each other.

### Key Improvements Over ROS 1

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Real-time** | Limited | Supported |
| **Security** | None | DDS Security |
| **Multi-robot** | Difficult | Native |
| **Platform** | Linux only | Linux, Windows, macOS |
| **Middleware** | Custom (TCPROS) | DDS standard |

---

## The Computation Graph

ROS 2 organizes robot software as a **computation graph**—a network of processes (nodes) connected by communication channels (topics, services, actions).

```
┌─────────────┐     /camera/image      ┌─────────────────┐
│   Camera    │ ──────────────────────▶│  Object         │
│   Driver    │                        │  Detector       │
└─────────────┘                        └────────┬────────┘
                                                │
                                       /detected_objects
                                                │
                                                ▼
┌─────────────┐     /cmd_vel           ┌─────────────────┐
│   Motor     │ ◀──────────────────────│  Navigation     │
│   Driver    │                        │  Planner        │
└─────────────┘                        └─────────────────┘
```

---

## Nodes

A **node** is a process that performs computation. Each node should do one thing well:

- Camera driver node → captures images
- Object detection node → finds objects in images
- Navigation node → plans paths
- Motor driver node → controls wheels

### Creating a Simple Node

```python
#!/usr/bin/env python3
"""Minimal ROS 2 node example."""

import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from ROS 2!')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Commands

```bash
# List all running nodes
ros2 node list

# Get information about a node
ros2 node info /minimal_node
```

---

## Topics

**Topics** are named buses for asynchronous message passing. Publishers send messages; subscribers receive them.

### Characteristics

- **Many-to-many**: Multiple publishers and subscribers per topic
- **Asynchronous**: Publishers don't wait for subscribers
- **Typed**: Each topic has a specific message type

### Topic Commands

```bash
# List all topics
ros2 topic list

# Show topic message type
ros2 topic info /camera/image

# Echo messages (useful for debugging)
ros2 topic echo /camera/image

# Publish a test message
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

### Publisher Example

```python
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Subscriber Example

```python
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
```

---

## Services

**Services** provide synchronous request-response communication. A client sends a request and waits for a response.

### When to Use Services

- Configuration changes
- One-time queries
- Actions that need confirmation

### Service Commands

```bash
# List all services
ros2 service list

# Show service type
ros2 service type /add_two_ints

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

### Service Example

```python
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

---

## Actions

**Actions** are for long-running tasks that need feedback and can be cancelled. They combine:

- **Goal**: What to achieve
- **Feedback**: Progress updates
- **Result**: Final outcome

### When to Use Actions

- Navigation (go to point)
- Manipulation (pick up object)
- Any task taking > 1 second

### Action Commands

```bash
# List all actions
ros2 action list

# Show action type
ros2 action info /navigate_to_pose

# Send a goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 2.0}}}}"
```

---

## DDS: The Middleware

ROS 2 uses **Data Distribution Service (DDS)** as its communication middleware. DDS is an industry standard providing:

- **Discovery**: Nodes find each other automatically
- **QoS**: Quality of Service policies (reliability, durability)
- **Security**: Authentication and encryption

### QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Use for sensor data that can tolerate drops
subscriber = node.create_subscription(Image, '/camera', callback, sensor_qos)
```

Common QoS profiles:
- **Sensor data**: Best effort, keep last
- **Commands**: Reliable, keep all
- **State**: Reliable, transient local (late joiners get last value)

---

## Hands-On Exercise

### Exercise 1.1: Explore the Demo Nodes

1. **Start the talker:**
```bash
ros2 run demo_nodes_cpp talker
```

2. **In another terminal, start the listener:**
```bash
ros2 run demo_nodes_cpp listener
```

3. **In a third terminal, explore:**
```bash
# List nodes
ros2 node list

# List topics
ros2 topic list

# Echo the topic
ros2 topic echo /chatter

# Get topic info
ros2 topic info /chatter -v
```

### Exercise 1.2: Modify Message Rate

1. Use `ros2 topic hz /chatter` to measure the publication rate
2. How would you change the talker to publish at 5 Hz instead of 1 Hz?

---

## Summary

| Concept | Use Case | Direction |
|---------|----------|-----------|
| **Node** | Unit of computation | N/A |
| **Topic** | Streaming data | Publisher → Subscriber |
| **Service** | Request/response | Client → Server |
| **Action** | Long-running tasks | Client → Server (with feedback) |

---

## Next Steps

In the next chapter, you'll build your first complete ROS 2 Python node from scratch.

**Continue to:** [1.2 Building Your First Python Node](/docs/module-1-ros2/first-node)

---

:::info Resources
- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [DDS Foundation](https://www.dds-foundation.org/)
:::
