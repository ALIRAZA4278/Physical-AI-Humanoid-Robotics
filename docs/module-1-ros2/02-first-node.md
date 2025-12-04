---
sidebar_position: 2
title: "1.2 Building Your First Python Node"
description: Create, build, and run your first ROS 2 Python package with publisher and subscriber nodes
---

# Building Your First Python Node

In this chapter, you'll create a complete ROS 2 Python package with publisher and subscriber nodes. By the end, you'll understand the full development workflow.

## Learning Objectives

By the end of this chapter, you will:
- Create a ROS 2 Python package
- Write publisher and subscriber nodes
- Build and run your package
- Understand package structure and dependencies

---

## Prerequisites

Ensure you have:
- ROS 2 Jazzy installed
- A colcon workspace (`~/ros2_ws`)
- Python 3.11+

```bash
# Verify ROS 2 is sourced
source /opt/ros/jazzy/setup.bash
ros2 --version
```

---

## Step 1: Create the Package

Navigate to your workspace's `src` directory and create a new package:

```bash
cd ~/ros2_ws/src

ros2 pkg create --build-type ament_python --license Apache-2.0 \
    --node-name talker my_first_package
```

This creates the following structure:

```
my_first_package/
├── my_first_package/
│   ├── __init__.py
│   └── talker.py          # Our node (created by --node-name)
├── resource/
│   └── my_first_package
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml            # Package metadata
├── setup.cfg              # Python setup config
└── setup.py               # Python build script
```

---

## Step 2: Understand package.xml

The `package.xml` file declares your package's metadata and dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_first_package</name>
  <version>0.0.1</version>
  <description>My first ROS 2 Python package</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build dependencies -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

:::tip Dependencies
Add `<exec_depend>` for every ROS 2 package you import in your Python code.
:::

---

## Step 3: Configure setup.py

The `setup.py` defines entry points—the executables your package provides:

```python
from setuptools import find_packages, setup

package_name = 'my_first_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='My first ROS 2 Python package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_first_package.talker:main',
            'listener = my_first_package.listener:main',
        ],
    },
)
```

The `entry_points` section maps command names to Python functions:
- `ros2 run my_first_package talker` → calls `my_first_package.talker:main()`

---

## Step 4: Write the Publisher (Talker)

Replace the contents of `my_first_package/talker.py`:

```python
#!/usr/bin/env python3
"""
ROS 2 Publisher Node - Talker

Publishes string messages to the 'chatter' topic at 2 Hz.
Part of the Physical AI & Humanoid Robotics book, Module 1.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """A simple publisher node that sends messages to a topic."""

    def __init__(self):
        super().__init__('talker')

        # Create publisher: message type, topic name, queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer: period in seconds, callback function
        timer_period = 0.5  # 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.count = 0
        self.get_logger().info('Talker node has started')

    def timer_callback(self):
        """Called every timer_period seconds."""
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1


def main(args=None):
    """Entry point for the talker node."""
    rclpy.init(args=args)

    node = TalkerNode()

    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 5: Write the Subscriber (Listener)

Create a new file `my_first_package/listener.py`:

```python
#!/usr/bin/env python3
"""
ROS 2 Subscriber Node - Listener

Subscribes to the 'chatter' topic and logs received messages.
Part of the Physical AI & Humanoid Robotics book, Module 1.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """A simple subscriber node that receives messages from a topic."""

    def __init__(self):
        super().__init__('listener')

        # Create subscription: message type, topic name, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        self.get_logger().info('Listener node has started')

    def listener_callback(self, msg: String):
        """Called when a message is received on the topic."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Entry point for the listener node."""
    rclpy.init(args=args)

    node = ListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 6: Build the Package

Build your workspace from the workspace root:

```bash
cd ~/ros2_ws

# Build only your package (faster)
colcon build --packages-select my_first_package

# Source the workspace
source install/setup.bash
```

:::caution Always Source After Build
You must source `install/setup.bash` after every build to use the updated package.
:::

---

## Step 7: Run the Nodes

Open two terminals and run the nodes:

**Terminal 1 - Talker:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_first_package talker
```

**Terminal 2 - Listener:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_first_package listener
```

**Expected Output:**

Talker terminal:
```
[INFO] [talker]: Talker node has started
[INFO] [talker]: Publishing: "Hello, ROS 2! Count: 0"
[INFO] [talker]: Publishing: "Hello, ROS 2! Count: 1"
```

Listener terminal:
```
[INFO] [listener]: Listener node has started
[INFO] [listener]: I heard: "Hello, ROS 2! Count: 0"
[INFO] [listener]: I heard: "Hello, ROS 2! Count: 1"
```

---

## Step 8: Inspect with CLI Tools

While the nodes are running, use a third terminal to inspect:

```bash
# List running nodes
ros2 node list
# Output:
# /talker
# /listener

# Get node info
ros2 node info /talker
# Output shows publishers, subscribers, services, actions

# List topics
ros2 topic list
# Output:
# /chatter
# /parameter_events
# /rosout

# Check topic info
ros2 topic info /chatter
# Output:
# Type: std_msgs/msg/String
# Publisher count: 1
# Subscription count: 1

# Measure frequency
ros2 topic hz /chatter
# Output:
# average rate: 2.000 Hz
```

---

## Understanding the Code

### Node Lifecycle

```
rclpy.init()      → Initialize ROS 2 client library
Node.__init__()   → Create node, publishers, subscribers
rclpy.spin()      → Process callbacks until shutdown
node.destroy()    → Clean up node resources
rclpy.shutdown()  → Shutdown client library
```

### Key Components

| Component | Purpose |
|-----------|---------|
| `rclpy.init()` | Initialize ROS 2 |
| `Node` | Base class for all nodes |
| `create_publisher()` | Create a topic publisher |
| `create_subscription()` | Create a topic subscriber |
| `create_timer()` | Create a periodic callback |
| `get_logger()` | Access the node's logger |
| `rclpy.spin()` | Keep processing callbacks |

---

## Hands-On Exercises

### Exercise 2.1: Change the Message Type

Modify the talker/listener to use `std_msgs/msg/Int32` instead of `String`.

<details>
<summary>Hint</summary>

```python
from std_msgs.msg import Int32

# In publisher
msg = Int32()
msg.data = self.count

# In subscriber
def listener_callback(self, msg: Int32):
    self.get_logger().info(f'Count: {msg.data}')
```
</details>

### Exercise 2.2: Add a Parameter

Add a parameter to control the publish frequency:

```python
# In __init__
self.declare_parameter('frequency', 2.0)
freq = self.get_parameter('frequency').value
timer_period = 1.0 / freq
```

Run with custom frequency:
```bash
ros2 run my_first_package talker --ros-args -p frequency:=10.0
```

### Exercise 2.3: Multiple Subscribers

What happens if you run two listener nodes? Try it!

---

## Downloadable Code

The complete code for this chapter is available:

- [talker.py](/code/module-1/talker.py)
- [listener.py](/code/module-1/listener.py)

---

## Summary

In this chapter, you:
- Created a ROS 2 Python package with `ros2 pkg create`
- Wrote publisher and subscriber nodes
- Built with `colcon build`
- Ran and inspected nodes with CLI tools

---

## Next Steps

In the next chapter, you'll learn about URDF—the format for describing robot structure.

**Continue to:** [1.3 URDF Robot Description Format](/docs/module-1-ros2/urdf)

---

:::info Resources
- [ROS 2 Python Package Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)
:::
