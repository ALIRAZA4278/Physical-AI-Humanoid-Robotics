---
sidebar_position: 3
title: "1.3 URDF Robot Description Format"
description: Learn to read, write, and visualize robot descriptions using the Unified Robot Description Format
---

# URDF Robot Description Format

URDF (Unified Robot Description Format) is how robots describe themselves in ROS 2. In this chapter, you'll learn to read, write, and visualize URDF files.

## Learning Objectives

By the end of this chapter, you will:
- Understand URDF structure (links, joints, visual, collision)
- Read and interpret existing URDF files
- Create a simple robot description
- Visualize URDFs in RViz2

---

## What is URDF?

**URDF** is an XML format that describes:
- **Links**: Rigid bodies (the "bones")
- **Joints**: Connections between links (the "joints")
- **Visual**: How the robot looks (meshes, colors)
- **Collision**: Simplified geometry for physics
- **Inertial**: Mass and moment of inertia

---

## URDF Structure Overview

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Links define the rigid bodies -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>

  <link name="wheel_link">...</link>

  <!-- Joints connect links -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

---

## Links

A **link** represents a rigid body with three components:

### Visual Element

How the link appears in visualization:

```xml
<link name="base_link">
  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
</link>
```

### Geometry Types

```xml
<!-- Box: length x width x height -->
<geometry>
  <box size="1.0 0.5 0.2"/>
</geometry>

<!-- Cylinder: radius and length -->
<geometry>
  <cylinder radius="0.1" length="0.5"/>
</geometry>

<!-- Sphere: radius -->
<geometry>
  <sphere radius="0.1"/>
</geometry>

<!-- Mesh: external 3D model file -->
<geometry>
  <mesh filename="package://my_robot/meshes/body.dae" scale="1 1 1"/>
</geometry>
```

### Collision Element

Simplified geometry for physics simulation:

```xml
<collision>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <geometry>
    <box size="0.5 0.3 0.2"/>
  </geometry>
</collision>
```

:::tip Performance
Use simple primitives (boxes, cylinders) for collision even if visual uses complex meshes.
:::

### Inertial Element

Mass and moment of inertia (required for simulation):

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="10.0"/>
  <inertia ixx="0.1" ixy="0" ixz="0"
           iyy="0.1" iyz="0"
           izz="0.1"/>
</inertial>
```

---

## Joints

A **joint** connects two links and defines their relative motion:

### Joint Types

| Type | Motion | Example |
|------|--------|---------|
| `fixed` | None | Sensor mount |
| `revolute` | Rotation with limits | Arm joint |
| `continuous` | Unlimited rotation | Wheel |
| `prismatic` | Linear sliding | Elevator |
| `floating` | 6-DOF | Free-floating base |
| `planar` | 2D translation + rotation | Mobile base |

### Joint Definition

```xml
<joint name="shoulder_joint" type="revolute">
  <!-- Parent link (fixed reference) -->
  <parent link="torso_link"/>

  <!-- Child link (moves relative to parent) -->
  <child link="upper_arm_link"/>

  <!-- Position and orientation of joint frame -->
  <origin xyz="0 0.2 0.5" rpy="0 0 0"/>

  <!-- Axis of rotation/translation -->
  <axis xyz="0 1 0"/>  <!-- Y-axis rotation -->

  <!-- Joint limits (for revolute/prismatic) -->
  <limit lower="-1.57" upper="1.57"
         effort="100" velocity="1.0"/>
</joint>
```

### Origin and Frames

The `origin` element positions the child link relative to the parent:
- `xyz`: Translation (meters)
- `rpy`: Rotation as roll-pitch-yaw (radians)

```xml
<!-- Child is 0.5m above and rotated 90° around Z -->
<origin xyz="0 0 0.5" rpy="0 0 1.5708"/>
```

---

## Example: iCub Humanoid Excerpt

The iCub humanoid robot demonstrates a complex URDF structure. Here's a simplified excerpt of its head:

```xml
<?xml version="1.0"?>
<robot name="icub_head">

  <!-- Neck base -->
  <link name="neck_base">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Head link -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.05"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.05"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck pitch joint -->
  <joint name="neck_pitch" type="revolute">
    <parent link="neck_base"/>
    <child link="head"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7" upper="0.5" effort="50" velocity="2.0"/>
  </joint>

  <!-- Left eye link -->
  <link name="left_eye">
    <visual>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
  </link>

  <!-- Left eye joint -->
  <joint name="left_eye_joint" type="fixed">
    <parent link="head"/>
    <child link="left_eye"/>
    <origin xyz="0.07 0.03 0.08" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## Visualizing URDF with RViz2

### Create a Launch File

Create `display.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf',
        'robot.urdf'
    )

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['cat ', urdf_file])
            }]
        ),

        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('my_robot'),
                'rviz',
                'display.rviz'
            )]
        ),
    ])
```

### Run Visualization

```bash
# Install required packages
sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-robot-state-publisher

# Launch
ros2 launch my_robot display.launch.py
```

### RViz2 Configuration

In RViz2:
1. Add `RobotModel` display
2. Set `Description Topic` to `/robot_description`
3. Set `Fixed Frame` to `base_link`

---

## URDF vs SDF vs MJCF

| Format | Primary Use | Strengths |
|--------|-------------|-----------|
| **URDF** | ROS visualization, kinematics | Standard in ROS, wide support |
| **SDF** | Gazebo simulation | Better physics, sensors, world |
| **MJCF** | MuJoCo simulation | Fast physics, reinforcement learning |

:::info Module 2 Preview
In Module 2, you'll learn SDF for Gazebo simulation. SDF extends URDF with simulation-specific features.
:::

---

## Hands-On Exercise

### Exercise 3.1: Build a Simple Arm

Create a 2-DOF robot arm with:
- `base_link`: Fixed base (box)
- `link_1`: Upper arm (cylinder)
- `link_2`: Forearm (cylinder)
- `shoulder_joint`: Revolute (connects base to link_1)
- `elbow_joint`: Revolute (connects link_1 to link_2)

<details>
<summary>Solution</summary>

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <link name="link_1">
    <visual>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <link name="link_2">
    <visual>
      <origin xyz="0 0 0.125"/>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <joint name="elbow_joint" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="50" velocity="1.0"/>
  </joint>

</robot>
```
</details>

---

## Downloadable Resources

- icub_sample.urdf - Simplified iCub head URDF (available in static/code/module-1/)
- simple_arm.urdf - Exercise solution (available in static/code/module-1/)

---

## Summary

In this chapter, you learned:
- URDF structure: links, joints, visual, collision, inertial
- Joint types and their use cases
- How to visualize URDFs in RViz2
- The relationship between URDF, SDF, and MJCF

---

## Next Steps

In the final chapter of Module 1, you'll set up your complete development environment.

**Continue to:** [1.4 Development Environment Setup](/docs/module-1-ros2/environment)

---

:::info Resources
- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [iCub Models Repository](https://github.com/robotology/icub-models)
- [URDF Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
:::
