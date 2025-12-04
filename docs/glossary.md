---
sidebar_position: 6
title: Glossary
description: Technical terms and definitions used throughout the Physical AI & Humanoid Robotics book
---

# Glossary

Technical terms and definitions used throughout this book, organized alphabetically.

---

## A

### Action (ROS 2)
A communication pattern in ROS 2 for long-running tasks that provide feedback and can be preempted. Actions combine goal request, result, and continuous feedback.

### AMCL
Adaptive Monte Carlo Localization. A probabilistic localization system that uses particle filters to track a robot's pose against a known map.

---

## C

### Colcon
The build tool for ROS 2 workspaces. Replaces catkin from ROS 1.

### cuVSLAM
NVIDIA's CUDA-accelerated Visual Simultaneous Localization and Mapping implementation in Isaac ROS.

---

## D

### DDS
Data Distribution Service. The middleware standard underlying ROS 2 communication, providing publish-subscribe messaging.

### Digital Twin
A virtual representation of a physical robot that mirrors its real-world counterpart in simulation.

---

## G

### Gazebo
An open-source 3D robotics simulator with physics engine, sensor simulation, and ROS integration. Gazebo Harmonic is the version paired with ROS 2 Jazzy.

---

## I

### Isaac ROS
NVIDIA's hardware-accelerated ROS 2 packages for perception, including cuVSLAM, nvblox, and Freespace Segmentation.

### Isaac Sim
NVIDIA's photorealistic simulation platform built on Omniverse, designed for AI robot training and testing.

### IMU
Inertial Measurement Unit. A sensor that measures acceleration and angular velocity, essential for robot balance.

---

## L

### LiDAR
Light Detection and Ranging. A sensor that measures distances using laser pulses to create 3D point clouds.

### LLM
Large Language Model. In VLA pipelines, LLMs translate natural language commands into robot action sequences.

---

## N

### Nav2
The Navigation 2 stack for ROS 2, providing path planning, localization, and obstacle avoidance.

### Node
The basic executable unit in ROS 2 that performs computation and communicates via topics, services, and actions.

### nvblox
NVIDIA's real-time 3D reconstruction library for building occupancy maps from depth sensors.

---

## P

### Publisher
A ROS 2 entity that sends messages on a topic.

### Python rclpy
The Python client library for ROS 2, providing APIs for nodes, publishers, subscribers, services, and actions.

---

## R

### ROS 2
Robot Operating System 2. A set of software libraries and tools for building robot applications. Version Jazzy is the LTS release supported until 2029.

---

## S

### SDF
Simulation Description Format. An XML format for describing robots and environments in Gazebo, more expressive than URDF for simulation properties.

### Service
A synchronous request-response communication pattern in ROS 2.

### Subscriber
A ROS 2 entity that receives messages from a topic.

---

## T

### Topic
A named bus over which nodes exchange messages in ROS 2's publish-subscribe pattern.

### TF2
The ROS 2 transform library that maintains relationships between coordinate frames over time.

---

## U

### URDF
Unified Robot Description Format. An XML format for describing robot kinematics, dynamics, and visual elements.

---

## V

### VLA
Vision-Language-Action. An AI architecture that connects visual perception, language understanding, and robot actions.

### VSLAM
Visual Simultaneous Localization and Mapping. Using camera images to simultaneously build a map and localize within it.

---

## W

### Whisper
OpenAI's automatic speech recognition model used for voice-to-text in VLA pipelines.

---

:::tip Adding Terms
This glossary will be updated as new chapters introduce additional technical concepts.
:::
