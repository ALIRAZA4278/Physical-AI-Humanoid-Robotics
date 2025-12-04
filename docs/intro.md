---
sidebar_position: 1
title: Introduction
description: Physical AI & Humanoid Robotics - From ROS 2 to Voice-Controlled Autonomous Humanoids
---

# Physical AI & Humanoid Robotics

Welcome to **Physical AI & Humanoid Robotics**, a comprehensive 4-module course that takes you from ROS 2 fundamentals to building voice-controlled autonomous humanoid robots.

## What You'll Learn

This book provides a progressive learning path through the essential technologies of modern humanoid robotics:

| Module | Focus | Hardware |
|--------|-------|----------|
| **Module 1** | ROS 2 Fundamentals | CPU only |
| **Module 2** | Digital Twin Simulation | CPU (GPU optional) |
| **Module 3** | NVIDIA Isaac AI | RTX 3070+ required |
| **Module 4** | VLA Capstone | RTX 3070+ required |

## Learning Approach

This book follows a **research-concurrent methodology**—every concept is verified against official documentation and tested on real hardware. You'll work with:

- **ROS 2 Jazzy**: The latest Long-Term Support release (supported until 2029)
- **iCub Humanoid**: A proven educational robot model with complete URDF/SDF files
- **Gazebo Harmonic**: Industry-standard physics simulation
- **NVIDIA Isaac Sim**: Photorealistic simulation with hardware-accelerated AI
- **OpenAI Whisper**: State-of-the-art speech recognition

## Target Audience

This book is designed for developers with:

- **1-3 years** of programming experience
- Familiarity with **Python** and **Linux command line**
- Interest in **robotics**, **AI**, or **autonomous systems**
- Access to **Ubuntu 24.04 LTS** (or 22.04 for Humble fallback)

No prior robotics experience is required—we start from fundamentals.

## Hardware Requirements

### Modules 1-2 (CPU Only)

- **OS**: Ubuntu 24.04 LTS
- **RAM**: 16GB minimum
- **Storage**: 50GB free space
- **CPU**: Modern x86_64 processor

### Modules 3-4 (GPU Required)

- **GPU**: NVIDIA RTX 3070 (8GB VRAM) minimum
- **Recommended**: RTX 4080 (16GB VRAM)
- **Driver**: 535.129 or newer
- **Optional**: Jetson Orin NX for edge deployment

## Quick Start

Ready to begin? Follow these steps:

```bash
# 1. Install ROS 2 Jazzy (Ubuntu 24.04)
sudo apt update && sudo apt install ros-jazzy-desktop

# 2. Source the environment
source /opt/ros/jazzy/setup.bash

# 3. Verify installation
ros2 --version
```

:::tip 30-Minute Setup
See the [Quickstart Guide](/docs/appendix/hardware-requirements) for detailed installation instructions.
:::

## Book Structure

### Module 1: ROS 2 Fundamentals

Learn the building blocks of robotic systems:
- ROS 2 architecture (nodes, topics, services, actions)
- Building your first Python node
- URDF robot description format
- Development environment setup

### Module 2: Digital Twin Simulation

Create realistic robotic simulations:
- Gazebo Harmonic + ROS 2 integration
- SDF robot modeling
- Sensor simulation (LiDAR, depth camera, IMU)
- Unity visualization (optional)

### Module 3: NVIDIA Isaac AI

Deploy AI perception and navigation:
- Isaac Sim setup and environment
- Perception pipelines (cuVSLAM, nvblox)
- Nav2 navigation for bipedal robots

### Module 4: VLA Capstone

Build a complete voice-controlled humanoid:
- Whisper voice-to-text integration
- LLM-based cognitive planning
- Full VLA (Vision-Language-Action) pipeline

## Reference Robot: iCub

Throughout this book, we use the **iCub humanoid robot** as our reference model:

- **53 degrees of freedom**
- **Complete URDF and SDF models**
- **Proven educational platform**
- **Open-source**: [robotology/icub-models](https://github.com/robotology/icub-models)

## Next Steps

Ready to start your journey into Physical AI?

1. **[Module 1: ROS 2 Architecture](/docs/module-1-ros2/architecture)** — Start here
2. **[Hardware Requirements](/docs/appendix/hardware-requirements)** — Verify your setup
3. **[Glossary](/docs/glossary)** — Reference technical terms

---

:::info Citation
This book uses official documentation from ROS 2, NVIDIA Isaac, Gazebo, and OpenAI Whisper as primary sources. All code examples are tested on the specified platforms.
:::
