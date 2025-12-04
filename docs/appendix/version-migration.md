---
sidebar_position: 2
title: Version Migration Guide
description: Instructions for running book examples on different ROS 2 and software versions
---

# Version Migration Guide

This book targets ROS 2 Jazzy (2024-2029 LTS). If you're using different versions, this guide explains necessary adaptations.

---

## Primary Target Versions

| Software | Version | Support Window |
|----------|---------|----------------|
| ROS 2 | **Jazzy Jalisco** | 2024-2029 (5-year LTS) |
| Ubuntu | **24.04 LTS** | 2024-2034 |
| Gazebo | **Harmonic** | Paired with Jazzy |
| Python | **3.11** | Isaac Sim requirement |
| Isaac Sim | **5.0+** | Current stable |

---

## ROS 2 Version Migration

### From Jazzy to Humble (2022-2027)

If you're on Ubuntu 22.04 with ROS 2 Humble:

#### Package Name Changes

Most packages work identically. Key differences:

```bash
# Jazzy
sudo apt install ros-jazzy-desktop

# Humble
sudo apt install ros-humble-desktop
```

#### Setup Script

```bash
# Jazzy
source /opt/ros/jazzy/setup.bash

# Humble
source /opt/ros/humble/setup.bash
```

#### Gazebo Version

```bash
# Jazzy uses Gazebo Harmonic
ros2 pkg list | grep gz

# Humble uses Gazebo Fortress (older)
ros2 pkg list | grep ignition
```

#### Code Changes

Most rclpy code is identical. Check API changes in:
- [ROS 2 Humble Migration Guide](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)

---

### From Jazzy to Iron (2023-2024)

Iron is not an LTS release (1-year support). Migration is straightforward:

```bash
# Package installation
sudo apt install ros-iron-desktop

# Source
source /opt/ros/iron/setup.bash
```

---

## Gazebo Version Migration

### Harmonic (Jazzy) vs Fortress (Humble)

| Feature | Harmonic | Fortress |
|---------|----------|----------|
| Physics | Bullet, DART, TPE | Same |
| ROS 2 bridge | `ros_gz_bridge` | `ros_ign_bridge` |
| CLI prefix | `gz` | `ign` |
| Python API | `gz.sim` | `ignition.gazebo` |

#### Command Examples

```bash
# Harmonic (Jazzy)
gz sim world.sdf

# Fortress (Humble)
ign gazebo world.sdf
```

#### ROS 2 Bridge Package

```bash
# Harmonic
ros2 run ros_gz_bridge parameter_bridge

# Fortress
ros2 run ros_ign_bridge parameter_bridge
```

---

## Isaac Sim Migration

### Version 5.0 vs 4.x

| Feature | 5.0+ | 4.x |
|---------|------|-----|
| ROS 2 | Jazzy/Humble | Humble only |
| Python | 3.11 | 3.10 |
| Isaac ROS | 3.0+ | 2.x |

#### Extension Changes

```python
# Isaac Sim 5.0+
from isaacsim import SimulationApp

# Isaac Sim 4.x (deprecated)
from omni.isaac.kit import SimulationApp
```

Check the [Isaac Sim Release Notes](https://docs.isaacsim.omniverse.nvidia.com/latest/release_notes.html) for detailed API changes.

---

## Python Version Considerations

### Python 3.11 vs 3.10

Isaac Sim 5.0+ requires Python 3.11. Most code is compatible, but check:

#### Type Hints

```python
# Python 3.11 (preferred)
def process(data: list[int]) -> dict[str, int]:
    pass

# Python 3.10 compatible
from typing import List, Dict
def process(data: List[int]) -> Dict[str, int]:
    pass
```

#### Using pyenv

If your system has Python 3.10:

```bash
# Install pyenv
curl https://pyenv.run | bash

# Install Python 3.11
pyenv install 3.11
pyenv global 3.11

# Verify
python --version  # Should show 3.11.x
```

---

## Nav2 Migration

### Nav2 1.2+ (Jazzy) vs 1.1 (Humble)

Most configuration is identical. Key changes:

#### Parameter File Format

```yaml
# Nav2 1.2+ (some new parameters)
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    use_realtime_priority: true  # New in 1.2

# Nav2 1.1 (omit new parameters)
controller_server:
  ros__parameters:
    controller_frequency: 20.0
```

---

## Whisper Migration

OpenAI Whisper has stable API. Use latest version:

```bash
# Latest (recommended)
pip install openai-whisper

# Specific version if needed
pip install openai-whisper==20231117
```

---

## Version Compatibility Matrix

| Book Version | ROS 2 | Ubuntu | Gazebo | Isaac Sim | Python |
|--------------|-------|--------|--------|-----------|--------|
| 1.0 (Current) | Jazzy | 24.04 | Harmonic | 5.0+ | 3.11 |
| Fallback | Humble | 22.04 | Fortress | 4.x | 3.10 |

---

## Reporting Issues

If you encounter version-specific issues:

1. Check the [GitHub Issues](https://github.com/your-github-username/my-research-paper/issues)
2. Include your version output:

```bash
ros2 --version
gz sim --version
python --version
nvidia-smi
```

---

## Next Steps

- [Hardware Requirements](/docs/appendix/hardware-requirements) — Verify your setup
- [Troubleshooting](/docs/appendix/troubleshooting) — Common version issues
