---
sidebar_position: 4
title: "1.4 Development Environment Setup"
description: Complete guide to setting up your ROS 2 Jazzy development environment on Ubuntu 24.04
---

# Development Environment Setup

This chapter provides a complete guide to setting up your ROS 2 development environment. Follow these steps to prepare for the hands-on exercises throughout this book.

## Learning Objectives

By the end of this chapter, you will have:
- ROS 2 Jazzy installed on Ubuntu 24.04
- A configured colcon workspace
- Essential development tools
- A verification script to confirm your setup

---

## System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **OS** | Ubuntu 24.04 LTS | Ubuntu 24.04 LTS |
| **RAM** | 8GB | 16GB+ |
| **Storage** | 30GB free | 50GB+ free |
| **CPU** | 4 cores | 8+ cores |

---

## Step 1: Install ROS 2 Jazzy

### Set Locale

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Add ROS 2 Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 Desktop

```bash
sudo apt update
sudo apt install ros-jazzy-desktop
```

This installs:
- ROS 2 core libraries
- RViz2 visualization
- Demo nodes
- Common message types

### Source the Environment

```bash
# Add to ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify Installation

```bash
ros2 --version
# Expected: ros2 0.10.x

ros2 run demo_nodes_cpp talker
# Should see: Publishing: 'Hello World: X'
```

---

## Step 2: Install Development Tools

### Colcon Build Tool

```bash
sudo apt install python3-colcon-common-extensions
```

### rosdep Dependency Manager

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### Additional Tools

```bash
# Code formatting
sudo apt install python3-autopep8 python3-flake8

# Debugging
sudo apt install ros-jazzy-rqt ros-jazzy-rqt-common-plugins

# URDF visualization
sudo apt install ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-robot-state-publisher \
                 ros-jazzy-xacro
```

---

## Step 3: Create Your Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build empty workspace (creates install/setup.bash)
colcon build

# Source the workspace
source install/setup.bash

# Add to ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Workspace Structure

```
~/ros2_ws/
├── src/           # Your packages go here
├── build/         # Build artifacts (generated)
├── install/       # Installed packages (generated)
└── log/           # Build logs (generated)
```

---

## Step 4: Configure Shell Environment

Add these lines to your `~/.bashrc`:

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Your workspace
source ~/ros2_ws/install/setup.bash

# Colcon autocomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Optional: ROS 2 domain ID (for multi-machine setups)
# export ROS_DOMAIN_ID=42

# Optional: Disable ROS 2 localhost-only mode
# export ROS_LOCALHOST_ONLY=0

# Useful aliases
alias cb='cd ~/ros2_ws && colcon build'
alias cbs='cd ~/ros2_ws && colcon build --symlink-install'
alias cbp='cd ~/ros2_ws && colcon build --packages-select'
alias cs='source ~/ros2_ws/install/setup.bash'
```

Reload:
```bash
source ~/.bashrc
```

---

## Step 5: IDE Setup (VS Code)

### Install VS Code

```bash
sudo snap install code --classic
```

### Install Extensions

Install these extensions for ROS 2 development:

1. **ROS** (ms-iot.vscode-ros)
2. **Python** (ms-python.python)
3. **C/C++** (ms-vscode.cpptools)
4. **YAML** (redhat.vscode-yaml)
5. **XML** (redhat.vscode-xml)

### Configure Settings

Create `.vscode/settings.json` in your workspace:

```json
{
    "python.analysis.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages",
        "/opt/ros/jazzy/local/lib/python3.12/dist-packages"
    ],
    "ros.distro": "jazzy",
    "files.associations": {
        "*.urdf": "xml",
        "*.sdf": "xml",
        "*.xacro": "xml"
    }
}
```

---

## Step 6: Verification Script

Create and run this script to verify your setup:

```bash
#!/bin/bash
# File: ~/verify_setup.sh

echo "=== ROS 2 Development Environment Verification ==="
echo ""

# ROS 2
echo -n "1. ROS 2 Jazzy: "
if ros2 --version 2>/dev/null | grep -q "0.10"; then
    echo "✓ INSTALLED"
else
    echo "✗ NOT FOUND"
fi

# Python
echo -n "2. Python 3.11+: "
python_version=$(python3 --version 2>&1 | cut -d' ' -f2)
if [[ "$python_version" > "3.11" ]]; then
    echo "✓ $python_version"
else
    echo "✗ $python_version (need 3.11+)"
fi

# Colcon
echo -n "3. Colcon: "
if command -v colcon &> /dev/null; then
    echo "✓ INSTALLED"
else
    echo "✗ NOT FOUND"
fi

# Workspace
echo -n "4. Workspace: "
if [ -d ~/ros2_ws/src ]; then
    echo "✓ ~/ros2_ws exists"
else
    echo "✗ ~/ros2_ws NOT FOUND"
fi

# RViz2
echo -n "5. RViz2: "
if command -v rviz2 &> /dev/null; then
    echo "✓ INSTALLED"
else
    echo "✗ NOT FOUND"
fi

# rosdep
echo -n "6. rosdep: "
if command -v rosdep &> /dev/null; then
    echo "✓ INSTALLED"
else
    echo "✗ NOT FOUND"
fi

echo ""
echo "=== Environment Variables ==="
echo "ROS_DISTRO: ${ROS_DISTRO:-NOT SET}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0 (default)}"
echo "AMENT_PREFIX_PATH: ${AMENT_PREFIX_PATH:0:50}..."

echo ""
echo "=== Verification Complete ==="
```

Run it:
```bash
chmod +x ~/verify_setup.sh
~/verify_setup.sh
```

**Expected Output:**
```
=== ROS 2 Development Environment Verification ===

1. ROS 2 Jazzy: ✓ INSTALLED
2. Python 3.11+: ✓ 3.12.3
3. Colcon: ✓ INSTALLED
4. Workspace: ✓ ~/ros2_ws exists
5. RViz2: ✓ INSTALLED
6. rosdep: ✓ INSTALLED

=== Environment Variables ===
ROS_DISTRO: jazzy
ROS_DOMAIN_ID: 0 (default)
AMENT_PREFIX_PATH: /home/user/ros2_ws/install:/opt/ros/jazzy...

=== Verification Complete ===
```

---

## Common Issues

### "ros2: command not found"

```bash
source /opt/ros/jazzy/setup.bash
```

### "colcon build" fails with import errors

```bash
# Ensure using system Python
which python3
# Should be /usr/bin/python3

# Not a virtual environment
deactivate  # if in venv
```

### RViz2 shows no robot model

1. Check `robot_state_publisher` is running
2. Verify Fixed Frame is set correctly
3. Check `/robot_description` topic has data

---

## Summary

Your development environment now includes:

| Component | Purpose |
|-----------|---------|
| ROS 2 Jazzy | Core robotics framework |
| Colcon | Build tool |
| rosdep | Dependency management |
| RViz2 | Visualization |
| VS Code | IDE with ROS extensions |

---

## Module 1 Complete!

Congratulations! You've completed Module 1: ROS 2 Fundamentals. You now understand:
- ROS 2 architecture (nodes, topics, services, actions)
- Building Python packages
- URDF robot descriptions
- Development environment setup

---

## Next Steps

Continue to Module 2 to learn simulation with Gazebo:

**Continue to:** [Module 2: Digital Twin Simulation](/docs/module-2-simulation/gazebo-setup)

---

## Downloadable Resources

- [verify_setup.sh](/code/module-1/setup_verify.sh) - Environment verification script

---

:::info Resources
- [ROS 2 Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
- [VS Code ROS Extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
- [Colcon Documentation](https://colcon.readthedocs.io/)
:::
