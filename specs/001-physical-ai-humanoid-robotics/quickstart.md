# Quickstart Guide: Physical AI & Humanoid Robotics Book

**Purpose**: Get readers set up and running the first example in under 30 minutes.

---

## Prerequisites

Before starting, ensure you have:

- **Computer**: x86_64 system with 16GB+ RAM
- **Operating System**: Ubuntu 24.04 LTS (or 22.04 for Humble fallback)
- **Internet**: For downloading packages and dependencies
- **Optional GPU**: NVIDIA RTX 3070+ (required for Module 3-4)

---

## Quick Setup (Module 1-2)

### Step 1: Install ROS 2 Jazzy (5 minutes)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop
```

### Step 2: Configure Environment (1 minute)

```bash
# Add to ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

**Expected Output:**
```
ros2 0.10.x
```

### Step 3: Create Workspace (2 minutes)

```bash
# Create colcon workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build (empty workspace)
colcon build
source install/setup.bash
```

### Step 4: Run Your First Node (2 minutes)

```bash
# Terminal 1: Run talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Run listener
ros2 run demo_nodes_cpp listener
```

**Expected Output (Terminal 2):**
```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
```

**Congratulations!** You're ready for Module 1.

---

## Full Setup (All Modules)

### Additional Requirements for Module 3-4

#### Check GPU Compatibility

```bash
# Check NVIDIA driver
nvidia-smi
```

**Minimum Requirements:**
- Driver: 535.129+
- GPU: RTX 3070 (8GB VRAM) minimum
- Recommended: RTX 4080 (16GB VRAM)

#### Install Isaac Sim (Module 3)

```bash
# Download Omniverse Launcher
# Visit: https://developer.nvidia.com/omniverse

# Install Isaac Sim via Omniverse
# Follow: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html
```

#### Install Whisper (Module 4)

```bash
# Install OpenAI Whisper
pip install openai-whisper

# Test installation
whisper --version
```

---

## Verify Complete Setup

Run this verification script:

```bash
#!/bin/bash
# File: verify_setup.sh

echo "=== Physical AI & Humanoid Robotics Setup Verification ==="
echo ""

# ROS 2
echo -n "ROS 2: "
if command -v ros2 &> /dev/null; then
    ros2 --version 2>/dev/null || echo "Installed"
else
    echo "NOT INSTALLED"
fi

# Gazebo
echo -n "Gazebo: "
if command -v gz &> /dev/null; then
    gz sim --version 2>/dev/null | head -1 || echo "Installed"
else
    echo "NOT INSTALLED (optional for Module 2)"
fi

# NVIDIA GPU
echo -n "NVIDIA GPU: "
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo "Detected"
else
    echo "NOT DETECTED (required for Module 3-4)"
fi

# Python
echo -n "Python: "
python3 --version 2>/dev/null || echo "NOT INSTALLED"

# Whisper
echo -n "Whisper: "
if python3 -c "import whisper" 2>/dev/null; then
    echo "Installed"
else
    echo "NOT INSTALLED (optional for Module 4)"
fi

echo ""
echo "=== Verification Complete ==="
```

Save and run:

```bash
chmod +x verify_setup.sh
./verify_setup.sh
```

**Expected Output (Modules 1-2 only):**
```
=== Physical AI & Humanoid Robotics Setup Verification ===

ROS 2: ros2 0.10.x
Gazebo: NOT INSTALLED (optional for Module 2)
NVIDIA GPU: NOT DETECTED (required for Module 3-4)
Python: Python 3.11.x
Whisper: NOT INSTALLED (optional for Module 4)

=== Verification Complete ===
```

---

## Learning Path

| Module | Prerequisites | Time Estimate | Hardware |
|--------|--------------|---------------|----------|
| 1: ROS 2 | Linux basics, Python | 10-15 hours | CPU only |
| 2: Simulation | Module 1 | 12-15 hours | GPU optional |
| 3: Isaac AI | Module 2 | 15-20 hours | RTX 3070+ required |
| 4: VLA Capstone | Modules 1-3 | 15-20 hours | RTX 3070+ required |

---

## Troubleshooting

### ROS 2 Not Found

```bash
# Ensure setup is sourced
source /opt/ros/jazzy/setup.bash
```

### Python Version Mismatch

Isaac Sim requires Python 3.11. Check your version:

```bash
python3 --version
```

If not 3.11, use pyenv:

```bash
# Install pyenv
curl https://pyenv.run | bash

# Install Python 3.11
pyenv install 3.11
pyenv global 3.11
```

### GPU Not Detected

1. Check driver installation: `nvidia-smi`
2. Reinstall NVIDIA driver if needed:
   ```bash
   sudo apt install nvidia-driver-535
   sudo reboot
   ```

---

## Next Steps

1. **Start Module 1**: [ROS 2 Architecture Essentials](./module-1-ros2/01-architecture.md)
2. **Clone Example Code**: `git clone https://github.com/your-repo/physical-ai-examples`
3. **Join Community**: [ROS Discourse](https://discourse.ros.org/)

---

## Version Reference

| Component | Version | Notes |
|-----------|---------|-------|
| Ubuntu | 24.04 LTS | Or 22.04 for Humble |
| ROS 2 | Jazzy | Primary target |
| Python | 3.11 | Isaac Sim requirement |
| Gazebo | Harmonic | For simulation |
| Isaac Sim | 5.0+ | Module 3-4 only |
| Docusaurus | 3.6+ | Book framework |
