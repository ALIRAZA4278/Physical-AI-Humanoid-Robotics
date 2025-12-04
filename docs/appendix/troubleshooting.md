---
sidebar_position: 3
title: Troubleshooting
description: Solutions to common issues when following the Physical AI & Humanoid Robotics book
---

# Troubleshooting

Common issues and solutions organized by module and category.

---

## General Issues

### ROS 2 Not Found

**Symptom:**
```bash
ros2: command not found
```

**Solution:**
```bash
# Source the ROS 2 setup
source /opt/ros/jazzy/setup.bash

# Add to ~/.bashrc for persistence
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Package Not Found

**Symptom:**
```bash
Package 'ros-jazzy-example' not found
```

**Solution:**
```bash
# Update package lists
sudo apt update

# Search for correct package name
apt search ros-jazzy | grep <keyword>

# Install the package
sudo apt install ros-jazzy-<package-name>
```

---

### Python Import Errors

**Symptom:**
```python
ModuleNotFoundError: No module named 'rclpy'
```

**Solution:**
```bash
# Ensure ROS 2 is sourced
source /opt/ros/jazzy/setup.bash

# Check Python path
python3 -c "import rclpy; print(rclpy.__file__)"
```

---

## Module 1: ROS 2 Issues

### Node Won't Start

**Symptom:**
```bash
[ERROR] [node_name]: Unable to initialize
```

**Solutions:**

1. **Check ROS domain ID:**
```bash
echo $ROS_DOMAIN_ID
# Should be empty or a number 0-232
```

2. **Kill zombie processes:**
```bash
killall -9 ros2
```

3. **Clear ROS 2 log files:**
```bash
rm -rf ~/.ros/log/*
```

---

### Topic Not Publishing

**Symptom:**
```bash
ros2 topic list  # Shows topic
ros2 topic echo /topic_name  # No output
```

**Solutions:**

1. **Check publisher is running:**
```bash
ros2 node list
ros2 node info /publisher_node
```

2. **Verify QoS compatibility:**
```bash
ros2 topic info /topic_name -v
```

---

### Service Timeout

**Symptom:**
```bash
[WARN] Waiting for service...
```

**Solution:**
```bash
# Verify service is available
ros2 service list | grep service_name

# Check service type matches
ros2 service type /service_name
```

---

## Module 2: Simulation Issues

### Gazebo Won't Launch

**Symptom:**
```bash
gz sim world.sdf
# Crashes or black screen
```

**Solutions:**

1. **Check OpenGL:**
```bash
glxinfo | grep "OpenGL version"
```

2. **Use software rendering:**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
gz sim world.sdf
```

3. **Update graphics drivers:**
```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

---

### Robot Falls Through Ground

**Symptom:** Robot mesh appears but drops through floor.

**Solutions:**

1. **Check collision elements in SDF/URDF**
2. **Verify physics parameters:**
```xml
<collision>
  <geometry>
    <mesh filename="model://robot/meshes/collision.dae"/>
  </geometry>
</collision>
```

---

### Sensor Data Not Publishing

**Symptom:**
```bash
ros2 topic echo /scan  # No LiDAR data
```

**Solutions:**

1. **Check sensor plugin loaded:**
```bash
gz topic -l | grep lidar
```

2. **Verify ROS bridge running:**
```bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

---

## Module 3: Isaac Issues

### Isaac Sim Won't Start

**Symptom:**
```bash
./isaac-sim.sh
# Crashes on startup
```

**Solutions:**

1. **Check GPU driver:**
```bash
nvidia-smi
# Must show driver 535.129+
```

2. **Verify VRAM:**
```bash
nvidia-smi --query-gpu=memory.total --format=csv
# Must be 8GB+
```

3. **Clear cache:**
```bash
rm -rf ~/.cache/ov
rm -rf ~/.local/share/ov
```

---

### CUDA Out of Memory

**Symptom:**
```
RuntimeError: CUDA out of memory
```

**Solutions:**

1. **Close other GPU applications**
2. **Reduce scene complexity**
3. **Lower rendering resolution:**
```python
carb.settings.get_settings().set("/app/window/width", 1280)
carb.settings.get_settings().set("/app/window/height", 720)
```

---

### Isaac ROS Package Missing

**Symptom:**
```bash
Package 'isaac_ros_vslam' not found
```

**Solution:**
```bash
# Clone Isaac ROS
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam

# Build
cd ~/workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

---

## Module 4: VLA Issues

### Whisper Not Responding

**Symptom:**
```python
import whisper
model = whisper.load_model("base")
# Hangs or crashes
```

**Solutions:**

1. **Install with CUDA support:**
```bash
pip uninstall whisper
pip install openai-whisper
```

2. **Check ffmpeg:**
```bash
ffmpeg -version
# If missing:
sudo apt install ffmpeg
```

---

### Microphone Not Detected

**Symptom:**
```bash
ALSA: Cannot open audio device
```

**Solutions:**

1. **List audio devices:**
```bash
arecord -l
```

2. **Set default device:**
```bash
export AUDIODEV=hw:1,0  # Adjust to your device
```

3. **Check permissions:**
```bash
sudo usermod -aG audio $USER
# Log out and back in
```

---

### LLM API Timeout

**Symptom:**
```
openai.error.Timeout: Request timed out
```

**Solutions:**

1. **Check API key:**
```bash
echo $OPENAI_API_KEY
```

2. **Increase timeout:**
```python
import openai
openai.api_request_timeout = 60  # seconds
```

3. **Use local model if needed:**
```python
# Consider Ollama for local LLM
# ollama.com
```

---

## Build Issues

### npm run build Fails

**Symptom:**
```bash
npm run build
# Error during build
```

**Solutions:**

1. **Clear cache:**
```bash
npm run clear
rm -rf node_modules
npm install
```

2. **Check Node version:**
```bash
node --version
# Must be 18+
```

3. **Fix broken links:**
```bash
# Check for 404s in build output
# Fix markdown link paths
```

---

### TypeScript Errors

**Symptom:**
```
TS2307: Cannot find module
```

**Solution:**
```bash
# Regenerate types
npx docusaurus clear
npm install
```

---

## Getting Help

If these solutions don't resolve your issue:

1. **Search existing issues:** [GitHub Issues](https://github.com/your-github-username/my-research-paper/issues)

2. **Create a new issue** with:
   - Module and chapter number
   - Full error message
   - Steps to reproduce
   - System information:
   ```bash
   ros2 --version
   python3 --version
   nvidia-smi
   uname -a
   ```

3. **Community resources:**
   - [ROS Discourse](https://discourse.ros.org/)
   - [NVIDIA Isaac Forum](https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac)
   - [Gazebo Community](https://community.gazebosim.org/)
