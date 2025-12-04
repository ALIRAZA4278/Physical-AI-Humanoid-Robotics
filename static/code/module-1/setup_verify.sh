#!/bin/bash
# ROS 2 Development Environment Verification Script
# Physical AI & Humanoid Robotics Book, Module 1
#
# Usage:
#   chmod +x setup_verify.sh
#   ./setup_verify.sh

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
