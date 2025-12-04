# Research Findings: Physical AI & Humanoid Robotics Book

**Date**: 2025-12-04
**Feature**: 001-physical-ai-humanoid-robotics

## Executive Summary

This research consolidates technical decisions for the 4-module Physical AI & Humanoid Robotics book. All decisions prioritize educational accessibility while maintaining technical accuracy per constitution principles.

---

## 1. ROS 2 Version Selection

### Decision: ROS 2 Jazzy Jalisco (Primary), Humble (Fallback)

**Rationale**:
- Jazzy offers 5-year LTS support until 2029 (vs Humble's 2027)
- Uses modern Gazebo Harmonic (better physics simulation)
- Strong community adoption trajectory in 2025
- Full compatibility with Isaac Sim 4.5+ and Nav2

**Alternatives Considered**:

| Version | Status | Why Not Primary |
|---------|--------|-----------------|
| Humble | Fallback | Shorter LTS, Gazebo Classic less future-proof |
| Iron | Excluded | End-of-Life, no longer supported |
| Kilted | Excluded | Too new (2026), not stable for education |

**Version Specification**:
```yaml
Primary: ROS 2 Jazzy (May 2024)
Ubuntu: 24.04 LTS (Noble)
Fallback: ROS 2 Humble on Ubuntu 22.04
```

---

## 2. Humanoid Robot Model Selection

### Decision: iCub (robotology/icub-models)

**Rationale**:
- Proven educational track record in university robotics programs worldwide
- Complete URDF/SDF models with multiple variants
- Excellent documentation and active community
- Works with both Gazebo Harmonic and Isaac Sim (community-supported)
- Full sensor suite support (cameras, IMU, force/torque sensors)

**Alternatives Considered**:

| Robot | Pros | Why Not Primary |
|-------|------|-----------------|
| Unitree H1 | Strong Isaac integration | Weaker Gazebo ecosystem, less educational docs |
| NAO | Good educational support | Outdated, smaller community |
| Atlas-like | No licensing | No existing quality URDF |
| Digit | Modern bipedal | Newer ecosystem, less documentation |

**Model Variants for Book**:
- `iCubGazebo` - Optimized for Gazebo Harmonic (Modules 1-2)
- `iCubSimulator` - Isaac Sim compatible (Module 3)
- Simplified iCub - Voice-controlled navigation (Module 4)

---

## 3. Simulation Engine Strategy

### Decision: Multi-Simulator Approach (Gazebo → Isaac Sim → Capstone)

**Rationale**:
- **Gazebo Harmonic** (Module 2): Lower hardware barrier, proven ROS 2 integration
- **Isaac Sim** (Module 3): NVIDIA GPU required, photorealistic rendering, hardware-accelerated perception
- **Combined** (Module 4): VLA capstone uses Isaac Sim with fallback to Gazebo

**Architecture**:
```
Module 1-2: Gazebo Harmonic + ROS 2 Jazzy
    - Works on CPU-only machines (slower)
    - Recommended: RTX 3070+ for smooth experience

Module 3-4: Isaac Sim 5.0+ + ROS 2 Humble/Jazzy
    - Requires: RTX 3070 minimum (8GB VRAM)
    - Recommended: RTX 4080 (16GB VRAM)
```

**Tradeoff Documentation**:

| Engine | Fidelity | Learning Curve | GPU Required |
|--------|----------|----------------|--------------|
| Gazebo Harmonic | Good | Low | No (but faster with) |
| Unity + ROS | High | Medium | Recommended |
| Isaac Sim | Photorealistic | High | Yes (RTX 3070+) |

---

## 4. Hardware Requirements

### Development Workstation

**Minimum Specification** (Modules 1-2):
- OS: Ubuntu 24.04 LTS (or 22.04 for Humble fallback)
- CPU: Intel Core i7-10th Gen or AMD Ryzen 5 5600X
- RAM: 16 GB DDR4
- Storage: 250 GB SSD
- GPU: None required (CPU simulation works)

**Recommended Specification** (All Modules):
- CPU: Intel Core i9-12th Gen or AMD Ryzen 9 5950X
- RAM: 32 GB DDR5
- Storage: 500 GB NVMe SSD
- GPU: NVIDIA RTX 4080 (16GB VRAM)
- **Minimum GPU for Isaac Sim**: RTX 3070 (8GB VRAM)

### Jetson Edge Device (Module 4 Deployment)

**Recommended: Jetson Orin NX** ($399-599)
- 8-core Arm CPU, 100 TOPS AI performance
- 8GB or 16GB RAM (16GB recommended)
- Supports Isaac ROS perception pipelines
- Full VLA pipeline with concurrent perception + planning

**Budget Option: Jetson Orin Nano** ($249)
- Limited to simplified capstone (quantized models only)
- Whisper.tiny for speech recognition
- Nav2 with reduced update rates

### Cloud Alternatives

| Provider | Cost | Use Case |
|----------|------|----------|
| Lambda Labs | $0.24-1.10/hr | Module 3 labs (~$2-22/student) |
| AWS EC2 g4dn | $1.19-4.48/hr | Scalable for cohorts |
| Google Colab | Free (limited) | Basic testing only |

---

## 5. Chapter Structure Per Module

### Module 1: ROS 2 Fundamentals (4 chapters, ~3,500-4,500 words)

| Chapter | Title | Words | Topics |
|---------|-------|-------|--------|
| 1.1 | ROS 2 Architecture Essentials | 900 | Nodes, topics, services, actions, DDS |
| 1.2 | Building Your First Python Node | 1,000 | rclpy fundamentals, pub/sub patterns |
| 1.3 | URDF Robot Description Format | 1,100 | iCub URDF parsing, joint structures |
| 1.4 | Development Environment Setup | 800 | ROS 2 Jazzy install, colcon, workspace |

### Module 2: Digital Twin Simulation (4 chapters, ~4,500-5,500 words)

| Chapter | Title | Words | Topics |
|---------|-------|-------|--------|
| 2.1 | Gazebo Harmonic + ROS 2 Integration | 1,200 | Setup, ros_gz_bridge, iCub launch |
| 2.2 | SDF Robot Modeling for Simulation | 1,300 | SDF format, physics tuning, bipedal |
| 2.3 | Simulating Humanoid Sensors | 1,200 | LiDAR, depth cameras, IMU plugins |
| 2.4 | Unity Visualization & Digital Twin | 1,100 | Unity + ROS 2 bridge, rendering |

### Module 3: NVIDIA Isaac AI (3 chapters, ~3,500-4,500 words)

| Chapter | Title | Words | Topics |
|---------|-------|-------|--------|
| 3.1 | Isaac Sim Setup & Environment | 1,100 | Hardware check, Omniverse, iCub import |
| 3.2 | Isaac ROS Perception Pipelines | 1,300 | cuVSLAM, camera integration, nvblox |
| 3.3 | Nav2 Navigation for Humanoids | 1,200 | Nav2 stack, Hybrid-A*, bipedal config |

### Module 4: VLA Capstone (3 chapters, ~3,500-5,000 words)

| Chapter | Title | Words | Topics |
|---------|-------|-------|--------|
| 4.1 | Voice-to-Text Pipeline (Whisper) | 1,100 | Whisper setup, ROS 2 audio nodes |
| 4.2 | Cognitive Planning: NLU to Actions | 1,300 | LLM-based planning, command translation |
| 4.3 | Full VLA Capstone Project | 1,500 | End-to-end voice → navigation → manipulation |

**Total**: 14 chapters, estimated 35-50 pages (within 30-80 page constraint)

---

## 6. VLA Pipeline Architecture

### End-to-End Flow

```
┌─────────────────────────────────────────────────────────┐
│              VOICE COMMAND INPUT                         │
│  Microphone → Whisper (ASR) → Text Command              │
│  Latency: 1-3 seconds                                    │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│              COGNITIVE PLANNING                          │
│  LLM/GPT Processing → Action Primitives                 │
│  Latency: 0.5-2 seconds                                  │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│              PERCEPTION & LOCALIZATION                   │
│  Isaac ROS (cuVSLAM, nvblox) → Robot Pose               │
│  Latency: 33ms (30Hz simulation)                         │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│              PATH PLANNING & CONTROL                     │
│  Nav2 (Hybrid-A*) → MPPI Controller → Joint Commands    │
│  Latency: 100-500ms                                      │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│              ROBOT EXECUTION                             │
│  Joint Controllers → Physics Simulation → Action        │
│  Total End-to-End: 2-6 seconds                          │
└─────────────────────────────────────────────────────────┘
```

### Whisper Integration Best Practices

- Use Whisper base model for education (1-2 second latency)
- Document latency expectations upfront
- Provide Moonshine as alternative (proportional latency)
- ROS 2 node publishes to `/voice_commands` topic

### Nav2 for Humanoids

- Use **Smac Hybrid-A* Planner** (supports arbitrary-shaped legged robots)
- Configure footprint for bipedal geometry
- Separate "route planning" (Nav2) from "locomotion control" (gait synthesis)
- 5-10 Hz planning frequency for smooth trajectories

---

## 7. Common Pitfalls to Avoid

### Setup & Installation

1. **Python 3.11 Mismatch**: Isaac Sim requires 3.11; Ubuntu may default to 3.10/3.12
2. **ROS 2 Library Path Issues**: Custom packages must match Python version
3. **GPU Driver Version**: Requires 535.129+ (check compatibility)

### Runtime & Integration

4. **Simulation Time Sync**: Set `use_sim_time: true` in all examples
5. **QoS Policy Mismatches**: Sensor data needs Best Effort for visualization
6. **Image Publishing Rate**: Large images cause network congestion

### VLA-Specific

7. **Whisper Latency Expectations**: Minimum 500ms-3s is normal
8. **Model Size Selection**: Tiny models = poor accuracy; Large = latency
9. **Actions vs Topics**: Voice commands need Actions for reliability

---

## 8. Version Reference Table

| Component | Version | Reason |
|-----------|---------|--------|
| Ubuntu | 24.04 LTS | Jazzy default |
| Python | 3.11 | Isaac Sim requirement |
| ROS 2 | Jazzy | Primary LTS |
| Gazebo | Harmonic | Modern physics |
| Isaac Sim | 5.0+ | Latest stable |
| iCub Models | v1.31.0+ | Latest release |
| Nav2 | 1.2.0+ | Stable, documented |
| Jetson | JetPack 6.0+ | ROS 2 Humble support |
| NVIDIA Driver | 535.129+ | Official minimum |

---

## 9. Architectural Decisions Requiring ADR

The following decisions meet ADR significance criteria:

1. **Multi-Simulator Strategy** (Gazebo → Isaac Sim progression)
   - Impact: Affects all modules, hardware requirements, learning path
   - Suggest: `/sp.adr simulation-engine-strategy`

2. **iCub as Reference Humanoid**
   - Impact: All code examples, URDF files, simulation models
   - Suggest: `/sp.adr humanoid-robot-selection`

3. **VLA Pipeline Architecture**
   - Impact: Module 4 capstone, perception/planning integration
   - Suggest: `/sp.adr vla-pipeline-architecture`

---

## Sources

- [ROS 2 REP 2000 - Releases](https://www.ros.org/reps/rep-2000.html)
- [iCub Models Repository](https://github.com/robotology/icub-models)
- [ROS 2 Gazebo Setup](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Isaac Sim Requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim ROS 2 Navigation](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/ros2_tutorials/tutorial_ros2_navigation.html)
- [Nav2 Documentation](https://docs.nav2.org/)
- [OpenVLA: Vision-Language-Action Model](https://openvla.github.io/)
- [OpenAI Whisper](https://github.com/openai/whisper)
- [Jetson Orin Developer Kit](https://developer.nvidia.com/blog/develop-ai-powered-robots-smart-vision-systems-and-more-with-nvidia-jetson-orin-nano-developer-kit/)
