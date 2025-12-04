---
sidebar_position: 1
title: Hardware Requirements
description: Complete hardware specifications for all modules in the Physical AI & Humanoid Robotics book
---

# Hardware Requirements

This appendix details the hardware requirements for each module, from CPU-only configurations to GPU-accelerated setups.

---

## Quick Reference

| Module | GPU Required | Minimum RAM | Storage |
|--------|--------------|-------------|---------|
| Module 1: ROS 2 | No | 8GB | 20GB |
| Module 2: Simulation | No (recommended) | 16GB | 50GB |
| Module 3: Isaac AI | **Yes** | 32GB | 100GB |
| Module 4: VLA | **Yes** | 32GB | 100GB |

---

## Modules 1-2: CPU Configuration

### Minimum Requirements

| Component | Specification |
|-----------|---------------|
| **OS** | Ubuntu 24.04 LTS (or 22.04 for Humble) |
| **CPU** | 4-core x86_64 (Intel i5 / AMD Ryzen 5) |
| **RAM** | 16GB DDR4 |
| **Storage** | 50GB SSD |
| **Network** | Internet connection for package downloads |

### Recommended Configuration

| Component | Specification |
|-----------|---------------|
| **OS** | Ubuntu 24.04 LTS |
| **CPU** | 8-core (Intel i7 / AMD Ryzen 7) |
| **RAM** | 32GB DDR4 |
| **Storage** | 100GB NVMe SSD |
| **GPU** | Any NVIDIA GPU (for Gazebo rendering) |

---

## Modules 3-4: GPU Configuration

:::warning Required Hardware
Modules 3 and 4 require an NVIDIA RTX GPU. Without compatible hardware, you can read the content but cannot execute the exercises.
:::

### Minimum Requirements

| Component | Specification |
|-----------|---------------|
| **OS** | Ubuntu 24.04 LTS |
| **CPU** | 8-core (Intel i7 / AMD Ryzen 7) |
| **RAM** | 32GB DDR4 |
| **GPU** | **NVIDIA RTX 3070 (8GB VRAM)** |
| **Driver** | 535.129 or newer |
| **Storage** | 100GB NVMe SSD |

### Recommended Configuration

| Component | Specification |
|-----------|---------------|
| **OS** | Ubuntu 24.04 LTS |
| **CPU** | 12-core (Intel i9 / AMD Ryzen 9) |
| **RAM** | 64GB DDR5 |
| **GPU** | **NVIDIA RTX 4080 (16GB VRAM)** |
| **Driver** | 545.x or newer |
| **Storage** | 500GB NVMe SSD |

---

## GPU Compatibility Chart

| GPU Model | VRAM | Isaac Sim | Modules |
|-----------|------|-----------|---------|
| RTX 3060 | 12GB | Limited | 3 only (slow) |
| RTX 3070 | 8GB | ✅ Minimum | 3-4 |
| RTX 3080 | 10GB | ✅ Good | 3-4 |
| RTX 3090 | 24GB | ✅ Excellent | 3-4 |
| RTX 4070 | 12GB | ✅ Good | 3-4 |
| RTX 4080 | 16GB | ✅ Recommended | 3-4 |
| RTX 4090 | 24GB | ✅ Excellent | 3-4 |

---

## Edge Deployment (Optional)

For deploying Module 4 VLA pipeline on edge hardware:

### NVIDIA Jetson Orin NX

| Specification | Value |
|---------------|-------|
| **GPU** | 1024 CUDA cores |
| **RAM** | 16GB unified |
| **Storage** | 256GB NVMe |
| **Power** | 10-25W |
| **JetPack** | 6.0+ |

### NVIDIA Jetson AGX Orin

| Specification | Value |
|---------------|-------|
| **GPU** | 2048 CUDA cores |
| **RAM** | 32GB/64GB unified |
| **Storage** | 64GB eMMC + NVMe |
| **Power** | 15-60W |
| **JetPack** | 6.0+ |

---

## Verification Commands

### Check CPU

```bash
lscpu | grep "Model name"
```

### Check RAM

```bash
free -h
```

### Check GPU

```bash
nvidia-smi
```

**Expected output (RTX 3070 example):**
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce RTX 3070  |   00000000:01:00.0  On |                  N/A |
|  0%   35C    P8    12W / 220W |    350MiB /  8192MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
```

### Check Storage

```bash
df -h /
```

---

## Cloud Alternatives

If you don't have local GPU hardware, these cloud options support Isaac Sim:

| Provider | GPU Options | Notes |
|----------|-------------|-------|
| AWS | g5.xlarge (A10G) | Good for learning |
| GCP | a2-highgpu-1g (A100) | Best performance |
| Lambda Labs | RTX 4090 | Cost-effective |
| Paperspace | RTX 4000/5000 | Easy setup |

:::info Cost Consideration
Cloud GPU instances typically cost $1-5/hour. Modules 3-4 require approximately 30-40 hours of hands-on time.
:::

---

## Next Steps

- [Module 1: ROS 2 Architecture](/docs/module-1-ros2/architecture) — Start learning (CPU only)
- [Version Migration Guide](/docs/appendix/version-migration) — If using different versions
- [Troubleshooting](/docs/appendix/troubleshooting) — Common setup issues
