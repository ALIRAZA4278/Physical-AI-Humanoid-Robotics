# Data Model: Physical AI & Humanoid Robotics Book

**Date**: 2025-12-04
**Feature**: 001-physical-ai-humanoid-robotics

## Overview

This document defines the content entities and their relationships for the Physical AI & Humanoid Robotics Docusaurus book. Since this is a documentation project (not a software application), the "data model" describes the content structure.

---

## Entity Definitions

### Module

A self-contained learning unit covering a major topic area.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| id | string | Unique identifier | Format: `module-{1-4}` |
| title | string | Display name | 50 chars max |
| slug | string | URL-friendly name | kebab-case |
| description | string | 1-2 sentence summary | 200 chars max |
| target_audience | string | Who should read this | Required |
| prerequisites | string[] | Required prior modules | Module IDs |
| chapters | Chapter[] | Ordered list of chapters | 2-5 chapters |
| word_count_range | object | Min/max words per chapter | {min, max} |
| hardware_required | boolean | GPU or special hardware | Default: false |

**Instances**:
```yaml
- id: module-1
  title: "The Robotic Nervous System (ROS 2)"
  slug: ros2-fundamentals
  word_count_range: {min: 800, max: 1200}
  hardware_required: false

- id: module-2
  title: "The Digital Twin (Gazebo & Unity)"
  slug: digital-twin-simulation
  word_count_range: {min: 1000, max: 1500}
  hardware_required: false

- id: module-3
  title: "The AI-Robot Brain (NVIDIA Isaac)"
  slug: nvidia-isaac-ai
  word_count_range: {min: 1000, max: 1500}
  hardware_required: true  # RTX GPU required

- id: module-4
  title: "Vision-Language-Action (VLA) & Capstone"
  slug: vla-capstone
  word_count_range: {min: 1200, max: 1800}
  hardware_required: true
```

---

### Chapter

A single document within a module covering a focused subtopic.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| id | string | Unique identifier | Format: `{module-id}-{seq}` |
| title | string | Display name | 60 chars max |
| slug | string | URL-friendly name | kebab-case |
| module_id | string | Parent module | FK to Module.id |
| sequence | number | Order in module | 1-5 |
| learning_objectives | string[] | What reader will learn | 2-4 objectives |
| word_count | number | Actual word count | Within module range |
| code_examples | CodeExample[] | Embedded code | 0-10 per chapter |
| has_exercise | boolean | Hands-on activity | Required for practical |
| estimated_time | number | Minutes to complete | 15-60 mins |

**Docusaurus Mapping**:
```
docs/
├── module-1-ros2/
│   ├── _category_.json        # Module metadata
│   ├── 01-architecture.md     # Chapter 1.1
│   ├── 02-first-node.md       # Chapter 1.2
│   ├── 03-urdf.md             # Chapter 1.3
│   └── 04-environment.md      # Chapter 1.4
├── module-2-simulation/
│   └── ...
└── module-4-vla/
    └── ...
```

---

### CodeExample

A functional, tested code snippet demonstrating a concept.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| id | string | Unique identifier | Auto-generated |
| chapter_id | string | Parent chapter | FK to Chapter.id |
| language | string | Programming language | python, bash, yaml, xml |
| title | string | Descriptive name | Optional |
| code | string | The actual code | Must be tested |
| expected_output | string | What running produces | Optional |
| file_path | string | Where to save code | Full path |
| dependencies | string[] | Required packages | npm, pip, apt |
| ros2_version | string | ROS 2 compatibility | humble, jazzy |

**Validation Rules**:
- All Python code MUST run on Python 3.11
- All ROS 2 code MUST specify version (Humble or Jazzy)
- Bash commands MUST show expected output when helpful
- YAML/XML MUST be valid and parseable

---

### RobotModel

URDF/SDF description of a humanoid robot used across modules.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| id | string | Unique identifier | Format: `robot-{name}` |
| name | string | Robot name | e.g., "iCub" |
| source | string | GitHub repository | URL |
| urdf_path | string | Path to URDF file | Relative to repo |
| sdf_path | string | Path to SDF file | Optional |
| simulator_compatibility | string[] | Supported simulators | gazebo, isaac, unity |
| sensors | Sensor[] | Available sensors | See Sensor entity |

**Primary Instance**:
```yaml
id: robot-icub
name: "iCub"
source: "https://github.com/robotology/icub-models"
urdf_path: "urdf/iCubGazebo/model.urdf"
sdf_path: "sdf/iCubGazebo/model.sdf"
simulator_compatibility: [gazebo, isaac, unity]
sensors:
  - type: camera
    count: 2
    location: head
  - type: imu
    count: 1
    location: torso
  - type: force_torque
    count: 4
    location: limbs
```

---

### Sensor

Sensor configuration for simulation.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| type | enum | Sensor category | lidar, camera, depth, imu |
| name | string | Specific model | e.g., "Intel RealSense D455" |
| topic | string | ROS 2 topic name | e.g., `/camera/depth/image_raw` |
| frequency | number | Publishing rate Hz | 10-60 Hz typical |
| frame_id | string | TF frame | e.g., `camera_link` |

---

### Exercise

Hands-on activity for readers to complete.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| id | string | Unique identifier | Format: `exercise-{module}-{seq}` |
| chapter_id | string | Parent chapter | FK to Chapter.id |
| title | string | Exercise name | 60 chars max |
| objective | string | What reader builds | 1 sentence |
| steps | string[] | Ordered instructions | 3-10 steps |
| success_criteria | string | How to verify | Testable outcome |
| estimated_time | number | Minutes to complete | 10-30 mins |

---

## Entity Relationships

```
Module (1) ────────< Chapter (many)
   │                    │
   │                    ├────< CodeExample (many)
   │                    │
   │                    └────< Exercise (many)
   │
   └─── uses ──> RobotModel (1)
                     │
                     └────< Sensor (many)
```

**Cardinality**:
- 1 Module contains 2-5 Chapters
- 1 Chapter contains 0-10 CodeExamples
- 1 Chapter contains 0-2 Exercises
- All Modules reference 1 primary RobotModel (iCub)
- 1 RobotModel has 1-10 Sensors

---

## Docusaurus File Structure

```text
my-research-paper/
├── docs/                           # Docusaurus content root
│   ├── intro.md                    # Book introduction
│   │
│   ├── module-1-ros2/              # Module 1
│   │   ├── _category_.json         # Sidebar config
│   │   ├── 01-architecture.md
│   │   ├── 02-first-node.md
│   │   ├── 03-urdf.md
│   │   └── 04-environment.md
│   │
│   ├── module-2-simulation/        # Module 2
│   │   ├── _category_.json
│   │   ├── 01-gazebo-setup.md
│   │   ├── 02-sdf-modeling.md
│   │   ├── 03-sensors.md
│   │   └── 04-unity-viz.md
│   │
│   ├── module-3-isaac/             # Module 3
│   │   ├── _category_.json
│   │   ├── 01-isaac-setup.md
│   │   ├── 02-perception.md
│   │   └── 03-nav2.md
│   │
│   ├── module-4-vla/               # Module 4
│   │   ├── _category_.json
│   │   ├── 01-whisper.md
│   │   ├── 02-planning.md
│   │   └── 03-capstone.md
│   │
│   ├── glossary.md                 # Technical terms
│   └── appendix/                   # Reference material
│       ├── hardware-requirements.md
│       ├── version-migration.md
│       └── troubleshooting.md
│
├── src/                            # Docusaurus customization
│   └── pages/
│       └── index.tsx               # Landing page
│
├── static/                         # Static assets
│   ├── img/                        # Diagrams, screenshots
│   └── code/                       # Downloadable examples
│
├── docusaurus.config.js            # Site configuration
├── sidebars.js                     # Navigation structure
└── package.json                    # Dependencies
```

---

## Content Validation Rules

### Chapter Validation

1. Word count MUST be within module's specified range
2. Learning objectives MUST be stated at chapter start
3. All code examples MUST be tested before inclusion
4. External links MUST use Markdown format with descriptive text
5. Version numbers MUST be explicit for all tools mentioned

### Code Example Validation

1. Python code MUST run on Python 3.11 without errors
2. ROS 2 code MUST specify target version (Humble/Jazzy)
3. Bash commands MUST show expected output
4. File paths MUST be relative to project root
5. Dependencies MUST be documented

### Cross-Reference Validation

1. Chapter prerequisites MUST reference existing chapters
2. Module dependencies MUST form a DAG (no cycles)
3. Robot model references MUST use consistent naming
4. Sensor topics MUST match ROS 2 conventions

---

## State Transitions

### Chapter Lifecycle

```
Draft → In Review → Tested → Published → Deprecated
  │         │          │         │
  ▼         ▼          ▼         ▼
[Author]  [Tech Review] [Code Test] [Live] [Version Update]
```

### Code Example Lifecycle

```
Written → Tested (Local) → Tested (CI) → Published
              │
              ▼
          Fix if fails
```

---

## Frontmatter Schema

Each chapter Markdown file MUST include:

```yaml
---
id: chapter-id
title: "Chapter Title"
sidebar_label: "Short Label"
sidebar_position: 1
description: "One-line description for SEO"
keywords: [ros2, humanoid, robotics]
---
```

Each `_category_.json` MUST include:

```json
{
  "label": "Module Title",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Module overview"
  }
}
```
