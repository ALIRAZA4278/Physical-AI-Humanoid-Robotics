# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/001-physical-ai-humanoid-robotics/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT explicitly requested in the feature specification. Code validation will be done manually per Testing Strategy in plan.md.

**Organization**: Tasks are grouped by user story (each module = one user story) to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Module 1, US2=Module 2, US3=Module 3, US4=Module 4)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `docs/`, `src/`, `static/` at repository root
- Chapter files: `docs/module-{n}-{slug}/{nn}-{chapter-slug}.md`
- Static code: `static/code/module-{n}/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus project and configure deployment

- [x] T001 Initialize Docusaurus project with `npx create-docusaurus@latest` in repository root
- [x] T002 Configure docusaurus.config.js with project metadata in docusaurus.config.js
- [x] T003 [P] Configure sidebars.js with 4-module navigation structure in sidebars.js
- [x] T004 [P] Create custom.css with book styling in src/css/custom.css
- [x] T005 [P] Create landing page component in src/pages/index.tsx
- [x] T006 Configure GitHub Pages deployment in .github/workflows/deploy.yml
- [x] T007 Create intro.md book introduction page in docs/intro.md
- [x] T008 [P] Create static/img/ directory structure for diagrams
- [x] T009 [P] Create static/code/ directory structure for downloadable examples
- [x] T010 Run initial `npm run build` to verify setup works

**Checkpoint**: Docusaurus site runs locally with empty module structure

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create shared assets and templates that ALL modules depend on

**⚠️ CRITICAL**: No chapter content can begin until this phase is complete

- [x] T011 Create _category_.json for Module 1 in docs/module-1-ros2/_category_.json
- [x] T012 [P] Create _category_.json for Module 2 in docs/module-2-simulation/_category_.json
- [x] T013 [P] Create _category_.json for Module 3 in docs/module-3-isaac/_category_.json
- [x] T014 [P] Create _category_.json for Module 4 in docs/module-4-vla/_category_.json
- [ ] T015 Download and configure iCub URDF model files in static/code/shared/icub-models/
- [ ] T016 Create ROS 2 architecture diagram in static/img/architecture/ros2-overview.svg
- [ ] T017 [P] Create VLA pipeline diagram in static/img/architecture/vla-pipeline.svg
- [x] T018 Create glossary.md technical terms document in docs/glossary.md
- [x] T019 [P] Create appendix _category_.json in docs/appendix/_category_.json
- [x] T020 Create hardware-requirements.md appendix in docs/appendix/hardware-requirements.md
- [x] T021 [P] Create version-migration.md appendix in docs/appendix/version-migration.md
- [x] T022 [P] Create troubleshooting.md appendix in docs/appendix/troubleshooting.md
- [x] T023 Run `npm run build` to verify foundation structure

**Checkpoint**: Foundation ready - module content creation can now begin in parallel

---

## Phase 3: User Story 1 - Learning ROS 2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Reader can create and run ROS 2 nodes, build packages, and interpret URDF files

**Independent Test**: Reader follows Module 1, creates pub/sub node, builds package, runs on ROS 2 Jazzy

### Implementation for User Story 1

- [x] T024 [US1] Write chapter 1.1 ROS 2 Architecture Essentials in docs/module-1-ros2/01-architecture.md
- [ ] T025 [P] [US1] Create ROS 2 node diagram in static/img/module-1/node-topic-diagram.svg
- [x] T026 [US1] Create talker.py example code in static/code/module-1/talker.py
- [x] T027 [P] [US1] Create listener.py example code in static/code/module-1/listener.py
- [x] T028 [US1] Write chapter 1.2 Building Your First Python Node in docs/module-1-ros2/02-first-node.md
- [ ] T029 [US1] Create service_server.py example in static/code/module-1/service_server.py
- [ ] T030 [P] [US1] Create service_client.py example in static/code/module-1/service_client.py
- [x] T031 [US1] Write chapter 1.3 URDF Robot Description Format in docs/module-1-ros2/03-urdf.md
- [ ] T032 [US1] Create annotated iCub URDF excerpt in static/code/module-1/icub_sample.urdf
- [ ] T033 [P] [US1] Create URDF visualization screenshot in static/img/module-1/urdf-rviz.png
- [x] T034 [US1] Write chapter 1.4 Development Environment Setup in docs/module-1-ros2/04-environment.md
- [x] T035 [US1] Create setup_verify.sh script in static/code/module-1/setup_verify.sh
- [ ] T036 [US1] Validate all Module 1 code examples run on ROS 2 Jazzy
- [x] T037 [US1] Run `npm run build` to verify Module 1 integration

**Checkpoint**: Module 1 complete - reader can learn ROS 2 fundamentals independently

---

## Phase 4: User Story 2 - Simulating Humanoid Robots (Priority: P2)

**Goal**: Reader can launch Gazebo simulation with humanoid robot and sensors

**Independent Test**: Reader follows Module 2, launches Gazebo world with iCub, sees sensor outputs

### Implementation for User Story 2

- [ ] T038 [US2] Write chapter 2.1 Gazebo Harmonic + ROS 2 Integration in docs/module-2-simulation/01-gazebo-setup.md
- [ ] T039 [US2] Create Gazebo launch file example in static/code/module-2/launch_gazebo.py
- [ ] T040 [P] [US2] Create empty Gazebo world file in static/code/module-2/empty_world.sdf
- [ ] T041 [US2] Write chapter 2.2 SDF Robot Modeling for Simulation in docs/module-2-simulation/02-sdf-modeling.md
- [ ] T042 [US2] Create iCub SDF configuration in static/code/module-2/icub_gazebo.sdf
- [ ] T043 [P] [US2] Create physics parameters YAML in static/code/module-2/physics_params.yaml
- [ ] T044 [US2] Write chapter 2.3 Simulating Humanoid Sensors in docs/module-2-simulation/03-sensors.md
- [ ] T045 [US2] Create LiDAR plugin configuration in static/code/module-2/lidar_plugin.sdf
- [ ] T046 [P] [US2] Create depth camera plugin configuration in static/code/module-2/depth_camera.sdf
- [ ] T047 [P] [US2] Create IMU plugin configuration in static/code/module-2/imu_plugin.sdf
- [ ] T048 [US2] Create sensor visualization screenshot in static/img/module-2/sensor-rviz.png
- [ ] T049 [US2] Write chapter 2.4 Unity Visualization & Digital Twin in docs/module-2-simulation/04-unity-viz.md
- [ ] T050 [US2] Create Unity ROS 2 bridge configuration in static/code/module-2/unity_ros_bridge.cs
- [ ] T051 [US2] Validate all Module 2 code examples run on Gazebo Harmonic
- [ ] T052 [US2] Run `npm run build` to verify Module 2 integration

**Checkpoint**: Module 2 complete - reader can simulate humanoid robots independently

---

## Phase 5: User Story 3 - Integrating AI Perception and Navigation (Priority: P3)

**Goal**: Reader can run Isaac Sim with perception pipelines and Nav2 navigation

**Independent Test**: Reader follows Module 3, launches Isaac Sim, runs VSLAM, navigates to goal

**Note**: Requires NVIDIA RTX 3070+ GPU

### Implementation for User Story 3

- [ ] T053 [US3] Write chapter 3.1 Isaac Sim Setup & Environment in docs/module-3-isaac/01-isaac-setup.md
- [ ] T054 [US3] Create Isaac Sim launch script in static/code/module-3/launch_isaac.py
- [ ] T055 [P] [US3] Create iCub Isaac environment in static/code/module-3/icub_isaac_env.usd
- [ ] T056 [P] [US3] Create hardware check script in static/code/module-3/check_gpu.sh
- [ ] T057 [US3] Write chapter 3.2 Isaac ROS Perception Pipelines in docs/module-3-isaac/02-perception.md
- [ ] T058 [US3] Create cuVSLAM launch configuration in static/code/module-3/vslam_launch.py
- [ ] T059 [P] [US3] Create nvblox configuration in static/code/module-3/nvblox_config.yaml
- [ ] T060 [US3] Create perception pipeline diagram in static/img/module-3/perception-pipeline.svg
- [ ] T061 [US3] Write chapter 3.3 Nav2 Navigation for Humanoids in docs/module-3-isaac/03-nav2.md
- [ ] T062 [US3] Create Nav2 parameters for bipedal robot in static/code/module-3/nav2_params.yaml
- [ ] T063 [P] [US3] Create hybrid-A* planner config in static/code/module-3/planner_config.yaml
- [ ] T064 [US3] Create navigation demo script in static/code/module-3/navigate_to_goal.py
- [ ] T065 [US3] Validate all Module 3 code examples run on Isaac Sim 5.0+
- [ ] T066 [US3] Run `npm run build` to verify Module 3 integration

**Checkpoint**: Module 3 complete - reader can integrate AI perception and navigation independently

---

## Phase 6: User Story 4 - Building Voice-Controlled Autonomous Humanoid (Priority: P4)

**Goal**: Reader can implement complete VLA pipeline from voice command to robot action

**Independent Test**: Reader speaks command, humanoid navigates and executes action in simulation

**Note**: Requires NVIDIA RTX 3070+ GPU or Jetson Orin

### Implementation for User Story 4

- [ ] T067 [US4] Write chapter 4.1 Voice-to-Text Pipeline (Whisper) in docs/module-4-vla/01-whisper.md
- [ ] T068 [US4] Create Whisper ROS 2 node in static/code/module-4/whisper_node.py
- [ ] T069 [P] [US4] Create audio capture utility in static/code/module-4/audio_capture.py
- [ ] T070 [US4] Write chapter 4.2 Cognitive Planning: NLU to Actions in docs/module-4-vla/02-planning.md
- [ ] T071 [US4] Create LLM planning node in static/code/module-4/planner_node.py
- [ ] T072 [P] [US4] Create action graph YAML in static/code/module-4/action_graph.yaml
- [ ] T073 [P] [US4] Create command mappings config in static/code/module-4/command_mappings.yaml
- [ ] T074 [US4] Write chapter 4.3 Full VLA Capstone Project in docs/module-4-vla/03-capstone.md
- [ ] T075 [US4] Create VLA orchestrator node in static/code/module-4/vla_orchestrator.py
- [ ] T076 [US4] Create capstone launch file in static/code/module-4/launch_capstone.py
- [ ] T077 [P] [US4] Create capstone demo script in static/code/module-4/run_demo.sh
- [ ] T078 [US4] Create VLA demo screenshot in static/img/module-4/vla-demo.png
- [ ] T079 [US4] Validate all Module 4 code examples run end-to-end
- [ ] T080 [US4] Run `npm run build` to verify Module 4 integration

**Checkpoint**: Module 4 complete - reader can build voice-controlled autonomous humanoid

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, documentation updates, and deployment

- [ ] T081 Update glossary.md with all technical terms from all modules in docs/glossary.md
- [ ] T082 [P] Add cross-references between related chapters across modules
- [ ] T083 [P] Verify all external documentation links resolve (ROS 2, Isaac, Gazebo docs)
- [ ] T084 Validate word counts per chapter meet module constraints
- [ ] T085 [P] Create book cover image in static/img/cover.svg
- [ ] T086 [P] Update intro.md with complete learning path overview
- [ ] T087 Run final `npm run build` with production settings
- [ ] T088 Deploy to GitHub Pages via GitHub Actions
- [ ] T089 Verify deployed site loads correctly at GitHub Pages URL
- [ ] T090 Create release notes for v1.0.0

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational (Phase 2) completion
  - User stories CAN proceed in parallel (if staffed)
  - Or sequentially in priority order (US1 → US2 → US3 → US4)
  - US1 (Module 1) has no dependencies on other stories
  - US2 (Module 2) can reference US1 but is independently testable
  - US3 (Module 3) can reference US1-US2 but is independently testable
  - US4 (Module 4) can reference US1-US3 but is independently testable
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - MVP
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Depends on shared iCub assets
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Requires RTX GPU
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Requires RTX GPU or Jetson

### Within Each User Story

- Chapters should be written in sequence (1→2→3→4)
- Code examples can be written in parallel within a chapter
- Diagrams/screenshots can be created in parallel with content
- Validation step MUST come after all chapter content

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational completes, all user stories can start in parallel
- Code examples marked [P] within a story can run in parallel
- Polish tasks marked [P] can run in parallel

---

## Parallel Example: User Story 1

```bash
# After T024 (chapter 1.1) is complete, launch these in parallel:
Task: T025 "Create ROS 2 node diagram in static/img/module-1/node-topic-diagram.svg"
Task: T026 "Create talker.py example code in static/code/module-1/talker.py"
Task: T027 "Create listener.py example code in static/code/module-1/listener.py"

# After T031 (chapter 1.3) is complete:
Task: T032 "Create annotated iCub URDF excerpt"
Task: T033 "Create URDF visualization screenshot"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Module 1 - ROS 2)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready - this is the MVP!

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy (MVP: ROS 2 Module!)
3. Add User Story 2 → Test independently → Deploy (Simulation Module)
4. Add User Story 3 → Test independently → Deploy (Isaac AI Module)
5. Add User Story 4 → Test independently → Deploy (VLA Capstone)
6. Complete Polish → Final Release v1.0.0

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (ROS 2)
   - Developer B: User Story 2 (Simulation)
   - Developer C: User Story 3 (Isaac) - requires RTX GPU
   - Developer D: User Story 4 (VLA) - requires RTX GPU
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific module/user story
- Each module should be independently completable and deployable
- Verify `npm run build` passes after each module completion
- Commit after each task or logical group
- Stop at any checkpoint to validate module independently
- Hardware dependencies: US3 and US4 require NVIDIA RTX GPU

---

## Summary

| Phase | Task Count | Parallel Opportunities |
|-------|------------|----------------------|
| Phase 1: Setup | 10 | 5 parallelizable |
| Phase 2: Foundational | 13 | 8 parallelizable |
| Phase 3: US1 (ROS 2) | 14 | 6 parallelizable |
| Phase 4: US2 (Simulation) | 15 | 7 parallelizable |
| Phase 5: US3 (Isaac) | 14 | 6 parallelizable |
| Phase 6: US4 (VLA) | 14 | 6 parallelizable |
| Phase 7: Polish | 10 | 5 parallelizable |
| **TOTAL** | **90** | **43 parallelizable** |

**MVP Scope**: Phase 1 + Phase 2 + Phase 3 (US1) = 37 tasks
**Full Delivery**: All 90 tasks
