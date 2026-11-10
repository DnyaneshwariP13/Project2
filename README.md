# Vision-Guided Robotic Pick-and-Place Simulation
### Autonomous Warehouse Shelf Manipulation Using NVIDIA Omniverse™ & Isaac Sim™

---

## 1. Project Overview

This repository houses a modular, production-grade vision-guided robotic pick-and-place simulation engineered exclusively within the **NVIDIA Omniverse™** ecosystem.

A stationary **Franka Emika Panda** robotic arm operates in a multi-rack warehouse storage environment. It uses an overhead **RTX Camera** sensor to perceive randomly positioned and oriented geometric objects (cubes, cuboids, cylinders) on storage shelves, estimates their 6D poses via depth back-projection and PCA, computes collision-free trajectories using **NVIDIA cuMotion RMPflow**, grasps the items with a parallel-jaw gripper, performs orientation correction, and places them onto destination shelves.

```
+---------------------------------------------------------------------------------------+
|                                    NVIDIA OMNIVERSE                                   |
|                                                                                       |
|   [Procedural USD Racks]      [RTX Camera Sensor]      [Franka Panda Articulation]    |
|             |                         |                             |                 |
|             v                         v                             v                 |
|       PhysX Collision          Depth + Semantics            PhysX 5 / Warp            |
|             |                         |                             |                 |
|             |                         v                             |                 |
|             |                  6D Pose Estimator                    |                 |
|             |                         |                             |                 |
|             |                         v                             |                 |
|             +---------------> cuMotion RMPflow <--------------------+                 |
|                                       |                                               |
|                                       v                                               |
|                          Pick-and-Place FSM Execution                                 |
+---------------------------------------------------------------------------------------+
```

---

## 2. Workstation & System Specifications

- **Operating System:** Ubuntu 24.04 LTS (Kernel: `7.0.0-38-generic`, x86_64)
- **NVIDIA GPU:** GeForce RTX 5090 (32 GB GDDR7 VRAM, Blackwell sm_120)
- **NVIDIA Driver:** `580.178.04` (CUDA 13.0)
- **NVIDIA Isaac Sim:** `6.2.0-rc.14+release.52805.02189a84.gl` (`/home/dmhai/isaacsim`)
- **Omniverse Kit SDK:** Version `110.4` (Python `3.12.14`)
- **Storage Status:** Critical operational caution: **9.36 GB available** on root partition. Zero local asset downloads are used; Franka USD assets stream seamlessly via NVIDIA's official S3 Cloud CDN, and warehouse racks are synthesized procedurally via OpenUSD primitives.

---

## 3. Repository Structure

```
vision_guided_robot_simulation/
├── config/                              # Declarative YAML configurations
│   ├── simulation_config.yaml           # Physics dt, device, gravity, lighting
│   ├── environment_config.yaml          # Rack dimensions, shelf elevations, object geometry
│   ├── robot_config.yaml                # Robot articulation paths, joint limits, gripper
│   ├── vision_config.yaml               # Camera resolution, intrinsics/extrinsics, depth thresholds
│   └── motion_config.yaml               # cuMotion parameters, collision margins, FSM frames
├── exts/                                # Custom Omniverse Kit extensions
│   └── omni.isaac.pick_place/           # Custom extension package
│       ├── config/extension.toml        # Package dependencies and metadata
│       └── omni/isaac/pick_place/
│           ├── extension.py             # omni.ext.IExt lifecycle implementation
│           └── ui/window.py             # omni.ui control and KPI dashboard
├── src/                                 # Modular simulation library
│   ├── core/                            # Configuration loader and simulation manager
│   ├── environment/                     # Procedural rack and object generation (Phase 2)
│   ├── robot/                           # Articulation, parallel-jaw gripper, and Lula kinematics (Phase 3)
│   │   ├── articulation.py              # FrankaRobot class with S3 asset streaming & 9-DOF control
│   │   ├── gripper.py                   # ParallelJawGripper class with grasp sensing
│   │   └── kinematics.py                # FrankaKinematics class wrapping Lula analytical FK & IK
│   ├── vision/                          # Camera, detector, depth, and 6D pose estimation (Phases 4-5)
│   │   ├── camera_manager.py            # CameraManager class wrapping Replicator RTX camera & intrinsics
│   │   ├── detector.py                  # SyntheticObjectDetector class for 2D BBoxes & 3D back-projection
│   │   └── pose_estimator.py            # PoseEstimator6D class for dense point clouds, PCA & OBB 6D pose
│   ├── motion/                          # Grasp planner and cuMotion RMPflow (Phases 6-7)
│   │   ├── __init__.py                  # Motion package export
│   │   ├── grasp_planner.py             # GraspPlanner & GraspCandidate
│   │   └── cumotion_planner.py          # CuMotionPlanner & trajectory generation
│   ├── execution/                       # Manipulation FSM and recovery (Phase 8)
│   │   ├── __init__.py                  # Execution package export
│   │   ├── manipulation_fsm.py          # ManipulationFSM & ManipulationState
│   │   └── attachment.py                # GraspAttachment relative transform locking
│   ├── testing/                         # Edge case benchmarking and Monte Carlo trials (Phase 9)
│   │   ├── __init__.py                  # Testing package export
│   │   └── edge_case_runner.py          # EdgeCaseRunner, TrialResult, EdgeCaseBenchmarkReport
│   └── metrics/                         # Live KPI collection and reporting (Phase 10)
│       ├── __init__.py                  # Metrics package export
│       └── telemetry_collector.py       # TelemetryCollector, TrialTelemetry, KPISummary
├── scripts/                             # Verification and execution scripts
│   ├── run_application.py               # Master standalone application launcher with UI (Phase 10)
│   ├── verify_environment.py            # Automated system inspection probe
│   ├── run_phase1_validation.py         # Phase 1 startup & extension test
│   ├── run_phase2_validation.py         # Phase 2 rack generation & object randomization test
│   ├── run_phase3_validation.py         # Phase 3 robot integration, gripper & kinematics test
│   ├── run_phase4_validation.py         # Phase 4 vision perception & 3D back-projection test
│   ├── run_phase5_validation.py         # Phase 5 6D pose estimation & validation test
│   ├── view_phase5_deliverables.py      # Interactive 3D GUI viewer for Phase 5 deliverables
│   ├── run_phase6_validation.py         # Phase 6 grasp planning & IK reachability test
│   ├── view_phase6_deliverables.py      # Interactive 3D GUI viewer for Phase 6 deliverables
│   ├── run_phase7_validation.py         # Phase 7 motion planning & cuMotion trajectory test
│   ├── view_phase7_deliverables.py      # Interactive 3D GUI viewer for Phase 7 deliverables
│   ├── run_phase8_validation.py         # Phase 8 manipulation FSM & closed-loop execution test
│   ├── view_phase8_deliverables.py      # Interactive 3D GUI viewer for Phase 8 deliverables
│   ├── run_phase9_validation.py         # Phase 9 Monte Carlo battery & edge-case test
│   ├── view_phase9_deliverables.py      # Interactive/headless Phase 9 benchmark inspector
│   ├── run_phase10_validation.py        # Phase 10 application & UI dashboard test
│   ├── view_phase10_deliverables.py     # Interactive/headless Phase 10 benchmark inspector
│   ├── render_scene_snapshot.py         # RTX camera render of warehouse racks and objects
│   ├── render_robot_warehouse_snapshot.py # RTX camera render of Franka arm between racks
│   └── view_warehouse_scene.py          # Interactive 3D GUI scene viewer
└── docs/                                # Complete technical documentation suite
```

---

## 4. Execution & Quick Start

### 4.1 Run Automated Environment Inspection
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/verify_environment.py
```

### 4.2 Run Phase 1 Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase1_validation.py
```

### 4.3 Run Phase 2 Warehouse Environment Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase2_validation.py
```

### 4.4 Run Phase 3 Robot Integration Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase3_validation.py
```

### 4.5 Run Phase 4 Vision Perception Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase4_validation.py
```

### 4.6 Run Phase 5 6D Pose Estimation Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase5_validation.py
```

### 4.7 View Phase 5 Deliverables Interactively
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/view_phase5_deliverables.py
```

### 4.8 Run Phase 6 Grasp Planning Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase6_validation.py
```

### 4.9 View Phase 6 Deliverables Interactively
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/view_phase6_deliverables.py
```

### 4.10 Run Phase 7 Motion Planning Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase7_validation.py
```

### 4.11 View Phase 7 Deliverables Interactively
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/view_phase7_deliverables.py
```

### 4.12 Run Phase 8 Manipulation Execution Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase8_validation.py
```

### 4.13 View Phase 8 Manipulation Execution Interactively
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/view_phase8_deliverables.py
```

### 4.14 Run Phase 9 Testing & Edge Cases Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase9_validation.py
```

### 4.15 View Phase 9 Deliverables & Telemetry Inspector
```bash
python3 /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/view_phase9_deliverables.py
```

### 4.16 Launch Full Interactive Application with UI Dashboard (Phase 10)
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_application.py
```

### 4.17 Run Phase 10 Application & UI Validation Test
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/run_phase10_validation.py
```

### 4.18 View Phase 10 Deliverables & KPI Inspector
```bash
python3 /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/view_phase10_deliverables.py
```

### 4.19 View Complete Warehouse Layout (6 Racks & Inventory)
```bash
/home/dmhai/isaacsim/python.sh /home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/scripts/view_warehouse_scene.py
```


---

## 5. Documentation Suite

For detailed architecture, mathematical algorithms, API references, and issue resolutions, see the documentation suite in [docs/](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/README.md):
- [docs/ENVIRONMENT_SETUP.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/ENVIRONMENT_SETUP.md)
- [docs/PROJECT_OVERVIEW.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/PROJECT_OVERVIEW.md)
- [docs/ARCHITECTURE.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/ARCHITECTURE.md)
- [docs/TECH_STACK.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/TECH_STACK.md)
- [docs/ALGORITHMS.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/ALGORITHMS.md)
- [docs/IMPLEMENTATION_DETAILS.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/IMPLEMENTATION_DETAILS.md)
- [docs/CHANGELOG.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/CHANGELOG.md)
- [docs/ISSUES_AND_SOLUTIONS.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/ISSUES_AND_SOLUTIONS.md)
- [docs/PHASE_WISE_PROGRESS.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/PHASE_WISE_PROGRESS.md)
- [docs/TESTING_AND_VALIDATION.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/TESTING_AND_VALIDATION.md)
- [docs/KPI_DEFINITIONS.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/KPI_DEFINITIONS.md)
- [docs/API_REFERENCE.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/API_REFERENCE.md)
- [docs/FUTURE_ENHANCEMENTS.md](file:///home/dmhai/Desktop/nvidia-omniverse/vision_guided_robot_simulation/docs/FUTURE_ENHANCEMENTS.md)
