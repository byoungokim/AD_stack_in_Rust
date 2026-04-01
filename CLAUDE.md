# Limo Drive - Autonomous Driving SW for Limo Pro

## Project Overview

Autonomous driving software stack for the **AgileX Limo Pro** robot equipped with **NVIDIA Jetson Orin Nano**. Built **without ROS** — using **ZeroMQ + Protobuf** as a lightweight middleware with a hybrid Python + C++ architecture.

## Hardware Platform

- **Robot**: AgileX Limo Pro (4-wheel differential/Ackermann drive)
- **Compute**: NVIDIA Jetson Orin Nano (8GB)
- **Sensors**: RGB camera, 2D LiDAR, IMU, wheel encoders
- **OS**: Ubuntu 22.04 + JetPack 6.x

## Tech Stack

- **Middleware**: ZeroMQ (pub/sub + req/rep) + Protobuf (serialization)
- **Languages**: Rust (SensPerc process), Python (Planning process), C++ (Control process)
- **Build**: Cargo (Rust), CMake (C++), pip/setuptools (Python), Makefile (top-level orchestration)
- **ML/DL**: PyTorch, TensorRT (for Orin Nano deployment)
- **Logging**: tracing (Rust), spdlog (C++), Python `logging` module
- **Config**: YAML config files loaded at startup
- **Recording**: Custom record/replay system using Protobuf-serialized messages

## 3-Process Architecture

```
┌─────────────────────┐    CH1: WorldState     ┌──────────────────┐    CH2: ControlCmd    ┌─────────────────┐
│  SENSING/PERCEPTION │ ──────(10 Hz)────────> │     PLANNING     │ ─────(10 Hz)───────> │     CONTROL     │
│     (Process 1)     │                        │   (Process 2)    │                      │   (Process 3)   │
│                     │    CH4: RawSensors     │                  │                      │                 │
│                     │ ──────(15 Hz)────────> │  [E2E Inference] │                      │                 │
│                     │   (E2E/shadow only)    │                  │                      │                 │
│                     │ <──────────────────────│──────────────────│── CH3: VehicleState  │                 │
└─────────────────────┘    (20 Hz)             └──────────────────┘       (20 Hz)        └─────────────────┘
                           ◄══════════ CH0: Heartbeat Bus (10 Hz, all 3) ══════════►
```

### Process 1: Sensing & Perception (Rust)

Full native Rust process for high-bandwidth sensor handling. Intra-process communication via crossbeam lock-free ring buffers and atomic latest-value slots. Built as a Cargo project at `sensperc/`.

| Thread | Rate | Role |
|--------|------|------|
| CameraDriver | 30 Hz | V4L2 capture → ring buffer (dummy fallback on non-Linux) |
| LidarDriver | 10 Hz | Serial LiDAR → ring buffer (dummy fallback) |
| ImuDriver | 100 Hz | Serial IMU → ring buffer (dummy fallback) |
| ObjectDetection | 15 Hz | TensorRT YOLO on GPU |
| LaneDetection | 15 Hz | TensorRT lane model on GPU |
| SlamFrontend | 10 Hz | Scan matching, pose estimation |
| SlamBackend | 1 Hz | Graph optimization |
| SensorFusion | 20 Hz | EKF fusing IMU + wheel odom + SLAM pose |
| Aggregator | 10 Hz | Combines all results → publishes `WorldState` on CH1 |

### Process 2: Planning (Python + C++)

All decision-making: behavior, global/local planning, E2E inference, and pipeline arbitration.

| Thread | Language | Rate | Role |
|--------|----------|------|------|
| BehaviorPlanner | Python | 5 Hz | State machine / behavior tree |
| GlobalPlanner | C++ | 1 Hz | A*/RRT path planning |
| LocalPlanner | C++ | 10 Hz | DWA/TEB local trajectory |
| E2EInference | Python/C++ | 15 Hz | End-to-end neural network (GPU) |
| PipelineArbitrator | Python | 10 Hz | Selects traditional vs E2E → publishes `ControlCommand` on CH2 |

### Process 3: Control (C++ only)

Safety-critical, purely C++ for determinism. Owns chassis hardware communication.

| Thread | Language | Rate | Role |
|--------|----------|------|------|
| ChassisDriver | C++ | 10 Hz | Serial comm with Limo Pro chassis |
| TrajectoryTracker | C++ | 10 Hz | Pure pursuit / Stanley controller |
| KinematicsEngine | C++ | 10 Hz | Ackermann/differential conversion |
| WatchdogTimer | C++ | 10 Hz | Monitors heartbeats, triggers E-stop on timeout |
| EmergencyStopHandler | C++ | event | Immediate stop, overrides all commands |
| StatePublisher | C++ | 20 Hz | Publishes `VehicleState` on CH3 |

## IPC Channels (ZMQ PUB/SUB)

| Channel | Port | Publisher | Subscriber(s) | Rate | Message |
|---------|------|-----------|---------------|------|---------|
| CH0 | tcp:5550 | All 3 | All 3 | 10 Hz each | `Heartbeat` |
| CH1 | tcp:5551 | SensPerc | Planning | 10 Hz | `WorldState` (aggregated perception) |
| CH2 | tcp:5552 | Planning | Control | 10 Hz | `ControlCommand` |
| CH3 | tcp:5553 | Control | SensPerc, Planning | 20 Hz | `VehicleState` |
| CH4 | tcp:5554 | SensPerc | Planning | 15 Hz | `SensorSnapshot` (E2E/shadow only) |

Only 3 data channels in traditional mode (CH4 inactive).

## Intra-Process Communication

| Pattern | Use Case | Implementation |
|---------|----------|---------------|
| SPSC Ring Buffer | Driver → processor (high-freq streams) | Lock-free circular buffer with atomic indices |
| Atomic Latest-Value Slot | Processor → aggregator (latest result) | `atomic<shared_ptr<const T>>` swap |
| Shared Mutex | Complex shared state (occupancy map) | `std::shared_mutex` readers-writer lock |

## Fault Tolerance

### 4-Layer Emergency Stop
1. **Software E-Stop**: Planning sets `emergency_stop=true` in ControlCommand
2. **Command Timeout**: Control's WatchdogTimer auto-stops after 200ms no command
3. **Chassis Firmware Timeout**: Limo Pro hardware stops motors after ~500ms no serial
4. **Physical E-Stop**: Hardware button

### Degradation Matrix

| If Dies | Control Response | Planning Response | SensPerc Response |
|---------|-----------------|-------------------|-------------------|
| SensPerc | Decelerate after 200ms cmd timeout | Dead-reckoning, plan safe stop | N/A |
| Planning | Decelerate after 200ms cmd timeout | N/A | Continue publishing (for logging) |
| Control | N/A | Alert operator | Log error, fusion degrades |

### Supervisor (Optional lightweight 4th process)
Launches/monitors the 3 main processes. Can restart crashed processes. Not a SPOF.

## E2E Autonomous Driving

E2E model lives in **Planning process** (E2EInference thread).

### Pipeline Modes
| Mode | Traditional | E2E | CH4 | Description |
|------|------------|-----|-----|-------------|
| `TRADITIONAL` | Active | Off | Off | Classical perception → planning → control |
| `E2E` | Off | Active | On | Neural net: sensors → control directly |
| `SHADOW` | Active (primary) | Active (logged) | On | Both run; traditional controls, E2E logged for comparison |

### PipelineArbitrator
- Selects output based on active mode
- In E2E mode: falls back to traditional or E-STOP if confidence < threshold
- Always applies **safety envelope** (max speed, max acceleration, max curvature)

## Project Structure

```
limo_drive/
├── proto/                     # Protobuf message definitions (shared contract)
│   ├── common.proto           # Pose2D, Twist2D, Header
│   ├── sensor.proto           # CameraFrame, LaserScan, ImuReading, SensorSnapshot
│   ├── perception.proto       # Detection, DetectionArray, LaneMarkings, OccupancyGrid
│   ├── planning.proto         # Path, Waypoint, Trajectory, PipelineMode
│   ├── control.proto          # ControlCommand, VehicleState, ChassisFeedback
│   ├── world_state.proto      # WorldState (aggregated perception output)
│   └── system.proto           # Heartbeat, ProcessStatus, SystemCommand
├── core/                      # Shared framework
│   ├── transport/             # ZMQ pub/sub wrappers, SPSC ring buffer, atomic slot
│   ├── node/                  # Base process class with lifecycle + heartbeat
│   ├── config/                # YAML config loader
│   ├── logging/               # Unified logging (spdlog + Python logging)
│   └── recorder/              # Message record/replay
├── sensperc/                  # Process 1: Sensing & Perception
│   ├── drivers/               # Camera, LiDAR, IMU drivers (C++)
│   ├── detection/             # Object detection (TensorRT)
│   ├── lane/                  # Lane detection (TensorRT)
│   ├── slam/                  # SLAM frontend + backend (C++)
│   ├── fusion/                # EKF sensor fusion (C++)
│   └── main.cpp               # Process entry point
├── planning/                  # Process 2: Planning
│   ├── behavior/              # Behavior planner / state machine
│   ├── global_planner/        # A*, RRT
│   ├── local_planner/         # DWA, TEB
│   ├── e2e/                   # E2E model inference
│   ├── arbitrator/            # Pipeline arbitrator + safety envelope
│   └── main.cpp               # Process entry point
├── control/                   # Process 3: Control (C++ only)
│   ├── chassis/               # Limo Pro serial driver
│   ├── tracker/               # Trajectory tracker (pure pursuit / Stanley)
│   ├── kinematics/            # Ackermann / differential
│   ├── watchdog/              # Watchdog timer + E-stop handler
│   └── main.cpp               # Process entry point
├── tools/                     # Utilities & dev tools
│   ├── launcher.py            # Process launcher (supervisor)
│   ├── visualizer/            # Visualization (OpenCV / matplotlib)
│   └── bag/                   # Record/replay CLI
├── tests/                     # All tests
│   ├── unit/                  # Per-module unit tests
│   ├── integration/           # Cross-module integration tests
│   └── sim/                   # Simulation-based system tests
├── config/                    # Runtime configuration files
│   ├── system.yaml            # ZMQ ports, pipeline mode, process launch
│   ├── fault_tolerance.yaml   # Heartbeat rate, timeout thresholds
│   ├── sensperc.yaml          # Camera, LiDAR, detection, SLAM params
│   ├── planning.yaml          # Planner params, E2E model path, safety envelope
│   ├── control.yaml           # PID gains, tracker, kinematics, watchdog
│   └── robot.yaml             # Hardware: serial ports, device IDs, geometry
├── CMakeLists.txt
├── requirements.txt
├── Makefile
└── CLAUDE.md
```

## Multi-Agent Team Structure (Hybrid)

### Module Agents
| Agent | Scope | Key Responsibilities |
|-------|-------|---------------------|
| **Perception Agent** | `sensperc/` | Drivers, object/lane detection, sensor fusion |
| **SLAM Agent** | `sensperc/slam/` | SLAM frontend/backend, localization |
| **Planning Agent** | `planning/` | Behavior, global/local planning, E2E, arbitrator |
| **Control Agent** | `control/` | Chassis driver, trajectory tracking, kinematics, watchdog |

### Role Agents
| Agent | Scope | Key Responsibilities |
|-------|-------|---------------------|
| **Architect Agent** | `proto/`, `core/` | Protobuf contracts, ZMQ transport, core framework |
| **Test & Integration Agent** | `tests/`, all modules | Unit/integration/sim tests, CI/CD |

## Coding Conventions

### C++
- C++17 standard, Google C++ Style Guide
- Namespace: `limo::<module>` (e.g., `limo::control`, `limo::sensperc`)
- Header guards: `LIMO_<MODULE>_<FILE>_HPP_`
- Smart pointers only, no raw `new`/`delete`
- spdlog for logging with module-prefixed loggers

### Python
- PEP 8, type hints on all public functions
- `dataclasses` or Protobuf-generated classes for structured data
- `logging` module with module-prefixed loggers

### General
- All inter-module interfaces defined in `proto/` — no ad-hoc serialization
- Config via YAML in `config/` — no hardcoded parameters
- Every module must be independently testable
- Log levels: DEBUG (dev), INFO (nominal), WARNING (degraded), ERROR (failures)

## Build & Run

```bash
make build          # Build everything (proto + C++ + Python)
make proto          # Build Protobuf only
make build-cpp      # Build C++ only
make test           # Run all tests
make test-unit      # Unit tests only
make test-integration  # Integration tests only

pip install -r requirements.txt  # Python deps

python tools/launcher.py config/system.yaml          # Launch full stack (real robot)
python tools/launcher.py config/system.yaml --sim     # Launch simulation
python tools/launcher.py config/system.yaml --mode e2e  # Launch in E2E mode
```

## Agent Workflow

1. **Architect Agent** defines Protobuf interfaces in `proto/` and core framework in `core/`
2. **Control Agent** implements `control/` (simplest, safety-critical, test first)
3. **Perception Agent + SLAM Agent** implement `sensperc/` in parallel
4. **Planning Agent** implements `planning/` including E2E and arbitrator
5. **Test Agent** validates each module and cross-module integration
6. All agents coordinate through Protobuf definitions as the shared contract
