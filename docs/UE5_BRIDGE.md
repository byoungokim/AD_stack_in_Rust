# UE5 Simulation Bridge — Milestone Design

Goal: photorealistic testing of the Limo Drive stack (Lumen GI, Nanite
geometry, cinematic assets) by replacing Gazebo with an **Unreal Engine 5**
world — *without touching the stack*. This works because the stack is
engine-agnostic: the entire simulator interface is three ZMQ channels
carrying Protobuf messages.

## The sim contract (already frozen)

| Channel | Dir | Endpoint | Rate | Message (proto/sim.proto) |
|---|---|---|---|---|
| CH5 | sim → stack | `tcp://*:5560` (sim binds) | 20 Hz | `SimSensorData` — ground-truth pose (spawn-anchored planner frame), 2D lidar scan (360×, 12 m), IMU |
| CH6 | sim → stack | `tcp://*:5561` (sim binds) | 20 Hz | `SimVehicleState` — wheel-odometry pose + twist |
| CH7 | stack → sim | `tcp://localhost:5562` (sim connects) | 10 Hz | `SimControlCommand` — linear velocity, angular velocity / steering, E-stop |

Anything that implements these three sockets *is* the simulator. The
Gazebo bridge (`simulation/bridge/gz_zmq_bridge.py`) is the reference
implementation; the contract details (frame anchoring at first pose,
scan-timestamp semantics) live in its comments.

## UE5-side architecture

- **Plugin `LimoBridge`** (C++): ZeroMQ (`libzmq` + `cppzmq`) and protobuf
  linked into a `UGameInstanceSubsystem`; a background thread owns the
  sockets, game-thread tick marshals data.
- **Robot pawn**: Chaos vehicle or simple kinematic pawn at Limo scale;
  applies CH7 commands (Ackermann: velocity + steering), publishes CH6
  odometry with injected drift/noise and CH5 ground truth from the actor
  transform.
- **Lidar**: 360 async line traces per scan (or the ray-cast batch API) at
  10 Hz against visibility collision — cheap and exact for a 2D scanner.
- **Camera**: `USceneCaptureComponent2D` → optional CH5 image field
  (contract extension) when perception work needs photoreal frames.
- **World**: city assets (Quixel Megascans / marketplace urban packs) laid
  out to the same limo-scale street grammar as `gen_city_world.py`; a
  UE-side generator (Python or Editor Utility) can consume the SAME
  `*_roadmap.yaml` / `*_peds.json` / `*_traffic.json` artifacts so
  scenarios stay identical across engines.
- **Actors**: UE behavior trees for pedestrians/traffic, mirroring the
  yield/erratic parameters of the Gazebo controllers.

## Phases

1. **Contract extraction** (small): move CH5/6/7 message + endpoint spec
   into a versioned doc; add a headless contract-conformance test (the
   stack's `--dummy` sim already exercises it).
2. **Minimal UE5 pawn** on a flat test level: drive CH7 → pawn, publish
   CH6/CH5, run the gauntlet scenario logic end-to-end. Exit: stack
   completes a straight-line scenario in UE5.
3. **City level**: streets/sidewalks/crosswalks at limo scale from the
   generated artifacts; lidar-visible geometry pass. Exit: sidewalk patrol
   lap parity with Gazebo (laps complete, accident log comparable).
4. **Photoreal + dynamic agents**: Lumen lighting, Megascans, UE-native
   pedestrians/traffic, camera capture for perception datasets.

## Constraints / risks

- **Apple Silicon**: UE5 editor runs on macOS but Nanite/Lumen support is
  limited; full fidelity wants a Windows/Linux + RTX box. Phases 1–2 are
  fine on the Mac.
- **Determinism**: UE physics is frame-rate coupled; fix the tick
  (`t.MaxFPS`, substepping) before comparing runs.
- **Scale**: keep the limo-scale world (1 uu = 1 cm ⇒ robot ≈ 32 uu) —
  do NOT rescale the robot to car size; the stack's dynamics are tuned.
- **Licensing**: UE5 is source-available (Epic EULA), fine for internal
  R&D use; ship nothing containing engine code.
