# Control Process (Process 3)

Safety-critical Rust process for vehicle actuation. Uses HAL `VehicleController` trait.

## Agent Working Here
- **Control Agent**: trajectory tracking, kinematics, watchdog, emergency stop

## Architecture
```
ControlCommand (CH2) → Tracker (pure pursuit/Stanley) → Kinematics
                                                          ↓
                                              VehicleController (HAL) → Motors
                                              VehicleController → Kinematics (odom)
                                                          ↓
                                              VehicleState (CH3) → SensPerc + Planning

Watchdog: monitors CH2 freshness + peer heartbeats → EmergencyStop
```

## Safety Rules (CRITICAL)
- NEVER weaken the watchdog timeout (200ms)
- NEVER bypass the emergency stop chain
- ALWAYS send zero velocity on shutdown
- ALWAYS enforce hardware limits in clamp_command()

## Key Files
- `src/main.rs` — process entry, HAL dispatch, control loop
- `src/tracker/` — pure pursuit + Stanley controllers
- `src/kinematics/` — Ackermann/differential drive, odometry
- `src/watchdog/` — command timeout, heartbeat monitoring, e-stop

## Build & Test
```bash
cargo check -p limo-control
cargo test -p limo-control    # 12 tests
```
