# SensPerc Process (Process 1)

Full native Rust process for sensing and perception. Uses HAL `SensorSource` trait for platform-agnostic sensor input.

## Agents Working Here
- **Perception Agent**: object detection, lane detection, sensor fusion, camera/LiDAR processing
- **SLAM Agent**: SLAM frontend/backend, localization, map building

## Architecture
```
SensorSource (HAL) → SensorReader thread → SensorStore (ring buffers)
                                              ↓
                                    Aggregator thread → WorldState (CH1)
                                              ↑
                               VehicleState (CH3) for odometry fallback
```

## Key Files
- `src/main.rs` — process entry, HAL dispatch, aggregator loop
- `src/store/` — ring buffers, atomic slots, sensor data types
- `src/config.rs` — YAML config for aggregator

## Build & Test
```bash
cargo check -p limo-sensperc
cargo test -p limo-sensperc
```
