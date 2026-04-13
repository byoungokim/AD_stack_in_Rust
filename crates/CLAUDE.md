# Shared Crates

Shared libraries used by all processes. Managed by the **Architect Agent**.

## Crates

| Crate | Purpose |
|-------|---------|
| `limo-proto` | Protobuf message definitions (compiled from `proto/`) |
| `limo-transport` | ZMQ pub/sub wrappers, channels (CH0-CH10), heartbeat |
| `limo-hal` | Hardware Abstraction Layer traits + implementations |
| `limo-sim-bridge` | Isaac Sim / Gazebo dummy simulator bridge |
| `limo-scenario` | Scenario manager for navigation goals |

## Rules
- Changes to proto files require updating `limo-proto/build.rs`
- New channels require updating `limo-transport/src/channels.rs`
- HAL type changes affect all processes — coordinate carefully
- Run `cargo test` across the full workspace after any shared crate change
