/// Limo Drive — Control process library.
///
/// The safety-critical loop body lives in `control_loop` so integration
/// tests (`control/tests/`) can drive it with injected commands and
/// wall-clock time. The `limo_control` binary in `main.rs` wires it to
/// ZMQ, the HAL, and the heartbeat bus.
pub mod config;
pub mod control_loop;
pub mod kinematics;
pub mod tracker;
pub mod watchdog;
