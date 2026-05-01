//! Wire-protocol parsers for serial sensors.
//!
//! Pure functions over byte slices / strings — no I/O. The runtime serial
//! glue (in `limo_hw.rs`) feeds bytes in; these modules turn them into
//! HAL-typed messages. Keeping parsers pure makes them fixture-testable
//! without hardware so bring-up day is just "wire serialport bytes into
//! the parser, HAL types out."
//!
//! Each protocol module exposes:
//!   - A typed `parse_*` function returning `Result<T, ParseError>`.
//!   - A small framing helper (`*FrameBuffer` / `*LineBuffer`) that aligns
//!     bytes from a streaming serial port into complete records.
//!
//! The two implementations here (RPLIDAR A1, ASCII-CSV IMU) are templates.
//! For a different sensor, add a new module following the same shape and
//! the rest of the stack stays unchanged.

pub mod imu_ascii;
pub mod rplidar_a1;
