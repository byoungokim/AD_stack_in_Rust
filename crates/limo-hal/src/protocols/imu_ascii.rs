//! ASCII-CSV IMU line parser.
//!
//! Many cheap USB 9-DOF IMUs emit lines like:
//!   $IMU,0.012,-0.015,9.812,0.001,0.000,0.002,0.0,-0.1,0.0\r\n
//!
//! Field order: linear acceleration x/y/z (m/s²), angular velocity x/y/z
//! (rad/s), orientation roll/pitch/yaw (rad).
//!
//! Timestamp + sequence aren't on the wire; the caller injects them when
//! constructing the HAL `ImuReading` (typically host-clock-on-receipt).

use nalgebra::Vector3;
use thiserror::Error;

use crate::ImuReading;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum ImuParseError {
    #[error("missing $IMU prefix")]
    MissingPrefix,
    #[error("expected 9 fields, got {0}")]
    WrongFieldCount(usize),
    #[error("non-numeric field at index {0}")]
    NonNumeric(usize),
}

/// Parse one ASCII-CSV IMU line. Trailing CR/LF is tolerated.
pub fn parse_imu_csv_line(
    line: &str,
    timestamp_ns: u64,
    sequence: u32,
) -> Result<ImuReading, ImuParseError> {
    let trimmed = line.trim_end_matches(['\r', '\n']);
    let body = trimmed
        .strip_prefix("$IMU,")
        .ok_or(ImuParseError::MissingPrefix)?;
    let fields: Vec<&str> = body.split(',').collect();
    if fields.len() != 9 {
        return Err(ImuParseError::WrongFieldCount(fields.len()));
    }
    let mut nums = [0.0f64; 9];
    for (i, f) in fields.iter().enumerate() {
        nums[i] = f
            .trim()
            .parse::<f64>()
            .map_err(|_| ImuParseError::NonNumeric(i))?;
    }
    Ok(ImuReading {
        timestamp_ns,
        linear_acceleration: Vector3::new(nums[0], nums[1], nums[2]),
        angular_velocity: Vector3::new(nums[3], nums[4], nums[5]),
        orientation_euler: Vector3::new(nums[6], nums[7], nums[8]),
        sequence,
    })
}

/// Streaming line buffer — emits a complete line when CR or LF is seen.
/// Non-ASCII bytes corrupt the current line and reset the buffer; this is
/// safer than partial UTF-8 in a sensor-grade format.
pub struct ImuLineBuffer {
    buf: String,
    max_len: usize,
}

impl Default for ImuLineBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuLineBuffer {
    pub fn new() -> Self {
        Self::with_max_len(256)
    }

    /// Bound the buffer so a sensor stuck mid-line can't grow memory unbounded.
    pub fn with_max_len(max_len: usize) -> Self {
        Self {
            buf: String::with_capacity(64),
            max_len,
        }
    }

    /// Push one byte. Returns Some(line) on terminator, None while buffering.
    pub fn push(&mut self, byte: u8) -> Option<String> {
        match byte {
            b'\r' | b'\n' => {
                if self.buf.is_empty() {
                    return None;
                }
                Some(std::mem::take(&mut self.buf))
            }
            _ => {
                if byte < 0x80 && self.buf.len() < self.max_len {
                    self.buf.push(byte as char);
                } else {
                    self.buf.clear();
                }
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_well_formed_line() {
        let line = "$IMU,0.012,-0.015,9.812,0.001,0.000,0.002,0.05,-0.10,1.57\r\n";
        let r = parse_imu_csv_line(line, 1_000, 7).unwrap();
        assert_eq!(r.timestamp_ns, 1_000);
        assert_eq!(r.sequence, 7);
        assert!((r.linear_acceleration.x - 0.012).abs() < 1e-9);
        assert!((r.linear_acceleration.z - 9.812).abs() < 1e-9);
        assert!((r.angular_velocity.z - 0.002).abs() < 1e-9);
        assert!((r.orientation_euler.z - 1.57).abs() < 1e-9);
    }

    #[test]
    fn tolerates_no_trailing_terminator() {
        let line = "$IMU,1,2,3,4,5,6,7,8,9";
        let r = parse_imu_csv_line(line, 0, 0).unwrap();
        assert_eq!(r.linear_acceleration.x, 1.0);
        assert_eq!(r.orientation_euler.z, 9.0);
    }

    #[test]
    fn rejects_missing_prefix() {
        let r = parse_imu_csv_line("0,0,0,0,0,0,0,0,0", 0, 0);
        assert_eq!(r.unwrap_err(), ImuParseError::MissingPrefix);
    }

    #[test]
    fn rejects_wrong_field_count() {
        let r = parse_imu_csv_line("$IMU,1,2,3", 0, 0);
        assert_eq!(r.unwrap_err(), ImuParseError::WrongFieldCount(3));
    }

    #[test]
    fn rejects_non_numeric_field() {
        let r = parse_imu_csv_line("$IMU,1,2,nope,4,5,6,7,8,9", 0, 0);
        assert_eq!(r.unwrap_err(), ImuParseError::NonNumeric(2));
    }

    #[test]
    fn line_buffer_emits_on_lf() {
        let mut lb = ImuLineBuffer::new();
        for &b in b"$IMU,1,2,3,4,5,6,7,8,9" {
            assert!(lb.push(b).is_none());
        }
        let line = lb.push(b'\n').expect("line should emit on LF");
        assert_eq!(line, "$IMU,1,2,3,4,5,6,7,8,9");
    }

    #[test]
    fn line_buffer_collapses_consecutive_terminators() {
        let mut lb = ImuLineBuffer::new();
        for &b in b"$IMU,1,2,3,4,5,6,7,8,9" {
            assert!(lb.push(b).is_none());
        }
        assert!(lb.push(b'\r').is_some());
        // Empty buffer + another terminator should not emit a phantom line.
        assert!(lb.push(b'\n').is_none());
    }

    #[test]
    fn line_buffer_drops_non_ascii() {
        let mut lb = ImuLineBuffer::new();
        lb.push(b'$');
        lb.push(0xFF); // Non-ASCII → reset
        lb.push(b'I');
        // Buffer was reset by 0xFF; only 'I' remains.
        let out = lb.push(b'\n');
        assert_eq!(out.as_deref(), Some("I"));
    }
}
