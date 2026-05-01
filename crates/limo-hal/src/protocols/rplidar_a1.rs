//! RPLIDAR A1 standard scan-mode parser.
//!
//! Slamtec's 2D LiDAR streams 5-byte measurement records over UART/USB.
//! Reference: Slamtec public RPLIDAR communication protocol.
//!
//! Byte layout (LSB first within each byte):
//!   B0: bit0 = S, bit1 = !S (start-flag invariant: S xor !S == 1),
//!       bits 2..=7 = quality (0..=63)
//!   B1: bit0 = check (must be 1), bits 1..=7 = angle low 7 bits (Q14.6 deg)
//!   B2: angle high 8 bits
//!   B3: distance low 8 bits  (Q14.2 mm)
//!   B4: distance high 8 bits
//!
//! distance == 0 means "no return for this beam" — we keep it as 0.0 m and
//! let the consumer decide how to mask.

use thiserror::Error;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum RplidarParseError {
    #[error("expected 5 bytes, got {0}")]
    ShortFrame(usize),
    #[error("start-flag invariant violated (S xor !S != 1)")]
    StartFlagInvariant,
    #[error("check bit must be 1")]
    CheckBit,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RplidarSample {
    /// True iff this sample begins a new 360° scan.
    pub start_flag: bool,
    /// 0..=63, higher = better. Hardware-scaled.
    pub quality: u8,
    /// Heading in degrees, 0.0 ≤ angle < 360.0 (per spec; we don't enforce).
    pub angle_deg: f32,
    /// Range in meters. 0.0 = no return.
    pub distance_m: f32,
}

/// Parse one 5-byte RPLIDAR A1 standard-mode sample.
pub fn parse_rplidar_sample(bytes: &[u8]) -> Result<RplidarSample, RplidarParseError> {
    if bytes.len() != 5 {
        return Err(RplidarParseError::ShortFrame(bytes.len()));
    }
    let s = bytes[0] & 0x01;
    let s_inv = (bytes[0] >> 1) & 0x01;
    if (s ^ s_inv) != 1 {
        return Err(RplidarParseError::StartFlagInvariant);
    }
    if (bytes[1] & 0x01) != 1 {
        return Err(RplidarParseError::CheckBit);
    }
    let quality = bytes[0] >> 2;
    let angle_q6 = (u16::from(bytes[1]) >> 1) | (u16::from(bytes[2]) << 7);
    let angle_deg = angle_q6 as f32 / 64.0;
    let distance_q2 = u16::from(bytes[3]) | (u16::from(bytes[4]) << 8);
    let distance_m = (distance_q2 as f32 / 4.0) / 1000.0;
    Ok(RplidarSample {
        start_flag: s == 1,
        quality,
        angle_deg,
        distance_m,
    })
}

/// Streaming framer: feeds bytes from a serial port into 5-byte samples.
/// On parse error, drops the oldest byte and resyncs (typical for unframed
/// streaming protocols where we may start mid-record).
pub struct RplidarFrameBuffer {
    buf: [u8; 5],
    len: usize,
}

impl Default for RplidarFrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl RplidarFrameBuffer {
    pub fn new() -> Self {
        Self {
            buf: [0; 5],
            len: 0,
        }
    }

    /// Push one byte. Returns Some(sample) on a successful parse, None while
    /// buffering or on a parse failure (which triggers a 1-byte resync).
    pub fn push(&mut self, byte: u8) -> Option<RplidarSample> {
        self.buf[self.len] = byte;
        self.len += 1;
        if self.len < 5 {
            return None;
        }
        match parse_rplidar_sample(&self.buf) {
            Ok(s) => {
                self.len = 0;
                Some(s)
            }
            Err(_) => {
                // Resync: shift one byte out, retry on next push.
                self.buf.copy_within(1..5, 0);
                self.len = 4;
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Construct a 5-byte sample with the given fields. Useful for fixtures.
    fn encode(start: bool, quality: u8, angle_deg: f32, distance_m: f32) -> [u8; 5] {
        let s = if start { 1u8 } else { 0u8 };
        let s_inv = 1 - s;
        let b0 = (quality << 2) | (s_inv << 1) | s;
        let angle_q6 = (angle_deg * 64.0).round() as u16;
        let b1 = ((angle_q6 & 0x7F) as u8) << 1 | 0x01;
        let b2 = (angle_q6 >> 7) as u8;
        let distance_q2 = (distance_m * 1000.0 * 4.0).round() as u16;
        let b3 = (distance_q2 & 0xFF) as u8;
        let b4 = (distance_q2 >> 8) as u8;
        [b0, b1, b2, b3, b4]
    }

    #[test]
    fn parses_known_fixture() {
        // start_flag=true, quality=42, angle=90°, distance=1.0m
        let bytes = [0xA9, 0x01, 0x2D, 0xA0, 0x0F];
        let s = parse_rplidar_sample(&bytes).unwrap();
        assert!(s.start_flag);
        assert_eq!(s.quality, 42);
        assert!((s.angle_deg - 90.0).abs() < 1e-3);
        assert!((s.distance_m - 1.0).abs() < 1e-6);
    }

    #[test]
    fn round_trip_via_encode() {
        let fixture = encode(false, 30, 180.5, 2.345);
        let s = parse_rplidar_sample(&fixture).unwrap();
        assert!(!s.start_flag);
        assert_eq!(s.quality, 30);
        assert!((s.angle_deg - 180.5).abs() < 0.05);
        assert!((s.distance_m - 2.345).abs() < 0.001);
    }

    #[test]
    fn rejects_short_frame() {
        assert_eq!(
            parse_rplidar_sample(&[0xA9, 0x01]),
            Err(RplidarParseError::ShortFrame(2))
        );
    }

    #[test]
    fn rejects_invalid_start_flag() {
        // S=0 and !S=0 → invariant broken
        let bytes = [0x00, 0x01, 0x00, 0x00, 0x00];
        assert_eq!(
            parse_rplidar_sample(&bytes),
            Err(RplidarParseError::StartFlagInvariant)
        );
    }

    #[test]
    fn rejects_missing_check_bit() {
        // S=1, !S=0, but C=0
        let bytes = [0x01, 0x00, 0x00, 0x00, 0x00];
        assert_eq!(
            parse_rplidar_sample(&bytes),
            Err(RplidarParseError::CheckBit)
        );
    }

    #[test]
    fn frame_buffer_emits_on_5th_byte() {
        let bytes = encode(true, 10, 45.0, 0.5);
        let mut fb = RplidarFrameBuffer::new();
        assert!(fb.push(bytes[0]).is_none());
        assert!(fb.push(bytes[1]).is_none());
        assert!(fb.push(bytes[2]).is_none());
        assert!(fb.push(bytes[3]).is_none());
        let s = fb.push(bytes[4]).expect("sample emitted");
        assert!(s.start_flag);
        assert!((s.angle_deg - 45.0).abs() < 0.05);
    }

    #[test]
    fn frame_buffer_resyncs_on_garbage_prefix() {
        // Prepend a garbage byte that breaks alignment.
        let good = encode(false, 5, 12.0, 0.3);
        let mut fb = RplidarFrameBuffer::new();
        // 0xFF will fail when treated as B0 of a frame (S=1, !S=1 → invariant fails).
        let _ = fb.push(0xFF);
        // Then push the real frame; framer should resync within ≤ 5 attempts.
        let mut emitted = None;
        for &b in &good {
            if let Some(s) = fb.push(b) {
                emitted = Some(s);
                break;
            }
        }
        // Push up to 4 more bytes if needed for full resync.
        for _ in 0..4 {
            if emitted.is_some() {
                break;
            }
            emitted = fb.push(good[good.len() - 1]);
        }
        let s = emitted.expect("framer should resync within 5 bytes");
        assert!(!s.start_flag);
    }
}
