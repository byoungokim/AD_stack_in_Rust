/// Dummy implementations for testing without any external system.
///
/// Generates synthetic sensor data and simulates vehicle physics
/// entirely in-process. Useful for unit testing AD algorithms.
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use anyhow::Result;
use tracing::info;

use crate::{
    CameraFrame, ChassisFeedback, ImuReading, LidarScan, MotorCommand, Pose2D, SensorSource,
    StampedPose, Twist2D, VehicleController,
};

// ======================== SensorSource ========================

pub struct DummySensorSource {
    start_time: Instant,
    sequence: u32,
    width: u32,
    height: u32,
    num_points: usize,
}

impl Default for DummySensorSource {
    fn default() -> Self {
        Self::new()
    }
}

impl DummySensorSource {
    pub fn new() -> Self {
        Self {
            start_time: Instant::now(),
            sequence: 0,
            width: 640,
            height: 480,
            num_points: 360,
        }
    }

    fn now_ns(&self) -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    }
}

impl SensorSource for DummySensorSource {
    fn start(&mut self) -> Result<()> {
        self.start_time = Instant::now();
        info!("DummySensorSource started");
        Ok(())
    }

    fn stop(&mut self) {
        info!("DummySensorSource stopped");
    }

    fn recv_camera(&mut self) -> Option<CameraFrame> {
        let size = (self.width * self.height * 3) as usize;
        let mut data = vec![128u8; size];
        let offset = (self.sequence as usize * 3) % 256;
        for (i, p) in data.iter_mut().enumerate() {
            *p = ((i + offset) % 256) as u8;
        }

        self.sequence += 1;
        Some(CameraFrame {
            timestamp_ns: self.now_ns(),
            width: self.width,
            height: self.height,
            encoding: "bgr8".into(),
            data,
            sequence: self.sequence,
        })
    }

    fn recv_lidar(&mut self) -> Option<LidarScan> {
        let t = self.start_time.elapsed().as_secs_f32();
        let ai = std::f32::consts::TAU / self.num_points as f32;
        let ranges: Vec<f32> = (0..self.num_points)
            .map(|i| {
                let a = i as f32 * ai;
                (4.0 + if (a - 1.5).abs() < 0.3 { -2.0 } else { 0.0 }
                    + (t * 2.0 + i as f32 * 0.05).sin() * 0.03)
                    .max(0.1)
            })
            .collect();

        Some(LidarScan {
            timestamp_ns: self.now_ns(),
            angle_min: 0.0,
            angle_max: std::f32::consts::TAU,
            angle_increment: ai,
            range_min: 0.1,
            range_max: 12.0,
            ranges,
            intensities: vec![200.0; self.num_points],
            sequence: self.sequence,
        })
    }

    fn recv_imu(&mut self) -> Option<ImuReading> {
        let t = self.start_time.elapsed().as_secs_f64();
        Some(ImuReading {
            timestamp_ns: self.now_ns(),
            linear_acceleration: nalgebra::Vector3::new(
                0.02 * (t * 5.0).sin(),
                0.01 * (t * 7.0).cos(),
                9.81,
            ),
            angular_velocity: nalgebra::Vector3::zeros(),
            orientation_euler: nalgebra::Vector3::zeros(),
            sequence: self.sequence,
        })
    }

    fn recv_pose(&mut self) -> Option<StampedPose> {
        Some(StampedPose {
            pose: Pose2D::default(),
            confidence: 1.0,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        })
    }

    fn recv_velocity(&mut self) -> Option<Twist2D> {
        Some(Twist2D::default())
    }

    fn name(&self) -> &str {
        "dummy"
    }
}

// ======================== VehicleController ========================

pub struct DummyVehicleController {
    last_command: MotorCommand,
    x: f64,
    y: f64,
    theta: f64,
    last_update: Instant,
}

impl Default for DummyVehicleController {
    fn default() -> Self {
        Self::new()
    }
}

impl DummyVehicleController {
    pub fn new() -> Self {
        Self {
            last_command: MotorCommand::default(),
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            last_update: Instant::now(),
        }
    }
}

impl VehicleController for DummyVehicleController {
    fn start(&mut self) -> Result<()> {
        self.last_update = Instant::now();
        info!("DummyVehicleController started");
        Ok(())
    }

    fn stop(&mut self) {
        info!("DummyVehicleController stopped");
    }

    fn send_command(&mut self, cmd: &MotorCommand) -> Result<()> {
        self.last_command = cmd.clone();
        Ok(())
    }

    fn recv_feedback(&mut self) -> Option<ChassisFeedback> {
        let dt = self.last_update.elapsed().as_secs_f64();
        self.last_update = Instant::now();

        // Simple kinematics
        self.theta += self.last_command.angular_vel * dt;
        self.x += self.last_command.linear_vel * self.theta.cos() * dt;
        self.y += self.last_command.linear_vel * self.theta.sin() * dt;

        let wheel_radius = 0.045;
        let track_width = 0.172;
        let lv = self.last_command.linear_vel - self.last_command.angular_vel * track_width / 2.0;
        let rv = self.last_command.linear_vel + self.last_command.angular_vel * track_width / 2.0;

        Some(ChassisFeedback {
            left_wheel_rpm: (lv / (2.0 * std::f64::consts::PI * wheel_radius) * 60.0) as f32,
            right_wheel_rpm: (rv / (2.0 * std::f64::consts::PI * wheel_radius) * 60.0) as f32,
            steering_angle: (self.last_command.angular_vel * 0.3) as f32,
            battery_voltage: 12.4,
            error_code: 0,
            timestamp_ns: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        })
    }

    fn name(&self) -> &str {
        "dummy"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dummy_sensor_source() {
        let mut src = DummySensorSource::new();
        src.start().unwrap();

        assert!(src.recv_camera().is_some());
        assert!(src.recv_lidar().is_some());
        assert!(src.recv_imu().is_some());
        assert!(src.recv_pose().is_some());
        assert!(src.recv_velocity().is_some());

        src.stop();
    }

    #[test]
    fn test_dummy_vehicle_controller() {
        let mut ctrl = DummyVehicleController::new();
        ctrl.start().unwrap();

        ctrl.send_command(&MotorCommand {
            linear_vel: 0.5,
            angular_vel: 0.1,
        })
        .unwrap();
        std::thread::sleep(std::time::Duration::from_millis(10));

        let fb = ctrl.recv_feedback().unwrap();
        assert!(fb.battery_voltage > 0.0);

        ctrl.stop();
    }
}
