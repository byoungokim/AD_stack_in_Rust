/// Simulator ZMQ-based implementations of SensorSource and VehicleController.
///
/// Subscribes to CH5 (SimSensorData) and CH6 (SimVehicleState) from
/// any simulator (Gazebo, Isaac Sim, dummy), and publishes CH7
/// (SimControlCommand) back to the simulator.
use std::cell::Cell;
use std::time::Duration;

use anyhow::Result;
use serde::Deserialize;
use tracing::info;

use limo_transport::{Channel, Publisher, Subscriber};

use crate::{
    CameraFrame, ChassisFeedback, ImuReading, LidarScan, MotorCommand, Pose2D, SensorSource,
    Twist2D, VehicleController,
};

// ======================== Configuration ========================

/// Ackermann geometry used by the sim HAL to convert (v, ω) to a steering angle.
/// Defaults match the Limo Pro.
#[derive(Clone, Debug)]
pub struct SimAckermannConfig {
    pub wheelbase: f64,
    pub max_steering_angle: f64,
    pub track_width: f64,
    pub wheel_radius: f64,
}

impl Default for SimAckermannConfig {
    fn default() -> Self {
        Self {
            wheelbase: 0.2,
            max_steering_angle: 0.48,
            track_width: 0.172,
            wheel_radius: 0.045,
        }
    }
}

/// Per-sensor drop rates in [0.0, 1.0]. A rate of 0 disables dropping.
/// Use `seed` to make drops reproducible across test runs.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct SimFaultConfig {
    // Sensor-side (CH5) drop rates.
    pub camera_drop_rate: f32,
    pub lidar_drop_rate: f32,
    pub imu_drop_rate: f32,
    pub pose_drop_rate: f32,
    pub velocity_drop_rate: f32,
    // Controller-side (CH6) drop rate — simulates lost/late VehicleState feedback.
    pub feedback_drop_rate: f32,
    pub seed: u64,
}

impl SimFaultConfig {
    /// True if any drop rate is non-zero (for startup logging).
    pub fn is_active(&self) -> bool {
        self.camera_drop_rate > 0.0
            || self.lidar_drop_rate > 0.0
            || self.imu_drop_rate > 0.0
            || self.pose_drop_rate > 0.0
            || self.velocity_drop_rate > 0.0
            || self.feedback_drop_rate > 0.0
    }
}

// xorshift64 PRNG. Cell so `should_drop` can take &self.
struct XorShift64 {
    state: Cell<u64>,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        let s = if seed == 0 {
            0xDEAD_BEEF_CAFE_BABE
        } else {
            seed
        };
        Self {
            state: Cell::new(s),
        }
    }

    fn next_f32(&self) -> f32 {
        let mut x = self.state.get();
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state.set(x);
        (x >> 32) as f32 / (1u64 << 32) as f32
    }
}

// ======================== SensorSource ========================

pub struct SimZmqSensorSource {
    ch5_sub: Option<Subscriber>,
    latest_camera: Option<CameraFrame>,
    latest_lidar: Option<LidarScan>,
    latest_imu: Option<ImuReading>,
    latest_pose: Option<(Pose2D, f32)>,
    latest_velocity: Option<Twist2D>,
    faults: SimFaultConfig,
    rng: XorShift64,
}

impl Default for SimZmqSensorSource {
    fn default() -> Self {
        Self::new()
    }
}

impl SimZmqSensorSource {
    pub fn new() -> Self {
        Self::with_faults(SimFaultConfig::default())
    }

    pub fn with_faults(faults: SimFaultConfig) -> Self {
        let rng = XorShift64::new(faults.seed);
        Self {
            ch5_sub: None,
            latest_camera: None,
            latest_lidar: None,
            latest_imu: None,
            latest_pose: None,
            latest_velocity: None,
            faults,
            rng,
        }
    }

    fn should_drop(&self, rate: f32) -> bool {
        rate > 0.0 && self.rng.next_f32() < rate
    }

    /// Poll CH5 and update latest values. Drains all available messages.
    fn poll(&mut self) {
        loop {
            let sim = match self.ch5_sub.as_mut() {
                Some(sub) => {
                    match sub.recv::<limo_proto::SimSensorData>(Duration::from_millis(0)) {
                        Ok(Some(m)) => m,
                        _ => return,
                    }
                }
                None => return,
            };
            // ch5_sub borrow released here; safe to call &self methods below.
            self.apply_sim_sensor(sim);
        }
    }

    fn apply_sim_sensor(&mut self, sim: limo_proto::SimSensorData) {
        let ts = sim.header.as_ref().map(|h| h.timestamp_ns).unwrap_or(0);
        let seq = sim.header.as_ref().map(|h| h.sequence).unwrap_or(0);

        if !sim.camera_image.is_empty() && !self.should_drop(self.faults.camera_drop_rate) {
            self.latest_camera = Some(CameraFrame {
                timestamp_ns: ts,
                width: sim.camera_width,
                height: sim.camera_height,
                encoding: sim.camera_encoding.clone(),
                data: sim.camera_image,
                sequence: seq,
            });
        }
        if let Some(scan) = sim.lidar_scan {
            if !self.should_drop(self.faults.lidar_drop_rate) {
                self.latest_lidar = Some(LidarScan {
                    timestamp_ns: ts,
                    angle_min: scan.angle_min,
                    angle_max: scan.angle_max,
                    angle_increment: scan.angle_increment,
                    range_min: scan.range_min,
                    range_max: scan.range_max,
                    ranges: scan.ranges,
                    intensities: scan.intensities,
                    sequence: seq,
                });
            }
        }
        if let Some(imu) = sim.imu {
            if !self.should_drop(self.faults.imu_drop_rate) {
                let a = imu.linear_acceleration.unwrap_or_default();
                let g = imu.angular_velocity.unwrap_or_default();
                let e = imu.orientation_euler.unwrap_or_default();
                self.latest_imu = Some(ImuReading {
                    timestamp_ns: ts,
                    linear_acceleration: nalgebra::Vector3::new(a.x, a.y, a.z),
                    angular_velocity: nalgebra::Vector3::new(g.x, g.y, g.z),
                    orientation_euler: nalgebra::Vector3::new(e.x, e.y, e.z),
                    sequence: seq,
                });
            }
        }
        if let Some(p) = sim.ground_truth_pose {
            if !self.should_drop(self.faults.pose_drop_rate) {
                self.latest_pose = Some((
                    Pose2D {
                        x: p.x,
                        y: p.y,
                        theta: p.theta,
                    },
                    1.0,
                ));
            }
        }
        if let Some(v) = sim.ground_truth_velocity {
            if !self.should_drop(self.faults.velocity_drop_rate) {
                self.latest_velocity = Some(Twist2D {
                    linear_x: v.linear_x,
                    linear_y: v.linear_y,
                    angular_z: v.angular_z,
                });
            }
        }
    }
}

impl SensorSource for SimZmqSensorSource {
    fn start(&mut self) -> Result<()> {
        let ctx = zmq::Context::new();
        self.ch5_sub = Some(Subscriber::connect(
            &ctx,
            Channel::SimSensors.connect_endpoint(),
            Channel::SimSensors.topic(),
        )?);
        info!(
            "SimZmqSensorSource started (CH5: {})",
            Channel::SimSensors.connect_endpoint()
        );
        Ok(())
    }

    fn stop(&mut self) {
        self.ch5_sub = None;
        info!("SimZmqSensorSource stopped");
    }

    fn recv_camera(&mut self) -> Option<CameraFrame> {
        self.poll();
        self.latest_camera.take()
    }

    fn recv_lidar(&mut self) -> Option<LidarScan> {
        self.poll();
        self.latest_lidar.take()
    }

    fn recv_imu(&mut self) -> Option<ImuReading> {
        self.poll();
        self.latest_imu.take()
    }

    fn recv_pose(&mut self) -> Option<(Pose2D, f32)> {
        self.poll();
        self.latest_pose.take()
    }

    fn recv_velocity(&mut self) -> Option<Twist2D> {
        self.poll();
        self.latest_velocity.take()
    }

    fn name(&self) -> &str {
        "sim_zmq"
    }
}

// ======================== VehicleController ========================

pub struct SimZmqVehicleController {
    ch6_sub: Option<Subscriber>,
    ch7_pub: Option<Publisher>,
    latest_feedback: Option<ChassisFeedback>,
    sequence: u32,
    kinematics: SimAckermannConfig,
    faults: SimFaultConfig,
    rng: XorShift64,
}

impl Default for SimZmqVehicleController {
    fn default() -> Self {
        Self::new()
    }
}

impl SimZmqVehicleController {
    pub fn new() -> Self {
        Self::with_config(SimAckermannConfig::default(), SimFaultConfig::default())
    }

    pub fn with_kinematics(kinematics: SimAckermannConfig) -> Self {
        Self::with_config(kinematics, SimFaultConfig::default())
    }

    pub fn with_config(kinematics: SimAckermannConfig, faults: SimFaultConfig) -> Self {
        let rng = XorShift64::new(faults.seed);
        Self {
            ch6_sub: None,
            ch7_pub: None,
            latest_feedback: None,
            sequence: 0,
            kinematics,
            faults,
            rng,
        }
    }

    fn should_drop(&self, rate: f32) -> bool {
        rate > 0.0 && self.rng.next_f32() < rate
    }

    /// Ackermann bicycle model: delta = atan(omega * L / v), clamped.
    fn compute_steering(&self, cmd: &MotorCommand) -> f32 {
        if cmd.linear_vel.abs() < 1e-6 {
            return 0.0;
        }
        let s = (cmd.angular_vel * self.kinematics.wheelbase / cmd.linear_vel).atan();
        s.clamp(
            -self.kinematics.max_steering_angle,
            self.kinematics.max_steering_angle,
        ) as f32
    }

    /// Synthesize wheel RPMs and a steering angle from the sim's body twist,
    /// so control's odometry integrates sim motion exactly as it would real
    /// encoder feedback. `reported_steering` from the sim wins when nonzero
    /// (the Gazebo bridge leaves it at 0).
    /// Returns (left_rpm, right_rpm, steering_angle).
    fn synthesize_feedback(
        &self,
        linear_vel: f64,
        angular_vel: f64,
        reported_steering: f32,
    ) -> (f32, f32, f32) {
        let half_track = self.kinematics.track_width / 2.0;
        let vel_to_rpm = |vel: f64| {
            (vel * 60.0 / (2.0 * std::f64::consts::PI * self.kinematics.wheel_radius)) as f32
        };
        let left_rpm = vel_to_rpm(linear_vel - angular_vel * half_track);
        let right_rpm = vel_to_rpm(linear_vel + angular_vel * half_track);
        let steering = if reported_steering.abs() > 1e-6 {
            reported_steering
        } else {
            self.compute_steering(&MotorCommand {
                linear_vel,
                angular_vel,
            })
        };
        (left_rpm, right_rpm, steering)
    }

    fn poll_feedback(&mut self) {
        loop {
            let vs = match self.ch6_sub.as_mut() {
                Some(sub) => {
                    match sub.recv::<limo_proto::SimVehicleState>(Duration::from_millis(0)) {
                        Ok(Some(m)) => m,
                        _ => return,
                    }
                }
                None => return,
            };
            // ch6_sub borrow released here; safe to call &self methods.
            if self.should_drop(self.faults.feedback_drop_rate) {
                continue;
            }
            let (linear_vel, angular_vel) = vs
                .velocity
                .as_ref()
                .map(|t| (t.linear_x, t.angular_z))
                .unwrap_or((0.0, 0.0));
            let (left_wheel_rpm, right_wheel_rpm, steering_angle) =
                self.synthesize_feedback(linear_vel, angular_vel, vs.steering_angle);
            self.latest_feedback = Some(ChassisFeedback {
                left_wheel_rpm,
                right_wheel_rpm,
                steering_angle,
                battery_voltage: vs.battery_voltage,
                error_code: 0,
                timestamp_ns: vs.header.as_ref().map(|h| h.timestamp_ns).unwrap_or(0),
            });
        }
    }
}

impl VehicleController for SimZmqVehicleController {
    fn start(&mut self) -> Result<()> {
        let ctx = zmq::Context::new();
        self.ch6_sub = Some(Subscriber::connect(
            &ctx,
            Channel::SimVehicleState.connect_endpoint(),
            Channel::SimVehicleState.topic(),
        )?);
        self.ch7_pub = Some(Publisher::bind(
            &ctx,
            Channel::SimControl.bind_endpoint(),
            Channel::SimControl.topic(),
        )?);
        info!(
            "SimZmqVehicleController started (CH6: {}, CH7: {})",
            Channel::SimVehicleState.connect_endpoint(),
            Channel::SimControl.bind_endpoint()
        );
        Ok(())
    }

    fn stop(&mut self) {
        self.ch6_sub = None;
        self.ch7_pub = None;
        info!("SimZmqVehicleController stopped");
    }

    fn send_command(&mut self, cmd: &MotorCommand) -> Result<()> {
        let steering = self.compute_steering(cmd);
        let sequence = self.sequence;
        if let Some(pub7) = &mut self.ch7_pub {
            let msg = limo_proto::SimControlCommand {
                header: Some(limo_proto::Header {
                    timestamp_ns: std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_nanos() as u64,
                    sequence,
                    frame_id: "".into(),
                }),
                linear_velocity: cmd.linear_vel as f32,
                angular_velocity: cmd.angular_vel as f32,
                steering_angle: steering,
                emergency_stop: false,
            };
            pub7.publish(&msg)?;
            self.sequence += 1;
        }
        Ok(())
    }

    fn recv_feedback(&mut self) -> Option<ChassisFeedback> {
        self.poll_feedback();
        self.latest_feedback.clone()
    }

    fn name(&self) -> &str {
        "sim_zmq"
    }
}

// ======================== Tests ========================

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Steering ----

    #[test]
    fn compute_steering_zero_velocity_returns_zero() {
        let ctrl = SimZmqVehicleController::new();
        let cmd = MotorCommand {
            linear_vel: 0.0,
            angular_vel: 1.0,
        };
        assert_eq!(ctrl.compute_steering(&cmd), 0.0);
    }

    #[test]
    fn compute_steering_left_turn_positive() {
        let ctrl = SimZmqVehicleController::new();
        let cmd = MotorCommand {
            linear_vel: 0.5,
            angular_vel: 0.5,
        };
        let s = ctrl.compute_steering(&cmd);
        assert!(s > 0.0 && s <= 0.48);
    }

    #[test]
    fn compute_steering_right_turn_negative() {
        let ctrl = SimZmqVehicleController::new();
        let cmd = MotorCommand {
            linear_vel: 0.5,
            angular_vel: -0.5,
        };
        let s = ctrl.compute_steering(&cmd);
        assert!((-0.48..0.0).contains(&s));
    }

    #[test]
    fn compute_steering_clamps_to_max() {
        let ctrl = SimZmqVehicleController::new();
        // Huge angular for small linear → raw atan approaches π/2, must clamp.
        let cmd = MotorCommand {
            linear_vel: 0.01,
            angular_vel: 5.0,
        };
        let s = ctrl.compute_steering(&cmd);
        assert!((s - 0.48).abs() < 1e-5);
    }

    #[test]
    fn compute_steering_honors_custom_wheelbase() {
        let ctrl = SimZmqVehicleController::with_kinematics(SimAckermannConfig {
            wheelbase: 1.0,
            max_steering_angle: 1.0,
            ..Default::default()
        });
        let cmd = MotorCommand {
            linear_vel: 1.0,
            angular_vel: 1.0,
        };
        // atan(1 * 1 / 1) = π/4
        assert!((ctrl.compute_steering(&cmd) - std::f32::consts::FRAC_PI_4).abs() < 1e-3);
    }

    // ---- Feedback synthesis ----

    /// Wheel velocity in m/s recovered from a synthesized RPM.
    fn rpm_to_vel(rpm: f32, wheel_radius: f64) -> f64 {
        rpm as f64 * 2.0 * std::f64::consts::PI * wheel_radius / 60.0
    }

    #[test]
    fn synthesize_feedback_straight_line() {
        let ctrl = SimZmqVehicleController::new();
        let cfg = SimAckermannConfig::default();
        let (left, right, steering) = ctrl.synthesize_feedback(0.5, 0.0, 0.0);
        assert!((left - right).abs() < 1e-6);
        assert!((rpm_to_vel(left, cfg.wheel_radius) - 0.5).abs() < 1e-6);
        assert_eq!(steering, 0.0);
    }

    #[test]
    fn synthesize_feedback_turn_roundtrips_twist() {
        // Differential model must recover the original (v, ω) exactly:
        // v = (v_l + v_r)/2, ω = (v_r - v_l)/track.
        let ctrl = SimZmqVehicleController::new();
        let cfg = SimAckermannConfig::default();
        let (v, w) = (0.4, 0.8);
        let (left, right, steering) = ctrl.synthesize_feedback(v, w, 0.0);
        let (lv, rv) = (
            rpm_to_vel(left, cfg.wheel_radius),
            rpm_to_vel(right, cfg.wheel_radius),
        );
        assert!(((lv + rv) / 2.0 - v).abs() < 1e-4);
        assert!(((rv - lv) / cfg.track_width - w).abs() < 1e-4);
        // Ackermann model must also recover ω: v·tan(δ)/L with δ = atan(ωL/v).
        let w_ackermann = v * (steering as f64).tan() / cfg.wheelbase;
        assert!((w_ackermann - w).abs() < 1e-4);
    }

    #[test]
    fn synthesize_feedback_prefers_reported_steering() {
        let ctrl = SimZmqVehicleController::new();
        let (_, _, steering) = ctrl.synthesize_feedback(0.5, 0.8, 0.2);
        assert_eq!(steering, 0.2);
    }

    #[test]
    fn synthesize_feedback_stationary_is_zero() {
        let ctrl = SimZmqVehicleController::new();
        let (left, right, steering) = ctrl.synthesize_feedback(0.0, 0.0, 0.0);
        assert_eq!(left, 0.0);
        assert_eq!(right, 0.0);
        assert_eq!(steering, 0.0);
    }

    // ---- Fault injection ----

    #[test]
    fn default_faults_never_drop() {
        let src = SimZmqSensorSource::new();
        for _ in 0..1000 {
            assert!(!src.should_drop(src.faults.camera_drop_rate));
        }
    }

    #[test]
    fn drop_rate_one_always_drops() {
        let src = SimZmqSensorSource::with_faults(SimFaultConfig {
            camera_drop_rate: 1.0,
            seed: 42,
            ..Default::default()
        });
        for _ in 0..100 {
            assert!(src.should_drop(src.faults.camera_drop_rate));
        }
    }

    #[test]
    fn drop_rate_half_is_approximate_half() {
        let src = SimZmqSensorSource::with_faults(SimFaultConfig {
            camera_drop_rate: 0.5,
            seed: 12345,
            ..Default::default()
        });
        let drops = (0..10_000)
            .filter(|_| src.should_drop(src.faults.camera_drop_rate))
            .count();
        // With N=10k, 3σ on a 0.5 Bernoulli is ~150; 4000..6000 is comfortable.
        assert!(drops > 4500 && drops < 5500, "drops = {}", drops);
    }

    #[test]
    fn xorshift_is_seed_deterministic() {
        let a = XorShift64::new(7);
        let b = XorShift64::new(7);
        for _ in 0..10 {
            assert_eq!(a.next_f32(), b.next_f32());
        }
    }

    // ---- Controller-side fault injection ----

    #[test]
    fn controller_feedback_drop_rate_zero_never_drops() {
        let ctrl = SimZmqVehicleController::with_config(
            SimAckermannConfig::default(),
            SimFaultConfig::default(),
        );
        for _ in 0..1000 {
            assert!(!ctrl.should_drop(ctrl.faults.feedback_drop_rate));
        }
    }

    #[test]
    fn controller_feedback_drop_rate_one_always_drops() {
        let ctrl = SimZmqVehicleController::with_config(
            SimAckermannConfig::default(),
            SimFaultConfig {
                feedback_drop_rate: 1.0,
                seed: 99,
                ..Default::default()
            },
        );
        for _ in 0..100 {
            assert!(ctrl.should_drop(ctrl.faults.feedback_drop_rate));
        }
    }

    #[test]
    fn is_active_detects_feedback_drop() {
        let cfg = SimFaultConfig {
            feedback_drop_rate: 0.2,
            ..Default::default()
        };
        assert!(cfg.is_active());
    }
}
