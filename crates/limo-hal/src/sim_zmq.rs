/// Simulator ZMQ-based implementations of SensorSource and VehicleController.
///
/// Subscribes to CH5 (SimSensorData) and CH6 (SimVehicleState) from
/// any simulator (Gazebo, Isaac Sim, dummy), and publishes CH7
/// (SimControlCommand) back to the simulator.
use std::time::Duration;

use anyhow::Result;
use tracing::{debug, info};

use limo_transport::{Channel, Publisher, Subscriber};

use crate::{
    CameraFrame, ChassisFeedback, ImuReading, LidarScan, MotorCommand,
    Pose2D, SensorSource, Twist2D, VehicleController,
};

// ======================== SensorSource ========================

pub struct SimZmqSensorSource {
    ch5_sub: Option<Subscriber>,
    latest_camera: Option<CameraFrame>,
    latest_lidar: Option<LidarScan>,
    latest_imu: Option<ImuReading>,
    latest_pose: Option<(Pose2D, f32)>,
    latest_velocity: Option<Twist2D>,
}

impl SimZmqSensorSource {
    pub fn new() -> Self {
        Self {
            ch5_sub: None,
            latest_camera: None,
            latest_lidar: None,
            latest_imu: None,
            latest_pose: None,
            latest_velocity: None,
        }
    }

    /// Poll CH5 and update latest values.
    fn poll(&mut self) {
        let sub = match &mut self.ch5_sub {
            Some(s) => s,
            None => return,
        };

        // Drain all available messages, keep latest
        loop {
            match sub.recv::<limo_proto::SimSensorData>(Duration::from_millis(0)) {
                Ok(Some(sim)) => {
                    let ts = sim.header.as_ref().map(|h| h.timestamp_ns).unwrap_or(0);
                    let seq = sim.header.as_ref().map(|h| h.sequence).unwrap_or(0);

                    if !sim.camera_image.is_empty() {
                        self.latest_camera = Some(CameraFrame {
                            timestamp_ns: ts, width: sim.camera_width,
                            height: sim.camera_height, encoding: sim.camera_encoding.clone(),
                            data: sim.camera_image, sequence: seq,
                        });
                    }
                    if let Some(scan) = sim.lidar_scan {
                        self.latest_lidar = Some(LidarScan {
                            timestamp_ns: ts, angle_min: scan.angle_min,
                            angle_max: scan.angle_max, angle_increment: scan.angle_increment,
                            range_min: scan.range_min, range_max: scan.range_max,
                            ranges: scan.ranges, intensities: scan.intensities, sequence: seq,
                        });
                    }
                    if let Some(imu) = sim.imu {
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
                    if let Some(p) = sim.ground_truth_pose {
                        self.latest_pose = Some((Pose2D { x: p.x, y: p.y, theta: p.theta }, 1.0));
                    }
                    if let Some(v) = sim.ground_truth_velocity {
                        self.latest_velocity = Some(Twist2D {
                            linear_x: v.linear_x, linear_y: v.linear_y, angular_z: v.angular_z,
                        });
                    }
                }
                Ok(None) => break, // no more messages
                Err(_) => break,
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
        info!("SimZmqSensorSource started (CH5: {})", Channel::SimSensors.connect_endpoint());
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

    fn name(&self) -> &str { "sim_zmq" }
}

// ======================== VehicleController ========================

pub struct SimZmqVehicleController {
    ch6_sub: Option<Subscriber>,
    ch7_pub: Option<Publisher>,
    latest_feedback: Option<ChassisFeedback>,
    sequence: u32,
}

impl SimZmqVehicleController {
    pub fn new() -> Self {
        Self {
            ch6_sub: None,
            ch7_pub: None,
            latest_feedback: None,
            sequence: 0,
        }
    }

    fn poll_feedback(&mut self) {
        let sub = match &mut self.ch6_sub {
            Some(s) => s,
            None => return,
        };

        loop {
            match sub.recv::<limo_proto::SimVehicleState>(Duration::from_millis(0)) {
                Ok(Some(vs)) => {
                    self.latest_feedback = Some(ChassisFeedback {
                        left_wheel_rpm: 0.0,
                        right_wheel_rpm: 0.0,
                        steering_angle: vs.steering_angle,
                        battery_voltage: vs.battery_voltage,
                        error_code: 0,
                        timestamp_ns: vs.header.as_ref().map(|h| h.timestamp_ns).unwrap_or(0),
                    });
                }
                Ok(None) => break,
                Err(_) => break,
            }
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
        info!("SimZmqVehicleController started (CH6: {}, CH7: {})",
              Channel::SimVehicleState.connect_endpoint(),
              Channel::SimControl.bind_endpoint());
        Ok(())
    }

    fn stop(&mut self) {
        self.ch6_sub = None;
        self.ch7_pub = None;
        info!("SimZmqVehicleController stopped");
    }

    fn send_command(&mut self, cmd: &MotorCommand) -> Result<()> {
        if let Some(pub7) = &mut self.ch7_pub {
            let msg = limo_proto::SimControlCommand {
                header: Some(limo_proto::Header {
                    timestamp_ns: std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64,
                    sequence: self.sequence,
                    frame_id: "".into(),
                }),
                linear_velocity: cmd.linear_vel as f32,
                angular_velocity: cmd.angular_vel as f32,
                steering_angle: 0.0, // TODO: from kinematics
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

    fn name(&self) -> &str { "sim_zmq" }
}
