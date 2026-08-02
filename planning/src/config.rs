/// Configuration for the Planning process.
use serde::Deserialize;

use crate::arbitrator::ArbitratorConfig;
use crate::behavior::BehaviorConfig;
use crate::e2e::E2EConfig;
use crate::global_planner::HybridAStarConfig;
use crate::local_planner::LocalPlannerConfig;

#[derive(Debug, Clone, Default, Deserialize)]
pub struct PlanningConfig {
    #[serde(default)]
    pub behavior: BehaviorConfig,
    #[serde(default)]
    pub global_planner: HybridAStarConfig,
    #[serde(default)]
    pub local_planner: LocalPlannerConfig,
    #[serde(default)]
    pub arbitrator: ArbitratorConfig,
    #[serde(default)]
    pub e2e: E2EConfig,
    #[serde(default)]
    pub transport: TransportConfig,
    #[serde(default)]
    pub fault_tolerance: FaultToleranceConfig,
    #[serde(default)]
    pub roadmap: RoadmapConfig,
}

/// Prior node-link roadmap layer for global routing (see
/// `planning/src/roadmap.rs`). Disabled (the default) reproduces the old
/// direct-to-mission-waypoint goal flow exactly.
#[derive(Debug, Clone, Deserialize)]
pub struct RoadmapConfig {
    #[serde(default)]
    pub enabled: bool,
    /// Roadmap YAML (nodes + links), path relative to the working directory.
    #[serde(default = "default_roadmap_file")]
    pub file: String,
    /// A reported link block expires after this many seconds — routing
    /// excludes blocked links, but never permanently.
    #[serde(default = "default_blocked_link_timeout_s")]
    pub blocked_link_timeout_s: f64,
    /// No progress toward the current route leg goal for this long marks
    /// that leg's link blocked and triggers a reroute.
    #[serde(default = "default_link_block_after_s")]
    pub link_block_after_s: f64,
    /// Route leg fed to Hybrid A*: the leg goal is kept until the robot
    /// closes within `leg_min_m` of it, then re-placed `leg_max_m` ahead
    /// along the route (snapped to a route node when one is in the window).
    #[serde(default = "default_leg_min_m")]
    pub leg_min_m: f64,
    #[serde(default = "default_leg_max_m")]
    pub leg_max_m: f64,
    /// Lateral deviation from the route polyline that forces a reroute.
    #[serde(default = "default_route_deviation_m")]
    pub deviation_m: f64,
}

fn default_roadmap_file() -> String {
    "config/maps/obstacle_gauntlet_roadmap.yaml".into()
}
fn default_blocked_link_timeout_s() -> f64 {
    20.0
}
fn default_link_block_after_s() -> f64 {
    10.0
}
fn default_leg_min_m() -> f64 {
    2.0
}
fn default_leg_max_m() -> f64 {
    4.0
}
fn default_route_deviation_m() -> f64 {
    1.5
}

impl Default for RoadmapConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            file: default_roadmap_file(),
            blocked_link_timeout_s: default_blocked_link_timeout_s(),
            link_block_after_s: default_link_block_after_s(),
            leg_min_m: default_leg_min_m(),
            leg_max_m: default_leg_max_m(),
            deviation_m: default_route_deviation_m(),
        }
    }
}

impl RoadmapConfig {
    /// Fail loudly on nonsense YAML: zero/negative timings would block every
    /// link instantly or never expire a block; a degenerate leg window would
    /// pin the route goal onto the robot.
    pub fn validate(&self) -> Result<(), String> {
        let positive = [
            (
                "roadmap.blocked_link_timeout_s",
                self.blocked_link_timeout_s,
            ),
            ("roadmap.link_block_after_s", self.link_block_after_s),
            ("roadmap.leg_min_m", self.leg_min_m),
            ("roadmap.leg_max_m", self.leg_max_m),
            ("roadmap.deviation_m", self.deviation_m),
        ];
        for (name, v) in positive {
            if !(v > 0.0 && v.is_finite()) {
                return Err(format!("{} must be positive, got {}", name, v));
            }
        }
        if self.leg_min_m >= self.leg_max_m {
            return Err(format!(
                "roadmap.leg_min_m ({}) must be < roadmap.leg_max_m ({})",
                self.leg_min_m, self.leg_max_m
            ));
        }
        if self.enabled && self.file.is_empty() {
            return Err("roadmap.file must be set when roadmap.enabled".into());
        }
        Ok(())
    }
}

/// Input-staleness thresholds driving the Layer-1 software E-stop.
#[derive(Debug, Clone, Deserialize)]
pub struct FaultToleranceConfig {
    /// WorldState (CH1) older than this → planning is blind → software E-stop.
    #[serde(default = "default_world_state_stale_ms")]
    pub world_state_stale_ms: u64,
    /// VehicleState (CH3) older than this → warn (degradation matrix: alert
    /// operator; Control's own watchdog covers the actuation side).
    #[serde(default = "default_vehicle_state_stale_ms")]
    pub vehicle_state_stale_ms: u64,
}

fn default_world_state_stale_ms() -> u64 {
    300
}
fn default_vehicle_state_stale_ms() -> u64 {
    500
}

impl Default for FaultToleranceConfig {
    fn default() -> Self {
        Self {
            world_state_stale_ms: default_world_state_stale_ms(),
            vehicle_state_stale_ms: default_vehicle_state_stale_ms(),
        }
    }
}

impl FaultToleranceConfig {
    /// A zero threshold would declare every cycle stale and latch the E-stop
    /// forever — a YAML typo must not disable the vehicle. Fail loudly.
    pub fn validate(&self) -> Result<(), String> {
        if self.world_state_stale_ms == 0 {
            return Err("fault_tolerance.world_state_stale_ms must be > 0".into());
        }
        if self.vehicle_state_stale_ms == 0 {
            return Err("fault_tolerance.vehicle_state_stale_ms must be > 0".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct TransportConfig {
    #[serde(default = "default_ch1_endpoint")]
    pub ch1_endpoint: String, // subscribe WorldState
    #[serde(default = "default_ch2_endpoint")]
    pub ch2_endpoint: String, // publish ControlCommand
    #[serde(default = "default_ch3_endpoint")]
    pub ch3_endpoint: String, // subscribe VehicleState
}

fn default_ch1_endpoint() -> String {
    "tcp://localhost:5551".into()
}
fn default_ch2_endpoint() -> String {
    "tcp://*:5552".into()
}
fn default_ch3_endpoint() -> String {
    "tcp://localhost:5553".into()
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            ch1_endpoint: default_ch1_endpoint(),
            ch2_endpoint: default_ch2_endpoint(),
            ch3_endpoint: default_ch3_endpoint(),
        }
    }
}

pub fn load_config(path: &str) -> anyhow::Result<PlanningConfig> {
    let contents = std::fs::read_to_string(path)?;
    let config: PlanningConfig = serde_yaml::from_str(&contents)?;
    Ok(config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fault_tolerance_defaults_are_valid() {
        let cfg = FaultToleranceConfig::default();
        assert_eq!(cfg.world_state_stale_ms, 300);
        assert_eq!(cfg.vehicle_state_stale_ms, 500);
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn fault_tolerance_rejects_zero_thresholds() {
        let cfg = FaultToleranceConfig {
            world_state_stale_ms: 0,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
        let cfg = FaultToleranceConfig {
            vehicle_state_stale_ms: 0,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn recovery_and_degraded_keys_parse_from_yaml() {
        // The new graduated-response / recovery keys as shipped in
        // config/planning.yaml must land in the right nested structs.
        let yaml = r#"
behavior:
  default_speed: 2.2
  obstacle_avoidance_speed_factor: 0.65
  stuck_cycles_before_recovery: 20
  recovery_reverse_max_s: 3.0
  recovery_max_attempts: 3
  recovery_reverse_min_m: 0.15
  recovery_reverse_max_m: 0.5
  recovery_exit_feasible_cycles: 3
  recovery_progress_reset_m: 0.5
  hold_retry_period_s: 10.0
global_planner:
  clearance_cost_weight: 2.0
  clearance_decay_m: 0.5
  smoothing_enabled: true
  smoothing_alpha: 0.25
  smoothing_clearance_beta: 0.1
  smoothing_iterations: 40
  path_improvement_threshold: 0.2
  start_escape_radius: 0.5
local_planner:
  mpc_trigger_curvature: 2.2
  pursuit:
    k_v: 0.9
    lookahead_min: 0.7
    lookahead_max: 2.4
    a_lat_max: 1.8
  dwa:
    max_speed: 2.2
    max_acceleration: 2.5
    max_deceleration: 3.0
    max_angular_speed: 4.5
    max_angular_accel: 8.0
    v_samples: 13
    w_samples: 15
    weight_continuity: 0.1
    recovery_margin_scale: 0.8
    robot_radius: 0.24
    margin_low_speed_scale: 0.4
    moving_obstacle_margin_gain: 0.4
    high_speed_margin_gain: 0.06
arbitrator:
  degraded_speed_cap: 0.15
  confidence_recovery_cycles: 5
"#;
        let cfg: PlanningConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(cfg.behavior.default_speed, 2.2);
        assert_eq!(cfg.behavior.obstacle_avoidance_speed_factor, 0.65);
        assert_eq!(cfg.behavior.stuck_cycles_before_recovery, 20);
        assert_eq!(cfg.behavior.recovery_reverse_max_s, 3.0);
        assert_eq!(cfg.behavior.recovery_max_attempts, 3);
        assert_eq!(cfg.behavior.recovery_reverse_min_m, 0.15);
        assert_eq!(cfg.behavior.recovery_reverse_max_m, 0.5);
        assert_eq!(cfg.behavior.recovery_exit_feasible_cycles, 3);
        assert_eq!(cfg.behavior.recovery_progress_reset_m, 0.5);
        assert_eq!(cfg.behavior.hold_retry_period_s, 10.0);
        assert_eq!(cfg.global_planner.clearance_cost_weight, 2.0);
        assert_eq!(cfg.global_planner.clearance_decay_m, 0.5);
        assert!(cfg.global_planner.smoothing_enabled);
        assert_eq!(cfg.global_planner.smoothing_alpha, 0.25);
        assert_eq!(cfg.global_planner.smoothing_clearance_beta, 0.1);
        assert_eq!(cfg.global_planner.smoothing_iterations, 40);
        assert_eq!(cfg.global_planner.path_improvement_threshold, 0.2);
        assert_eq!(cfg.global_planner.start_escape_radius, 0.5);
        assert_eq!(cfg.local_planner.mpc_trigger_curvature, 2.2);
        assert_eq!(cfg.local_planner.pursuit.k_v, 0.9);
        assert_eq!(cfg.local_planner.pursuit.lookahead_min, 0.7);
        assert_eq!(cfg.local_planner.pursuit.lookahead_max, 2.4);
        assert_eq!(cfg.local_planner.pursuit.a_lat_max, 1.8);
        assert_eq!(cfg.local_planner.dwa.max_speed, 2.2);
        assert_eq!(cfg.local_planner.dwa.max_acceleration, 2.5);
        assert_eq!(cfg.local_planner.dwa.max_deceleration, 3.0);
        assert_eq!(cfg.local_planner.dwa.max_angular_speed, 4.5);
        assert_eq!(cfg.local_planner.dwa.max_angular_accel, 8.0);
        assert_eq!(cfg.local_planner.dwa.v_samples, 13);
        assert_eq!(cfg.local_planner.dwa.w_samples, 15);
        assert_eq!(cfg.local_planner.dwa.weight_continuity, 0.1);
        assert_eq!(cfg.local_planner.dwa.recovery_margin_scale, 0.8);
        assert_eq!(cfg.local_planner.dwa.robot_radius, 0.24);
        assert_eq!(cfg.local_planner.dwa.margin_low_speed_scale, 0.4);
        assert_eq!(cfg.local_planner.dwa.moving_obstacle_margin_gain, 0.4);
        assert_eq!(cfg.local_planner.dwa.high_speed_margin_gain, 0.06);
        assert_eq!(cfg.arbitrator.degraded_speed_cap, 0.15);
        assert_eq!(cfg.arbitrator.confidence_recovery_cycles, 5);
        assert!(cfg.behavior.validate().is_ok());
        assert!(cfg.arbitrator.validate().is_ok());
        assert!(cfg.global_planner.validate().is_ok());
        assert!(cfg.local_planner.validate().is_ok());
    }

    #[test]
    fn shipped_planning_yaml_parses_and_validates() {
        // The checked-in config/planning.yaml must never drift from the
        // structs: a key that silently lands in the wrong place is a runtime
        // behavior change with no error.
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../config/planning.yaml");
        let cfg = load_config(path).expect("shipped config/planning.yaml must parse");
        assert!(cfg.behavior.validate().is_ok());
        assert!(cfg.arbitrator.validate().is_ok());
        assert!(cfg.fault_tolerance.validate().is_ok());
        assert!(cfg.global_planner.validate().is_ok());
        assert!(cfg.local_planner.validate().is_ok());
        // Raised dynamics chain: DWA, behavior, and the envelope move together.
        assert_eq!(cfg.behavior.default_speed, 2.2);
        assert_eq!(cfg.behavior.obstacle_avoidance_speed_factor, 0.65);
        assert_eq!(cfg.local_planner.dwa.max_speed, 2.2);
        assert_eq!(cfg.local_planner.dwa.max_acceleration, 2.5);
        assert_eq!(cfg.local_planner.dwa.max_deceleration, 3.0);
        assert_eq!(cfg.local_planner.dwa.max_angular_speed, 5.5);
        assert_eq!(cfg.local_planner.dwa.max_curvature, 2.4);
        assert_eq!(cfg.local_planner.dwa.max_angular_accel, 20.0);
        assert_eq!(cfg.local_planner.dwa.high_speed_margin_gain, 0.03);
        assert_eq!(cfg.arbitrator.safety.max_speed, 2.2);
        assert_eq!(cfg.arbitrator.safety.max_acceleration, 2.5);
        assert_eq!(cfg.arbitrator.safety.max_deceleration, 3.0);
        assert_eq!(cfg.arbitrator.safety.max_angular_speed, 5.5);
        assert_eq!(cfg.arbitrator.safety.max_curvature, 2.4);
        // Coherence: DWA never verifies dynamics the envelope would clamp,
        // and the curvature envelope is executable at full cruise.
        assert!(cfg.local_planner.dwa.max_speed <= cfg.arbitrator.safety.max_speed);
        assert!(cfg.local_planner.dwa.max_deceleration <= cfg.arbitrator.safety.max_deceleration);
        assert!(
            cfg.local_planner.dwa.max_curvature * cfg.local_planner.dwa.max_speed
                <= cfg.arbitrator.safety.max_angular_speed
        );
        assert_eq!(cfg.behavior.recovery_reverse_min_m, 0.15);
        assert_eq!(cfg.behavior.recovery_reverse_max_m, 0.5);
        assert_eq!(cfg.behavior.recovery_exit_feasible_cycles, 3);
        assert_eq!(cfg.behavior.recovery_progress_reset_m, 0.5);
        assert_eq!(cfg.behavior.hold_retry_period_s, 10.0);
        assert_eq!(cfg.local_planner.dwa.robot_radius, 0.24);
        assert_eq!(cfg.local_planner.dwa.margin_low_speed_scale, 0.4);
        assert_eq!(cfg.local_planner.dwa.moving_obstacle_margin_gain, 0.4);
        assert_eq!(cfg.global_planner.clearance_cost_weight, 3.0);
        assert_eq!(cfg.global_planner.clearance_decay_m, 0.5);
        // Maneuver planning ships enabled: bidirectional primitives with the
        // tested penalties, RS goal expansion, and the pursuit reverse cap.
        assert!(cfg.global_planner.reverse_enabled);
        assert_eq!(cfg.global_planner.direction_switch_penalty, 0.6);
        assert_eq!(cfg.global_planner.reverse_cost_multiplier, 2.0);
        assert!(cfg.global_planner.rs_expansion_enabled);
        assert_eq!(cfg.global_planner.rs_expansion_radius, 2.0);
        // Start-pocket escape ships enabled at the tested default radius.
        assert_eq!(cfg.global_planner.start_escape_radius, 0.6);
        assert_eq!(cfg.local_planner.pursuit.reverse_speed_cap, 0.4);
        // Path smoothing + hysteresis ship enabled with the tested defaults.
        assert!(cfg.global_planner.smoothing_enabled);
        assert_eq!(cfg.global_planner.smoothing_alpha, 0.3);
        assert_eq!(cfg.global_planner.smoothing_clearance_beta, 0.2);
        assert_eq!(cfg.global_planner.smoothing_iterations, 50);
        assert_eq!(cfg.global_planner.path_improvement_threshold, 0.15);
        // Pure-pursuit primary executor (apex-hitch fix): pursuit keys ship
        // with the defaults, and the stuck detector is calmed back down —
        // recovery is the exception again, not the apex-transition workaround.
        assert_eq!(cfg.local_planner.pursuit.k_v, 1.0);
        assert_eq!(cfg.local_planner.pursuit.lookahead_min, 0.6);
        assert_eq!(cfg.local_planner.pursuit.lookahead_max, 2.5);
        assert_eq!(cfg.local_planner.pursuit.a_lat_max, 2.0);
        assert_eq!(cfg.behavior.stuck_cycles_before_recovery, 15);
        // Roadmap layer ships enabled for the gauntlet with the tested map
        // and timing defaults.
        assert!(cfg.roadmap.enabled);
        assert_eq!(
            cfg.roadmap.file,
            "config/maps/obstacle_gauntlet_roadmap.yaml"
        );
        assert_eq!(cfg.roadmap.blocked_link_timeout_s, 20.0);
        assert_eq!(cfg.roadmap.link_block_after_s, 10.0);
        assert_eq!(cfg.roadmap.leg_min_m, 2.0);
        assert_eq!(cfg.roadmap.leg_max_m, 4.0);
        assert_eq!(cfg.roadmap.deviation_m, 1.5);
        assert!(cfg.roadmap.validate().is_ok());
    }

    #[test]
    fn roadmap_config_defaults_are_disabled_and_valid() {
        let cfg = RoadmapConfig::default();
        assert!(!cfg.enabled, "absent roadmap key must mean old behavior");
        assert_eq!(cfg.blocked_link_timeout_s, 20.0);
        assert_eq!(cfg.link_block_after_s, 10.0);
        assert_eq!(cfg.leg_min_m, 2.0);
        assert_eq!(cfg.leg_max_m, 4.0);
        assert_eq!(cfg.deviation_m, 1.5);
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn roadmap_config_rejects_nonsense() {
        let bad = |f: fn(&mut RoadmapConfig)| {
            let mut cfg = RoadmapConfig::default();
            f(&mut cfg);
            cfg.validate()
        };
        assert!(bad(|c| c.blocked_link_timeout_s = 0.0).is_err());
        assert!(bad(|c| c.link_block_after_s = -1.0).is_err());
        assert!(bad(|c| c.deviation_m = 0.0).is_err());
        assert!(bad(|c| c.leg_min_m = 5.0).is_err(), "min >= max");
        assert!(bad(|c| {
            c.enabled = true;
            c.file = String::new();
        })
        .is_err());
    }

    #[test]
    fn roadmap_config_parses_from_yaml() {
        let yaml = r#"
roadmap:
  enabled: true
  file: "config/maps/obstacle_gauntlet_roadmap.yaml"
  blocked_link_timeout_s: 25.0
  link_block_after_s: 8.0
  leg_min_m: 1.5
  leg_max_m: 3.5
  deviation_m: 2.0
"#;
        let cfg: PlanningConfig = serde_yaml::from_str(yaml).unwrap();
        assert!(cfg.roadmap.enabled);
        assert_eq!(
            cfg.roadmap.file,
            "config/maps/obstacle_gauntlet_roadmap.yaml"
        );
        assert_eq!(cfg.roadmap.blocked_link_timeout_s, 25.0);
        assert_eq!(cfg.roadmap.link_block_after_s, 8.0);
        assert_eq!(cfg.roadmap.leg_min_m, 1.5);
        assert_eq!(cfg.roadmap.leg_max_m, 3.5);
        assert_eq!(cfg.roadmap.deviation_m, 2.0);
        assert!(cfg.roadmap.validate().is_ok());
    }

    #[test]
    fn fault_tolerance_parses_from_yaml() {
        let yaml = "fault_tolerance:\n  world_state_stale_ms: 250\n  vehicle_state_stale_ms: 400\n";
        let cfg: PlanningConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(cfg.fault_tolerance.world_state_stale_ms, 250);
        assert_eq!(cfg.fault_tolerance.vehicle_state_stale_ms, 400);
    }
}
