/// Behavior planner: high-level driving state machine at 5Hz.
///
/// Decides the current driving mode based on perception data and goals.
/// Feeds mode decisions to the global/local planners and arbitrator.
use serde::Deserialize;
use tracing::info;

/// Driving behavior states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrivingState {
    /// Waiting for a goal or initial localization.
    Idle,
    /// Following a planned path toward a goal.
    Following,
    /// Approaching the goal, decelerating.
    Approaching,
    /// Reached the goal, holding position.
    GoalReached,
    /// Obstacle detected, replanning or waiting.
    ObstacleAvoidance,
    /// Lost localization or perception failure, controlled stop.
    Degraded,
    /// Emergency stop active.
    EmergencyStop,
}

#[derive(Debug, Clone, Deserialize)]
pub struct BehaviorConfig {
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
    #[serde(default = "default_goal_tolerance")]
    pub goal_tolerance: f64,       // meters
    #[serde(default = "default_approach_distance")]
    pub approach_distance: f64,    // meters, start decelerating
    #[serde(default = "default_obstacle_distance")]
    pub obstacle_stop_distance: f64, // meters, stop if obstacle closer
    #[serde(default = "default_speed")]
    pub default_speed: f64,        // m/s
}

fn default_rate_hz() -> u32 { 5 }
fn default_goal_tolerance() -> f64 { 0.15 }
fn default_approach_distance() -> f64 { 0.5 }
fn default_obstacle_distance() -> f64 { 0.3 }
fn default_speed() -> f64 { 0.5 }

impl Default for BehaviorConfig {
    fn default() -> Self {
        Self {
            rate_hz: default_rate_hz(),
            goal_tolerance: default_goal_tolerance(),
            approach_distance: default_approach_distance(),
            obstacle_stop_distance: default_obstacle_distance(),
            default_speed: default_speed(),
        }
    }
}

/// Goal for the behavior planner.
#[derive(Debug, Clone)]
pub struct Goal {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// Input to the behavior planner from perception.
// `robot_theta` is part of the public pose contract; Debug-logged and will feed heading-aware checks.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct BehaviorInput {
    pub robot_x: f64,
    pub robot_y: f64,
    pub robot_theta: f64,
    pub localization_confidence: f32,
    pub nearest_obstacle_distance: f64,
    pub emergency_stop: bool,
}

/// Output from the behavior planner.
#[derive(Debug, Clone)]
pub struct BehaviorOutput {
    pub state: DrivingState,
    pub desired_speed: f64, // m/s
    pub replan_requested: bool,
}

pub struct BehaviorPlanner {
    config: BehaviorConfig,
    state: DrivingState,
    goal: Option<Goal>,
}

impl BehaviorPlanner {
    pub fn new(config: BehaviorConfig) -> Self {
        Self {
            config,
            state: DrivingState::Idle,
            goal: None,
        }
    }

    pub fn set_goal(&mut self, goal: Goal) {
        info!("Behavior: new goal ({:.2}, {:.2}, {:.1}°)",
              goal.x, goal.y, goal.theta.to_degrees());
        self.goal = Some(goal);
        self.state = DrivingState::Following;
    }

    pub fn clear_goal(&mut self) {
        self.goal = None;
        self.state = DrivingState::Idle;
    }

    pub fn state(&self) -> DrivingState {
        self.state
    }

    /// Update behavior state based on current perception input.
    pub fn update(&mut self, input: &BehaviorInput) -> BehaviorOutput {
        // Emergency stop overrides everything
        if input.emergency_stop {
            self.state = DrivingState::EmergencyStop;
            return BehaviorOutput {
                state: self.state,
                desired_speed: 0.0,
                replan_requested: false,
            };
        }

        // Degraded if localization is poor
        if input.localization_confidence < 0.3 {
            self.state = DrivingState::Degraded;
            return BehaviorOutput {
                state: self.state,
                desired_speed: 0.0,
                replan_requested: false,
            };
        }

        // State machine transitions
        match self.state {
            DrivingState::EmergencyStop => {
                // Recover from e-stop
                if !input.emergency_stop {
                    self.state = if self.goal.is_some() {
                        DrivingState::Following
                    } else {
                        DrivingState::Idle
                    };
                }
            }
            DrivingState::Degraded => {
                if input.localization_confidence >= 0.5 {
                    self.state = if self.goal.is_some() {
                        DrivingState::Following
                    } else {
                        DrivingState::Idle
                    };
                }
            }
            DrivingState::Idle => {
                if self.goal.is_some() {
                    self.state = DrivingState::Following;
                }
            }
            DrivingState::Following => {
                if let Some(goal) = &self.goal {
                    let dist = distance(input.robot_x, input.robot_y, goal.x, goal.y);

                    if dist < self.config.goal_tolerance {
                        self.state = DrivingState::GoalReached;
                    } else if dist < self.config.approach_distance {
                        self.state = DrivingState::Approaching;
                    } else if input.nearest_obstacle_distance < self.config.obstacle_stop_distance {
                        self.state = DrivingState::ObstacleAvoidance;
                    }
                } else {
                    self.state = DrivingState::Idle;
                }
            }
            DrivingState::Approaching => {
                if let Some(goal) = &self.goal {
                    let dist = distance(input.robot_x, input.robot_y, goal.x, goal.y);
                    if dist < self.config.goal_tolerance {
                        self.state = DrivingState::GoalReached;
                    }
                }
            }
            DrivingState::GoalReached => {
                // Stay until a new goal is set
            }
            DrivingState::ObstacleAvoidance => {
                if input.nearest_obstacle_distance > self.config.obstacle_stop_distance * 1.5 {
                    self.state = DrivingState::Following;
                }
            }
        }

        // Compute desired speed
        let desired_speed = match self.state {
            DrivingState::Following => self.config.default_speed,
            DrivingState::Approaching => {
                if let Some(goal) = &self.goal {
                    let dist = distance(input.robot_x, input.robot_y, goal.x, goal.y);
                    let ratio = (dist / self.config.approach_distance).clamp(0.1, 1.0);
                    self.config.default_speed * ratio
                } else {
                    0.0
                }
            }
            DrivingState::ObstacleAvoidance => self.config.default_speed * 0.3,
            _ => 0.0,
        };

        let replan_requested = matches!(
            self.state,
            DrivingState::ObstacleAvoidance | DrivingState::Following
        );

        BehaviorOutput {
            state: self.state,
            desired_speed,
            replan_requested,
        }
    }
}

fn distance(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_input() -> BehaviorInput {
        BehaviorInput {
            robot_x: 0.0, robot_y: 0.0, robot_theta: 0.0,
            localization_confidence: 0.9,
            nearest_obstacle_distance: 5.0,
            emergency_stop: false,
        }
    }

    #[test]
    fn test_idle_to_following() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        assert_eq!(bp.state(), DrivingState::Idle);

        bp.set_goal(Goal { x: 5.0, y: 0.0, theta: 0.0 });
        let out = bp.update(&default_input());
        assert_eq!(out.state, DrivingState::Following);
        assert!(out.desired_speed > 0.0);
    }

    #[test]
    fn test_goal_reached() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal { x: 0.05, y: 0.05, theta: 0.0 });

        let out = bp.update(&default_input());
        assert_eq!(out.state, DrivingState::GoalReached);
        assert_eq!(out.desired_speed, 0.0);
    }

    #[test]
    fn test_emergency_stop_override() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal { x: 5.0, y: 0.0, theta: 0.0 });

        let mut input = default_input();
        input.emergency_stop = true;
        let out = bp.update(&input);
        assert_eq!(out.state, DrivingState::EmergencyStop);
        assert_eq!(out.desired_speed, 0.0);
    }

    #[test]
    fn test_obstacle_avoidance() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal { x: 5.0, y: 0.0, theta: 0.0 });
        bp.update(&default_input()); // transition to Following

        let mut input = default_input();
        input.nearest_obstacle_distance = 0.2; // below threshold
        let out = bp.update(&input);
        assert_eq!(out.state, DrivingState::ObstacleAvoidance);
        assert!(out.desired_speed < 0.5); // reduced speed
    }
}
