/// Behavior planner: high-level driving state machine at 5Hz.
///
/// Decides the current driving mode based on perception data and goals.
/// Feeds mode decisions to the global/local planners and arbitrator.
use serde::Deserialize;
use tracing::{error, info, warn};

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
    /// Stuck without a feasible plan: phased self-recovery (relaxed-margin
    /// forward retry, then slow straight reverse while the rear is clear).
    Recovery,
    /// Lost localization or perception failure, controlled stop.
    Degraded,
    /// Emergency stop active.
    EmergencyStop,
}

/// Active sub-phase while in `DrivingState::Recovery`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryPhase {
    /// Phase A: retry forward planning with relaxed DWA margin at a crawl.
    ForwardRetry,
    /// Phase B: steered slow reverse (only while the rear corridor is clear).
    Reverse,
    /// All attempts exhausted: stay stopped, retrying one phase-A round every
    /// `hold_retry_period_s` (never a permanently terminal state).
    Hold,
}

/// Robot speeds below this (m/s) count as "effectively stationary" for stuck
/// detection.
const STATIONARY_SPEED: f64 = 0.05;
/// Recovery crawl speed (m/s): phase A forward retry and phase B reverse both
/// run at this speed (the main loop scripts the sign). Mirrors main.rs's
/// RECOVERY_REVERSE_SPEED.
const RECOVERY_CRAWL_SPEED: f64 = 0.1;
/// Slack (s) added to the distance-derived reverse time allowance, covering
/// spin-up and tracking lag before the commanded crawl produces displacement.
const REVERSE_TIME_SLACK_S: f64 = 1.0;
/// Duration of recovery phase A (relaxed forward retry) before trying reverse.
const RECOVERY_FORWARD_RETRY_S: f64 = 3.0;
/// Movement since recovery entry that counts as progress and exits Recovery.
const RECOVERY_EXIT_DISTANCE_M: f64 = 0.3;
/// Continuous time in Following after which a recovery episode is considered
/// genuinely over (attempt counter cleared).
const EPISODE_FOLLOWING_CLEAR_S: f64 = 5.0;

/// Book-keeping for an active recovery (Some iff state == Recovery).
#[derive(Debug, Clone)]
struct RecoveryStatus {
    phase: RecoveryPhase,
    phase_elapsed: f64,
    /// Pose at THIS Recovery activation (immediate 0.3m movement exit).
    entry_x: f64,
    entry_y: f64,
    /// Pose at the start of the current reverse burst; Some iff phase is
    /// Reverse. Actual displacement from here (not commanded time) gates the
    /// committed-reverse window.
    reverse_start: Option<(f64, f64)>,
    /// Consecutive feasible forward cycles (sticky exit).
    feasible_streak: u32,
    /// Time spent holding since entering Hold / since the last retry round.
    hold_elapsed: f64,
    /// The current ForwardRetry round was launched from Hold (periodic retry):
    /// on exhaustion it returns to Hold instead of Reverse.
    hold_retry: bool,
    /// The exhaustion ERROR was already logged once for this recovery.
    abort_logged: bool,
}

impl RecoveryStatus {
    fn new(x: f64, y: f64) -> Self {
        Self {
            phase: RecoveryPhase::ForwardRetry,
            phase_elapsed: 0.0,
            entry_x: x,
            entry_y: y,
            reverse_start: None,
            feasible_streak: 0,
            hold_elapsed: 0.0,
            hold_retry: false,
            abort_logged: false,
        }
    }
}

/// A recovery EPISODE spans multiple Recovery activations: it starts at the
/// first Recovery entry and only ends on real net progress
/// (`recovery_progress_reset_m` of displacement from the episode origin) or
/// on a stable return to Following (`EPISODE_FOLLOWING_CLEAR_S`). The attempt
/// counter lives here so that a marginal one-cycle exit followed by an
/// immediate re-stick cannot reset it — the live oscillation that burned
/// through attempt counters without the robot moving an inch.
#[derive(Debug, Clone)]
struct RecoveryEpisode {
    origin_x: f64,
    origin_y: f64,
    /// Completed A+B rounds without progress across the whole episode.
    attempts: u32,
    /// Continuous time spent in Following since the last Recovery exit.
    following_time: f64,
}

impl RecoveryEpisode {
    fn new(x: f64, y: f64) -> Self {
        Self {
            origin_x: x,
            origin_y: y,
            attempts: 0,
            following_time: 0.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct BehaviorConfig {
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
    #[serde(default = "default_goal_tolerance")]
    pub goal_tolerance: f64, // meters
    #[serde(default = "default_approach_distance")]
    pub approach_distance: f64, // meters, start decelerating
    #[serde(default = "default_obstacle_distance")]
    pub obstacle_stop_distance: f64, // meters, stop if obstacle closer
    #[serde(default = "default_speed")]
    pub default_speed: f64, // m/s
    /// Speed factor applied in ObstacleAvoidance, relative to the per-leg
    /// speed. The historical hardwired 0.3 was sized for a 0.5 m/s default —
    /// at a 2.2 m/s default it dragged the whole slalom to 0.66 m/s. The
    /// speed-scaled margins, high-speed margin growth, and braking-aware
    /// feasibility in DWA now carry the safety burden near obstacles; this
    /// factor is a moderation, not the primary protection.
    #[serde(default = "default_obstacle_avoidance_speed_factor")]
    pub obstacle_avoidance_speed_factor: f64,
    /// Consecutive update cycles stationary-and-infeasible (in Following /
    /// ObstacleAvoidance) before entering Recovery. ~2s at the 10Hz loop.
    #[serde(default = "default_stuck_cycles")]
    pub stuck_cycles_before_recovery: u32,
    /// Maximum duration (s) of one recovery reverse burst (phase B).
    #[serde(default = "default_recovery_reverse_max_s")]
    pub recovery_reverse_max_s: f64,
    /// Full A+B recovery rounds without progress before aborting (hold
    /// stopped, ERROR log — operator attention).
    #[serde(default = "default_recovery_max_attempts")]
    pub recovery_max_attempts: u32,
    /// Committed reverse: once phase B starts, forward feasibility is not
    /// re-checked until the robot has actually reversed this many meters
    /// (tracked from pose displacement, not commanded time) or the rear
    /// becomes blocked mid-maneuver. This is the ROUND-0 base: attempt round
    /// n within an episode escalates the gate to min((n+1)·this,
    /// `recovery_reverse_max_m`) — identical short retreats just re-run the
    /// same failed experiment, while escalation shows the 1Hz global replan
    /// genuinely new geometry.
    #[serde(default = "default_recovery_reverse_min_m")]
    pub recovery_reverse_min_m: f64,
    /// Cap (m) on the escalating per-round committed reverse distance.
    #[serde(default = "default_recovery_reverse_max_m")]
    pub recovery_reverse_max_m: f64,
    /// Sticky exit: consecutive feasible forward cycles (confidence above the
    /// arbitrator threshold AND positive linear speed) required to leave
    /// Recovery for Following. The moved->0.3m exit stays immediate.
    #[serde(default = "default_recovery_exit_feasible_cycles")]
    pub recovery_exit_feasible_cycles: u32,
    /// Net displacement from the episode origin (first Recovery entry) that
    /// counts as real progress and clears the episode attempt counter.
    #[serde(default = "default_recovery_progress_reset_m")]
    pub recovery_progress_reset_m: f64,
    /// Hold is not terminal: after max attempts it retries one phase-A round
    /// every this many seconds.
    #[serde(default = "default_hold_retry_period_s")]
    pub hold_retry_period_s: f64,
}

fn default_rate_hz() -> u32 {
    5
}
fn default_goal_tolerance() -> f64 {
    0.15
}
fn default_approach_distance() -> f64 {
    0.5
}
fn default_obstacle_distance() -> f64 {
    0.3
}
fn default_speed() -> f64 {
    // Gauntlet cruise target; matches dwa.max_speed and the arbitrator
    // envelope max_speed so behavior can actually request full cruise.
    2.2
}
fn default_obstacle_avoidance_speed_factor() -> f64 {
    0.65
}
fn default_stuck_cycles() -> u32 {
    20
}
fn default_recovery_reverse_max_s() -> f64 {
    3.0
}
fn default_recovery_max_attempts() -> u32 {
    3
}
fn default_recovery_reverse_min_m() -> f64 {
    0.15
}
fn default_recovery_reverse_max_m() -> f64 {
    0.5
}
fn default_recovery_exit_feasible_cycles() -> u32 {
    3
}
fn default_recovery_progress_reset_m() -> f64 {
    0.5
}
fn default_hold_retry_period_s() -> f64 {
    10.0
}

impl Default for BehaviorConfig {
    fn default() -> Self {
        Self {
            rate_hz: default_rate_hz(),
            goal_tolerance: default_goal_tolerance(),
            approach_distance: default_approach_distance(),
            obstacle_stop_distance: default_obstacle_distance(),
            default_speed: default_speed(),
            obstacle_avoidance_speed_factor: default_obstacle_avoidance_speed_factor(),
            stuck_cycles_before_recovery: default_stuck_cycles(),
            recovery_reverse_max_s: default_recovery_reverse_max_s(),
            recovery_max_attempts: default_recovery_max_attempts(),
            recovery_reverse_min_m: default_recovery_reverse_min_m(),
            recovery_reverse_max_m: default_recovery_reverse_max_m(),
            recovery_exit_feasible_cycles: default_recovery_exit_feasible_cycles(),
            recovery_progress_reset_m: default_recovery_progress_reset_m(),
            hold_retry_period_s: default_hold_retry_period_s(),
        }
    }
}

impl BehaviorConfig {
    /// Fail loudly on YAML values that would break the recovery state machine
    /// (zero cycles would enter Recovery on the first stationary cycle; zero
    /// attempts would abort instantly; a non-positive reverse window would
    /// skip phase B entirely).
    pub fn validate(&self) -> Result<(), String> {
        if self.stuck_cycles_before_recovery == 0 {
            return Err("behavior.stuck_cycles_before_recovery must be >= 1".into());
        }
        if !(self.obstacle_avoidance_speed_factor > 0.0
            && self.obstacle_avoidance_speed_factor <= 1.0)
        {
            return Err(format!(
                "behavior.obstacle_avoidance_speed_factor must be in (0, 1], got {}",
                self.obstacle_avoidance_speed_factor
            ));
        }
        if !(self.recovery_reverse_max_s > 0.0 && self.recovery_reverse_max_s.is_finite()) {
            return Err(format!(
                "behavior.recovery_reverse_max_s must be > 0, got {}",
                self.recovery_reverse_max_s
            ));
        }
        if self.recovery_max_attempts == 0 {
            return Err("behavior.recovery_max_attempts must be >= 1".into());
        }
        if !(self.recovery_reverse_min_m > 0.0 && self.recovery_reverse_min_m.is_finite()) {
            return Err(format!(
                "behavior.recovery_reverse_min_m must be > 0, got {}",
                self.recovery_reverse_min_m
            ));
        }
        if !(self.recovery_reverse_max_m >= self.recovery_reverse_min_m
            && self.recovery_reverse_max_m.is_finite())
        {
            return Err(format!(
                "behavior.recovery_reverse_max_m must be finite and >= recovery_reverse_min_m \
                 ({}), got {}",
                self.recovery_reverse_min_m, self.recovery_reverse_max_m
            ));
        }
        if self.recovery_exit_feasible_cycles == 0 {
            return Err("behavior.recovery_exit_feasible_cycles must be >= 1".into());
        }
        if !(self.recovery_progress_reset_m > 0.0 && self.recovery_progress_reset_m.is_finite()) {
            return Err(format!(
                "behavior.recovery_progress_reset_m must be > 0, got {}",
                self.recovery_progress_reset_m
            ));
        }
        if !(self.hold_retry_period_s > 0.0 && self.hold_retry_period_s.is_finite()) {
            return Err(format!(
                "behavior.hold_retry_period_s must be > 0, got {}",
                self.hold_retry_period_s
            ));
        }
        Ok(())
    }
}

/// Goal for the behavior planner.
#[derive(Debug, Clone)]
pub struct Goal {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    /// Per-goal arrival tolerance in meters (e.g. from a scenario waypoint).
    /// Falls back to `BehaviorConfig::goal_tolerance` when None.
    pub tolerance: Option<f64>,
    /// Per-leg desired speed (m/s) from the scenario waypoint
    /// (`NavigationGoal.desired_speed`). CAPS the leg's driving speed: the
    /// effective base speed is `min(default_speed, speed)`. None (or a
    /// non-positive wire value, mapped to None by the caller) means the
    /// configured `default_speed` governs the leg.
    pub speed: Option<f64>,
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
    /// Seconds since the previous `update` call (main-loop cycle period).
    pub dt: f64,
    /// Measured linear speed (m/s), for stuck detection.
    pub robot_speed: f64,
    /// Previous cycle's local plan was infeasible: (near-)zero command with
    /// low confidence (DWA found no collision-free trajectory).
    pub planner_infeasible: bool,
    /// Previous cycle's local plan was a feasible forward plan: non-zero
    /// command with confidence above the arbitrator threshold.
    pub planner_feasible: bool,
    /// Rear corridor is clear for a slow straight reverse (recovery phase B).
    pub rear_clear: bool,
}

/// Output from the behavior planner.
#[derive(Debug, Clone)]
pub struct BehaviorOutput {
    pub state: DrivingState,
    pub desired_speed: f64, // m/s
    pub replan_requested: bool,
    /// Active recovery sub-phase; Some iff `state == DrivingState::Recovery`.
    pub recovery_phase: Option<RecoveryPhase>,
}

pub struct BehaviorPlanner {
    config: BehaviorConfig,
    state: DrivingState,
    goal: Option<Goal>,
    /// Consecutive stationary-and-infeasible cycles (stuck detector).
    stuck_count: u32,
    /// Book-keeping for an active recovery; Some iff state == Recovery.
    recovery: Option<RecoveryStatus>,
    /// Recovery episode spanning Recovery activations; survives exits until
    /// real progress or a stable return to Following.
    episode: Option<RecoveryEpisode>,
}

impl BehaviorPlanner {
    pub fn new(config: BehaviorConfig) -> Self {
        Self {
            config,
            state: DrivingState::Idle,
            goal: None,
            stuck_count: 0,
            recovery: None,
            episode: None,
        }
    }

    pub fn set_goal(&mut self, goal: Goal) {
        info!(
            "Behavior: new goal ({:.2}, {:.2}, {:.1}°)",
            goal.x,
            goal.y,
            goal.theta.to_degrees()
        );
        self.goal = Some(goal);
        self.state = DrivingState::Following;
    }

    /// Arrival tolerance for a goal: per-goal value if set (and positive),
    /// otherwise the configured default.
    fn arrival_tolerance(&self, goal: &Goal) -> f64 {
        goal.tolerance
            .filter(|t| *t > 0.0)
            .unwrap_or(self.config.goal_tolerance)
    }

    /// Base driving speed for the current leg: the configured default, capped
    /// by the active goal's per-leg speed when set. All state-dependent speed
    /// shaping (approach ramp, obstacle-avoidance factor) applies on top.
    fn leg_speed(&self) -> f64 {
        let base = self.config.default_speed;
        self.goal
            .as_ref()
            .and_then(|g| g.speed.filter(|s| *s > 0.0))
            .map_or(base, |s| s.min(base))
    }

    pub fn clear_goal(&mut self) {
        self.goal = None;
        self.state = DrivingState::Idle;
        self.stuck_count = 0;
        self.recovery = None;
        self.episode = None;
    }

    #[allow(dead_code)] // Inspection accessor for tests / future status publishing.
    pub fn state(&self) -> DrivingState {
        self.state
    }

    /// Committed reverse distance (m) for the CURRENT attempt round: round n
    /// within an episode (n = completed A+B rounds so far) escalates from the
    /// configured base as min((n+1)·recovery_reverse_min_m,
    /// recovery_reverse_max_m). Identical 0.15m retreats re-run the same
    /// failed experiment; escalation lets the 1Hz global replan see genuinely
    /// new geometry. The main loop uses this both for the swept rear-corridor
    /// check and as the scripted reverse arc length.
    pub fn reverse_target_m(&self) -> f64 {
        let n = self.episode.as_ref().map(|e| e.attempts).unwrap_or(0);
        (self.config.recovery_reverse_min_m * (n + 1) as f64)
            .min(self.config.recovery_reverse_max_m)
    }

    /// Time allowance (s) for the current round's reverse burst: the time the
    /// crawl needs to cover `reverse_target_m` plus slack, never below the
    /// configured per-burst duration floor (`recovery_reverse_max_s`, which
    /// preserves the round-0 behavior the gauntlet was tuned on).
    fn reverse_time_allowance_s(&self) -> f64 {
        (self.reverse_target_m() / RECOVERY_CRAWL_SPEED + REVERSE_TIME_SLACK_S)
            .max(self.config.recovery_reverse_max_s)
    }

    /// Update behavior state based on current perception input.
    pub fn update(&mut self, input: &BehaviorInput) -> BehaviorOutput {
        // Emergency stop overrides everything
        if input.emergency_stop {
            self.state = DrivingState::EmergencyStop;
            self.stuck_count = 0;
            self.recovery = None;
            return BehaviorOutput {
                state: self.state,
                desired_speed: 0.0,
                replan_requested: false,
                recovery_phase: None,
            };
        }

        // Degraded if localization is poor
        if input.localization_confidence < 0.3 {
            self.state = DrivingState::Degraded;
            self.stuck_count = 0;
            self.recovery = None;
            return BehaviorOutput {
                state: self.state,
                desired_speed: 0.0,
                replan_requested: false,
                recovery_phase: None,
            };
        }

        // Stuck detection: stationary with an infeasible local plan while
        // actively trying to drive. Only Following/ObstacleAvoidance can enter
        // Recovery — never GoalReached/EmergencyStop (there is nothing to
        // recover toward) and never Idle (a parked robot is not stuck).
        if matches!(
            self.state,
            DrivingState::Following | DrivingState::ObstacleAvoidance
        ) && input.robot_speed.abs() < STATIONARY_SPEED
            && input.planner_infeasible
        {
            self.stuck_count += 1;
            if self.stuck_count >= self.config.stuck_cycles_before_recovery {
                // Episode: created on the FIRST entry, reused (attempts and
                // origin preserved) when re-sticking before real progress.
                let ep = self
                    .episode
                    .get_or_insert_with(|| RecoveryEpisode::new(input.robot_x, input.robot_y));
                ep.following_time = 0.0;
                warn!(
                    "Behavior: stuck for {} cycles without a feasible plan — entering Recovery \
                     (episode attempts so far: {})",
                    self.stuck_count, ep.attempts
                );
                self.state = DrivingState::Recovery;
                self.recovery = Some(RecoveryStatus::new(input.robot_x, input.robot_y));
                self.stuck_count = 0;
            }
        } else if self.state != DrivingState::Recovery {
            self.stuck_count = 0;
        }

        // Episode bookkeeping (progress-gated attempt reset): the episode —
        // and with it the attempt counter — only clears on real net
        // displacement from the episode origin or on a stable stretch of
        // Following. Exiting Recovery alone must NOT reset attempts.
        if let Some(ep) = self.episode.as_mut() {
            let net = distance(input.robot_x, input.robot_y, ep.origin_x, ep.origin_y);
            if net > self.config.recovery_progress_reset_m {
                info!(
                    "Behavior: recovery episode cleared — net progress {:.2}m from episode origin",
                    net
                );
                self.episode = None;
            } else if self.state == DrivingState::Following {
                ep.following_time += input.dt;
                if ep.following_time >= EPISODE_FOLLOWING_CLEAR_S {
                    info!(
                        "Behavior: recovery episode cleared — stable Following for {:.0}s",
                        EPISODE_FOLLOWING_CLEAR_S
                    );
                    self.episode = None;
                }
            } else {
                ep.following_time = 0.0;
            }
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

                    if dist < self.arrival_tolerance(goal) {
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
                    if dist < self.arrival_tolerance(goal) {
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
            DrivingState::Recovery => {
                self.update_recovery(input);
            }
        }

        let recovery_phase = if self.state == DrivingState::Recovery {
            self.recovery.as_ref().map(|r| r.phase)
        } else {
            None
        };

        // Compute desired speed. The per-leg base (`leg_speed`) is the
        // configured default capped by the active waypoint's desired_speed.
        let desired_speed = match self.state {
            DrivingState::Following => self.leg_speed(),
            DrivingState::Approaching => {
                if let Some(goal) = &self.goal {
                    let dist = distance(input.robot_x, input.robot_y, goal.x, goal.y);
                    let ratio = (dist / self.config.approach_distance).clamp(0.1, 1.0);
                    self.leg_speed() * ratio
                } else {
                    0.0
                }
            }
            DrivingState::ObstacleAvoidance => {
                self.leg_speed() * self.config.obstacle_avoidance_speed_factor
            }
            // Recovery crawls: phase A retries forward at the crawl, phase B
            // reverses at the crawl (the main loop scripts the sign), Hold
            // stops.
            DrivingState::Recovery => match recovery_phase {
                Some(RecoveryPhase::ForwardRetry) | Some(RecoveryPhase::Reverse) => {
                    RECOVERY_CRAWL_SPEED
                }
                _ => 0.0,
            },
            _ => 0.0,
        };

        let replan_requested = matches!(
            self.state,
            DrivingState::ObstacleAvoidance | DrivingState::Following | DrivingState::Recovery
        );

        BehaviorOutput {
            state: self.state,
            desired_speed,
            replan_requested,
            recovery_phase,
        }
    }

    /// Leave Recovery for Following (or Idle without a goal). The episode —
    /// and the attempt counter it carries — deliberately survives the exit;
    /// only the progress/stable-Following gates in `update` clear it.
    fn exit_recovery(&mut self, reason: &str) {
        let attempts = self.episode.as_ref().map(|e| e.attempts).unwrap_or(0);
        info!(
            "Behavior: exiting Recovery ({}) after {} attempt round(s)",
            reason, attempts
        );
        self.recovery = None;
        self.stuck_count = 0;
        self.state = if self.goal.is_some() {
            DrivingState::Following
        } else {
            DrivingState::Idle
        };
    }

    /// Advance the Recovery sub-state machine by one cycle.
    ///
    /// Exits to Following on real displacement (> `RECOVERY_EXIT_DISTANCE_M`
    /// since this Recovery entry, immediate) or on
    /// `recovery_exit_feasible_cycles` CONSECUTIVE feasible forward cycles
    /// (sticky exit — one marginal relaxed-margin cycle no longer bounces the
    /// robot out only to re-stick 2s later). While a reverse burst is
    /// committed (less than `recovery_reverse_min_m` of actual displacement
    /// since the burst began and the rear still clear), forward feasibility
    /// is not consulted at all: the live failure aborted every reverse after
    /// ~7cm on a single marginal cycle.
    ///
    /// Phases alternate A (relaxed forward retry, ~3s) and B (steered slow
    /// reverse, up to `recovery_reverse_max_s`, only while the rear corridor
    /// is clear). A finished phase B closes one A+B round; the round counter
    /// lives on the EPISODE, so exits without progress cannot reset it. After
    /// `recovery_max_attempts` rounds the recovery holds stopped (ERROR —
    /// operator attention) but retries one phase-A round every
    /// `hold_retry_period_s` instead of being terminal.
    fn update_recovery(&mut self, input: &BehaviorInput) {
        // Escalating round parameters (computed before borrowing the
        // recovery book-keeping mutably).
        let reverse_min_m = self.reverse_target_m();
        let reverse_allowance_s = self.reverse_time_allowance_s();
        let Some(rec) = self.recovery.as_mut() else {
            // Defensive: Recovery state without book-keeping — re-resolve.
            self.state = if self.goal.is_some() {
                DrivingState::Following
            } else {
                DrivingState::Idle
            };
            return;
        };

        // Immediate exit on real displacement since this Recovery entry.
        if distance(input.robot_x, input.robot_y, rec.entry_x, rec.entry_y)
            > RECOVERY_EXIT_DISTANCE_M
        {
            self.exit_recovery("moved > 0.3m");
            return;
        }

        // Committed reverse: below the minimum actual reverse displacement
        // (and with the rear still open), do not even count feasible cycles.
        let committed_reverse = rec.phase == RecoveryPhase::Reverse
            && input.rear_clear
            && rec.reverse_start.is_none_or(|(sx, sy)| {
                distance(input.robot_x, input.robot_y, sx, sy) < reverse_min_m
            });

        if committed_reverse {
            rec.feasible_streak = 0;
        } else if input.planner_feasible {
            rec.feasible_streak += 1;
            if rec.feasible_streak >= self.config.recovery_exit_feasible_cycles {
                let n = rec.feasible_streak;
                self.exit_recovery(&format!("{} consecutive feasible forward cycles", n));
                return;
            }
        } else {
            rec.feasible_streak = 0;
        }

        match rec.phase {
            RecoveryPhase::ForwardRetry => {
                rec.phase_elapsed += input.dt;
                if rec.phase_elapsed >= RECOVERY_FORWARD_RETRY_S {
                    rec.phase_elapsed = 0.0;
                    if rec.hold_retry {
                        rec.hold_retry = false;
                        if input.rear_clear {
                            // Forward found nothing, but the world may have
                            // opened BEHIND since the episode aborted (live
                            // gauntlet: robot pinched between cone and wall,
                            // forward permanently infeasible, rear open —
                            // phase-A-only retries held it there forever).
                            // Spend the retry round's phase B; the Reverse arm
                            // returns to Hold afterwards via the attempts gate.
                            rec.phase = RecoveryPhase::Reverse;
                            rec.reverse_start = Some((input.robot_x, input.robot_y));
                            warn!(
                                "Behavior: Hold retry found no forward plan — trying reverse leg"
                            );
                        } else {
                            // Nothing forward, nothing behind: hold and rearm.
                            rec.phase = RecoveryPhase::Hold;
                            rec.hold_elapsed = 0.0;
                            warn!(
                                "Behavior: Hold retry round found no feasible plan (rear blocked) — holding"
                            );
                        }
                    } else {
                        rec.phase = RecoveryPhase::Reverse;
                        rec.reverse_start = Some((input.robot_x, input.robot_y));
                        info!("Behavior: recovery phase A exhausted, trying steered slow reverse");
                    }
                }
            }
            RecoveryPhase::Reverse => {
                let round_over = if !input.rear_clear {
                    info!("Behavior: rear corridor blocked, ending reverse and retrying forward");
                    true
                } else {
                    rec.phase_elapsed += input.dt;
                    // Escalating allowance: time to cover this round's
                    // committed distance at the crawl, plus slack.
                    rec.phase_elapsed >= reverse_allowance_s
                };
                if round_over {
                    rec.phase_elapsed = 0.0;
                    rec.reverse_start = None;
                    let ep = self
                        .episode
                        .get_or_insert_with(|| RecoveryEpisode::new(input.robot_x, input.robot_y));
                    ep.attempts += 1;
                    if ep.attempts >= self.config.recovery_max_attempts {
                        rec.phase = RecoveryPhase::Hold;
                        rec.hold_elapsed = 0.0;
                        if !rec.abort_logged {
                            rec.abort_logged = true;
                            error!(
                                "Behavior: recovery FAILED after {} A+B rounds without progress — \
                                 holding stopped (phase-A retry every {:.0}s), operator attention \
                                 required",
                                ep.attempts, self.config.hold_retry_period_s
                            );
                        }
                    } else {
                        rec.phase = RecoveryPhase::ForwardRetry;
                    }
                }
            }
            RecoveryPhase::Hold => {
                rec.hold_elapsed += input.dt;
                if rec.hold_elapsed >= self.config.hold_retry_period_s {
                    rec.hold_elapsed = 0.0;
                    rec.phase = RecoveryPhase::ForwardRetry;
                    rec.phase_elapsed = 0.0;
                    rec.hold_retry = true;
                    warn!("Behavior: Hold — periodic phase-A retry");
                }
            }
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
            robot_x: 0.0,
            robot_y: 0.0,
            robot_theta: 0.0,
            localization_confidence: 0.9,
            nearest_obstacle_distance: 5.0,
            emergency_stop: false,
            dt: 0.1,
            robot_speed: 0.0,
            planner_infeasible: false,
            planner_feasible: true,
            rear_clear: true,
        }
    }

    /// Input describing a stuck robot: stationary, planner infeasible.
    fn stuck_input() -> BehaviorInput {
        BehaviorInput {
            robot_speed: 0.0,
            planner_infeasible: true,
            planner_feasible: false,
            ..default_input()
        }
    }

    /// Drive the planner into Recovery via the stuck counter.
    fn enter_recovery(bp: &mut BehaviorPlanner) {
        bp.set_goal(Goal {
            x: 5.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });
        bp.update(&default_input()); // Following
        for _ in 0..BehaviorConfig::default().stuck_cycles_before_recovery {
            bp.update(&stuck_input());
        }
        assert_eq!(bp.state(), DrivingState::Recovery);
    }

    #[test]
    fn test_idle_to_following() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        assert_eq!(bp.state(), DrivingState::Idle);

        bp.set_goal(Goal {
            x: 5.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });
        let out = bp.update(&default_input());
        assert_eq!(out.state, DrivingState::Following);
        assert!(out.desired_speed > 0.0);
    }

    #[test]
    fn test_goal_reached() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal {
            x: 0.05,
            y: 0.05,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });

        let out = bp.update(&default_input());
        assert_eq!(out.state, DrivingState::GoalReached);
        assert_eq!(out.desired_speed, 0.0);
    }

    #[test]
    fn test_emergency_stop_override() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal {
            x: 5.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });

        let mut input = default_input();
        input.emergency_stop = true;
        let out = bp.update(&input);
        assert_eq!(out.state, DrivingState::EmergencyStop);
        assert_eq!(out.desired_speed, 0.0);
    }

    #[test]
    fn test_obstacle_avoidance() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal {
            x: 5.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });
        bp.update(&default_input()); // transition to Following

        let mut input = default_input();
        input.nearest_obstacle_distance = 0.2; // below threshold
        let out = bp.update(&input);
        assert_eq!(out.state, DrivingState::ObstacleAvoidance);
        // Reduced relative to cruise by the configured factor — no longer a
        // blanket crawl (the DWA margins + braking check carry the safety).
        let cfg = BehaviorConfig::default();
        assert!(out.desired_speed < cfg.default_speed);
        assert!(
            (out.desired_speed - cfg.default_speed * cfg.obstacle_avoidance_speed_factor).abs()
                < 1e-9
        );
    }

    #[test]
    fn test_obstacle_avoidance_speed_factor_is_configurable() {
        let cfg = BehaviorConfig {
            obstacle_avoidance_speed_factor: 0.5,
            ..Default::default()
        };
        assert!(cfg.validate().is_ok());
        let mut bp = BehaviorPlanner::new(cfg.clone());
        bp.set_goal(Goal {
            x: 5.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });
        bp.update(&default_input()); // Following
        let mut input = default_input();
        input.nearest_obstacle_distance = 0.2;
        let out = bp.update(&input);
        assert_eq!(out.state, DrivingState::ObstacleAvoidance);
        assert!((out.desired_speed - cfg.default_speed * 0.5).abs() < 1e-9);

        // Nonsense factors fail loudly at startup.
        assert!(BehaviorConfig {
            obstacle_avoidance_speed_factor: 0.0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(BehaviorConfig {
            obstacle_avoidance_speed_factor: 1.5,
            ..Default::default()
        }
        .validate()
        .is_err());
    }

    #[test]
    fn test_per_leg_speed_caps_desired_speed() {
        let cfg = BehaviorConfig::default();
        // A waypoint's desired_speed below the default caps the whole leg…
        let mut bp = BehaviorPlanner::new(cfg.clone());
        bp.set_goal(Goal {
            x: 5.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: Some(1.0),
        });
        let out = bp.update(&default_input());
        assert_eq!(out.state, DrivingState::Following);
        assert!((out.desired_speed - 1.0).abs() < 1e-9);

        // …including the ObstacleAvoidance factor, which applies on top.
        let mut input = default_input();
        input.nearest_obstacle_distance = 0.2;
        let out = bp.update(&input);
        assert_eq!(out.state, DrivingState::ObstacleAvoidance);
        assert!((out.desired_speed - 1.0 * cfg.obstacle_avoidance_speed_factor).abs() < 1e-9);

        // A waypoint speed ABOVE the default never raises the leg beyond it.
        let mut bp = BehaviorPlanner::new(cfg.clone());
        bp.set_goal(Goal {
            x: 5.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: Some(9.9),
        });
        let out = bp.update(&default_input());
        assert!((out.desired_speed - cfg.default_speed).abs() < 1e-9);

        // Unset (None) or nonsensical (<= 0) per-leg speeds fall back to the
        // configured default.
        let mut bp = BehaviorPlanner::new(cfg.clone());
        bp.set_goal(Goal {
            x: 5.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: Some(0.0),
        });
        let out = bp.update(&default_input());
        assert!((out.desired_speed - cfg.default_speed).abs() < 1e-9);
    }

    #[test]
    fn test_per_goal_tolerance_overrides_config() {
        // Goal at 0.2m: outside the config default (0.15) but inside the
        // per-goal tolerance (0.3) -> must count as reached.
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal {
            x: 0.2,
            y: 0.0,
            theta: 0.0,
            tolerance: Some(0.3),
            speed: None,
        });
        let out = bp.update(&default_input());
        assert_eq!(out.state, DrivingState::GoalReached);
    }

    #[test]
    fn test_config_tolerance_when_goal_has_none() {
        // Same 0.2m distance without a per-goal tolerance -> not reached
        // (falls back to config 0.15), robot keeps approaching.
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal {
            x: 0.2,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });
        let out = bp.update(&default_input());
        assert_eq!(out.state, DrivingState::Approaching);
    }

    #[test]
    fn test_recovery_entry_requires_consecutive_stuck_cycles() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal {
            x: 5.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });
        bp.update(&default_input());
        assert_eq!(bp.state(), DrivingState::Following);

        let n = BehaviorConfig::default().stuck_cycles_before_recovery;

        // N-1 stuck cycles: not yet.
        for _ in 0..n - 1 {
            let out = bp.update(&stuck_input());
            assert_ne!(out.state, DrivingState::Recovery);
        }
        // A feasible cycle resets the counter...
        bp.update(&default_input());
        for _ in 0..n - 1 {
            let out = bp.update(&stuck_input());
            assert_ne!(
                out.state,
                DrivingState::Recovery,
                "stuck counter must reset on a feasible cycle"
            );
        }
        // ...and N consecutive stuck cycles enter Recovery, phase A first.
        let out = bp.update(&stuck_input());
        assert_eq!(out.state, DrivingState::Recovery);
        assert_eq!(out.recovery_phase, Some(RecoveryPhase::ForwardRetry));
        assert!((out.desired_speed - 0.1).abs() < 1e-9, "phase A crawls");
    }

    #[test]
    fn test_recovery_never_entered_from_goal_reached_or_idle() {
        // GoalReached: stationary with an infeasible planner must NOT recover.
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal {
            x: 0.05,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });
        bp.update(&default_input());
        assert_eq!(bp.state(), DrivingState::GoalReached);
        for _ in 0..3 * BehaviorConfig::default().stuck_cycles_before_recovery {
            let out = bp.update(&stuck_input());
            assert_eq!(out.state, DrivingState::GoalReached);
        }

        // Idle (no goal): same.
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        for _ in 0..3 * BehaviorConfig::default().stuck_cycles_before_recovery {
            let out = bp.update(&stuck_input());
            assert_eq!(out.state, DrivingState::Idle);
        }
    }

    #[test]
    fn test_recovery_exit_requires_consecutive_feasible_cycles() {
        // Sticky exit (recovery v2): a SINGLE marginal feasible cycle must
        // not exit Recovery — that instant-exit → re-stick → attempt-reset
        // loop is exactly the live oscillation. Only
        // `recovery_exit_feasible_cycles` CONSECUTIVE feasible cycles exit.
        let n = BehaviorConfig::default().recovery_exit_feasible_cycles;
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        enter_recovery(&mut bp);

        // n-1 feasible cycles: still in Recovery.
        for i in 0..n - 1 {
            let out = bp.update(&default_input());
            assert_eq!(
                out.state,
                DrivingState::Recovery,
                "exited after only {} feasible cycle(s)",
                i + 1
            );
        }
        // An infeasible cycle resets the streak...
        bp.update(&stuck_input());
        for _ in 0..n - 1 {
            let out = bp.update(&default_input());
            assert_eq!(
                out.state,
                DrivingState::Recovery,
                "feasible streak must reset on an infeasible cycle"
            );
        }
        // ...and n consecutive feasible cycles finally exit.
        let out = bp.update(&default_input());
        assert_eq!(out.state, DrivingState::Following);
        assert_eq!(out.recovery_phase, None);
    }

    #[test]
    fn test_recovery_exits_on_movement() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        enter_recovery(&mut bp);

        // Still infeasible, but the robot has moved > 0.3m since entry.
        let moved = BehaviorInput {
            robot_x: 0.35,
            ..stuck_input()
        };
        let out = bp.update(&moved);
        assert_eq!(out.state, DrivingState::Following);
    }

    #[test]
    fn test_recovery_phase_a_then_reverse_when_rear_clear() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        enter_recovery(&mut bp);

        // Phase A lasts ~3s (30 cycles at dt=0.1).
        let mut out = bp.update(&stuck_input());
        assert_eq!(out.recovery_phase, Some(RecoveryPhase::ForwardRetry));
        for _ in 0..30 {
            out = bp.update(&stuck_input());
        }
        assert_eq!(
            out.recovery_phase,
            Some(RecoveryPhase::Reverse),
            "phase A must hand over to reverse after ~3s without progress"
        );
        assert!((out.desired_speed - 0.1).abs() < 1e-9);

        // Reverse times out after recovery_reverse_max_s and returns to A.
        for _ in 0..31 {
            out = bp.update(&stuck_input());
        }
        assert_eq!(out.recovery_phase, Some(RecoveryPhase::ForwardRetry));
    }

    #[test]
    fn test_recovery_reverse_blocked_rear_returns_to_forward_retry() {
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        enter_recovery(&mut bp);

        // Run phase A out.
        let mut out = bp.update(&stuck_input());
        for _ in 0..30 {
            out = bp.update(&stuck_input());
        }
        assert_eq!(out.recovery_phase, Some(RecoveryPhase::Reverse));

        // Rear blocked: never reverse; alternate back to phase A.
        let blocked = BehaviorInput {
            rear_clear: false,
            ..stuck_input()
        };
        let out = bp.update(&blocked);
        assert_eq!(
            out.recovery_phase,
            Some(RecoveryPhase::ForwardRetry),
            "blocked rear must fall back to forward retries, not reverse"
        );
        assert_ne!(out.recovery_phase, Some(RecoveryPhase::Reverse));
    }

    #[test]
    fn test_recovery_aborts_after_max_attempts_and_holds() {
        let cfg = BehaviorConfig::default();
        let mut bp = BehaviorPlanner::new(cfg.clone());
        enter_recovery(&mut bp);

        // Run full A+B rounds (durations escalate with the reverse allowance)
        // until Hold; it must take exactly recovery_max_attempts rounds.
        let mut out = bp.update(&stuck_input());
        let mut rounds = 0;
        while out.recovery_phase != Some(RecoveryPhase::Hold) {
            out = run_one_round(&mut bp, &stuck_input());
            rounds += 1;
            assert!(
                rounds <= cfg.recovery_max_attempts,
                "Hold not reached after {} rounds",
                rounds
            );
        }
        assert_eq!(rounds, cfg.recovery_max_attempts);
        assert_eq!(out.state, DrivingState::Recovery);
        assert_eq!(
            out.recovery_phase,
            Some(RecoveryPhase::Hold),
            "exhausted recovery must hold stopped"
        );
        assert_eq!(out.desired_speed, 0.0);

        // Still exits if the world opens up later (sticky exit: needs the
        // configured consecutive feasible cycles).
        let mut out = bp.update(&default_input());
        for _ in 0..cfg.recovery_exit_feasible_cycles {
            out = bp.update(&default_input());
        }
        assert_eq!(out.state, DrivingState::Following);
    }

    /// Drive an in-Recovery planner with stuck cycles until phase B starts,
    /// then until the round closes (reverse timeout with no movement).
    /// Returns the output of the cycle on which the reverse ended.
    fn run_one_round(bp: &mut BehaviorPlanner, template: &BehaviorInput) -> BehaviorOutput {
        let mut out = bp.update(template);
        let mut guard = 0;
        while out.recovery_phase != Some(RecoveryPhase::Reverse) {
            out = bp.update(template);
            guard += 1;
            assert!(guard < 100, "never reached Reverse phase");
        }
        while out.recovery_phase == Some(RecoveryPhase::Reverse) {
            out = bp.update(template);
            guard += 1;
            assert!(guard < 200, "reverse phase never ended");
        }
        out
    }

    #[test]
    fn test_committed_reverse_ignores_feasibility_until_min_distance() {
        // Recovery v2 item 1: once Reverse starts, marginal feasible cycles
        // must NOT abort it (live: every reverse died after ~7cm / <1s to a
        // single-cycle feasible exit). Only after the robot has ACTUALLY
        // reversed recovery_reverse_min_m does the sticky feasible exit apply.
        let cfg = BehaviorConfig::default();
        let mut bp = BehaviorPlanner::new(cfg.clone());
        enter_recovery(&mut bp);

        // Run phase A out into Reverse.
        let mut out = bp.update(&stuck_input());
        while out.recovery_phase != Some(RecoveryPhase::Reverse) {
            out = bp.update(&stuck_input());
        }

        // Feasible cycles with NO displacement: committed — must stay Reverse.
        for i in 0..2 * cfg.recovery_exit_feasible_cycles {
            let out = bp.update(&default_input());
            assert_eq!(
                out.recovery_phase,
                Some(RecoveryPhase::Reverse),
                "committed reverse aborted by feasible cycle {} before any displacement",
                i + 1
            );
        }

        // Displacement past the minimum (reverse started at the origin pose):
        // feasibility is consulted again; the sticky exit then applies.
        let reversed = BehaviorInput {
            robot_x: -(cfg.recovery_reverse_min_m + 0.01),
            ..default_input()
        };
        let mut out = bp.update(&reversed);
        for _ in 0..cfg.recovery_exit_feasible_cycles {
            out = bp.update(&reversed);
        }
        assert_eq!(
            out.state,
            DrivingState::Following,
            "after min reverse distance, consecutive feasible cycles must exit"
        );
    }

    #[test]
    fn test_committed_reverse_ends_on_blocked_rear_or_movement() {
        let cfg = BehaviorConfig::default();
        let mut bp = BehaviorPlanner::new(cfg);
        enter_recovery(&mut bp);
        let mut out = bp.update(&stuck_input());
        while out.recovery_phase != Some(RecoveryPhase::Reverse) {
            out = bp.update(&stuck_input());
        }

        // Rear blocked mid-maneuver ends the committed reverse immediately.
        let blocked = BehaviorInput {
            rear_clear: false,
            ..stuck_input()
        };
        let out = bp.update(&blocked);
        assert_eq!(out.recovery_phase, Some(RecoveryPhase::ForwardRetry));

        // And the moved > 0.3m exit stays immediate even mid-reverse.
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        enter_recovery(&mut bp);
        let mut out = bp.update(&stuck_input());
        while out.recovery_phase != Some(RecoveryPhase::Reverse) {
            out = bp.update(&stuck_input());
        }
        let moved = BehaviorInput {
            robot_x: -0.35,
            ..stuck_input()
        };
        let out = bp.update(&moved);
        assert_eq!(out.state, DrivingState::Following);
    }

    #[test]
    fn test_reverse_distance_gate_escalates_per_round_and_caps() {
        // Round n uses min((n+1)·recovery_reverse_min_m, recovery_reverse_max_m):
        // 0.15 → 0.30 → 0.45 → capped 0.50 with defaults.
        let cfg = BehaviorConfig::default();
        assert_eq!(cfg.recovery_max_attempts, 3, "test assumes 3 attempts");
        let mut bp = BehaviorPlanner::new(cfg.clone());
        enter_recovery(&mut bp);
        assert!((bp.reverse_target_m() - 0.15).abs() < 1e-9);

        run_one_round(&mut bp, &stuck_input()); // attempts = 1
        assert!((bp.reverse_target_m() - 0.30).abs() < 1e-9);
        run_one_round(&mut bp, &stuck_input()); // attempts = 2
        assert!((bp.reverse_target_m() - 0.45).abs() < 1e-9);
        run_one_round(&mut bp, &stuck_input()); // attempts = 3 (Hold)
        assert!(
            (bp.reverse_target_m() - cfg.recovery_reverse_max_m).abs() < 1e-9,
            "gate must cap at recovery_reverse_max_m, got {}",
            bp.reverse_target_m()
        );
    }

    #[test]
    fn test_committed_reverse_gate_uses_escalated_distance() {
        // Round 2 (one completed round → gate 0.30m): feasible cycles after
        // only 0.2m of actual reverse displacement must NOT abort the burst —
        // under the flat round-0 gate (0.15m) they would have. Identical
        // retreats re-running the same failed experiment is the bug.
        let cfg = BehaviorConfig::default();
        let mut bp = BehaviorPlanner::new(cfg.clone());
        enter_recovery(&mut bp);
        run_one_round(&mut bp, &stuck_input()); // attempts = 1
        assert!((bp.reverse_target_m() - 0.30).abs() < 1e-9);

        // Drive round 2 into Reverse (reverse_start at the origin pose).
        let mut out = bp.update(&stuck_input());
        while out.recovery_phase != Some(RecoveryPhase::Reverse) {
            out = bp.update(&stuck_input());
        }

        // 0.2m reversed (> base 0.15, < escalated gate 0.30 and < the 0.3m
        // movement exit): feasible cycles must be ignored — still committed.
        let reversed = BehaviorInput {
            robot_x: -0.2,
            ..default_input()
        };
        for i in 0..2 * cfg.recovery_exit_feasible_cycles {
            let out = bp.update(&reversed);
            assert_eq!(
                out.recovery_phase,
                Some(RecoveryPhase::Reverse),
                "escalated committed reverse aborted by feasible cycle {} at only 0.2m",
                i + 1
            );
        }
    }

    #[test]
    fn test_reverse_time_allowance_scales_with_escalated_distance() {
        // Round 2 targets 0.30m at the 0.1 m/s crawl → allowance
        // max(recovery_reverse_max_s, 0.30/0.1 + 1.0s slack) = 4.0s: the
        // burst must still be running at 3.5s and end by ~4.1s.
        let cfg = BehaviorConfig::default();
        let mut bp = BehaviorPlanner::new(cfg.clone());
        enter_recovery(&mut bp);
        run_one_round(&mut bp, &stuck_input()); // attempts = 1

        let mut out = bp.update(&stuck_input());
        while out.recovery_phase != Some(RecoveryPhase::Reverse) {
            out = bp.update(&stuck_input());
        }
        // 3.5s of stuck reversing (dt = 0.1): past the old flat 3.0s timeout,
        // still inside the escalated allowance.
        for _ in 0..35 {
            out = bp.update(&stuck_input());
            assert_eq!(
                out.recovery_phase,
                Some(RecoveryPhase::Reverse),
                "escalated round's reverse must outlast the flat 3s timeout"
            );
        }
        // ...and it does end shortly after 4.0s.
        for _ in 0..7 {
            out = bp.update(&stuck_input());
        }
        assert_ne!(
            out.recovery_phase,
            Some(RecoveryPhase::Reverse),
            "reverse burst must end once the scaled allowance elapses"
        );
    }

    #[test]
    fn test_attempts_persist_across_no_progress_exits() {
        // Recovery v2 item 3: exiting Recovery without net displacement must
        // NOT reset the attempt counter. One round here + a marginal exit +
        // re-stick + two more rounds = max_attempts -> Hold. (A reset counter
        // would need three fresh rounds after re-entry.)
        let cfg = BehaviorConfig::default();
        assert_eq!(cfg.recovery_max_attempts, 3, "test assumes 3 attempts");
        let mut bp = BehaviorPlanner::new(cfg.clone());
        enter_recovery(&mut bp);

        let out = run_one_round(&mut bp, &stuck_input()); // attempts = 1
        assert_eq!(out.recovery_phase, Some(RecoveryPhase::ForwardRetry));

        // Marginal exit with zero displacement.
        for _ in 0..cfg.recovery_exit_feasible_cycles {
            bp.update(&default_input());
        }
        assert_eq!(bp.state(), DrivingState::Following);

        // Re-stick at the same spot.
        for _ in 0..cfg.stuck_cycles_before_recovery {
            bp.update(&stuck_input());
        }
        assert_eq!(bp.state(), DrivingState::Recovery);

        let out = run_one_round(&mut bp, &stuck_input()); // attempts = 2
        assert_eq!(out.recovery_phase, Some(RecoveryPhase::ForwardRetry));
        let out = run_one_round(&mut bp, &stuck_input()); // attempts = 3
        assert_eq!(
            out.recovery_phase,
            Some(RecoveryPhase::Hold),
            "attempt counter must persist across exits without net progress"
        );
    }

    #[test]
    fn test_attempts_reset_on_net_displacement() {
        // Net displacement beyond recovery_progress_reset_m from the episode
        // origin clears the episode: a later re-stick starts a fresh count.
        let cfg = BehaviorConfig::default();
        let mut bp = BehaviorPlanner::new(cfg.clone());
        enter_recovery(&mut bp);
        run_one_round(&mut bp, &stuck_input()); // attempts = 1

        // Exit and drive well past the progress threshold.
        let far = BehaviorInput {
            robot_x: cfg.recovery_progress_reset_m + 0.2,
            ..default_input()
        };
        for _ in 0..cfg.recovery_exit_feasible_cycles + 1 {
            bp.update(&far);
        }
        assert_eq!(bp.state(), DrivingState::Following);

        // Re-stick at the new spot; two rounds must NOT reach Hold (a
        // persisted counter would hit 3 here).
        let stuck_far = BehaviorInput {
            robot_x: cfg.recovery_progress_reset_m + 0.2,
            ..stuck_input()
        };
        for _ in 0..cfg.stuck_cycles_before_recovery {
            bp.update(&stuck_far);
        }
        assert_eq!(bp.state(), DrivingState::Recovery);
        run_one_round(&mut bp, &stuck_far);
        let out = run_one_round(&mut bp, &stuck_far);
        assert_eq!(
            out.recovery_phase,
            Some(RecoveryPhase::ForwardRetry),
            "episode must have been cleared by net displacement"
        );
    }

    #[test]
    fn test_attempts_reset_after_stable_following() {
        // Reaching Following for >= 5s ends the episode even without 0.5m of
        // displacement (e.g. progress toward the goal in small increments).
        let cfg = BehaviorConfig::default();
        let mut bp = BehaviorPlanner::new(cfg.clone());
        enter_recovery(&mut bp);
        run_one_round(&mut bp, &stuck_input()); // attempts = 1

        for _ in 0..cfg.recovery_exit_feasible_cycles {
            bp.update(&default_input());
        }
        assert_eq!(bp.state(), DrivingState::Following);
        // 5s of stable Following at dt = 0.1.
        for _ in 0..51 {
            bp.update(&default_input());
        }

        for _ in 0..cfg.stuck_cycles_before_recovery {
            bp.update(&stuck_input());
        }
        assert_eq!(bp.state(), DrivingState::Recovery);
        run_one_round(&mut bp, &stuck_input());
        let out = run_one_round(&mut bp, &stuck_input());
        assert_eq!(
            out.recovery_phase,
            Some(RecoveryPhase::ForwardRetry),
            "episode must have been cleared by 5s of stable Following"
        );
    }

    #[test]
    fn test_hold_retries_phase_a_periodically() {
        // Recovery v2 item 4: Hold is not terminal — every hold_retry_period_s
        // it runs one phase-A round (crawl retry), then returns to Hold (never
        // straight to Reverse) if nothing was found.
        let cfg = BehaviorConfig::default();
        let mut bp = BehaviorPlanner::new(cfg.clone());
        enter_recovery(&mut bp);
        let mut out = run_one_round(&mut bp, &stuck_input());
        while out.recovery_phase != Some(RecoveryPhase::Hold) {
            out = run_one_round(&mut bp, &stuck_input());
        }

        // The retry fires after ~hold_retry_period_s of holding, not sooner.
        let period_cycles = (cfg.hold_retry_period_s / 0.1) as u32;
        let mut cycles = 0;
        loop {
            let out = bp.update(&stuck_input());
            cycles += 1;
            if out.recovery_phase == Some(RecoveryPhase::ForwardRetry) {
                assert!((out.desired_speed - 0.1).abs() < 1e-9, "retry round crawls");
                break;
            }
            assert_eq!(out.recovery_phase, Some(RecoveryPhase::Hold));
            assert_eq!(out.desired_speed, 0.0);
            assert!(cycles <= period_cycles + 2, "hold retry never fired");
        }
        assert!(
            cycles >= period_cycles - 1,
            "hold retry fired after only {} cycles",
            cycles
        );

        // A barren retry round with the rear CLEAR spends its phase B —
        // a reverse leg — before returning to Hold (live gauntlet: forward
        // permanently pinched, rear open, phase-A-only retries held forever).
        let mut saw_reverse = false;
        let mut guard = 0;
        loop {
            let out = bp.update(&stuck_input());
            if out.recovery_phase == Some(RecoveryPhase::Reverse) {
                saw_reverse = true;
            }
            if out.recovery_phase == Some(RecoveryPhase::Hold) {
                break;
            }
            guard += 1;
            assert!(guard < 200, "retry round never returned to Hold");
        }
        assert!(
            saw_reverse,
            "clear-rear hold retry must attempt a reverse leg"
        );

        // With the rear BLOCKED the retry round returns straight to Hold.
        let mut cycles = 0;
        loop {
            let out = bp.update(&stuck_input());
            cycles += 1;
            if out.recovery_phase == Some(RecoveryPhase::ForwardRetry) {
                break;
            }
            assert!(cycles < period_cycles + 5, "second hold retry never fired");
        }
        let blocked = BehaviorInput {
            rear_clear: false,
            ..stuck_input()
        };
        let mut guard = 0;
        loop {
            let out = bp.update(&blocked);
            assert_ne!(
                out.recovery_phase,
                Some(RecoveryPhase::Reverse),
                "blocked-rear hold retry must not reverse"
            );
            if out.recovery_phase == Some(RecoveryPhase::Hold) {
                break;
            }
            guard += 1;
            assert!(guard < 40, "blocked-rear retry never returned to Hold");
        }
    }

    #[test]
    fn test_behavior_config_validation() {
        assert!(BehaviorConfig::default().validate().is_ok());
        assert!(BehaviorConfig {
            stuck_cycles_before_recovery: 0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(BehaviorConfig {
            recovery_reverse_max_s: 0.0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(BehaviorConfig {
            recovery_max_attempts: 0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(BehaviorConfig {
            recovery_reverse_min_m: 0.0,
            ..Default::default()
        }
        .validate()
        .is_err());
        // Cap below the base gate would silently de-escalate every round.
        assert!(BehaviorConfig {
            recovery_reverse_max_m: 0.1,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(BehaviorConfig {
            recovery_reverse_max_m: f64::INFINITY,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(BehaviorConfig {
            recovery_exit_feasible_cycles: 0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(BehaviorConfig {
            recovery_progress_reset_m: f64::NAN,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(BehaviorConfig {
            hold_retry_period_s: -1.0,
            ..Default::default()
        }
        .validate()
        .is_err());
    }

    #[test]
    fn test_nonpositive_goal_tolerance_falls_back_to_config() {
        // A zero tolerance (unset proto field) must not make the goal
        // unreachable; it falls back to the config default.
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal {
            x: 0.05,
            y: 0.0,
            theta: 0.0,
            tolerance: Some(0.0),
            speed: None,
        });
        let out = bp.update(&default_input());
        assert_eq!(out.state, DrivingState::GoalReached);
    }
}
