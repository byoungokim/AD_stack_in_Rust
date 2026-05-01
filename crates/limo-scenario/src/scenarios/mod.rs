/// Built-in scenario presets.
///
/// Each scenario returns a ScenarioCommand that can be sent to Planning
/// via CH8. Custom scenarios can be loaded from YAML files.
pub mod presets;

use serde::Deserialize;

/// A waypoint definition for YAML-based scenarios.
#[derive(Debug, Clone, Deserialize)]
pub struct WaypointDef {
    pub x: f64,
    pub y: f64,
    #[serde(default)]
    pub theta: f64,
    #[serde(default = "default_tolerance")]
    pub tolerance: f32,
    #[serde(default = "default_speed")]
    pub speed: f32,
    #[serde(default)]
    pub label: String,
}

fn default_tolerance() -> f32 {
    0.15
}
fn default_speed() -> f32 {
    0.5
}

/// A scenario definition loadable from YAML.
#[derive(Debug, Clone, Deserialize)]
pub struct ScenarioDef {
    pub name: String,
    #[serde(default = "default_scenario_type")]
    pub scenario_type: String, // waypoint, sequence, patrol, parking
    pub waypoints: Vec<WaypointDef>,
    #[serde(default = "default_speed")]
    pub speed_limit: f32,
}

fn default_scenario_type() -> String {
    "sequence".into()
}

impl ScenarioDef {
    /// Convert to a proto ScenarioCommand.
    pub fn to_command(&self, sequence: u32) -> limo_proto::ScenarioCommand {
        let scenario_type = match self.scenario_type.as_str() {
            "waypoint" => limo_proto::ScenarioType::ScenarioWaypoint as i32,
            "sequence" => limo_proto::ScenarioType::ScenarioWaypointSequence as i32,
            "patrol" => limo_proto::ScenarioType::ScenarioPatrol as i32,
            "parking" => limo_proto::ScenarioType::ScenarioParking as i32,
            "follow" => limo_proto::ScenarioType::ScenarioFollowPath as i32,
            "explore" => limo_proto::ScenarioType::ScenarioExplore as i32,
            _ => limo_proto::ScenarioType::ScenarioWaypointSequence as i32,
        };

        let waypoints: Vec<limo_proto::NavigationGoal> = self
            .waypoints
            .iter()
            .enumerate()
            .map(|(i, wp)| limo_proto::NavigationGoal {
                header: Some(limo_proto::Header {
                    timestamp_ns: 0,
                    sequence: i as u32,
                    frame_id: "world".into(),
                }),
                goal_pose: Some(limo_proto::Pose2D {
                    x: wp.x,
                    y: wp.y,
                    theta: wp.theta,
                }),
                goal_tolerance: wp.tolerance,
                desired_speed: wp.speed,
                label: if wp.label.is_empty() {
                    format!("wp_{}", i)
                } else {
                    wp.label.clone()
                },
            })
            .collect();

        limo_proto::ScenarioCommand {
            header: Some(limo_proto::Header {
                timestamp_ns: now_ns(),
                sequence,
                frame_id: "".into(),
            }),
            r#type: scenario_type,
            goal: waypoints.first().cloned(),
            waypoints,
            start: true,
            pause: false,
            global_speed_limit: self.speed_limit,
        }
    }
}

/// Load a scenario from a YAML file.
pub fn load_scenario(path: &str) -> anyhow::Result<ScenarioDef> {
    let contents = std::fs::read_to_string(path)?;
    let scenario: ScenarioDef = serde_yaml::from_str(&contents)?;
    Ok(scenario)
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}
