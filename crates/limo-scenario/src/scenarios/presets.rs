/// Built-in scenario presets for common test cases.
use super::{ScenarioDef, WaypointDef};

/// Simple straight line: go 3m forward.
pub fn straight_line() -> ScenarioDef {
    ScenarioDef {
        name: "straight_line".into(),
        scenario_type: "waypoint".into(),
        waypoints: vec![
            WaypointDef {
                x: 3.0, y: 0.0, theta: 0.0,
                tolerance: 0.15, speed: 0.5, label: "goal".into(),
            },
        ],
        speed_limit: 0.5,
    }
}

/// Square patrol: drive in a 4m square loop.
pub fn square_patrol() -> ScenarioDef {
    ScenarioDef {
        name: "square_patrol".into(),
        scenario_type: "patrol".into(),
        waypoints: vec![
            WaypointDef { x: 2.0, y: 0.0, theta: 0.0, tolerance: 0.2, speed: 0.4, label: "A".into() },
            WaypointDef { x: 2.0, y: 2.0, theta: std::f64::consts::FRAC_PI_2, tolerance: 0.2, speed: 0.4, label: "B".into() },
            WaypointDef { x: 0.0, y: 2.0, theta: std::f64::consts::PI, tolerance: 0.2, speed: 0.4, label: "C".into() },
            WaypointDef { x: 0.0, y: 0.0, theta: -std::f64::consts::FRAC_PI_2, tolerance: 0.2, speed: 0.4, label: "D".into() },
        ],
        speed_limit: 0.5,
    }
}

/// Slalom: weave through cone-like waypoints.
pub fn slalom() -> ScenarioDef {
    ScenarioDef {
        name: "slalom".into(),
        scenario_type: "sequence".into(),
        waypoints: vec![
            WaypointDef { x: 1.0, y: 0.5, theta: 0.0, tolerance: 0.2, speed: 0.3, label: "s1".into() },
            WaypointDef { x: 2.0, y: -0.5, theta: 0.0, tolerance: 0.2, speed: 0.3, label: "s2".into() },
            WaypointDef { x: 3.0, y: 0.5, theta: 0.0, tolerance: 0.2, speed: 0.3, label: "s3".into() },
            WaypointDef { x: 4.0, y: -0.5, theta: 0.0, tolerance: 0.2, speed: 0.3, label: "s4".into() },
            WaypointDef { x: 5.0, y: 0.0, theta: 0.0, tolerance: 0.15, speed: 0.3, label: "finish".into() },
        ],
        speed_limit: 0.4,
    }
}

/// Parking: navigate to a tight parking spot.
pub fn parking() -> ScenarioDef {
    ScenarioDef {
        name: "parking".into(),
        scenario_type: "parking".into(),
        waypoints: vec![
            WaypointDef { x: 3.0, y: 1.0, theta: 0.0, tolerance: 0.3, speed: 0.3, label: "approach".into() },
            WaypointDef { x: 4.0, y: 1.0, theta: 0.0, tolerance: 0.05, speed: 0.1, label: "park".into() },
        ],
        speed_limit: 0.3,
    }
}

/// Figure-8 pattern.
pub fn figure_eight() -> ScenarioDef {
    let r = 2.0_f64;
    let n = 16;
    let mut waypoints = Vec::new();

    // First circle (counter-clockwise)
    for i in 0..n {
        let angle = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
        waypoints.push(WaypointDef {
            x: r * angle.cos(),
            y: r + r * angle.sin(),
            theta: angle + std::f64::consts::FRAC_PI_2,
            tolerance: 0.3, speed: 0.3,
            label: format!("f8_a_{}", i),
        });
    }
    // Second circle (clockwise, shifted)
    for i in 0..n {
        let angle = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
        waypoints.push(WaypointDef {
            x: r * angle.cos(),
            y: -r + r * (-angle).sin(),
            theta: -angle - std::f64::consts::FRAC_PI_2,
            tolerance: 0.3, speed: 0.3,
            label: format!("f8_b_{}", i),
        });
    }

    ScenarioDef {
        name: "figure_eight".into(),
        scenario_type: "patrol".into(),
        waypoints,
        speed_limit: 0.4,
    }
}
