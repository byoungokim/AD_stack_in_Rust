//! Prior node-link roadmap for global routing.
//!
//! Today's failure mode: route TOPOLOGY was re-derived from live perception
//! by Hybrid A* on every replan, which caused route flapping and pocket
//! traps. The roadmap gives the robot standing global knowledge — nodes at
//! meaningful places, links with traversable width and a cruise speed cap —
//! so routing becomes a graph search (Dijkstra over travel time =
//! length/speed) and the metric planner handles only local execution of the
//! next short leg and deviations from it.
//!
//! Live perception feeds back through temporary link blocking:
//! `report_blocked` excludes a link from routing for a configured timeout,
//! and routing retries ignoring blocks when the exclusions leave no route at
//! all (a stale block must never strand the robot).

use serde::Deserialize;
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Matching slack (m) when deduplicating coincident route points.
const COINCIDENT_EPS: f64 = 1e-6;

// ---------------------------------------------------------------------------
// YAML schema
// ---------------------------------------------------------------------------

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RoadmapFile {
    /// Documentation-only frame tag (e.g. "planner"); not interpreted.
    #[serde(default)]
    #[allow(dead_code)]
    frame: Option<String>,
    nodes: Vec<NodeSpec>,
    links: Vec<LinkSpec>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct NodeSpec {
    id: String,
    x: f64,
    y: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct LinkSpec {
    from: String,
    to: String,
    /// Usable free width (m) of the link corridor.
    width: f64,
    /// Cruise speed cap (m/s) on the link.
    speed: f64,
    /// Traversable only from `from` to `to` when true.
    #[serde(default)]
    oneway: bool,
}

// ---------------------------------------------------------------------------
// Graph
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Node {
    pub id: String,
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone)]
pub struct Link {
    pub a: usize,
    pub b: usize,
    pub width: f64,
    pub speed: f64,
    pub length: f64,
    pub oneway: bool,
}

#[derive(Debug)]
pub struct Roadmap {
    nodes: Vec<Node>,
    links: Vec<Link>,
    /// Per-link temporary block expiry (None = not blocked).
    blocked_until: Vec<Option<Instant>>,
    blocked_timeout: Duration,
}

/// One vertex of a computed route polyline.
#[derive(Debug, Clone)]
pub struct RouteWaypoint {
    pub x: f64,
    pub y: f64,
    /// Speed cap (m/s) of the link on the segment LEAVING this waypoint
    /// (for the final waypoint: of the incoming link).
    pub speed: f64,
    /// Usable free width (m) of that same link.
    pub width: f64,
    /// Roadmap link index of that same link.
    pub link: usize,
    /// Node id when this vertex is a network node; empty for the snapped
    /// entry/exit points.
    pub node_id: String,
}

/// A computed route: polyline waypoints plus summary metrics.
#[derive(Debug, Clone)]
pub struct Route {
    pub waypoints: Vec<RouteWaypoint>,
    /// Total polyline length (m).
    pub length: f64,
    /// Estimated travel time (s) = sum of segment length / link speed cap.
    pub travel_time: f64,
}

impl Route {
    /// "spawn→lane_n_w→…→goal" — named nodes only (snapped entry/exit
    /// points are anonymous).
    pub fn describe(&self) -> String {
        let named: Vec<&str> = self
            .waypoints
            .iter()
            .filter(|w| !w.node_id.is_empty())
            .map(|w| w.node_id.as_str())
            .collect();
        if named.is_empty() {
            "<on-link>".to_string()
        } else {
            named.join("→")
        }
    }

    /// Narrowest usable link width (m) along the route.
    pub fn min_width(&self) -> f64 {
        self.waypoints
            .iter()
            .map(|w| w.width)
            .fold(f64::INFINITY, f64::min)
    }
}

/// Result of snapping a free point onto the nearest link segment.
#[derive(Debug, Clone, Copy)]
struct Snap {
    link: usize,
    /// Segment parameter in [0, 1] from link node `a` to node `b`.
    t: f64,
    x: f64,
    y: f64,
}

impl Roadmap {
    pub fn load(path: &str, blocked_timeout: Duration) -> Result<Self, String> {
        let contents =
            std::fs::read_to_string(path).map_err(|e| format!("cannot read '{}': {}", path, e))?;
        Self::from_yaml(&contents, blocked_timeout)
    }

    pub fn from_yaml(yaml: &str, blocked_timeout: Duration) -> Result<Self, String> {
        let file: RoadmapFile =
            serde_yaml::from_str(yaml).map_err(|e| format!("roadmap parse error: {}", e))?;
        Self::build(file, blocked_timeout)
    }

    fn build(file: RoadmapFile, blocked_timeout: Duration) -> Result<Self, String> {
        if file.nodes.is_empty() {
            return Err("roadmap has no nodes".into());
        }
        if file.links.is_empty() {
            return Err("roadmap has no links".into());
        }

        let mut index: HashMap<String, usize> = HashMap::new();
        let mut nodes = Vec::with_capacity(file.nodes.len());
        for n in file.nodes {
            if !(n.x.is_finite() && n.y.is_finite()) {
                return Err(format!("node '{}' has non-finite coordinates", n.id));
            }
            if index.insert(n.id.clone(), nodes.len()).is_some() {
                return Err(format!("duplicate node id '{}'", n.id));
            }
            nodes.push(Node {
                id: n.id,
                x: n.x,
                y: n.y,
            });
        }

        let mut links = Vec::with_capacity(file.links.len());
        for l in file.links {
            let a = *index
                .get(&l.from)
                .ok_or_else(|| format!("link '{}→{}': unknown node '{}'", l.from, l.to, l.from))?;
            let b = *index
                .get(&l.to)
                .ok_or_else(|| format!("link '{}→{}': unknown node '{}'", l.from, l.to, l.to))?;
            if a == b {
                return Err(format!("link '{}→{}' is a self-loop", l.from, l.to));
            }
            if !(l.width > 0.0 && l.width.is_finite()) {
                return Err(format!(
                    "link '{}→{}': width must be positive, got {}",
                    l.from, l.to, l.width
                ));
            }
            if !(l.speed > 0.0 && l.speed.is_finite()) {
                return Err(format!(
                    "link '{}→{}': speed must be positive, got {}",
                    l.from, l.to, l.speed
                ));
            }
            let length =
                ((nodes[b].x - nodes[a].x).powi(2) + (nodes[b].y - nodes[a].y).powi(2)).sqrt();
            if length < COINCIDENT_EPS {
                return Err(format!("link '{}→{}' has zero length", l.from, l.to));
            }
            links.push(Link {
                a,
                b,
                width: l.width,
                speed: l.speed,
                length,
                oneway: l.oneway,
            });
        }

        let blocked_until = vec![None; links.len()];
        Ok(Self {
            nodes,
            links,
            blocked_until,
            blocked_timeout,
        })
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn link_count(&self) -> usize {
        self.links.len()
    }

    /// "from→to" in authored orientation, for logs.
    pub fn link_name(&self, link: usize) -> String {
        match self.links.get(link) {
            Some(l) => format!("{}→{}", self.nodes[l.a].id, self.nodes[l.b].id),
            None => format!("<link {}>", link),
        }
    }

    /// Link index by endpoint node ids, matching either orientation.
    #[cfg(test)]
    pub fn find_link(&self, from: &str, to: &str) -> Option<usize> {
        self.links.iter().position(|l| {
            let (a, b) = (self.nodes[l.a].id.as_str(), self.nodes[l.b].id.as_str());
            (a == from && b == to) || (a == to && b == from)
        })
    }

    /// Mark a link temporarily blocked (routing excludes it until the
    /// configured timeout expires).
    pub fn report_blocked(&mut self, link: usize, now: Instant) {
        if let Some(slot) = self.blocked_until.get_mut(link) {
            *slot = Some(now + self.blocked_timeout);
        }
    }

    pub fn is_blocked(&self, link: usize, now: Instant) -> bool {
        self.blocked_until
            .get(link)
            .copied()
            .flatten()
            .is_some_and(|until| now < until)
    }

    fn any_blocked(&self, now: Instant) -> bool {
        (0..self.links.len()).any(|i| self.is_blocked(i, now))
    }

    /// Nearest point on any link segment to (x, y). Snapping considers ALL
    /// links (blocked included): a block excludes full-link traversal from
    /// routing, but the robot or goal may physically sit on the link.
    fn snap(&self, x: f64, y: f64) -> Option<Snap> {
        let mut best: Option<(f64, Snap)> = None;
        for (i, l) in self.links.iter().enumerate() {
            let (ax, ay) = (self.nodes[l.a].x, self.nodes[l.a].y);
            let (bx, by) = (self.nodes[l.b].x, self.nodes[l.b].y);
            let (dx, dy) = (bx - ax, by - ay);
            let t = (((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            let (px, py) = (ax + t * dx, ay + t * dy);
            let d2 = (x - px).powi(2) + (y - py).powi(2);
            if best.is_none_or(|(bd, _)| d2 < bd) {
                best = Some((
                    d2,
                    Snap {
                        link: i,
                        t,
                        x: px,
                        y: py,
                    },
                ));
            }
        }
        best.map(|(_, s)| s)
    }

    /// Route from `from_xy` to `to_xy`: snap both onto the nearest link
    /// segment (mid-link positions included), Dijkstra over travel time,
    /// excluding currently-blocked links. If the exclusions leave no route,
    /// retry ignoring them — stale blocks must never strand the robot.
    pub fn route(&self, from_xy: (f64, f64), to_xy: (f64, f64), now: Instant) -> Option<Route> {
        let start = self.snap(from_xy.0, from_xy.1)?;
        let goal = self.snap(to_xy.0, to_xy.1)?;
        self.route_snapped(&start, &goal, true, now).or_else(|| {
            if self.any_blocked(now) {
                self.route_snapped(&start, &goal, false, now)
            } else {
                None
            }
        })
    }

    fn route_snapped(
        &self,
        start: &Snap,
        goal: &Snap,
        respect_blocks: bool,
        now: Instant,
    ) -> Option<Route> {
        let n = self.nodes.len();
        let (vs, vg) = (n, n + 1); // virtual start / goal vertices

        // Edge list: (to, cost_seconds, via_link).
        let mut edges: Vec<Vec<(usize, f64, usize)>> = vec![Vec::new(); n + 2];
        for (i, l) in self.links.iter().enumerate() {
            if respect_blocks && self.is_blocked(i, now) {
                continue;
            }
            let t = l.length / l.speed;
            edges[l.a].push((l.b, t, i));
            if !l.oneway {
                edges[l.b].push((l.a, t, i));
            }
        }

        let dist_to_node = |s: &Snap, node: usize| -> f64 {
            ((s.x - self.nodes[node].x).powi(2) + (s.y - self.nodes[node].y).powi(2)).sqrt()
        };

        // Virtual edges for the snapped endpoints are block-exempt: the
        // robot (or the mission goal) physically sits on that link and the
        // partial traversal to its endpoints must stay possible.
        let ls = &self.links[start.link];
        edges[vs].push((ls.b, dist_to_node(start, ls.b) / ls.speed, start.link));
        if !ls.oneway {
            edges[vs].push((ls.a, dist_to_node(start, ls.a) / ls.speed, start.link));
        }
        let lg = &self.links[goal.link];
        edges[lg.a].push((vg, dist_to_node(goal, lg.a) / lg.speed, goal.link));
        if !lg.oneway {
            edges[lg.b].push((vg, dist_to_node(goal, lg.b) / lg.speed, goal.link));
        }
        // Same-link shortcut: both endpoints on one link — direct partial
        // traversal (direction-checked for oneway links).
        if start.link == goal.link {
            let along = (goal.t - start.t) * ls.length;
            if !ls.oneway || along >= 0.0 {
                edges[vs].push((vg, along.abs() / ls.speed, start.link));
            }
        }

        // Dense Dijkstra (graph is tens of vertices).
        let total = n + 2;
        let mut dist = vec![f64::INFINITY; total];
        let mut prev: Vec<Option<(usize, usize)>> = vec![None; total]; // (from, via_link)
        let mut done = vec![false; total];
        dist[vs] = 0.0;
        while let Some(u) = (0..total)
            .filter(|&v| !done[v] && dist[v].is_finite())
            .min_by(|&a, &b| dist[a].total_cmp(&dist[b]))
        {
            if u == vg {
                break;
            }
            done[u] = true;
            for &(to, cost, via) in &edges[u] {
                if dist[u] + cost < dist[to] {
                    dist[to] = dist[u] + cost;
                    prev[to] = Some((u, via));
                }
            }
        }
        if !dist[vg].is_finite() {
            return None;
        }

        // Name a snapped endpoint after a link node it coincides with:
        // Dijkstra tie-breaking may route the virtual vertex past the node
        // vertex itself, and the route must still read "…→slalom_exit".
        let endpoint_name = |s: &Snap| -> String {
            let l = &self.links[s.link];
            [l.a, l.b]
                .iter()
                .map(|&ni| &self.nodes[ni])
                .find(|nd| {
                    (s.x - nd.x).abs() < COINCIDENT_EPS && (s.y - nd.y).abs() < COINCIDENT_EPS
                })
                .map(|nd| nd.id.clone())
                .unwrap_or_default()
        };

        // Reconstruct (position, node_id, incoming link) items.
        let mut rev: Vec<(f64, f64, String, usize)> = Vec::new();
        let mut cur = vg;
        // Walk predecessors back to vs (the only vertex with prev == None).
        while let Some((from, via)) = prev[cur] {
            let item = if cur == vg {
                (goal.x, goal.y, endpoint_name(goal), via)
            } else {
                let nd = &self.nodes[cur];
                (nd.x, nd.y, nd.id.clone(), via)
            };
            rev.push(item);
            cur = from;
        }
        rev.push((start.x, start.y, endpoint_name(start), start.link));
        rev.reverse();

        // Deduplicate coincident points (snapped endpoint exactly on a
        // node), preferring the named vertex.
        let mut items: Vec<(f64, f64, String, usize)> = Vec::new();
        for it in rev {
            match items.last_mut() {
                Some(last)
                    if (last.0 - it.0).abs() < COINCIDENT_EPS
                        && (last.1 - it.1).abs() < COINCIDENT_EPS =>
                {
                    if last.2.is_empty() {
                        last.2 = it.2;
                    }
                    last.3 = it.3;
                }
                _ => items.push(it),
            }
        }

        // Waypoints carry the OUTGOING leg's link (last: incoming).
        let waypoints: Vec<RouteWaypoint> = (0..items.len())
            .map(|i| {
                let link = if i + 1 < items.len() {
                    items[i + 1].3
                } else {
                    items[i].3
                };
                let l = &self.links[link];
                RouteWaypoint {
                    x: items[i].0,
                    y: items[i].1,
                    speed: l.speed,
                    width: l.width,
                    link,
                    node_id: items[i].2.clone(),
                }
            })
            .collect();

        let mut length = 0.0;
        let mut travel_time = 0.0;
        for w in waypoints.windows(2) {
            let seg = ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
            length += seg;
            travel_time += seg / w[0].speed;
        }
        Some(Route {
            waypoints,
            length,
            travel_time,
        })
    }
}

// ---------------------------------------------------------------------------
// Route-following geometry (pure functions, unit-testable)
// ---------------------------------------------------------------------------

/// Projection of a point onto a route polyline.
#[derive(Debug, Clone, Copy)]
pub struct RouteProjection {
    /// Segment index i: the projection lies between waypoints i and i+1.
    pub segment: usize,
    /// Arc length (m) from the route start to the projected point.
    pub s: f64,
    /// Lateral distance (m) from the point to the route.
    pub distance: f64,
}

/// Project (x, y) onto the route polyline (nearest point on any segment).
pub fn project_onto_route(route: &Route, x: f64, y: f64) -> RouteProjection {
    let wps = &route.waypoints;
    if wps.len() < 2 {
        let d = wps
            .first()
            .map(|w| ((x - w.x).powi(2) + (y - w.y).powi(2)).sqrt())
            .unwrap_or(f64::INFINITY);
        return RouteProjection {
            segment: 0,
            s: 0.0,
            distance: d,
        };
    }
    let mut best = RouteProjection {
        segment: 0,
        s: 0.0,
        distance: f64::INFINITY,
    };
    let mut arc = 0.0;
    for i in 0..wps.len() - 1 {
        let (ax, ay) = (wps[i].x, wps[i].y);
        let (bx, by) = (wps[i + 1].x, wps[i + 1].y);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let seg_len = len2.sqrt();
        if seg_len < COINCIDENT_EPS {
            continue;
        }
        let t = (((x - ax) * dx + (y - ay) * dy) / len2).clamp(0.0, 1.0);
        let (px, py) = (ax + t * dx, ay + t * dy);
        let d = ((x - px).powi(2) + (y - py).powi(2)).sqrt();
        if d < best.distance {
            best = RouteProjection {
                segment: i,
                s: arc + t * seg_len,
                distance: d,
            };
        }
        arc += seg_len;
    }
    best
}

/// The point of the route at arc length `s` (clamped), with the local
/// tangent heading and the link of the segment it lies on.
#[derive(Debug, Clone, Copy)]
pub struct RouteGoal {
    /// Arc position (m) along the route this goal was placed at.
    pub s: f64,
    pub x: f64,
    pub y: f64,
    /// Tangent heading of the route at the goal.
    pub heading: f64,
    /// Link of the segment the goal lies on (the leg being executed).
    pub link: usize,
    /// True when the goal is the route's end — the caller should then feed
    /// the exact mission pose to the metric planner instead.
    pub is_final: bool,
}

/// Evaluate the route at arc length `s` (clamped to [0, length]).
pub fn goal_at_arc(route: &Route, s: f64) -> Option<RouteGoal> {
    let wps = &route.waypoints;
    let first = wps.first()?;
    if wps.len() < 2 {
        return Some(RouteGoal {
            s: 0.0,
            x: first.x,
            y: first.y,
            heading: 0.0,
            link: first.link,
            is_final: true,
        });
    }
    let s = s.clamp(0.0, route.length);
    let mut arc = 0.0;
    for i in 0..wps.len() - 1 {
        let (ax, ay) = (wps[i].x, wps[i].y);
        let (bx, by) = (wps[i + 1].x, wps[i + 1].y);
        let seg_len = ((bx - ax).powi(2) + (by - ay).powi(2)).sqrt();
        if seg_len < COINCIDENT_EPS {
            continue;
        }
        let last_seg = i == wps.len() - 2;
        if s <= arc + seg_len + COINCIDENT_EPS || last_seg {
            let t = ((s - arc) / seg_len).clamp(0.0, 1.0);
            return Some(RouteGoal {
                s,
                x: ax + t * (bx - ax),
                y: ay + t * (by - ay),
                heading: (by - ay).atan2(bx - ax),
                link: wps[i].link,
                is_final: s >= route.length - COINCIDENT_EPS,
            });
        }
        arc += seg_len;
    }
    None
}

/// Index of the first route vertex at or beyond arc position `s` — the
/// vertex the current leg is heading toward (visualization highlight).
pub fn vertex_at_or_after(route: &Route, s: f64) -> usize {
    let mut arc = 0.0;
    for (i, w) in route.waypoints.windows(2).enumerate() {
        arc += ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
        if arc >= s - COINCIDENT_EPS {
            return i + 1;
        }
    }
    route.waypoints.len().saturating_sub(1)
}

/// Hysteretic carrot advance: pick the arc position of the leg goal fed to
/// the metric planner. The previous goal is KEPT while it is still ahead
/// and more than `min_leg` away (a discretely-hopping goal keeps the global
/// planner's path hysteresis effective — a continuously sliding goal would
/// force a "goal changed" replacement every replan). When the robot closes
/// within `min_leg` (or had no goal), the new goal is placed `max_leg`
/// ahead — snapped back to the farthest route VERTEX inside
/// (robot + min_leg, robot + max_leg] when one exists, because network
/// nodes are the meaningful places — clamped to the route end.
pub fn advance_goal_arc(
    route: &Route,
    robot_s: f64,
    prev_goal_s: Option<f64>,
    min_leg: f64,
    max_leg: f64,
) -> f64 {
    let end = route.length;
    if let Some(g) = prev_goal_s {
        let g = g.min(end);
        if g >= robot_s && (g - robot_s > min_leg || g >= end - COINCIDENT_EPS) {
            return g;
        }
    }
    let target = (robot_s + max_leg).min(end);
    // Farthest waypoint arc inside the acceptance window.
    let mut arc = 0.0;
    let mut snapped: Option<f64> = None;
    for w in route.waypoints.windows(2) {
        arc += ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
        if arc > robot_s + min_leg && arc <= target + COINCIDENT_EPS {
            snapped = Some(arc);
        }
    }
    snapped.unwrap_or(target)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn secs(s: u64) -> Duration {
        Duration::from_secs(s)
    }

    fn gauntlet() -> Roadmap {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../config/maps/obstacle_gauntlet_roadmap.yaml"
        );
        Roadmap::load(path, secs(20)).expect("gauntlet roadmap must load")
    }

    fn node_ids(route: &Route) -> Vec<String> {
        route
            .waypoints
            .iter()
            .filter(|w| !w.node_id.is_empty())
            .map(|w| w.node_id.clone())
            .collect()
    }

    // ---- Loader + validation ----

    #[test]
    fn gauntlet_roadmap_loads() {
        let rm = gauntlet();
        assert_eq!(rm.node_count(), 14);
        assert_eq!(rm.link_count(), 14);
        assert!(rm.find_link("spawn", "lane_n_w").is_some());
        assert!(
            rm.find_link("lane_n_w", "spawn").is_some(),
            "either orientation"
        );
    }

    #[test]
    fn loader_rejects_unknown_link_endpoint() {
        let yaml = r#"
nodes:
  - {id: a, x: 0.0, y: 0.0}
  - {id: b, x: 1.0, y: 0.0}
links:
  - {from: a, to: ghost, width: 1.0, speed: 1.0}
"#;
        let err = Roadmap::from_yaml(yaml, secs(20)).unwrap_err();
        assert!(err.contains("unknown node 'ghost'"), "{}", err);
    }

    #[test]
    fn loader_rejects_non_positive_width_and_speed() {
        for (w, s) in [(0.0, 1.0), (-1.0, 1.0), (1.0, 0.0), (1.0, -0.5)] {
            let yaml = format!(
                "nodes:\n  - {{id: a, x: 0.0, y: 0.0}}\n  - {{id: b, x: 1.0, y: 0.0}}\n\
                 links:\n  - {{from: a, to: b, width: {}, speed: {}}}\n",
                w, s
            );
            let err = Roadmap::from_yaml(&yaml, secs(20)).unwrap_err();
            assert!(
                err.contains("must be positive"),
                "width={} speed={}: {}",
                w,
                s,
                err
            );
        }
    }

    #[test]
    fn loader_rejects_duplicate_node_ids_and_self_loops() {
        let dup = r#"
nodes:
  - {id: a, x: 0.0, y: 0.0}
  - {id: a, x: 1.0, y: 0.0}
links:
  - {from: a, to: a, width: 1.0, speed: 1.0}
"#;
        assert!(Roadmap::from_yaml(dup, secs(20))
            .unwrap_err()
            .contains("duplicate node id"));

        let selfloop = r#"
nodes:
  - {id: a, x: 0.0, y: 0.0}
  - {id: b, x: 1.0, y: 0.0}
links:
  - {from: a, to: a, width: 1.0, speed: 1.0}
"#;
        assert!(Roadmap::from_yaml(selfloop, secs(20))
            .unwrap_err()
            .contains("self-loop"));
    }

    #[test]
    fn loader_rejects_empty_and_typo_keys() {
        assert!(Roadmap::from_yaml("nodes: []\nlinks: []\n", secs(20)).is_err());
        // deny_unknown_fields: a typo'd key must fail loudly, not silently drop.
        let typo = r#"
nodes:
  - {id: a, x: 0.0, y: 0.0}
  - {id: b, x: 1.0, y: 0.0}
links:
  - {from: a, to: b, widht: 1.0, speed: 1.0}
"#;
        assert!(Roadmap::from_yaml(typo, secs(20)).is_err());
    }

    // ---- Routing ----

    #[test]
    fn north_lane_beats_apex_chain_on_travel_time() {
        // Hand math (spawn → slalom_exit):
        //   lane:  2.707/1.2 + 6.3/2.2 + 1.118/1.0 + 1.7/1.0 ≈ 7.94 s
        //   weave: 7.906 m at 0.8 ≈ 9.88 s
        let rm = gauntlet();
        let route = rm
            .route((0.0, 0.0), (7.0, 0.0), Instant::now())
            .expect("route must exist");
        let ids = node_ids(&route);
        assert_eq!(
            ids,
            vec!["spawn", "lane_n_w", "lane_n_e", "chan_e", "slalom_exit"],
            "default route must take the north lane, got {:?}",
            ids
        );
        assert!(
            (route.travel_time - 7.94).abs() < 0.05,
            "lane travel time ≈7.94s, got {:.3}",
            route.travel_time
        );
    }

    #[test]
    fn apex_chain_chosen_when_lane_links_blocked() {
        let mut rm = gauntlet();
        let now = Instant::now();
        for (a, b) in [
            ("spawn", "lane_n_w"),
            ("lane_n_w", "lane_n_e"),
            ("lane_n_e", "chan_e"),
            ("chan_e", "slalom_exit"),
        ] {
            let l = rm.find_link(a, b).unwrap();
            rm.report_blocked(l, now);
        }
        let route = rm.route((0.0, 0.0), (7.0, 0.0), now).expect("route");
        let ids = node_ids(&route);
        assert!(
            ids.iter().any(|id| id == "apex_2"),
            "blocked lane must fall back to the apex chain, got {:?}",
            ids
        );
        assert!(!ids.iter().any(|id| id == "lane_n_e"));
    }

    #[test]
    fn mid_link_snap_starts_route_along_the_link() {
        // Robot mid-lane at (3.0, 2.7): must snap onto lane_n_w→lane_n_e and
        // continue east — not detour back through lane_n_w.
        let rm = gauntlet();
        let route = rm.route((3.0, 2.7), (7.0, 0.0), Instant::now()).unwrap();
        let entry = &route.waypoints[0];
        assert!((entry.x - 3.0).abs() < 1e-9 && (entry.y - 2.7).abs() < 1e-9);
        assert_eq!(entry.link, rm.find_link("lane_n_w", "lane_n_e").unwrap());
        assert_eq!(route.waypoints[1].node_id, "lane_n_e");
        assert!(!node_ids(&route).iter().any(|id| id == "lane_n_w"));
    }

    #[test]
    fn blocked_link_timeout_expiry_restores_link() {
        let mut rm = gauntlet();
        let t0 = Instant::now();
        let lane = rm.find_link("lane_n_w", "lane_n_e").unwrap();
        rm.report_blocked(lane, t0);
        assert!(rm.is_blocked(lane, t0 + secs(19)));
        assert!(!rm.is_blocked(lane, t0 + secs(21)));

        // While blocked: routed around. After expiry: lane again.
        let during = rm.route((0.0, 0.0), (7.0, 0.0), t0 + secs(5)).unwrap();
        assert!(!node_ids(&during).iter().any(|id| id == "lane_n_e"));
        let after = rm.route((0.0, 0.0), (7.0, 0.0), t0 + secs(21)).unwrap();
        assert!(node_ids(&after).iter().any(|id| id == "lane_n_e"));
    }

    #[test]
    fn fully_blocked_graph_falls_back_to_ignoring_blocks() {
        // Never strand the robot on stale blocks: with EVERY link blocked the
        // exclusion pass finds nothing and the retry must ignore blocks.
        let mut rm = gauntlet();
        let now = Instant::now();
        for i in 0..rm.link_count() {
            rm.report_blocked(i, now);
        }
        let route = rm.route((0.0, 0.0), (17.9, 0.5), now);
        assert!(route.is_some(), "fallback route must exist despite blocks");
    }

    #[test]
    fn full_gauntlet_route_reaches_goal_via_gate_and_plaza() {
        let rm = gauntlet();
        let route = rm.route((0.0, 0.0), (17.9, 0.5), Instant::now()).unwrap();
        let ids = node_ids(&route);
        assert_eq!(
            ids,
            vec![
                "spawn",
                "lane_n_w",
                "lane_n_e",
                "chan_e",
                "slalom_exit",
                "gate_mid",
                "gate_e",
                "plaza",
                "goal"
            ]
        );
        assert!((route.travel_time - 14.70).abs() < 0.1);
        assert!((route.length - 23.4).abs() < 0.2);
    }

    #[test]
    fn current_scenario_waypoints_all_route_cleanly() {
        // Mission checkpoints from config/scenarios/obstacle_gauntlet.yaml
        // must snap onto the network (< 0.3 m) and be routable in sequence.
        let rm = gauntlet();
        let mission = [
            (0.2, 2.4),  // lane_entry
            (6.4, 2.4),  // lane_east
            (7.0, 0.0),  // slalom_exit
            (8.7, 0.9),  // gate_cleared
            (13.2, 0.0), // plaza_crossed
            (17.9, 0.5), // gauntlet_goal
        ];
        let mut from = (0.0, 0.0);
        for goal in mission {
            let route = rm
                .route(from, goal, Instant::now())
                .unwrap_or_else(|| panic!("no route {:?}→{:?}", from, goal));
            let end = route.waypoints.last().unwrap();
            let off = ((end.x - goal.0).powi(2) + (end.y - goal.1).powi(2)).sqrt();
            assert!(
                off < 0.3,
                "mission wp {:?} snapped {:.2}m off-network",
                goal,
                off
            );
            from = goal;
        }
    }

    #[test]
    fn oneway_link_is_direction_restricted() {
        let yaml = r#"
nodes:
  - {id: a, x: 0.0, y: 0.0}
  - {id: b, x: 2.0, y: 0.0}
  - {id: c, x: 1.0, y: 2.0}
links:
  - {from: a, to: b, width: 1.0, speed: 1.0, oneway: true}
  - {from: b, to: c, width: 1.0, speed: 1.0}
  - {from: c, to: a, width: 1.0, speed: 1.0}
"#;
        let rm = Roadmap::from_yaml(yaml, secs(20)).unwrap();
        // b→a must go the long way around (b→c→a), not backwards on a→b.
        let route = rm.route((2.0, 0.0), (0.0, 0.0), Instant::now()).unwrap();
        assert!(node_ids(&route).iter().any(|id| id == "c"));
    }

    // ---- Route-following geometry ----

    #[test]
    fn projection_reports_lateral_deviation() {
        let rm = gauntlet();
        let route = rm.route((0.2, 2.7), (7.0, 0.0), Instant::now()).unwrap();
        // 2m south of the lane midpoint.
        let p = project_onto_route(&route, 3.0, 0.7);
        assert!((p.distance - 2.0).abs() < 1e-6);
        // On the lane: zero deviation, arc ≈ 2.8m from lane_n_w.
        let q = project_onto_route(&route, 3.0, 2.7);
        assert!(q.distance < 1e-6);
        assert!((q.s - 2.8).abs() < 1e-6);
    }

    #[test]
    fn advance_goal_hops_discretely_and_prefers_nodes() {
        let rm = gauntlet();
        let route = rm.route((0.0, 0.0), (7.0, 0.0), Instant::now()).unwrap();
        // From the start: window (2.0, 4.0] contains lane_n_w (arc 2.707) —
        // snapped to the node, not the raw 4.0m carrot.
        let g0 = advance_goal_arc(&route, 0.0, None, 2.0, 4.0);
        assert!((g0 - 2.7074).abs() < 1e-3, "got {}", g0);
        // Still >min_leg ahead: goal KEPT (hysteresis, no slide).
        assert_eq!(advance_goal_arc(&route, 0.5, Some(g0), 2.0, 4.0), g0);
        // Robot closes within min_leg: new goal 4m ahead; no node in
        // (2.9+2, 2.9+4] (lane_n_e is at arc 9.0) → raw carrot at 6.9.
        let g1 = advance_goal_arc(&route, 0.9, Some(g0), 2.0, 4.0);
        assert!((g1 - 4.9).abs() < 1e-9, "got {}", g1);
        // Near the end the goal clamps to the route end and stays there.
        let end = advance_goal_arc(&route, route.length - 1.0, Some(g1), 2.0, 4.0);
        assert!((end - route.length).abs() < 1e-9);
        assert_eq!(
            advance_goal_arc(&route, route.length - 0.2, Some(end), 2.0, 4.0),
            end
        );
    }

    #[test]
    fn goal_at_arc_interpolates_with_tangent_heading() {
        let rm = gauntlet();
        let route = rm.route((0.2, 2.7), (7.0, 0.0), Instant::now()).unwrap();
        // 2.8m along: on the west-east lane, heading ~0.
        let g = goal_at_arc(&route, 2.8).unwrap();
        assert!((g.x - 3.0).abs() < 1e-6 && (g.y - 2.7).abs() < 1e-6);
        assert!(g.heading.abs() < 1e-6);
        assert!(!g.is_final);
        assert_eq!(g.link, rm.find_link("lane_n_w", "lane_n_e").unwrap());
        // At the end: final flag set.
        let end = goal_at_arc(&route, route.length).unwrap();
        assert!(end.is_final);
        assert!((end.x - 7.0).abs() < 1e-6 && (end.y - 0.0).abs() < 1e-6);
    }

    #[test]
    fn route_speed_and_width_follow_the_active_link() {
        let rm = gauntlet();
        let route = rm.route((0.0, 0.0), (7.0, 0.0), Instant::now()).unwrap();
        // Segment under a mid-lane robot: the 2.2 m/s, 1.6m lane link.
        let p = project_onto_route(&route, 3.0, 2.7);
        let wp = &route.waypoints[p.segment];
        assert_eq!(wp.speed, 2.2);
        assert_eq!(wp.width, 1.6);
    }

    #[test]
    fn describe_names_the_route() {
        let rm = gauntlet();
        let route = rm.route((0.0, 0.0), (7.0, 0.0), Instant::now()).unwrap();
        assert_eq!(
            route.describe(),
            "spawn→lane_n_w→lane_n_e→chan_e→slalom_exit"
        );
    }
}
