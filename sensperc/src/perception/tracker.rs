//! Lidar obstacle clustering and tracking with velocity estimation.
//!
//! Pipeline per scan:
//!   1. `cluster_scan_points` — segment world-frame scan points (in beam
//!      order) into clusters by adjacent-point distance.
//!   2. `ClusterTracker::update` — associate compact clusters with existing
//!      tracks (nearest-neighbor within a gate, against the velocity-
//!      predicted position), update a smoothed constant-velocity estimate,
//!      coast missed tracks briefly, and drop stale ones.
//!
//! Large/elongated clusters (walls) are NOT tracked — they stay in the
//! per-sector point representation, which the grid inflation handles well.

/// One spatial cluster of scan points.
#[derive(Debug, Clone)]
pub struct Cluster {
    pub cx: f64,
    pub cy: f64,
    /// Max distance from centroid to any member point.
    pub radius: f64,
    /// Indices into the input point slice.
    pub point_indices: Vec<usize>,
}

/// Segment scan points (world frame, beam order) into clusters: consecutive
/// points within `eps` of each other belong to the same cluster. Clusters
/// with fewer than `min_pts` points are discarded as noise.
pub fn cluster_scan_points(pts: &[(f64, f64)], eps: f64, min_pts: usize) -> Vec<Cluster> {
    let mut clusters = Vec::new();
    let mut current: Vec<usize> = Vec::new();

    let flush = |current: &mut Vec<usize>, clusters: &mut Vec<Cluster>| {
        if current.len() >= min_pts {
            let n = current.len() as f64;
            let (sx, sy) = current
                .iter()
                .fold((0.0, 0.0), |(ax, ay), &i| (ax + pts[i].0, ay + pts[i].1));
            let (cx, cy) = (sx / n, sy / n);
            let radius = current
                .iter()
                .map(|&i| ((pts[i].0 - cx).powi(2) + (pts[i].1 - cy).powi(2)).sqrt())
                .fold(0.0, f64::max);
            clusters.push(Cluster {
                cx,
                cy,
                radius,
                point_indices: std::mem::take(current),
            });
        } else {
            current.clear();
        }
    };

    for i in 0..pts.len() {
        if let Some(&last) = current.last() {
            let d = ((pts[i].0 - pts[last].0).powi(2) + (pts[i].1 - pts[last].1).powi(2)).sqrt();
            if d > eps {
                flush(&mut current, &mut clusters);
            }
        }
        current.push(i);
    }
    flush(&mut current, &mut clusters);

    // A 360° scan wraps: merge first and last clusters if their endpoints
    // are adjacent (same physical object split across the seam).
    if clusters.len() >= 2 {
        let first_pt = pts[clusters[0].point_indices[0]];
        let last_cluster = clusters.last().unwrap();
        let last_pt = pts[*last_cluster.point_indices.last().unwrap()];
        let d = ((first_pt.0 - last_pt.0).powi(2) + (first_pt.1 - last_pt.1).powi(2)).sqrt();
        if d <= eps {
            let tail = clusters.pop().unwrap();
            let head = &mut clusters[0];
            head.point_indices.extend(tail.point_indices);
            let n = head.point_indices.len() as f64;
            let (sx, sy) = head
                .point_indices
                .iter()
                .fold((0.0, 0.0), |(ax, ay), &i| (ax + pts[i].0, ay + pts[i].1));
            head.cx = sx / n;
            head.cy = sy / n;
            head.radius = head
                .point_indices
                .iter()
                .map(|&i| ((pts[i].0 - head.cx).powi(2) + (pts[i].1 - head.cy).powi(2)).sqrt())
                .fold(0.0, f64::max);
        }
    }

    clusters
}

#[derive(Debug, Clone)]
pub struct TrackerConfig {
    /// Max association distance between a predicted track and a cluster.
    pub gate: f64,
    /// Smoothing factor for the velocity estimate (1.0 = raw differences).
    pub ema_alpha: f64,
    /// Updates before a track's velocity estimate is trusted (published).
    pub confirm_hits: u32,
    /// Consecutive missed scans before a track is dropped.
    pub max_misses: u32,
    /// Speeds below this are reported as zero. Must sit above the jitter
    /// residual: EMA of alternating +-v converges to v*a/(2-a), and 1cm
    /// centroid jitter at 10Hz is an alternating 0.2 m/s (-> 0.05 residual
    /// at alpha=0.4).
    pub static_speed: f64,
    /// Velocity estimates above this are clamped (association glitches).
    pub max_speed: f64,
}

impl Default for TrackerConfig {
    fn default() -> Self {
        Self {
            gate: 0.6,
            ema_alpha: 0.4,
            confirm_hits: 3,
            max_misses: 3,
            static_speed: 0.12,
            max_speed: 3.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TrackedObstacle {
    pub id: u32,
    pub x: f64,
    pub y: f64,
    /// Zero until the track is confirmed or when below the static dead-band.
    pub vx: f64,
    pub vy: f64,
    pub radius: f64,
}

#[derive(Debug, Clone)]
struct Track {
    id: u32,
    x: f64,
    y: f64,
    vx: f64,
    vy: f64,
    radius: f64,
    hits: u32,
    misses: u32,
}

pub struct ClusterTracker {
    config: TrackerConfig,
    tracks: Vec<Track>,
    next_id: u32,
}

impl ClusterTracker {
    pub fn new(config: TrackerConfig) -> Self {
        Self {
            config,
            tracks: Vec::new(),
            next_id: 1,
        }
    }

    /// Update tracks with this scan's compact clusters. `dt` is the time
    /// since the previous update in seconds.
    pub fn update(&mut self, clusters: &[Cluster], dt: f64) -> Vec<TrackedObstacle> {
        let dt = dt.max(1e-3);
        let mut assigned = vec![false; clusters.len()];

        // Greedy nearest-neighbor association against predicted positions.
        for track in &mut self.tracks {
            let px = track.x + track.vx * dt;
            let py = track.y + track.vy * dt;
            let mut best: Option<(usize, f64)> = None;
            for (ci, c) in clusters.iter().enumerate() {
                if assigned[ci] {
                    continue;
                }
                let d = ((c.cx - px).powi(2) + (c.cy - py).powi(2)).sqrt();
                if d <= self.config.gate && best.is_none_or(|(_, bd)| d < bd) {
                    best = Some((ci, d));
                }
            }
            if let Some((ci, _)) = best {
                assigned[ci] = true;
                let c = &clusters[ci];
                let raw_vx = (c.cx - track.x) / dt;
                let raw_vy = (c.cy - track.y) / dt;
                let a = self.config.ema_alpha;
                track.vx = a * raw_vx + (1.0 - a) * track.vx;
                track.vy = a * raw_vy + (1.0 - a) * track.vy;
                let speed = (track.vx * track.vx + track.vy * track.vy).sqrt();
                if speed > self.config.max_speed {
                    let s = self.config.max_speed / speed;
                    track.vx *= s;
                    track.vy *= s;
                }
                track.x = c.cx;
                track.y = c.cy;
                track.radius = 0.5 * (track.radius + c.radius);
                track.hits += 1;
                track.misses = 0;
            } else {
                // Coast briefly on the current velocity estimate.
                track.x = px;
                track.y = py;
                track.misses += 1;
            }
        }

        // Unmatched clusters spawn new tracks.
        for (ci, c) in clusters.iter().enumerate() {
            if !assigned[ci] {
                self.tracks.push(Track {
                    id: self.next_id,
                    x: c.cx,
                    y: c.cy,
                    vx: 0.0,
                    vy: 0.0,
                    radius: c.radius,
                    hits: 1,
                    misses: 0,
                });
                self.next_id += 1;
            }
        }

        let max_misses = self.config.max_misses;
        self.tracks.retain(|t| t.misses <= max_misses);

        self.tracks
            .iter()
            .map(|t| {
                let confirmed = t.hits >= self.config.confirm_hits;
                let speed = (t.vx * t.vx + t.vy * t.vy).sqrt();
                let publish_vel = confirmed && speed >= self.config.static_speed;
                TrackedObstacle {
                    id: t.id,
                    x: t.x,
                    y: t.y,
                    vx: if publish_vel { t.vx } else { 0.0 },
                    vy: if publish_vel { t.vy } else { 0.0 },
                    radius: t.radius,
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn blob(cx: f64, cy: f64) -> Vec<(f64, f64)> {
        vec![(cx - 0.05, cy), (cx, cy), (cx + 0.05, cy)]
    }

    #[test]
    fn clusters_two_separated_groups() {
        let mut pts = blob(1.0, 0.0);
        pts.extend(blob(3.0, 0.0));
        let clusters = cluster_scan_points(&pts, 0.25, 3);
        assert_eq!(clusters.len(), 2);
        assert!((clusters[0].cx - 1.0).abs() < 1e-9);
        assert!((clusters[1].cx - 3.0).abs() < 1e-9);
        assert!(clusters[0].radius <= 0.06);
    }

    #[test]
    fn discards_noise_below_min_pts() {
        let pts = vec![(1.0, 0.0), (5.0, 0.0)]; // two isolated returns
        assert!(cluster_scan_points(&pts, 0.25, 3).is_empty());
    }

    #[test]
    fn wall_forms_one_large_cluster() {
        // 2m wall sampled every 5cm: one cluster with radius ~1m.
        let pts: Vec<_> = (0..=40).map(|i| (i as f64 * 0.05, 2.0)).collect();
        let clusters = cluster_scan_points(&pts, 0.25, 3);
        assert_eq!(clusters.len(), 1);
        assert!(clusters[0].radius > 0.9);
    }

    #[test]
    fn merges_wraparound_cluster() {
        // Object split across the 360° seam: tail points adjacent to head.
        let mut pts = blob(1.0, 0.0);
        pts.extend(blob(5.0, 5.0)); // unrelated middle cluster
        pts.extend(vec![(0.8, 0.0), (0.85, 0.0), (0.9, 0.0)]); // seam side of the first blob
        let clusters = cluster_scan_points(&pts, 0.25, 3);
        assert_eq!(clusters.len(), 2);
        assert_eq!(clusters[0].point_indices.len(), 6);
    }

    fn one_cluster(cx: f64, cy: f64) -> Vec<Cluster> {
        vec![Cluster {
            cx,
            cy,
            radius: 0.15,
            point_indices: vec![],
        }]
    }

    #[test]
    fn estimates_velocity_of_moving_object() {
        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        // Object moving +x at 1.0 m/s, observed at 10 Hz.
        for step in 0..6 {
            tracker.update(&one_cluster(step as f64 * 0.1, 0.0), 0.1);
        }
        let out = tracker.update(&one_cluster(0.6, 0.0), 0.1);
        assert_eq!(out.len(), 1);
        assert!(
            (out[0].vx - 1.0).abs() < 0.15,
            "vx = {} should approach 1.0",
            out[0].vx
        );
        assert!(out[0].vy.abs() < 0.05);
        assert_eq!(out[0].id, 1);
    }

    #[test]
    fn static_object_reports_zero_velocity() {
        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        let mut out = Vec::new();
        for _ in 0..5 {
            // Centimeter-scale jitter around a fixed position.
            tracker.update(&one_cluster(2.0 + 0.01, 1.0 - 0.01), 0.1);
            out = tracker.update(&one_cluster(2.0 - 0.01, 1.0 + 0.01), 0.1);
        }
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].vx, 0.0);
        assert_eq!(out[0].vy, 0.0);
    }

    #[test]
    fn velocity_suppressed_until_confirmed() {
        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        // Two updates < confirm_hits(3): apparent motion must not publish yet.
        tracker.update(&one_cluster(0.0, 0.0), 0.1);
        let out = tracker.update(&one_cluster(0.2, 0.0), 0.1);
        assert_eq!(out[0].vx, 0.0);
    }

    #[test]
    fn track_coasts_through_misses_then_drops() {
        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        for step in 0..4 {
            tracker.update(&one_cluster(step as f64 * 0.1, 0.0), 0.1);
        }
        // Occlusion: no clusters for max_misses scans -> coasts, survives.
        for _ in 0..3 {
            let out = tracker.update(&[], 0.1);
            assert_eq!(out.len(), 1, "track should coast through misses");
        }
        // One more miss -> dropped.
        let out = tracker.update(&[], 0.1);
        assert!(out.is_empty());
    }

    #[test]
    fn two_objects_keep_distinct_ids() {
        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        let two = |x1: f64, x2: f64| {
            vec![
                Cluster {
                    cx: x1,
                    cy: 0.0,
                    radius: 0.1,
                    point_indices: vec![],
                },
                Cluster {
                    cx: x2,
                    cy: 2.0,
                    radius: 0.1,
                    point_indices: vec![],
                },
            ]
        };
        tracker.update(&two(0.0, 5.0), 0.1);
        let out = tracker.update(&two(0.05, 4.95), 0.1);
        assert_eq!(out.len(), 2);
        assert_ne!(out[0].id, out[1].id);
    }
}
