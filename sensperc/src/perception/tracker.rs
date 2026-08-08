//! Lidar obstacle clustering and tracking with velocity estimation.
//!
//! Pipeline per scan:
//!   1. `cluster_scan_returns` — segment world-frame scan points (in beam
//!      order) into clusters by adjacent-point distance. Only returns within
//!      `ClusterParams::max_range` participate (a range gate: beyond it the
//!      beam-arc spacing rivals the eps threshold and walls fragment into
//!      phantom objects), and the pairwise eps grows with range to follow
//!      arc spacing (`eps(r) = max(eps_base, k * r * angle_increment)`).
//!   2. Extent gate — clusters with fitted radius above `MAX_TRACK_EXTENT`
//!      are structure, not objects; they stay in the sector points / grid.
//!      Shape gate — elongated clusters (PCA aspect above
//!      `cluster_max_aspect` AND major-axis length above `min_major_len`)
//!      are wall fragments seen at grazing incidence, likewise structure.
//!   3. `ClusterTracker::update` — associate compact clusters with existing
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
    /// PCA elongation: sqrt(major eigenvalue / minor eigenvalue) of the
    /// point covariance. `f64::INFINITY` for perfectly collinear clusters.
    pub aspect: f64,
    /// Full extent of the points along the PCA major axis (meters).
    pub major_len: f64,
    /// Oriented-box half extent along the major axis (m), floored at the
    /// lidar-noise scale. Zero for synthetic clusters (circle fallback).
    pub half_major: f64,
    /// Oriented-box half extent along the minor axis (m).
    pub half_minor: f64,
    /// Major-axis heading (radians, world frame), normalized to
    /// [-PI/2, PI/2) — a box axis is direction-ambiguous.
    pub orientation: f64,
    /// Indices into the input point slice.
    pub point_indices: Vec<usize>,
}

impl Cluster {
    /// Build a cluster from member indices: centroid, extent radius, and
    /// planar-PCA shape statistics (aspect + major-axis length).
    fn from_indices(pts: &[(f64, f64)], point_indices: Vec<usize>) -> Self {
        let n = point_indices.len() as f64;
        let (sx, sy) = point_indices
            .iter()
            .fold((0.0, 0.0), |(ax, ay), &i| (ax + pts[i].0, ay + pts[i].1));
        let (cx, cy) = (sx / n, sy / n);
        let radius = point_indices
            .iter()
            .map(|&i| ((pts[i].0 - cx).powi(2) + (pts[i].1 - cy).powi(2)).sqrt())
            .fold(0.0, f64::max);

        // Planar PCA of the centered points (population covariance).
        let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
        for &i in &point_indices {
            let dx = pts[i].0 - cx;
            let dy = pts[i].1 - cy;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }
        let (a, b, c) = (sxx / n, sxy / n, syy / n);
        let half_tr = 0.5 * (a + c);
        let disc = (0.25 * (a - c).powi(2) + b * b).sqrt();
        let l_max = half_tr + disc;
        let l_min = (half_tr - disc).max(0.0);

        // Major-axis direction (eigenvector of l_max).
        let (ex, ey) = if b.abs() > 1e-12 {
            (l_max - c, b)
        } else if a >= c {
            (1.0, 0.0)
        } else {
            (0.0, 1.0)
        };
        let norm = (ex * ex + ey * ey).sqrt().max(1e-12);
        let (ex, ey) = (ex / norm, ey / norm);
        let (mut p_min, mut p_max) = (f64::INFINITY, f64::NEG_INFINITY);
        for &i in &point_indices {
            let p = (pts[i].0 - cx) * ex + (pts[i].1 - cy) * ey;
            p_min = p_min.min(p);
            p_max = p_max.max(p);
        }
        let major_len = p_max - p_min;
        // Minor-axis extent for the oriented-box representation: project on
        // the perpendicular. The floor covers one-face visibility (a box
        // seen edge-on has near-zero observed depth) and sensor noise.
        const HALF_EXTENT_FLOOR: f64 = 0.05;
        let (mx, my) = (-ey, ex);
        let (mut q_min, mut q_max) = (f64::INFINITY, f64::NEG_INFINITY);
        for &i in &point_indices {
            let q = (pts[i].0 - cx) * mx + (pts[i].1 - cy) * my;
            q_min = q_min.min(q);
            q_max = q_max.max(q);
        }
        let half_major = (0.5 * major_len).max(HALF_EXTENT_FLOOR);
        let half_minor = (0.5 * (q_max - q_min)).max(HALF_EXTENT_FLOOR);
        let mut orientation = ey.atan2(ex);
        if orientation >= std::f64::consts::FRAC_PI_2 {
            orientation -= std::f64::consts::PI;
        } else if orientation < -std::f64::consts::FRAC_PI_2 {
            orientation += std::f64::consts::PI;
        }
        // 1e-10 m^2 == 1e-5 m std: far below sensor noise, so only truly
        // degenerate (collinear) clusters map to infinity.
        let aspect = if l_min < 1e-10 {
            f64::INFINITY
        } else {
            (l_max / l_min).sqrt()
        };

        Self {
            cx,
            cy,
            radius,
            aspect,
            major_len,
            half_major,
            half_minor,
            orientation,
            point_indices,
        }
    }

    /// Shape gate: true for clusters that look like wall fragments — highly
    /// elongated AND long enough along the major axis that the elongation is
    /// meaningful. The major-length floor spares small 3-8 return clusters
    /// (cones) whose minor axis is near-degenerate at range: a r=0.15 cone
    /// can never exceed ~0.30 m of major extent, while grazing-incidence
    /// wall slivers that slip under the extent gate span 0.35 m and more.
    pub fn is_wall_like(&self, params: &ClusterParams) -> bool {
        self.aspect > params.cluster_max_aspect && self.major_len > params.min_major_len
    }
}

/// Parameters for range-gated, range-adaptive clustering of scan returns.
#[derive(Debug, Clone)]
pub struct ClusterParams {
    /// Floor for the adjacent-point distance threshold (meters).
    pub eps_base: f64,
    /// Multiplier `k` in `eps(r) = max(eps_base, k * r * angle_increment)`.
    /// Adjacent returns on one surface are ~`r * angle_increment` apart, so
    /// `k` gives headroom for slanted surfaces and dropped beams without
    /// merging separate objects.
    pub eps_range_factor: f64,
    /// Returns beyond this range never participate in cluster formation —
    /// they still feed the sector points and the SLAM grid. Past ~4.5 m the
    /// arc spacing between beams approaches the eps floor, and wall segments
    /// fragment into phantom "objects" that pollute tracking.
    pub max_range: f64,
    /// Minimum points for a cluster (smaller groups are noise).
    pub min_pts: usize,
    /// Shape gate: clusters whose PCA aspect (sqrt of major/minor
    /// eigenvalue) exceeds this AND whose major-axis length exceeds
    /// `min_major_len` are wall fragments, not compact objects. Beyond ~3 m
    /// at oblique incidence, sparse wall returns fragment into clusters
    /// small enough to slip the extent gate; elongation catches them.
    pub cluster_max_aspect: f64,
    /// Major-axis length floor for the shape gate (meters). Small 3-8
    /// return clusters have a near-degenerate minor axis (huge aspect) but
    /// tiny extent — they must stay trackable. A r=0.15 cone tops out at
    /// ~0.30 m major extent (its full diameter), so 0.35 keeps every cone
    /// while a 0.4 m grazing wall sliver still fails.
    pub min_major_len: f64,
}

impl Default for ClusterParams {
    fn default() -> Self {
        Self {
            eps_base: 0.25,
            eps_range_factor: 2.5,
            max_range: 4.5,
            min_pts: 3,
            cluster_max_aspect: 2.5,
            min_major_len: 0.35,
        }
    }
}

/// Clusters whose fitted extent radius exceeds this are structure (walls,
/// gate frames), not compact objects, and must never become or update
/// tracks. Structure is already represented by the occupancy grid and the
/// nearest-per-sector points.
pub const MAX_TRACK_EXTENT: f64 = 0.45;

/// Segment scan points (world frame, beam order) into clusters: consecutive
/// points within a fixed `eps` of each other belong to the same cluster.
/// Clusters with fewer than `min_pts` points are discarded as noise.
///
/// Production uses the range-gated `cluster_scan_returns`; this fixed-eps
/// variant remains as the reference behavior for the clustering core tests.
#[cfg(test)]
pub fn cluster_scan_points(pts: &[(f64, f64)], eps: f64, min_pts: usize) -> Vec<Cluster> {
    cluster_by(pts, min_pts, |_| true, |_, _| eps)
}

/// Range-gated, range-adaptive clustering of scan returns.
///
/// `pts` are world-frame points in beam order and `ranges` the matching
/// sensor-frame ranges. Returns beyond `params.max_range` are skipped: they
/// never join a cluster, and they do not break a chain either (a far
/// background return between two beams on the same nearby object must not
/// split it — the pairwise eps check between the surviving neighbors handles
/// genuine separation).
///
/// The pairwise threshold grows with range to track beam-arc spacing:
/// `eps(a, b) = max(eps_base, eps_range_factor * max(r_a, r_b) * angle_increment)`.
pub fn cluster_scan_returns(
    pts: &[(f64, f64)],
    ranges: &[f64],
    angle_increment: f64,
    params: &ClusterParams,
) -> Vec<Cluster> {
    assert_eq!(
        pts.len(),
        ranges.len(),
        "pts and ranges must be parallel arrays"
    );
    cluster_by(
        pts,
        params.min_pts,
        |i| ranges[i] <= params.max_range,
        |a, b| {
            let r = ranges[a].max(ranges[b]);
            (params.eps_range_factor * r * angle_increment).max(params.eps_base)
        },
    )
}

/// Shared clustering core: chain consecutive included points whose pairwise
/// distance is within `eps_between`, then merge across the 360° seam.
fn cluster_by(
    pts: &[(f64, f64)],
    min_pts: usize,
    include: impl Fn(usize) -> bool,
    eps_between: impl Fn(usize, usize) -> f64,
) -> Vec<Cluster> {
    let mut clusters = Vec::new();
    let mut current: Vec<usize> = Vec::new();

    let flush = |current: &mut Vec<usize>, clusters: &mut Vec<Cluster>| {
        if current.len() >= min_pts {
            clusters.push(Cluster::from_indices(pts, std::mem::take(current)));
        } else {
            current.clear();
        }
    };

    for i in 0..pts.len() {
        if !include(i) {
            continue;
        }
        if let Some(&last) = current.last() {
            let d = ((pts[i].0 - pts[last].0).powi(2) + (pts[i].1 - pts[last].1).powi(2)).sqrt();
            if d > eps_between(last, i) {
                flush(&mut current, &mut clusters);
            }
        }
        current.push(i);
    }
    flush(&mut current, &mut clusters);

    // A 360° scan wraps: merge first and last clusters if their endpoints
    // are adjacent (same physical object split across the seam).
    if clusters.len() >= 2 {
        let first_idx = clusters[0].point_indices[0];
        let first_pt = pts[first_idx];
        let last_cluster = clusters.last().unwrap();
        let last_idx = *last_cluster.point_indices.last().unwrap();
        let last_pt = pts[last_idx];
        let d = ((first_pt.0 - last_pt.0).powi(2) + (first_pt.1 - last_pt.1).powi(2)).sqrt();
        if d <= eps_between(last_idx, first_idx) {
            let tail = clusters.pop().unwrap();
            let mut indices = std::mem::take(&mut clusters[0].point_indices);
            indices.extend(tail.point_indices);
            clusters[0] = Cluster::from_indices(pts, indices);
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
    /// Oriented-box half extents (m) and heading; zero extents = circle.
    pub half_major: f64,
    pub half_minor: f64,
    pub orientation: f64,
}

#[derive(Debug, Clone)]
struct Track {
    id: u32,
    x: f64,
    y: f64,
    vx: f64,
    vy: f64,
    radius: f64,
    half_major: f64,
    half_minor: f64,
    orientation: f64,
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
                // Box smoothing: extents like the radius; the orientation
                // via the doubled-angle circular mean (a box axis is
                // direction-ambiguous, so blend 2θ, halve back).
                track.half_major = 0.5 * (track.half_major + c.half_major);
                track.half_minor = 0.5 * (track.half_minor + c.half_minor);
                let (s2, c2) = (
                    0.5 * ((2.0 * track.orientation).sin() + (2.0 * c.orientation).sin()),
                    0.5 * ((2.0 * track.orientation).cos() + (2.0 * c.orientation).cos()),
                );
                track.orientation = 0.5 * s2.atan2(c2);
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
                    half_major: c.half_major,
                    half_minor: c.half_minor,
                    orientation: c.orientation,
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
                    half_major: t.half_major,
                    half_minor: t.half_minor,
                    orientation: t.orientation,
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

    /// Compact, round cluster stub for tracker-level tests.
    fn compact_cluster(cx: f64, cy: f64, radius: f64) -> Cluster {
        Cluster {
            cx,
            cy,
            radius,
            aspect: 1.0,
            major_len: radius,
            half_major: 0.0,
            half_minor: 0.0,
            orientation: 0.0,
            point_indices: vec![],
        }
    }

    fn one_cluster(cx: f64, cy: f64) -> Vec<Cluster> {
        vec![compact_cluster(cx, cy, 0.15)]
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
        let two =
            |x1: f64, x2: f64| vec![compact_cluster(x1, 0.0, 0.1), compact_cluster(x2, 2.0, 0.1)];
        tracker.update(&two(0.0, 5.0), 0.1);
        let out = tracker.update(&two(0.05, 4.95), 0.1);
        assert_eq!(out.len(), 2);
        assert_ne!(out[0].id, out[1].id);
    }

    // ---- Range gate / extent gate / adaptive eps (scan-level scenarios) ----

    /// RPLIDAR A1-like angular resolution (1 degree).
    const INC: f64 = 0.017_453_3;

    /// Beams (angle, range) for `n` returns on a compact object's front
    /// face: consecutive beams centered on `center_angle`, all at range `r`.
    fn cone_beams(r: f64, center_angle: f64, n: usize, inc: f64) -> Vec<(f64, f64)> {
        let half = (n as f64 - 1.0) / 2.0;
        (0..n)
            .map(|k| (center_angle + (k as f64 - half) * inc, r))
            .collect()
    }

    /// Beams (angle, range) for a straight wall `y = d` swept from `a0_deg`
    /// to `a1_deg` at `inc` angular spacing.
    fn wall_beams(d: f64, a0_deg: f64, a1_deg: f64, inc: f64) -> Vec<(f64, f64)> {
        let mut beams = Vec::new();
        let mut a = a0_deg.to_radians();
        while a <= a1_deg.to_radians() + 1e-9 {
            beams.push((a, d / a.sin()));
            a += inc;
        }
        beams
    }

    fn project(beams: &[(f64, f64)]) -> (Vec<(f64, f64)>, Vec<f64>) {
        beams
            .iter()
            .map(|&(a, r)| ((r * a.cos(), r * a.sin()), r))
            .unzip()
    }

    /// Cluster + gate exactly like the aggregator: compact clusters must
    /// pass BOTH the extent gate and the PCA shape gate; everything else is
    /// structure.
    fn run_pipeline(beams: &[(f64, f64)], inc: f64) -> (Vec<Cluster>, Vec<Cluster>) {
        let params = ClusterParams::default();
        let (pts, ranges) = project(beams);
        let clusters = cluster_scan_returns(&pts, &ranges, inc, &params);
        clusters
            .into_iter()
            .partition(|c| c.radius <= MAX_TRACK_EXTENT && !c.is_wall_like(&params))
    }

    /// Beams (angle, range) ray-cast against a round object of radius
    /// `obj_r` centered at range `d` straight ahead: physically correct
    /// front-face arc (ranges shorten toward the center beam), unlike
    /// `cone_beams`, which puts every return at the same range.
    fn circle_beams(d: f64, obj_r: f64, inc: f64) -> Vec<(f64, f64)> {
        let half = (obj_r / d).asin();
        let n_half = (half / inc).floor() as i64;
        (-n_half..=n_half)
            .map(|k| {
                let a = k as f64 * inc;
                // Ray-circle: t = d cos a - sqrt(r^2 - d^2 sin^2 a).
                let disc = obj_r * obj_r - (d * a.sin()).powi(2);
                (a, d * a.cos() - disc.sqrt())
            })
            .collect()
    }

    /// Beams (angle, range) hitting a straight wall at grazing incidence:
    /// the wall line passes through (dist, 0) with direction `incidence_deg`
    /// from the x-axis. Beams start at angle 0 and step by `inc` until the
    /// returns span `span` meters along the wall.
    fn line_beams(dist: f64, incidence_deg: f64, span: f64, inc: f64) -> Vec<(f64, f64)> {
        let phi = incidence_deg.to_radians();
        let (ux, uy) = (phi.cos(), phi.sin());
        let mut out: Vec<(f64, f64)> = Vec::new();
        let mut first: Option<(f64, f64)> = None;
        for k in 0..90 {
            let a = k as f64 * inc;
            let (dx, dy) = (a.cos(), a.sin());
            let det = ux * dy - dx * uy;
            if det.abs() < 1e-9 {
                break; // beam parallel to the wall
            }
            let t = -dist * uy / det;
            if t <= 0.0 {
                break;
            }
            let p = (t * dx, t * dy);
            match first {
                None => first = Some(p),
                Some(p0) => {
                    if ((p.0 - p0.0).powi(2) + (p.1 - p0.1).powi(2)).sqrt() > span {
                        break;
                    }
                }
            }
            out.push((a, t));
        }
        out
    }

    #[test]
    fn wall_at_3m_extent_gated_cone_tracked() {
        // (a) Straight wall at 3 m (inside the range gate) + cone at 2 m:
        // exactly one track (the cone); the wall clusters but is rejected by
        // the extent gate as structure.
        let mut beams = cone_beams(2.0, (-30.0_f64).to_radians(), 3, INC);
        beams.extend(wall_beams(3.0, 60.0, 120.0, INC));
        let (compact, structure) = run_pipeline(&beams, INC);
        assert_eq!(compact.len(), 1, "only the cone may pass the extent gate");
        assert_eq!(structure.len(), 1, "wall must cluster but be extent-gated");
        assert!(structure[0].radius > MAX_TRACK_EXTENT);

        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        let mut out = Vec::new();
        for _ in 0..3 {
            out = tracker.update(&compact, 0.1);
        }
        assert_eq!(out.len(), 1);
        let (ex, ey) = (
            2.0 * (-30.0_f64).to_radians().cos(),
            2.0 * (-30.0_f64).to_radians().sin(),
        );
        assert!((out[0].x - ex).abs() < 0.05 && (out[0].y - ey).abs() < 0.05);
        assert!(out[0].radius < 0.1, "cone track must stay cone-sized");
    }

    #[test]
    fn wall_beyond_cluster_range_never_clusters_but_stays_static() {
        // (b) Wall at 6 m: beyond cluster_max_range (4.5) but inside
        // MAX_OBSTACLE_RANGE (8.0) — zero clusters/tracks, yet every return
        // survives to the static/sector output.
        let beams = wall_beams(6.0, 60.0, 120.0, INC);
        let (pts, ranges) = project(&beams);
        let clusters = cluster_scan_returns(&pts, &ranges, INC, &ClusterParams::default());
        assert!(
            clusters.is_empty(),
            "range-gated wall must form no clusters"
        );

        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        assert!(tracker.update(&clusters, 0.1).is_empty());

        // The planning-lookahead filter still admits these returns...
        let max_r = ranges.iter().cloned().fold(0.0, f64::max);
        assert!(max_r < crate::MAX_OBSTACLE_RANGE as f64);
        // ...and with no cluster owning any point, all of them reach the
        // nearest-per-sector static representation (aggregator wiring).
        let static_returns: Vec<(f32, f32)> =
            beams.iter().map(|&(a, r)| (a as f32, r as f32)).collect();
        let sectors = crate::nearest_per_sector(&static_returns, 72);
        assert!(
            sectors.len() >= 12,
            "wall must remain in sector output, got {} sectors",
            sectors.len()
        );
    }

    #[test]
    fn two_cones_half_meter_apart_at_4m_stay_distinct() {
        // (c) Two cones with 0.5 m center separation at ~4 m, 1-degree beam
        // spacing: adaptive eps must not merge them.
        let sep_angle = (0.5_f64 / 4.0).atan();
        let r2 = (4.0_f64.powi(2) + 0.5_f64.powi(2)).sqrt();
        let mut beams = cone_beams(4.0, 0.0, 3, INC);
        beams.extend(cone_beams(r2, sep_angle, 3, INC));
        let (compact, structure) = run_pipeline(&beams, INC);
        assert_eq!(compact.len(), 2, "cones 0.5 m apart must not merge");
        assert!(structure.is_empty());

        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        let out = tracker.update(&compact, 0.1);
        assert_eq!(out.len(), 2);
        assert_ne!(out[0].id, out[1].id);
    }

    #[test]
    fn coarse_beam_spacing_does_not_fragment_cone_at_4m() {
        // (d) Cone at 4 m whose returns sit a full beam-arc apart
        // (4 m * angle_increment). With inc = 0.07 rad the spacing is
        // 0.28 m > eps_base, so a fixed eps fragments it; the adaptive eps
        // (2.5 * 4 * 0.07 = 0.7) keeps it whole. (Clustering-level test:
        // gates are exercised separately.)
        let inc = 0.07;
        let beams = cone_beams(4.0, 0.0, 3, inc);
        let (pts, ranges) = project(&beams);

        // Fixed base eps fragments (each chain < min_pts -> nothing).
        assert!(cluster_scan_points(&pts, 0.25, 3).is_empty());

        let clusters = cluster_scan_returns(&pts, &ranges, inc, &ClusterParams::default());
        assert_eq!(clusters.len(), 1, "adaptive eps must keep the cone whole");
        assert_eq!(clusters[0].point_indices.len(), ranges.len());
    }

    #[test]
    fn pedestrian_arc_is_tracked() {
        // (e) Pedestrian (r = 0.25) at 3.5 m: ~8 physically ray-cast
        // returns curving around the front face. Inside the range gate and
        // the extent gate, and round enough (aspect < 2.5) for the shape
        // gate despite a ~0.48 m major extent -> tracked.
        let beams = circle_beams(3.5, 0.25, INC);
        assert!(
            (7..=10).contains(&beams.len()),
            "expected ~8 returns, got {}",
            beams.len()
        );
        let (compact, structure) = run_pipeline(&beams, INC);
        assert_eq!(compact.len(), 1, "pedestrian must pass both gates");
        assert!(structure.is_empty());
        assert!(compact[0].radius > 0.15 && compact[0].radius < MAX_TRACK_EXTENT);
        assert!(
            compact[0].aspect < 2.5,
            "front-face curvature must keep aspect low, got {}",
            compact[0].aspect
        );

        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        let out = tracker.update(&compact, 0.1);
        assert_eq!(out.len(), 1);
    }

    // ---- Shape gate (PCA aspect + major-axis length) ----

    #[test]
    fn wall_sliver_at_grazing_incidence_is_shape_gated() {
        // Wall fragment at 3.5 m seen at 75 deg incidence: ~7 sparse
        // returns spanning ~0.4 m along the wall. Radius ~0.2 slips the
        // extent gate — exactly the phantom-object defect — but the points
        // are collinear, so the shape gate must reject it as structure.
        let beams = line_beams(3.5, 75.0, 0.40, INC);
        assert!(beams.len() >= 5, "fixture must produce a sliver");
        let (compact, structure) = run_pipeline(&beams, INC);
        assert!(
            compact.is_empty(),
            "wall sliver must not become a trackable cluster"
        );
        assert_eq!(structure.len(), 1);
        let sliver = &structure[0];
        assert!(
            sliver.radius <= MAX_TRACK_EXTENT,
            "fixture must actually slip the extent gate (radius {})",
            sliver.radius
        );
        assert!(sliver.aspect > ClusterParams::default().cluster_max_aspect);
        assert!(sliver.major_len > ClusterParams::default().min_major_len);
    }

    #[test]
    fn round_cone_with_few_returns_passes_shape_gate() {
        // Cone r = 0.15 at 3 m: 5 ray-cast returns. The shallow arc has a
        // near-degenerate minor axis (aspect >> 2.5), but its major extent
        // stays under the 0.35 m floor, so the gate must keep it.
        let beams = circle_beams(3.0, 0.15, INC);
        assert!(
            (3..=8).contains(&beams.len()),
            "expected 3-8 returns, got {}",
            beams.len()
        );
        let (compact, structure) = run_pipeline(&beams, INC);
        assert_eq!(compact.len(), 1, "cone must remain trackable");
        assert!(structure.is_empty());
        assert!(
            compact[0].major_len < ClusterParams::default().min_major_len,
            "cone major extent {} must sit under the floor",
            compact[0].major_len
        );

        let mut tracker = ClusterTracker::new(TrackerConfig::default());
        assert_eq!(tracker.update(&compact, 0.1).len(), 1);
    }

    #[test]
    fn sliver_and_cone_in_one_scan_only_cone_tracked() {
        // Regression for the live defect: a grazing wall sliver beyond 3 m
        // and a real cone in the same scan — only the cone may be tracked;
        // the sliver stays in the structure representation.
        let mut beams = circle_beams(2.0, 0.15, INC); // cone near angle 0
                                                      // Sliver well away from the cone (wall through bearing ~40 deg).
        let sliver: Vec<(f64, f64)> = line_beams(3.5, 75.0, 0.40, INC)
            .into_iter()
            .map(|(a, r)| (a + 0.7, r))
            .collect();
        beams.extend(sliver);
        let (compact, structure) = run_pipeline(&beams, INC);
        assert_eq!(compact.len(), 1, "only the cone may pass");
        assert_eq!(structure.len(), 1, "sliver must be gated to structure");
        assert!(compact[0].cx < 2.1, "compact cluster must be the cone");
    }

    #[test]
    fn gated_far_return_does_not_split_nearby_object() {
        // A background return (beyond the range gate) interleaved between
        // beams on one nearby object is skipped, not a chain breaker.
        let beams = vec![
            (0.0, 3.0),
            (INC, 3.0),
            (2.0 * INC, 6.0), // far background return, gated out
            (3.0 * INC, 3.0),
            (4.0 * INC, 3.0),
        ];
        let (pts, ranges) = project(&beams);
        let clusters = cluster_scan_returns(&pts, &ranges, INC, &ClusterParams::default());
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].point_indices.len(), 4);
        assert!(!clusters[0].point_indices.contains(&2));
    }

    #[test]
    fn wraparound_merge_uses_adaptive_eps() {
        // Object split across the 360-degree seam at 4 m with coarse beam
        // spacing: seam endpoints are ~0.3 m apart — beyond eps_base but
        // within the adaptive eps — so the halves must still merge.
        let inc = 0.07;
        let mut beams: Vec<(f64, f64)> = (0..3).map(|k| (k as f64 * inc, 4.0)).collect();
        beams.extend(cone_beams(2.0, 2.0, 3, inc)); // unrelated middle object
        beams.extend((0..3).map(|k| (std::f64::consts::TAU - (3 - k) as f64 * inc - 0.005, 4.0)));
        let (pts, ranges) = project(&beams);
        let clusters = cluster_scan_returns(&pts, &ranges, inc, &ClusterParams::default());
        assert_eq!(clusters.len(), 2, "seam halves must merge into one");
        assert_eq!(clusters[0].point_indices.len(), 6);
    }
}
