/// Feature-based scan matching.
///
/// Matches line segments between consecutive scans to estimate
/// rigid body transformation (dx, dy, dtheta).
use nalgebra::{Matrix2, Vector2};

use super::features::LineSegment;

/// Result of scan matching: relative motion between two scans.
#[derive(Clone, Debug)]
pub struct MatchResult {
    pub dx: f64,
    pub dy: f64,
    pub dtheta: f64,
    pub confidence: f32, // [0.0, 1.0] based on match quality
    pub num_matches: usize,
}

impl Default for MatchResult {
    fn default() -> Self {
        Self {
            dx: 0.0,
            dy: 0.0,
            dtheta: 0.0,
            confidence: 0.0,
            num_matches: 0,
        }
    }
}

/// Match two sets of line segments and compute the rigid transform.
///
/// Uses closest-line matching by angle + midpoint distance,
/// then SVD-based point alignment on matched segment endpoints.
pub fn match_scans(
    prev_lines: &[LineSegment],
    curr_lines: &[LineSegment],
    max_angle_diff: f64,
    max_dist: f64,
) -> MatchResult {
    if prev_lines.is_empty() || curr_lines.is_empty() {
        return MatchResult::default();
    }

    // Find matching pairs: for each current line, find closest previous line
    let mut src_points = Vec::new();
    let mut dst_points = Vec::new();

    for curr in curr_lines {
        let curr_mid = (curr.start + curr.end) * 0.5;

        let mut best_dist = f64::INFINITY;
        let mut best_prev: Option<&LineSegment> = None;

        for prev in prev_lines {
            let angle_diff = normalize_angle(curr.angle - prev.angle).abs();
            if angle_diff > max_angle_diff {
                continue;
            }

            let prev_mid = (prev.start + prev.end) * 0.5;
            let dist = (curr_mid - prev_mid).norm();
            if dist < best_dist && dist < max_dist {
                best_dist = dist;
                best_prev = Some(prev);
            }
        }

        if let Some(prev) = best_prev {
            // Use midpoints of matched segments as correspondence
            src_points.push((curr.start + curr.end) * 0.5);
            dst_points.push((prev.start + prev.end) * 0.5);

            // Also use endpoints for better rotation estimation
            src_points.push(curr.start);
            dst_points.push(prev.start);
            src_points.push(curr.end);
            dst_points.push(prev.end);
        }
    }

    let num_matches = src_points.len() / 3; // each match contributes 3 point pairs

    if num_matches < 2 {
        return MatchResult {
            confidence: 0.1,
            num_matches,
            ..Default::default()
        };
    }

    // Compute rigid transform via SVD (point-to-point)
    let (dx, dy, dtheta) = compute_rigid_transform(&src_points, &dst_points);

    // Confidence based on number and quality of matches
    let confidence = ((num_matches as f32) / (curr_lines.len().max(1) as f32)).min(1.0) * 0.8;

    MatchResult {
        dx,
        dy,
        dtheta,
        confidence,
        num_matches,
    }
}

/// Compute rigid transform (R, t) that maps src points to dst points.
/// Uses SVD decomposition: dst = R * src + t
fn compute_rigid_transform(src: &[Vector2<f64>], dst: &[Vector2<f64>]) -> (f64, f64, f64) {
    if src.len() != dst.len() || src.len() < 2 {
        return (0.0, 0.0, 0.0);
    }

    let n = src.len() as f64;

    // Compute centroids
    let src_centroid: Vector2<f64> = src.iter().sum::<Vector2<f64>>() / n;
    let dst_centroid: Vector2<f64> = dst.iter().sum::<Vector2<f64>>() / n;

    // Build covariance matrix H = sum((src_i - src_c) * (dst_i - dst_c)^T)
    let mut h = Matrix2::zeros();
    for (s, d) in src.iter().zip(dst.iter()) {
        let ps = s - src_centroid;
        let pd = d - dst_centroid;
        h += ps * pd.transpose();
    }

    // SVD of H
    let svd = h.svd(true, true);
    let u = svd.u.unwrap();
    let v_t = svd.v_t.unwrap();

    // Rotation matrix R = V * U^T
    let mut r = v_t.transpose() * u.transpose();

    // Ensure proper rotation (det(R) = 1, not -1)
    if r.determinant() < 0.0 {
        let mut v = v_t.transpose();
        v.column_mut(1).neg_mut();
        r = v * u.transpose();
    }

    // Translation t = dst_centroid - R * src_centroid
    let t = dst_centroid - r * src_centroid;

    // Extract rotation angle
    let dtheta = r[(1, 0)].atan2(r[(0, 0)]);

    (t.x, t.y, dtheta)
}

fn normalize_angle(a: f64) -> f64 {
    let mut v = a;
    while v > std::f64::consts::PI {
        v -= 2.0 * std::f64::consts::PI;
    }
    while v < -std::f64::consts::PI {
        v += 2.0 * std::f64::consts::PI;
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_wall(x_start: f64, x_end: f64, y: f64) -> LineSegment {
        let start = Vector2::new(x_start, y);
        let end = Vector2::new(x_end, y);
        LineSegment {
            start,
            end,
            angle: 0.0,
            length: (end - start).norm(),
            point_count: 10,
        }
    }

    #[test]
    fn test_match_identical_scans() {
        let lines = vec![make_wall(0.0, 2.0, 3.0), make_wall(0.0, 2.0, -3.0)];

        let result = match_scans(&lines, &lines, 0.3, 2.0);
        assert!(result.dx.abs() < 0.01);
        assert!(result.dy.abs() < 0.01);
        assert!(result.dtheta.abs() < 0.01);
        assert!(result.num_matches >= 2);
    }

    #[test]
    fn test_match_translated_scan() {
        let prev = vec![make_wall(0.0, 2.0, 3.0), make_wall(0.0, 2.0, -3.0)];
        // Shift current scan by (0.5, 0) — robot moved forward 0.5m
        let curr = vec![make_wall(0.5, 2.5, 3.0), make_wall(0.5, 2.5, -3.0)];

        let result = match_scans(&prev, &curr, 0.3, 2.0);
        // The transform should detect ~0.5m shift
        assert!(result.num_matches >= 2);
        assert!(result.confidence > 0.3);
    }

    #[test]
    fn test_rigid_transform_translation() {
        let src = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(0.0, 1.0),
        ];
        let dst = vec![
            Vector2::new(0.5, 0.3),
            Vector2::new(1.5, 0.3),
            Vector2::new(0.5, 1.3),
        ];
        let (dx, dy, dtheta) = compute_rigid_transform(&src, &dst);
        assert!((dx - 0.5).abs() < 0.01);
        assert!((dy - 0.3).abs() < 0.01);
        assert!(dtheta.abs() < 0.01);
    }
}
