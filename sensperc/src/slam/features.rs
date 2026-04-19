/// Feature extraction from 2D LiDAR scans.
///
/// Extracts line segments and corners using split-and-merge algorithm.
/// Features are used for scan-to-scan matching in the SLAM frontend.
use nalgebra::Vector2;

/// A line segment extracted from a LiDAR scan.
#[derive(Clone, Debug)]
pub struct LineSegment {
    pub start: Vector2<f64>,
    pub end: Vector2<f64>,
    pub angle: f64,         // radians, direction of the line
    pub length: f64,        // meters
    pub point_count: usize, // number of LiDAR points on this segment
}

/// A corner (intersection of two line segments).
// `angle` is read via Debug logging and by downstream scan matchers.
#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct Corner {
    pub position: Vector2<f64>,
    pub angle: f64, // angle between the two lines (radians)
}

/// Convert polar LiDAR scan to Cartesian points in robot frame.
pub fn scan_to_points(
    ranges: &[f32],
    angle_min: f32,
    angle_increment: f32,
    range_min: f32,
    range_max: f32,
) -> Vec<Vector2<f64>> {
    ranges.iter().enumerate().filter_map(|(i, &r)| {
        if r >= range_min && r <= range_max {
            let angle = angle_min + i as f32 * angle_increment;
            Some(Vector2::new(
                r as f64 * (angle as f64).cos(),
                r as f64 * (angle as f64).sin(),
            ))
        } else {
            None
        }
    }).collect()
}

/// Extract line segments from Cartesian points using split-and-merge.
pub fn extract_lines(points: &[Vector2<f64>], split_threshold: f64) -> Vec<LineSegment> {
    if points.len() < 2 {
        return vec![];
    }

    let mut segments = Vec::new();
    split_and_merge(points, 0, points.len() - 1, split_threshold, &mut segments);

    // Merge collinear adjacent segments
    merge_segments(&mut segments, 0.15, 0.3); // angle_threshold, gap_threshold

    segments
}

/// Recursive split-and-merge line fitting.
fn split_and_merge(
    points: &[Vector2<f64>],
    start: usize,
    end: usize,
    threshold: f64,
    segments: &mut Vec<LineSegment>,
) {
    if end <= start + 1 {
        return;
    }

    // Find point with maximum distance to the line from start to end
    let line_start = &points[start];
    let line_end = &points[end];
    let line_dir = line_end - line_start;
    let line_len = line_dir.norm();

    if line_len < 1e-6 {
        return;
    }

    let line_unit = line_dir / line_len;
    let line_normal = Vector2::new(-line_unit.y, line_unit.x);

    let mut max_dist = 0.0;
    let mut max_idx = start;

    for i in (start + 1)..end {
        let diff = points[i] - line_start;
        let dist = diff.dot(&line_normal).abs();
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }

    if max_dist > threshold {
        // Split at max distance point
        split_and_merge(points, start, max_idx, threshold, segments);
        split_and_merge(points, max_idx, end, threshold, segments);
    } else {
        // All points are close to the line — create segment
        let angle = line_unit.y.atan2(line_unit.x);
        segments.push(LineSegment {
            start: *line_start,
            end: *line_end,
            angle,
            length: line_len,
            point_count: end - start + 1,
        });
    }
}

/// Merge collinear adjacent segments.
fn merge_segments(segments: &mut Vec<LineSegment>, angle_threshold: f64, gap_threshold: f64) {
    if segments.len() < 2 {
        return;
    }

    let mut merged = Vec::with_capacity(segments.len());
    let mut current = segments[0].clone();

    for next in segments.iter().skip(1) {
        let angle_diff = normalize_angle(next.angle - current.angle).abs();
        let gap = (next.start - current.end).norm();

        if angle_diff < angle_threshold && gap < gap_threshold {
            // Merge: extend current segment to next's end
            current.end = next.end;
            current.length = (current.end - current.start).norm();
            current.point_count += next.point_count;
            current.angle = (current.end - current.start).y.atan2(
                (current.end - current.start).x,
            );
        } else {
            merged.push(current);
            current = next.clone();
        }
    }
    merged.push(current);

    *segments = merged;
}

/// Extract corners from adjacent line segments.
pub fn extract_corners(segments: &[LineSegment], min_angle: f64) -> Vec<Corner> {
    if segments.len() < 2 {
        return vec![];
    }

    let mut corners = Vec::new();

    for i in 0..segments.len() - 1 {
        let s1 = &segments[i];
        let s2 = &segments[i + 1];

        // Check if segments are close (endpoint of s1 near start of s2)
        let gap = (s2.start - s1.end).norm();
        if gap > 0.5 {
            continue;
        }

        let angle_between = normalize_angle(s2.angle - s1.angle).abs();
        if angle_between > min_angle {
            // Corner at the junction
            let position = (s1.end + s2.start) * 0.5;
            corners.push(Corner {
                position,
                angle: angle_between,
            });
        }
    }

    corners
}

fn normalize_angle(a: f64) -> f64 {
    let mut v = a;
    while v > std::f64::consts::PI { v -= 2.0 * std::f64::consts::PI; }
    while v < -std::f64::consts::PI { v += 2.0 * std::f64::consts::PI; }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scan_to_points() {
        // 4 points at 1m, 90 degrees apart
        let ranges = vec![1.0, 1.0, 1.0, 1.0];
        let pts = scan_to_points(&ranges, 0.0, std::f32::consts::FRAC_PI_2, 0.1, 10.0);
        assert_eq!(pts.len(), 4);
        assert!((pts[0].x - 1.0).abs() < 1e-6); // 0 deg
        assert!((pts[1].y - 1.0).abs() < 1e-6); // 90 deg
    }

    #[test]
    fn test_extract_lines_straight_wall() {
        // Points along a straight wall at y=2
        let points: Vec<Vector2<f64>> = (0..20)
            .map(|i| Vector2::new(i as f64 * 0.1, 2.0))
            .collect();

        let lines = extract_lines(&points, 0.05);
        assert_eq!(lines.len(), 1, "Should fit one line segment");
        assert!(lines[0].length > 1.5, "Line should be ~1.9m long");
    }

    #[test]
    fn test_extract_lines_corner() {
        // L-shaped points: wall along x then wall along y
        let mut points: Vec<Vector2<f64>> = (0..10)
            .map(|i| Vector2::new(i as f64 * 0.2, 0.0))
            .collect();
        points.extend((1..10).map(|i| Vector2::new(1.8, i as f64 * 0.2)));

        let lines = extract_lines(&points, 0.05);
        assert!(lines.len() >= 2, "Should find at least 2 segments for L-shape");
    }

    #[test]
    fn test_extract_corners() {
        let segments = vec![
            LineSegment {
                start: Vector2::new(0.0, 0.0),
                end: Vector2::new(1.0, 0.0),
                angle: 0.0, length: 1.0, point_count: 10,
            },
            LineSegment {
                start: Vector2::new(1.0, 0.0),
                end: Vector2::new(1.0, 1.0),
                angle: std::f64::consts::FRAC_PI_2, length: 1.0, point_count: 10,
            },
        ];

        let corners = extract_corners(&segments, 0.3);
        assert_eq!(corners.len(), 1);
        assert!((corners[0].position.x - 1.0).abs() < 0.1);
    }
}
