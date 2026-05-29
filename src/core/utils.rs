use std::sync::atomic::{AtomicU64, Ordering};

use super::{
    Point, Point2Ext, Polygon, point_distance, polygon_from_points, polygon_open_vertices,
    segment_distance_to_point, segment_new,
};

static NEXT_ID: AtomicU64 = AtomicU64::new(1);

pub(crate) fn next_id(prefix: &str) -> String {
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    format!("{prefix}_{id}")
}

pub fn float_to_byte(v: f32) -> u8 {
    float_to_byte_in_range(v, 0.0, 255.0)
}

pub fn float_to_byte_in_range(v: f32, min: f32, max: f32) -> u8 {
    let scaled = v.clamp(0.0, 1.0) * 255.0;
    scaled.clamp(min, max).round() as u8
}

pub fn point_to_line_distance(point: Point, line_start: Point, line_end: Point) -> f64 {
    segment_distance_to_point(segment_new(line_start, line_end), point)
}

pub fn angle_between(v1: Point, v2: Point) -> f64 {
    let v1_mag = (v1.x() * v1.x() + v1.y() * v1.y()).sqrt();
    let v2_mag = (v2.x() * v2.x() + v2.y() * v2.y()).sqrt();
    if v1_mag <= 1e-10 || v2_mag <= 1e-10 {
        return 0.0;
    }
    let dot = ((v1.x() * v2.x()) + (v1.y() * v2.y())) / (v1_mag * v2_mag);
    dot.clamp(-1.0, 1.0).acos()
}

pub fn are_colinear(p1: Point, p2: Point, p3: Point, epsilon: f64) -> bool {
    let area =
        (p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()))
            * 0.5;
    area.abs() < epsilon
}

pub fn remove_colinear_points(polygon: &Polygon, epsilon: f64) -> Polygon {
    let points = polygon_open_vertices(polygon);
    if points.len() < 4 {
        return polygon.clone();
    }

    let mut result = Vec::new();
    for i in 0..points.len() {
        let prev = points[(i + points.len() - 1) % points.len()];
        let curr = points[i];
        let next = points[(i + 1) % points.len()];
        if !are_colinear(prev, curr, next, epsilon) {
            result.push(curr);
        }
    }

    polygon_from_points(result)
}

pub fn points_equal(p1: Point, p2: Point, epsilon: f64) -> bool {
    point_distance(p1, p2) < epsilon
}

pub fn heading_between(from: Point, to: Point) -> f64 {
    (to.y() - from.y()).atan2(to.x() - from.x())
}

pub fn normalize_angle(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }
    while angle < -std::f64::consts::PI {
        angle += 2.0 * std::f64::consts::PI;
    }
    angle
}

pub fn angle_difference(from: f64, to: f64) -> f64 {
    normalize_angle(to - from)
}
