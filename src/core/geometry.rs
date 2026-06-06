use std::cmp::Ordering;

pub use datapod::{Aabb, Linestring, Point, Polygon, Segment};

pub trait Point2Ext {
    fn x(&self) -> f64;
    fn y(&self) -> f64;
}

impl Point2Ext for Point {
    fn x(&self) -> f64 {
        self.x
    }

    fn y(&self) -> f64 {
        self.y
    }
}

pub fn point_xy(x: f64, y: f64) -> Point {
    Point::new(x, y, 0.0)
}

pub fn segment_new(start: Point, end: Point) -> Segment {
    Segment::new(start, end)
}

pub fn segment_start(segment: Segment) -> Point {
    segment.start
}

pub fn segment_end(segment: Segment) -> Point {
    segment.end
}

pub fn segment_length(segment: Segment) -> f64 {
    point_distance(segment_start(segment), segment_end(segment))
}

pub fn point_distance(a: Point, b: Point) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    (dx * dx + dy * dy).sqrt()
}

pub fn point_lerp(a: Point, b: Point, t: f64) -> Point {
    point_xy(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

pub fn segment_distance_to_point(segment: Segment, point: Point) -> f64 {
    let a = segment_start(segment);
    let b = segment_end(segment);
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let apx = point.x - a.x;
    let apy = point.y - a.y;
    let denom = abx * abx + aby * aby;
    if denom <= 1e-12 {
        return point_distance(point, a);
    }
    let t = ((apx * abx) + (apy * aby)) / denom;
    let t = t.clamp(0.0, 1.0);
    point_distance(point, point_lerp(a, b, t))
}

pub fn aabb_from_points(points: &[Point]) -> Aabb {
    let mut minx = points[0].x;
    let mut miny = points[0].y;
    let mut maxx = points[0].x;
    let mut maxy = points[0].y;
    for point in points.iter().copied().skip(1) {
        minx = minx.min(point.x);
        miny = miny.min(point.y);
        maxx = maxx.max(point.x);
        maxy = maxy.max(point.y);
    }
    Aabb::new(point_xy(minx, miny), point_xy(maxx, maxy))
}

pub fn aabb_center(aabb: Aabb) -> Point {
    point_xy(
        (aabb.min_point.x + aabb.max_point.x) * 0.5,
        (aabb.min_point.y + aabb.max_point.y) * 0.5,
    )
}

pub fn aabb_width(aabb: Aabb) -> f64 {
    aabb.max_point.x - aabb.min_point.x
}

pub fn aabb_height(aabb: Aabb) -> f64 {
    aabb.max_point.y - aabb.min_point.y
}

pub fn polygon_from_points(mut points: Vec<Point>) -> Polygon {
    if points.first() != points.last() {
        if let Some(first) = points.first().copied() {
            points.push(first);
        }
    }
    Polygon::new(points)
}

pub fn polygon_exterior_points(polygon: &Polygon) -> Vec<Point> {
    polygon.vertices.clone()
}

pub fn polygon_open_vertices(polygon: &Polygon) -> Vec<Point> {
    let mut points = polygon_exterior_points(polygon);
    if points.first() == points.last() {
        points.pop();
    }
    points
}

pub fn polygon_area(polygon: &Polygon) -> f64 {
    let ring = polygon_open_vertices(polygon);
    if ring.len() < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        sum += a.x * b.y - b.x * a.y;
    }
    0.5 * sum
}

pub fn polygon_ensure_ccw(polygon: &Polygon) -> Polygon {
    if polygon_area(polygon) >= 0.0 {
        return polygon.clone();
    }
    let mut points = polygon_open_vertices(polygon);
    points.reverse();
    polygon_from_points(points)
}

pub fn polygon_aabb(polygon: &Polygon) -> Option<Aabb> {
    let ring = polygon_open_vertices(polygon);
    (!ring.is_empty()).then(|| aabb_from_points(&ring))
}

pub fn polygon_is_axis_aligned_rectangle(polygon: &Polygon) -> bool {
    let ring = polygon_open_vertices(polygon);
    if ring.len() != 4 {
        return false;
    }
    let Some(bb) = polygon_aabb(polygon) else {
        return false;
    };
    ring.iter().all(|point| {
        (point.x - bb.min_point.x).abs() < 1e-9
            || (point.x - bb.max_point.x).abs() < 1e-9
            || (point.y - bb.min_point.y).abs() < 1e-9
            || (point.y - bb.max_point.y).abs() < 1e-9
    })
}

pub fn polygon_buffer(polygon: &Polygon, distance: f64) -> Option<Polygon> {
    let normalized = polygon_ensure_ccw(polygon);
    let ring = polygon_open_vertices(&normalized);
    if ring.len() < 3 {
        return None;
    }

    let mut buffered = Vec::with_capacity(ring.len());
    for i in 0..ring.len() {
        let prev = ring[(i + ring.len() - 1) % ring.len()];
        let curr = ring[i];
        let next = ring[(i + 1) % ring.len()];
        buffered.push(compute_miter_offset(prev, curr, next, distance));
    }

    let polygon = polygon_from_points(buffered);
    let area = polygon_area(&polygon);
    if !area.is_finite() {
        return None;
    }
    if distance < 0.0 && area <= 1.0 {
        return None;
    }
    Some(polygon)
}

pub fn polygon_shrink(polygon: &Polygon, distance: f64) -> Option<Polygon> {
    polygon_buffer(polygon, -distance.abs())
}

fn compute_miter_offset(prev: Point, curr: Point, next: Point, distance: f64) -> Point {
    let (mut e1x, mut e1y) = (curr.x - prev.x, curr.y - prev.y);
    let (mut e2x, mut e2y) = (next.x - curr.x, next.y - curr.y);
    normalize_2d(&mut e1x, &mut e1y);
    normalize_2d(&mut e2x, &mut e2y);

    let n1x = e1y;
    let n1y = -e1x;
    let n2x = e2y;
    let n2y = -e2x;

    let mut bx = n1x + n2x;
    let mut by = n1y + n2y;
    let blen = (bx * bx + by * by).sqrt();
    if blen < 1e-10 {
        return point_xy(curr.x + n1x * distance, curr.y + n1y * distance);
    }

    bx /= blen;
    by /= blen;

    let cos_half = n1x * bx + n1y * by;
    let mut miter_length = distance;
    if cos_half.abs() > 1e-10 {
        miter_length = distance / cos_half;
        const MITER_LIMIT: f64 = 2.0;
        if miter_length.abs() > distance.abs() * MITER_LIMIT {
            miter_length = distance.signum() * distance.abs() * MITER_LIMIT * miter_length.signum();
        }
    }

    point_xy(curr.x + bx * miter_length, curr.y + by * miter_length)
}

fn normalize_2d(x: &mut f64, y: &mut f64) {
    let len = (*x * *x + *y * *y).sqrt();
    if len > 1e-10 {
        *x /= len;
        *y /= len;
    }
}

/// Intersection of two polygons assuming the `clipper` is convex
/// (Sutherland-Hodgman). Returns `None` if the intersection is empty or
/// degenerate. Both polygons should be CCW-wound.
pub fn polygon_intersection(subject: &Polygon, clipper: &Polygon) -> Option<Polygon> {
    let subject_verts = polygon_open_vertices(subject);
    let clipper_verts = polygon_open_vertices(clipper);
    if subject_verts.len() < 3 || clipper_verts.len() < 3 {
        return None;
    }

    let mut output: Vec<Point> = subject_verts;
    let clipper_ccw = polygon_ensure_ccw(clipper);
    let clipper_verts = polygon_open_vertices(&clipper_ccw);

    for i in 0..clipper_verts.len() {
        if output.is_empty() {
            break;
        }
        let edge_a = clipper_verts[i];
        let edge_b = clipper_verts[(i + 1) % clipper_verts.len()];
        let input = std::mem::take(&mut output);
        let mut prev = *input.last().unwrap();
        let mut prev_inside = point_is_inside_edge(prev, edge_a, edge_b);
        for curr in input {
            let curr_inside = point_is_inside_edge(curr, edge_a, edge_b);
            if curr_inside {
                if !prev_inside {
                    if let Some(cross) = line_line_intersection(prev, curr, edge_a, edge_b) {
                        output.push(cross);
                    }
                }
                output.push(curr);
            } else if prev_inside {
                if let Some(cross) = line_line_intersection(prev, curr, edge_a, edge_b) {
                    output.push(cross);
                }
            }
            prev = curr;
            prev_inside = curr_inside;
        }
    }

    if output.len() < 3 {
        return None;
    }
    let polygon = polygon_from_points(output);
    if polygon_area(&polygon).abs() < 1e-9 {
        return None;
    }
    Some(polygon)
}

fn point_is_inside_edge(point: Point, edge_a: Point, edge_b: Point) -> bool {
    // CCW polygon: a point is inside (to the left of) edge (a -> b) when the
    // cross product is non-negative.
    let cross =
        (edge_b.x - edge_a.x) * (point.y - edge_a.y) - (edge_b.y - edge_a.y) * (point.x - edge_a.x);
    cross >= -1e-9
}

fn line_line_intersection(p1: Point, p2: Point, p3: Point, p4: Point) -> Option<Point> {
    let denom = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    if denom.abs() < 1e-12 {
        return None;
    }
    let t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / denom;
    Some(point_xy(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y)))
}

pub fn polygon_unique_sorted_intersections_with_line(
    polygon: &Polygon,
    normal: (f64, f64),
    tangent: (f64, f64),
    offset: f64,
) -> Vec<Point> {
    let ring = polygon_open_vertices(polygon);
    if ring.len() < 2 {
        return Vec::new();
    }

    let mut hits: Vec<(f64, Point)> = Vec::new();
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        let fa = a.x * normal.0 + a.y * normal.1 - offset;
        let fb = b.x * normal.0 + b.y * normal.1 - offset;

        if fa.abs() <= 1e-9 && fb.abs() <= 1e-9 {
            hits.push((a.x * tangent.0 + a.y * tangent.1, a));
            hits.push((b.x * tangent.0 + b.y * tangent.1, b));
            continue;
        }

        if (fa > 0.0 && fb > 0.0) || (fa < 0.0 && fb < 0.0) {
            continue;
        }

        let denom = fa - fb;
        if denom.abs() <= 1e-12 {
            continue;
        }

        let t = fa / (fa - fb);
        if !(-1e-9..=1.0 + 1e-9).contains(&t) {
            continue;
        }
        let point = point_lerp(a, b, t.clamp(0.0, 1.0));
        let along = point.x * tangent.0 + point.y * tangent.1;
        hits.push((along, point));
    }

    hits.sort_by(|(a, _), (b, _)| a.partial_cmp(b).unwrap_or(Ordering::Equal));
    let mut unique: Vec<Point> = Vec::new();
    for (_, point) in hits {
        if unique
            .last()
            .is_none_or(|last| point_distance(*last, point) > 1e-6)
        {
            unique.push(point);
        }
    }
    unique
}
