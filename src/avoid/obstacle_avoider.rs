use concord::Geo;
use geo::{Contains, Point, Polygon};

use crate::core::{
    Segment, aabb_from_points, next_id, point_distance, point_lerp, point_xy, points_equal,
    polygon_buffer, polygon_open_vertices, segment_distance_to_point, segment_end, segment_length,
    segment_new, segment_start,
};
use crate::field::{Swath, SwathType, create_swath};

#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleAvoider {
    obstacles: Vec<Polygon>,
    inflated_obstacles: Vec<Polygon>,
    transit_obstacles: Vec<Polygon>,
    inflation_distance: f64,
    datum: Geo,
    field_boundary: Option<Polygon>,
}

impl ObstacleAvoider {
    pub fn new(obstacles: Vec<Polygon>, datum: Geo) -> Self {
        Self {
            obstacles,
            inflated_obstacles: Vec::new(),
            transit_obstacles: Vec::new(),
            inflation_distance: 0.0,
            datum,
            field_boundary: None,
        }
    }

    pub fn set_field_boundary(&mut self, boundary: Polygon) {
        self.field_boundary = Some(boundary);
    }

    pub fn obstacles(&self) -> &[Polygon] {
        &self.obstacles
    }

    pub fn get_obstacles(&self) -> &[Polygon] {
        self.obstacles()
    }

    pub fn inflated_obstacles(&self) -> &[Polygon] {
        &self.inflated_obstacles
    }

    pub fn get_inflated_obstacles(&self) -> &[Polygon] {
        self.inflated_obstacles()
    }

    pub fn transit_obstacles(&self) -> &[Polygon] {
        &self.transit_obstacles
    }

    pub fn datum(&self) -> Geo {
        self.datum
    }

    pub fn avoid(&mut self, input_swaths: &[Swath], inflation_distance: f64) -> Vec<Swath> {
        self.inflation_distance = inflation_distance;
        self.inflate_obstacles();

        let mut result = Vec::new();
        for swath in input_swaths {
            result.extend(self.process_swath(swath));
        }
        result
    }

    fn inflate_obstacles(&mut self) {
        self.inflated_obstacles.clear();
        self.transit_obstacles.clear();

        for obstacle in &self.obstacles {
            let Some(inflated) = inflate_polygon(obstacle, self.inflation_distance) else {
                continue;
            };
            let touches_boundary = self
                .field_boundary
                .as_ref()
                .is_some_and(|boundary| polygon_touches_boundary(&inflated, boundary, 1e-6));
            self.inflated_obstacles.push(inflated.clone());
            if !touches_boundary {
                self.transit_obstacles.push(inflated);
            }
        }
    }

    fn process_swath(&self, swath: &Swath) -> Vec<Swath> {
        let intersects = self
            .inflated_obstacles
            .iter()
            .any(|obstacle| segment_intersects_polygon(swath.line, obstacle));
        if !intersects {
            return vec![swath.clone()];
        }
        self.cut_swath_around_obstacles(swath)
    }

    fn cut_swath_around_obstacles(&self, swath: &Swath) -> Vec<Swath> {
        let mut current_segments = vec![swath.line];
        for obstacle in &self.inflated_obstacles {
            let mut next_segments = Vec::new();
            for segment in current_segments {
                next_segments.extend(difference_segment_polygon(segment, obstacle));
            }
            current_segments = next_segments;
        }

        let mut result = Vec::new();
        for (index, segment) in current_segments.iter().enumerate() {
            if segment_length(*segment) < 0.1 {
                continue;
            }
            let start = segment_start(*segment);
            let end = segment_end(*segment);
            let mut cut_swath = create_swath(start, end, swath.r#type, "");
            cut_swath.id = swath.id;
            cut_swath.width = swath.width;
            cut_swath.points = vec![start, end];
            result.push(cut_swath);
        }

        result
    }
}

fn polygon_touches_boundary(polygon: &Polygon, boundary: &Polygon, tol: f64) -> bool {
    let boundary_ring = polygon_open_vertices(boundary);
    let obstacle_ring = polygon_open_vertices(polygon);
    if boundary_ring.len() < 2 || obstacle_ring.is_empty() {
        return false;
    }

    obstacle_ring.iter().any(|point| point_near_polygon_boundary(*point, boundary, tol))
        || obstacle_ring.iter().any(|point| point_in_polygon(*point, boundary) && point_near_polygon_boundary(*point, boundary, tol))
        || (0..boundary_ring.len()).any(|i| {
            let edge = segment_new(boundary_ring[i], boundary_ring[(i + 1) % boundary_ring.len()]);
            segment_intersects_polygon(edge, polygon)
        })
}

fn inflate_polygon(polygon: &Polygon, inflation_distance: f64) -> Option<Polygon> {
    polygon_buffer(polygon, inflation_distance.max(0.0))
}

fn segment_intersects_polygon(segment: Segment, polygon: &Polygon) -> bool {
    point_in_polygon(segment_start(segment), polygon)
        || point_in_polygon(segment_end(segment), polygon)
        || !segment_polygon_intersections(segment, polygon).is_empty()
}

fn difference_segment_polygon(segment: Segment, polygon: &Polygon) -> Vec<Segment> {
    let mut params: Vec<f64> = vec![0.0_f64, 1.0_f64];
    for (t, _) in segment_polygon_intersections(segment, polygon) {
        if (1e-9..=1.0 - 1e-9).contains(&t) {
            params.push(t);
        }
    }
    params.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    params.dedup_by(|a, b| (*a - *b).abs() < 1e-9);

    let mut result = Vec::new();
    for window in params.windows(2) {
        let t0 = window[0];
        let t1 = window[1];
        if (t1 - t0).abs() < 1e-9 {
            continue;
        }
        let mid = point_lerp(
            segment_start(segment),
            segment_end(segment),
            (t0 + t1) * 0.5,
        );
        if point_in_polygon(mid, polygon) {
            continue;
        }
        result.push(segment_new(
            point_lerp(segment_start(segment), segment_end(segment), t0),
            point_lerp(segment_start(segment), segment_end(segment), t1),
        ));
    }
    result
}

fn segment_polygon_intersections(segment: Segment, polygon: &Polygon) -> Vec<(f64, Point)> {
    let ring = polygon_open_vertices(polygon);
    let mut hits: Vec<(f64, Point)> = Vec::new();
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        if let Some((t, point)) = segment_intersection_param(segment, segment_new(a, b)) {
            if hits.iter().all(|(existing_t, existing_point)| {
                (*existing_t - t).abs() > 1e-8 || !points_equal(*existing_point, point, 1e-8)
            }) {
                hits.push((t, point));
            }
        }
    }
    hits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    hits
}

fn segment_intersection_param(a: Segment, b: Segment) -> Option<(f64, Point)> {
    let r = point_xy(
        segment_end(a).x() - segment_start(a).x(),
        segment_end(a).y() - segment_start(a).y(),
    );
    let s = point_xy(
        segment_end(b).x() - segment_start(b).x(),
        segment_end(b).y() - segment_start(b).y(),
    );
    let qp = point_xy(
        segment_start(b).x() - segment_start(a).x(),
        segment_start(b).y() - segment_start(a).y(),
    );

    let rxs = cross2(r, s);
    let qpxr = cross2(qp, r);

    if rxs.abs() < 1e-12 && qpxr.abs() < 1e-12 {
        return None;
    }
    if rxs.abs() < 1e-12 {
        return None;
    }

    let t = cross2(qp, s) / rxs;
    let u = cross2(qp, r) / rxs;
    if !(-1e-9..=1.0 + 1e-9).contains(&t) || !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }

    let t = t.clamp(0.0, 1.0);
    let point = point_lerp(segment_start(a), segment_end(a), t);
    Some((t, point))
}

fn point_in_polygon(point: Point, polygon: &Polygon) -> bool {
    polygon.contains(&point)
        || polygon_open_vertices(polygon)
            .iter()
            .any(|vertex| points_equal(*vertex, point, 1e-8))
}

fn cross2(a: Point, b: Point) -> f64 {
    a.x() * b.y() - a.y() * b.x()
}

fn boundary_detour_points(polygon: &Polygon, start: Point, end: Point) -> Option<Vec<Point>> {
    let ring = polygon_open_vertices(polygon);
    if ring.len() < 3 {
        return None;
    }

    let start_edge = find_boundary_edge(&ring, start)?;
    let end_edge = find_boundary_edge(&ring, end)?;

    let forward = walk_boundary(&ring, start, start_edge, end, end_edge, 1);
    let backward = walk_boundary(&ring, start, start_edge, end, end_edge, -1);

    let forward_len = polyline_length(&forward);
    let backward_len = polyline_length(&backward);
    Some(if forward_len <= backward_len {
        dedup_polyline(forward)
    } else {
        dedup_polyline(backward)
    })
}

fn walk_boundary(
    ring: &[Point],
    start: Point,
    start_edge: usize,
    end: Point,
    end_edge: usize,
    direction: isize,
) -> Vec<Point> {
    if start_edge == end_edge {
        return vec![start, end];
    }

    let len = ring.len() as isize;
    let mut points = vec![start];
    let mut edge = start_edge as isize;
    while edge != end_edge as isize {
        let vertex = if direction > 0 {
            ring[((edge + 1).rem_euclid(len)) as usize]
        } else {
            ring[edge.rem_euclid(len) as usize]
        };
        points.push(vertex);
        edge = (edge + direction).rem_euclid(len);
    }
    points.push(end);
    points
}

fn find_boundary_edge(ring: &[Point], point: Point) -> Option<usize> {
    (0..ring.len()).find(|&i| {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        point_on_segment(point, a, b, 1e-5)
    })
}

fn point_on_segment(point: Point, a: Point, b: Point, tol: f64) -> bool {
    let seg = segment_new(a, b);
    if segment_distance_to_point(seg, point) > tol {
        return false;
    }

    let min_x = a.x().min(b.x()) - tol;
    let max_x = a.x().max(b.x()) + tol;
    let min_y = a.y().min(b.y()) - tol;
    let max_y = a.y().max(b.y()) + tol;
    (min_x..=max_x).contains(&point.x()) && (min_y..=max_y).contains(&point.y())
}

fn polyline_length(points: &[Point]) -> f64 {
    points
        .windows(2)
        .map(|pair| point_distance(pair[0], pair[1]))
        .sum()
}

fn dedup_polyline(mut points: Vec<Point>) -> Vec<Point> {
    points.dedup_by(|a, b| points_equal(*a, *b, 1e-8));
    points
}

fn polygon_boundary_distance(point: Point, polygon: &Polygon) -> f64 {
    let ring = polygon_open_vertices(polygon);
    if ring.is_empty() {
        return f64::INFINITY;
    }

    let mut best = f64::INFINITY;
    for i in 0..ring.len() {
        let seg = segment_new(ring[i], ring[(i + 1) % ring.len()]);
        best = best.min(segment_distance_to_point(seg, point));
    }
    best
}

fn point_near_polygon_boundary(point: Point, polygon: &Polygon, tol: f64) -> bool {
    polygon_boundary_distance(point, polygon) <= tol
}
