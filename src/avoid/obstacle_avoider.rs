use concord::Geo;
use geo::{Point, Polygon};

use crate::core::{
    Segment, point_distance, point_lerp, point_xy, polygon_aabb, polygon_from_points,
    polygon_open_vertices, segment_end, segment_length, segment_new, segment_start, next_id, points_equal,
};
use crate::field::{Swath, SwathType, create_swath};

#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleAvoider {
    obstacles: Vec<Polygon>,
    inflated_obstacles: Vec<Polygon>,
    inflation_distance: f64,
    datum: Geo,
}

impl ObstacleAvoider {
    pub fn new(obstacles: Vec<Polygon>, datum: Geo) -> Self {
        Self {
            obstacles,
            inflated_obstacles: Vec::new(),
            inflation_distance: 0.0,
            datum,
        }
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
        self.inflated_obstacles = self
            .obstacles
            .iter()
            .filter_map(|obstacle| inflate_polygon(obstacle, self.inflation_distance))
            .collect();
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

            if let Some(next_segment) = current_segments.get(index + 1) {
                let gap = point_distance(segment_end(*segment), segment_start(*next_segment));
                if gap > 0.1 {
                    let mut around = create_swath(
                        segment_end(*segment),
                        segment_start(*next_segment),
                        SwathType::Around,
                        "",
                    );
                    around.points = vec![segment_end(*segment), segment_start(*next_segment)];
                    around.uuid = next_id("around");
                    result.push(around);
                }
            }
        }

        result
    }
}

fn inflate_polygon(polygon: &Polygon, inflation_distance: f64) -> Option<Polygon> {
    let bb = polygon_aabb(polygon)?;
    Some(polygon_from_points(vec![
        point_xy(bb.min().x - inflation_distance, bb.min().y - inflation_distance),
        point_xy(bb.max().x + inflation_distance, bb.min().y - inflation_distance),
        point_xy(bb.max().x + inflation_distance, bb.max().y + inflation_distance),
        point_xy(bb.min().x - inflation_distance, bb.max().y + inflation_distance),
    ]))
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
        let mid = point_lerp(segment_start(segment), segment_end(segment), (t0 + t1) * 0.5);
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
    let ring = polygon_open_vertices(polygon);
    if ring.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = ring.len() - 1;
    for i in 0..ring.len() {
        let pi = ring[i];
        let pj = ring[j];
        let intersects = ((pi.y() > point.y()) != (pj.y() > point.y()))
            && (point.x()
                < (pj.x() - pi.x()) * (point.y() - pi.y())
                    / ((pj.y() - pi.y()).abs().max(1e-12))
                    + pi.x());
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside || ring.iter().any(|vertex| points_equal(*vertex, point, 1e-8))
}

fn cross2(a: Point, b: Point) -> f64 {
    a.x() * b.y() - a.y() * b.x()
}
