use geo::{Point, Polygon};

use crate::core::{
    aabb_from_points, heading_between, point_distance, polygon_open_vertices, segment_end,
    segment_new, segment_start,
};
use crate::field::{Part, Swath, SwathType, create_swath};
use crate::turners::{Dubins, Pose2D, ReedsShepp, Sharper};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TurnPlannerModel {
    Auto,
    Dubins,
    ReedsShepp,
    Sharper,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TurnPlannerConfig {
    pub model: TurnPlannerModel,
    pub min_turning_radius: f64,
    pub step_size: f64,
    pub machine_length: f64,
    pub machine_width: f64,
    pub sharper_pattern: String,
    pub swath_width: f64,
    pub headland_threshold_rows: f64,
}

impl Default for TurnPlannerConfig {
    fn default() -> Self {
        Self {
            model: TurnPlannerModel::Auto,
            min_turning_radius: 2.0,
            step_size: 0.2,
            machine_length: 6.0,
            machine_width: 0.0,
            sharper_pattern: "auto".to_string(),
            swath_width: 0.0,
            headland_threshold_rows: 2.0,
        }
    }
}

pub struct TourBuilder;

impl TourBuilder {
    pub fn build(part: &Part, ordered_swaths: &[Swath], cfg: &TurnPlannerConfig) -> Vec<Swath> {
        if ordered_swaths.is_empty() {
            return Vec::new();
        }

        let ring = select_headland_ring(part);
        let mut out = Vec::with_capacity(ordered_swaths.len() * 2);

        for (index, swath) in ordered_swaths.iter().enumerate() {
            out.push(swath.clone());
            if let Some(next) = ordered_swaths.get(index + 1) {
                out.extend(connect_between_swaths(swath, next, ring, cfg));
            }
        }

        out
    }
}

fn select_headland_ring(part: &Part) -> &Polygon {
    part.headlands
        .first()
        .map(|ring| &ring.polygon)
        .unwrap_or(&part.boundary.polygon)
}

fn connect_between_swaths(
    from: &Swath,
    to: &Swath,
    headland_ring: &Polygon,
    cfg: &TurnPlannerConfig,
) -> Vec<Swath> {
    let from_end = from.tail();
    let to_start = to.head();

    let swath_width = if cfg.swath_width > 0.0 {
        cfg.swath_width
    } else if from.width > 0.0 {
        from.width
    } else {
        to.width
    };

    let rows = lateral_rows_between(from, to, from_end, to_start, swath_width);
    if rows <= cfg.headland_threshold_rows {
        if let Some(swath) = direct_connection_swath(from, to, cfg) {
            return vec![swath];
        }
        return Vec::new();
    }

    let start_proj = project_to_ring(headland_ring, from_end);
    let goal_proj = project_to_ring(headland_ring, to_start);
    let ring_path = shorter_ring_path(headland_ring, start_proj, goal_proj);

    let mut out = Vec::new();
    if let Some(mut enter) = direct_connection_swath_points(
        from_end,
        heading_between(from.head(), from.tail()),
        start_proj,
        heading_between(start_proj, ring_path.get(1).copied().unwrap_or(start_proj)),
        cfg,
    ) {
        enter.r#type = SwathType::Connection;
        out.push(enter);
    }

    if ring_path.len() >= 2 {
        let mut ring_swath = create_swath(
            ring_path[0],
            *ring_path.last().unwrap(),
            SwathType::Connection,
            "",
        );
        ring_swath.points = ring_path.clone();
        ring_swath.bounding_box = aabb_from_points(&ring_path);
        out.push(ring_swath);
    }

    if let Some(mut exit) = direct_connection_swath_points(
        goal_proj,
        heading_between(
            ring_path
                .get(ring_path.len().saturating_sub(2))
                .copied()
                .unwrap_or(goal_proj),
            goal_proj,
        ),
        to_start,
        heading_between(to.head(), to.tail()),
        cfg,
    ) {
        exit.r#type = SwathType::Connection;
        out.push(exit);
    }

    out
}

fn direct_connection_swath(from: &Swath, to: &Swath, cfg: &TurnPlannerConfig) -> Option<Swath> {
    direct_connection_swath_points(
        from.tail(),
        heading_between(from.head(), from.tail()),
        to.head(),
        heading_between(to.head(), to.tail()),
        cfg,
    )
}

fn direct_connection_swath_points(
    start_point: Point,
    start_yaw: f64,
    goal_point: Point,
    goal_yaw: f64,
    cfg: &TurnPlannerConfig,
) -> Option<Swath> {
    let start = Pose2D::from_point(start_point, start_yaw);
    let goal = Pose2D::from_point(goal_point, goal_yaw);
    let points = match cfg.model {
        TurnPlannerModel::Dubins => {
            Dubins::new(cfg.min_turning_radius)
                .plan_path(start, goal, cfg.step_size)
                .waypoints
        }
        TurnPlannerModel::Sharper => {
            Sharper::new(
                cfg.min_turning_radius,
                cfg.machine_length,
                cfg.machine_width,
            )
            .plan_sharp_turn(start, goal, &cfg.sharper_pattern)
            .waypoints
        }
        TurnPlannerModel::ReedsShepp => {
            ReedsShepp::new(cfg.min_turning_radius)
                .plan_path(start, goal, cfg.step_size)
                .waypoints
        }
        TurnPlannerModel::Auto => {
            if point_distance(start.point, goal.point) < cfg.min_turning_radius * 0.25 {
                Sharper::new(
                    cfg.min_turning_radius,
                    cfg.machine_length,
                    cfg.machine_width,
                )
                .plan_sharp_turn(start, goal, &cfg.sharper_pattern)
                .waypoints
            } else {
                ReedsShepp::new(cfg.min_turning_radius)
                    .plan_path(start, goal, cfg.step_size)
                    .waypoints
            }
        }
    };

    let polyline: Vec<Point> = points.into_iter().map(|pose| pose.point).collect();
    if polyline.len() < 2 {
        return None;
    }

    let mut swath = create_swath(
        polyline[0],
        *polyline.last().unwrap(),
        SwathType::Connection,
        "",
    );
    swath.points = polyline.clone();
    swath.bounding_box = aabb_from_points(&polyline);
    Some(swath)
}

fn lateral_rows_between(
    from: &Swath,
    _to: &Swath,
    from_end: Point,
    to_start: Point,
    swath_width: f64,
) -> f64 {
    if swath_width <= 0.0 {
        return 0.0;
    }
    let dx = from.tail().x() - from.head().x();
    let dy = from.tail().y() - from.head().y();
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-9 {
        return 0.0;
    }
    let nx = -dy / len;
    let ny = dx / len;
    let vx = to_start.x() - from_end.x();
    let vy = to_start.y() - from_end.y();
    (vx * nx + vy * ny).abs() / swath_width
}

fn project_to_ring(ring: &Polygon, point: Point) -> Point {
    let vertices = polygon_open_vertices(ring);
    let mut best = vertices[0];
    let mut best_distance = f64::INFINITY;
    for i in 0..vertices.len() {
        let a = vertices[i];
        let b = vertices[(i + 1) % vertices.len()];
        let seg = segment_new(a, b);
        let start = segment_start(seg);
        let end = segment_end(seg);
        let ax = start.x();
        let ay = start.y();
        let bx = end.x();
        let by = end.y();
        let abx = bx - ax;
        let aby = by - ay;
        let denom = abx * abx + aby * aby;
        let t = if denom <= 1e-12 {
            0.0
        } else {
            (((point.x() - ax) * abx) + ((point.y() - ay) * aby)) / denom
        }
        .clamp(0.0, 1.0);
        let projected = Point::new(ax + abx * t, ay + aby * t);
        let distance = point_distance(point, projected);
        if distance < best_distance {
            best_distance = distance;
            best = projected;
        }
    }
    best
}

fn shorter_ring_path(ring: &Polygon, start: Point, goal: Point) -> Vec<Point> {
    let vertices = polygon_open_vertices(ring);
    let mut base = Vec::with_capacity(vertices.len() + 2);
    base.push(start);
    base.extend_from_slice(&vertices);
    base.push(goal);

    let mut forward = base.clone();
    forward.dedup_by(|a, b| point_distance(*a, *b) < 1e-9);

    let mut backward = vec![start];
    backward.extend(vertices.iter().rev().copied());
    backward.push(goal);
    backward.dedup_by(|a, b| point_distance(*a, *b) < 1e-9);

    let forward_len: f64 = forward
        .windows(2)
        .map(|pair| point_distance(pair[0], pair[1]))
        .sum();
    let backward_len: f64 = backward
        .windows(2)
        .map(|pair| point_distance(pair[0], pair[1]))
        .sum();
    if forward_len <= backward_len {
        forward
    } else {
        backward
    }
}
