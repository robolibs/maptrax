use geo::{Point, Polygon};

use crate::core::{
    aabb_from_points, angle_difference, heading_between, point_distance, points_equal,
    polygon_open_vertices, segment_end, segment_new, segment_start,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectorMode {
    Auto,
    Direct,
    Headland,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TurnPlannerConfig {
    pub model: TurnPlannerModel,
    pub connector_mode: ConnectorMode,
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
            connector_mode: ConnectorMode::Auto,
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

        let obstacle_rings = part
            .transit_rings
            .iter()
            .map(|ring| &ring.polygon)
            .collect::<Vec<_>>();
        let field_ring = select_field_ring(part);
        let mut out = Vec::with_capacity(ordered_swaths.len() * 2);

        for (index, swath) in ordered_swaths.iter().enumerate() {
            out.push(swath.clone());
            if let Some(next) = ordered_swaths.get(index + 1) {
                out.extend(connect_between_swaths(
                    swath,
                    next,
                    &obstacle_rings,
                    field_ring,
                    cfg,
                ));
            }
        }

        out
    }
}

fn select_field_ring(part: &Part) -> &Polygon {
    part.headlands
        .first()
        .map(|ring| &ring.polygon)
        .unwrap_or(&part.boundary.polygon)
}

#[derive(Clone)]
struct ConnectorPlan {
    segments: Vec<Swath>,
    cost: f64,
}

fn connect_between_swaths(
    from: &Swath,
    to: &Swath,
    obstacle_rings: &[&Polygon],
    field_ring: &Polygon,
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
    let direct = direct_connection_plan(from, to, cfg);
    let headland = best_transit_connection_plan(from, to, obstacle_rings, field_ring, cfg);

    match cfg.connector_mode {
        ConnectorMode::Direct => {
            return direct
                .or(headland)
                .map(|plan| plan.segments)
                .unwrap_or_default();
        }
        ConnectorMode::Headland => {
            return headland
                .or(direct)
                .map(|plan| plan.segments)
                .unwrap_or_default();
        }
        ConnectorMode::Auto => {}
    }

    if rows <= cfg.headland_threshold_rows {
        return direct
            .or(headland)
            .map(|plan| plan.segments)
            .unwrap_or_default();
    }

    match (direct, headland) {
        (Some(direct), Some(headland)) => {
            if rows >= cfg.headland_threshold_rows * 1.5 && headland.segments.len() >= 2 {
                return headland.segments;
            }
            let direct_cost = direct.cost + rows * swath_width.max(cfg.min_turning_radius);
            if headland.cost <= direct_cost * 1.05 {
                headland.segments
            } else {
                direct.segments
            }
        }
        (Some(plan), None) | (None, Some(plan)) => plan.segments,
        (None, None) => Vec::new(),
    }
}

fn best_transit_connection_plan(
    from: &Swath,
    to: &Swath,
    obstacle_rings: &[&Polygon],
    field_ring: &Polygon,
    cfg: &TurnPlannerConfig,
) -> Option<ConnectorPlan> {
    let direct_gap = segment_new(from.tail(), to.head());

    let obstacle_plan = obstacle_rings
        .iter()
        .filter(|ring| connector_needs_obstacle_ring(direct_gap, ring))
        .filter_map(|ring| headland_connection_plan(from, to, ring, cfg))
        .min_by(|a, b| a.cost.partial_cmp(&b.cost).unwrap_or(std::cmp::Ordering::Equal))
        ;

    obstacle_plan.or_else(|| headland_connection_plan(from, to, field_ring, cfg))
}

fn connector_needs_obstacle_ring(direct_gap: geo::Line<f64>, ring: &Polygon) -> bool {
    segment_intersects_polygon(direct_gap, ring)
        || point_in_polygon(segment_start(direct_gap), ring)
        || point_in_polygon(segment_end(direct_gap), ring)
}

fn direct_connection_plan(
    from: &Swath,
    to: &Swath,
    cfg: &TurnPlannerConfig,
) -> Option<ConnectorPlan> {
    direct_connection_swath_points(
        from.tail(),
        heading_between(from.head(), from.tail()),
        to.head(),
        heading_between(to.head(), to.tail()),
        cfg,
    )
    .map(|swath| ConnectorPlan {
        cost: polyline_length(&swath.points),
        segments: vec![swath],
    })
}

fn headland_connection_plan(
    from: &Swath,
    to: &Swath,
    headland_ring: &Polygon,
    cfg: &TurnPlannerConfig,
) -> Option<ConnectorPlan> {
    let from_end = from.tail();
    let to_start = to.head();
    let start_proj = project_to_ring(headland_ring, from_end)?;
    let goal_proj = project_to_ring(headland_ring, to_start)?;
    let ring_path = shorter_ring_path(headland_ring, start_proj, goal_proj);
    if ring_path.len() < 2 {
        return None;
    }
    let ring_path = smooth_headland_path(&ring_path, cfg);

    let mut out = Vec::new();
    if !points_equal(from_end, start_proj.point, 1e-6) {
        if let Some(mut enter) = direct_connection_swath_points(
            from_end,
            heading_between(from.head(), from.tail()),
            start_proj.point,
            heading_between(
                start_proj.point,
                ring_path.get(1).copied().unwrap_or(start_proj.point),
            ),
            cfg,
        ) {
            enter.r#type = SwathType::Connection;
            out.push(enter);
        }
    }

    let mut ring_swath = create_swath(
        ring_path[0],
        *ring_path.last().unwrap(),
        SwathType::Connection,
        "",
    );
    ring_swath.points = ring_path.clone();
    ring_swath.bounding_box = aabb_from_points(&ring_path);
    out.push(ring_swath);

    if !points_equal(goal_proj.point, to_start, 1e-6) {
        if let Some(mut exit) = direct_connection_swath_points(
            goal_proj.point,
            heading_between(
                ring_path
                    .get(ring_path.len().saturating_sub(2))
                    .copied()
                    .unwrap_or(goal_proj.point),
                goal_proj.point,
            ),
            to_start,
            heading_between(to.head(), to.tail()),
            cfg,
        ) {
            exit.r#type = SwathType::Connection;
            out.push(exit);
        }
    }

    Some(ConnectorPlan {
        cost: out.iter().map(|swath| polyline_length(&swath.points)).sum(),
        segments: out,
    })
}

fn direct_connection_swath_points(
    start_point: Point,
    start_yaw: f64,
    goal_point: Point,
    goal_yaw: f64,
    cfg: &TurnPlannerConfig,
) -> Option<Swath> {
    let distance = point_distance(start_point, goal_point);
    let heading_delta = angle_difference(start_yaw, goal_yaw).abs();
    let tiny_hop = distance <= cfg.min_turning_radius.max(cfg.step_size) * 0.75;
    let near_aligned = heading_delta <= 25.0_f64.to_radians();
    if points_equal(start_point, goal_point, 1e-6) || (tiny_hop && near_aligned) {
        return straight_connection_swath(start_point, goal_point);
    }

    let start = Pose2D::from_point(start_point, start_yaw);
    let goal = Pose2D::from_point(goal_point, goal_yaw);
    let (poses, densify) = match cfg.model {
        TurnPlannerModel::Dubins => (
            Dubins::new(cfg.min_turning_radius)
                .plan_path(start, goal, cfg.step_size)
                .waypoints,
            false,
        ),
        TurnPlannerModel::Sharper => (
            Sharper::new(
                cfg.min_turning_radius,
                cfg.machine_length,
                cfg.machine_width,
            )
            .plan_sharp_turn(start, goal, &cfg.sharper_pattern)
            .waypoints,
            false,
        ),
        TurnPlannerModel::ReedsShepp => (
            ReedsShepp::new(cfg.min_turning_radius)
                .plan_path(start, goal, cfg.step_size)
                .waypoints,
            false,
        ),
        TurnPlannerModel::Auto => {
            if point_distance(start.point, goal.point) < cfg.min_turning_radius * 0.25 {
                (
                    Sharper::new(
                        cfg.min_turning_radius,
                        cfg.machine_length,
                        cfg.machine_width,
                    )
                    .plan_sharp_turn(start, goal, &cfg.sharper_pattern)
                    .waypoints,
                    false,
                )
            } else {
                (
                    ReedsShepp::new(cfg.min_turning_radius)
                        .plan_path(start, goal, cfg.step_size)
                        .waypoints,
                    false,
                )
            }
        }
    };

    let polyline: Vec<Point> = if densify {
        sample_pose_curve(&poses, cfg.step_size.max(0.1))
    } else {
        poses.into_iter().map(|pose| pose.point).collect()
    };
    if polyline.len() < 2 {
        return straight_connection_swath(start_point, goal_point);
    }

    if polyline
        .windows(2)
        .all(|pair| point_distance(pair[0], pair[1]) <= 1e-9)
    {
        return straight_connection_swath(start_point, goal_point);
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

fn straight_connection_swath(start_point: Point, goal_point: Point) -> Option<Swath> {
    if points_equal(start_point, goal_point, 1e-6) {
        return None;
    }
    let mut swath = create_swath(start_point, goal_point, SwathType::Connection, "");
    swath.points = vec![start_point, goal_point];
    swath.bounding_box = aabb_from_points(&swath.points);
    Some(swath)
}

fn sample_pose_curve(poses: &[Pose2D], step_size: f64) -> Vec<Point> {
    if poses.len() < 2 {
        return poses.iter().map(|pose| pose.point).collect();
    }

    let mut out = vec![poses[0].point];
    for pair in poses.windows(2) {
        let a = pair[0];
        let b = pair[1];
        let distance = point_distance(a.point, b.point);
        let steps = ((distance / step_size.max(1e-3)).ceil() as usize).max(4);
        let scale = distance.max(step_size);
        let m0 = (scale * a.yaw.cos(), scale * a.yaw.sin());
        let m1 = (scale * b.yaw.cos(), scale * b.yaw.sin());

        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let t2 = t * t;
            let t3 = t2 * t;
            let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
            let h10 = t3 - 2.0 * t2 + t;
            let h01 = -2.0 * t3 + 3.0 * t2;
            let h11 = t3 - t2;
            let x = h00 * a.point.x() + h10 * m0.0 + h01 * b.point.x() + h11 * m1.0;
            let y = h00 * a.point.y() + h10 * m0.1 + h01 * b.point.y() + h11 * m1.1;
            let point = Point::new(x, y);
            if !points_equal(*out.last().unwrap(), point, 1e-6) {
                out.push(point);
            }
        }
    }
    out
}

fn smooth_headland_path(points: &[Point], cfg: &TurnPlannerConfig) -> Vec<Point> {
    if points.len() < 3 {
        return points.to_vec();
    }

    let mut out = vec![points[0]];
    for i in 1..points.len() - 1 {
        let prev = points[i - 1];
        let corner = points[i];
        let next = points[i + 1];
        let in_len = point_distance(prev, corner);
        let out_len = point_distance(corner, next);
        if in_len <= 1e-6 || out_len <= 1e-6 {
            continue;
        }

        let in_heading = heading_between(prev, corner);
        let out_heading = heading_between(corner, next);
        if angle_difference(in_heading, out_heading).abs() <= 8.0_f64.to_radians() {
            if !points_equal(*out.last().unwrap(), corner, 1e-6) {
                out.push(corner);
            }
            continue;
        }

        let trim = (cfg.min_turning_radius * 0.75)
            .max(cfg.step_size * 4.0)
            .min(in_len * 0.35)
            .min(out_len * 0.35);
        if trim <= 1e-6 {
            if !points_equal(*out.last().unwrap(), corner, 1e-6) {
                out.push(corner);
            }
            continue;
        }

        let corner_in = Point::new(
            corner.x() + (prev.x() - corner.x()) * (trim / in_len),
            corner.y() + (prev.y() - corner.y()) * (trim / in_len),
        );
        let corner_out = Point::new(
            corner.x() + (next.x() - corner.x()) * (trim / out_len),
            corner.y() + (next.y() - corner.y()) * (trim / out_len),
        );

        if !points_equal(*out.last().unwrap(), corner_in, 1e-6) {
            out.push(corner_in);
        }
        if let Some(turn) =
            direct_connection_swath_points(corner_in, in_heading, corner_out, out_heading, cfg)
        {
            for point in turn.points.into_iter().skip(1) {
                if !points_equal(*out.last().unwrap(), point, 1e-6) {
                    out.push(point);
                }
            }
        } else if !points_equal(*out.last().unwrap(), corner_out, 1e-6) {
            out.push(corner_out);
        }
    }

    if !points_equal(*out.last().unwrap(), *points.last().unwrap(), 1e-6) {
        out.push(*points.last().unwrap());
    }
    dedup_polyline(out)
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

#[derive(Clone, Copy)]
struct Projection {
    point: Point,
    seg_idx: usize,
    t: f64,
}

fn project_to_ring(ring: &Polygon, point: Point) -> Option<Projection> {
    let vertices = polygon_open_vertices(ring);
    if vertices.len() < 2 {
        return None;
    }
    let mut best = vertices[0];
    let mut best_distance = f64::INFINITY;
    let mut best_seg_idx = 0usize;
    let mut best_t = 0.0;
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
            best_seg_idx = i;
            best_t = t;
        }
    }
    Some(Projection {
        point: best,
        seg_idx: best_seg_idx,
        t: best_t,
    })
}

fn shorter_ring_path(ring: &Polygon, start: Projection, goal: Projection) -> Vec<Point> {
    let forward = path_along_ring(ring, start, goal, true);
    let backward = path_along_ring(ring, start, goal, false);
    if polyline_length(&forward) <= polyline_length(&backward) {
        dedup_polyline(forward)
    } else {
        dedup_polyline(backward)
    }
}

fn path_along_ring(
    ring: &Polygon,
    start: Projection,
    goal: Projection,
    forward: bool,
) -> Vec<Point> {
    let vertices = polygon_open_vertices(ring);
    let nseg = vertices.len();
    if nseg == 0 {
        return Vec::new();
    }

    let next_seg = |idx: usize| (idx + 1) % nseg;
    let prev_seg = |idx: usize| (idx + nseg - 1) % nseg;

    let mut out = vec![start.point];
    if start.seg_idx == goal.seg_idx
        && ((forward && start.t <= goal.t) || (!forward && start.t >= goal.t))
    {
        out.push(goal.point);
        return out;
    }

    if forward {
        out.push(vertices[next_seg(start.seg_idx)]);
        let mut idx = next_seg(start.seg_idx);
        while idx != goal.seg_idx {
            out.push(vertices[next_seg(idx)]);
            idx = next_seg(idx);
        }
        out.push(goal.point);
    } else {
        out.push(vertices[start.seg_idx]);
        let mut idx = prev_seg(start.seg_idx);
        while idx != goal.seg_idx {
            out.push(vertices[idx]);
            idx = prev_seg(idx);
        }
        out.push(goal.point);
    }
    out
}

fn polyline_length(points: &[Point]) -> f64 {
    points
        .windows(2)
        .map(|pair| point_distance(pair[0], pair[1]))
        .sum()
}

fn segment_intersects_polygon(segment: geo::Line<f64>, polygon: &Polygon) -> bool {
    point_in_polygon(segment_start(segment), polygon)
        || point_in_polygon(segment_end(segment), polygon)
        || !segment_polygon_intersections(segment, polygon).is_empty()
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
                < (pj.x() - pi.x()) * (point.y() - pi.y()) / ((pj.y() - pi.y()).abs().max(1e-12))
                    + pi.x());
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside || ring.iter().any(|vertex| points_equal(*vertex, point, 1e-8))
}

fn segment_polygon_intersections(segment: geo::Line<f64>, polygon: &Polygon) -> Vec<(f64, Point)> {
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

fn segment_intersection_param(a: geo::Line<f64>, b: geo::Line<f64>) -> Option<(f64, Point)> {
    let r = Point::new(
        segment_end(a).x() - segment_start(a).x(),
        segment_end(a).y() - segment_start(a).y(),
    );
    let s = Point::new(
        segment_end(b).x() - segment_start(b).x(),
        segment_end(b).y() - segment_start(b).y(),
    );
    let qp = Point::new(
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
    let point = Point::new(
        segment_start(a).x() + (segment_end(a).x() - segment_start(a).x()) * t,
        segment_start(a).y() + (segment_end(a).y() - segment_start(a).y()) * t,
    );
    Some((t, point))
}

fn cross2(a: Point, b: Point) -> f64 {
    a.x() * b.y() - a.y() * b.x()
}

fn dedup_polyline(mut points: Vec<Point>) -> Vec<Point> {
    points.dedup_by(|a, b| point_distance(*a, *b) < 1e-9);
    points
}
