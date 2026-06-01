use crate::core::{
    Point, Point2Ext, Polygon, Segment, aabb_from_points, angle_difference, heading_between,
    point_distance, point_xy, points_equal, polygon_open_vertices, segment_end, segment_new,
    segment_start,
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
            // Default to headland routing. Realistic farming always transits
            // through the headland band between rows — never directly across
            // already-worked swaths.
            connector_mode: ConnectorMode::Headland,
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

        let headland_ring = outer_headland_ring(part);
        // Turns must not cut through the work area — the region inside the
        // innermost headland ring (where swaths live).
        let work_area = innermost_headland_ring(part);
        let mut out = Vec::with_capacity(ordered_swaths.len() * 2);

        for (index, swath) in ordered_swaths.iter().enumerate() {
            out.push(swath.clone());
            if let Some(next) = ordered_swaths.get(index + 1) {
                out.extend(connect_between_swaths(
                    swath,
                    next,
                    headland_ring,
                    work_area,
                    cfg,
                ));
            }
        }

        out
    }

    /// Build a complete machine tour that INCLUDES driving the assigned
    /// headland rings as work, before transitioning to the interior swaths.
    /// Output sequence:
    ///   1. For each headland arc (outermost → inner), drive it as a
    ///      `SwathType::Headland` segment with a connector between arcs.
    ///   2. Connector from last headland arc to the first interior swath.
    ///   3. Interior swaths (same as `build`) with their normal connectors.
    ///
    /// Falls back to `build` when no headland arcs are supplied.
    pub fn build_with_headlands(
        part: &Part,
        headland_arcs: &[Vec<Point>],
        ordered_swaths: &[Swath],
        cfg: &TurnPlannerConfig,
    ) -> Vec<Swath> {
        if headland_arcs.is_empty() {
            return Self::build(part, ordered_swaths, cfg);
        }

        let work_area = innermost_headland_ring(part);
        let mut out: Vec<Swath> = Vec::new();

        for arc in headland_arcs {
            if arc.len() < 2 {
                continue;
            }

            if let Some(prev) = out.last() {
                if let Some(conn) = build_connector(prev, arc[0], arc[1], work_area, cfg) {
                    out.push(conn);
                }
            }

            let mut swath = create_swath(
                arc[0],
                *arc.last().unwrap(),
                SwathType::Headland,
                String::new(),
            );
            swath.points = arc.clone();
            swath.bounding_box = aabb_from_points(arc);
            out.push(swath);
        }

        // Hand-off from the last driven headland ring to the first interior
        // swath. A direct Dubins arc can fly straight across the field when
        // the headland endpoint and the swath head sit on opposite sides of
        // the innermost ring. Walk along the innermost ring instead — that
        // path stays on the boundary and can never cut through the crop.
        if let (Some(last_hdl), Some(first_sw)) = (out.last().cloned(), ordered_swaths.first()) {
            let from_end = last_hdl.tail();
            let to_start = first_sw.head();
            if !points_equal(from_end, to_start, 1e-6) {
                let inner_polygon: &Polygon = part
                    .headlands
                    .last()
                    .map(|ring| &ring.polygon)
                    .unwrap_or(&part.boundary.polygon);
                if let (Some(start_proj), Some(goal_proj)) = (
                    project_to_ring(inner_polygon, from_end),
                    project_to_ring(inner_polygon, to_start),
                ) {
                    let ring_path = shorter_ring_path(inner_polygon, start_proj, goal_proj);
                    let mut ring_path = smooth_headland_path(&ring_path, cfg);
                    // Force endpoints to match the actual previous segment's
                    // tail and next swath's head so the tour stays gap-free.
                    if let Some(first) = ring_path.first().copied() {
                        if !points_equal(first, from_end, 1e-6) {
                            ring_path.insert(0, from_end);
                        }
                    } else {
                        ring_path.push(from_end);
                    }
                    if let Some(last) = ring_path.last().copied() {
                        if !points_equal(last, to_start, 1e-6) {
                            ring_path.push(to_start);
                        }
                    }
                    if ring_path.len() >= 2 {
                        let mut ring_swath = create_swath(
                            ring_path[0],
                            *ring_path.last().unwrap(),
                            SwathType::Connection,
                            String::new(),
                        );
                        ring_swath.bounding_box = aabb_from_points(&ring_path);
                        ring_swath.points = ring_path;
                        out.push(ring_swath);
                    }
                }
            }
        }

        out.extend(Self::build(part, ordered_swaths, cfg));
        out
    }
}

fn build_connector(
    prev: &Swath,
    next_start: Point,
    next_second: Point,
    work_area: Option<&Polygon>,
    cfg: &TurnPlannerConfig,
) -> Option<Swath> {
    if points_equal(prev.tail(), next_start, 1e-6) {
        return None;
    }
    let from_heading = if prev.points.len() >= 2 {
        heading_between(
            prev.points[prev.points.len() - 2],
            *prev.points.last().unwrap(),
        )
    } else {
        heading_between(prev.head(), prev.tail())
    };
    let to_heading = heading_between(next_start, next_second);
    let mut connector = direct_connection_swath_points(
        prev.tail(),
        from_heading,
        next_start,
        to_heading,
        work_area,
        cfg,
    )?;
    connector.r#type = SwathType::Connection;
    Some(connector)
}

/// The headland ring that connectors route along when transitioning
/// between swaths. Needs to be FARTHER from the swath endpoints than the
/// innermost ring — otherwise the enter/exit Dubins arcs collapse to
/// zero and you get no visible turn geometry. Rule:
///   * 0 rings           → field boundary
///   * 1-2 rings         → outermost (`headlands.first()`)
///   * 3 or more rings   → middle (`headlands[len / 2]`)
/// With 3+ rings the middle option gives a shorter, more realistic
/// detour than the outermost while still leaving turn room.
fn outer_headland_ring(part: &Part) -> &Polygon {
    if part.headlands.is_empty() {
        return &part.boundary.polygon;
    }
    if part.headlands.len() >= 3 {
        return &part.headlands[part.headlands.len() / 2].polygon;
    }
    &part.headlands[0].polygon
}

/// Innermost headland ring — the boundary of the work area. Turn arcs must
/// stay OUTSIDE this polygon (i.e., in the headland band, not crossing
/// already-worked swaths). Returns None when no headlands exist.
fn innermost_headland_ring(part: &Part) -> Option<&Polygon> {
    part.headlands.last().map(|ring| &ring.polygon)
}

#[derive(Clone)]
struct ConnectorPlan {
    segments: Vec<Swath>,
}

fn connect_between_swaths(
    from: &Swath,
    to: &Swath,
    field_ring: &Polygon,
    work_area: Option<&Polygon>,
    cfg: &TurnPlannerConfig,
) -> Vec<Swath> {
    let headland = headland_connection_plan(from, to, field_ring, work_area, cfg);

    match cfg.connector_mode {
        ConnectorMode::Direct => {
            // Rare: caller explicitly wants a tight swath-to-swath turn and
            // accepts that it may pass across finished rows when no headland
            // detour is possible. Still prefer the headland route if it is
            // available.
            let direct = direct_connection_plan(from, to, work_area, cfg);
            return direct
                .or(headland)
                .map(|plan| plan.segments)
                .unwrap_or_default();
        }
        ConnectorMode::Headland | ConnectorMode::Auto => {
            // Realistic farming rule: never cut across finished swaths.
            // Always route through the headland band.
            if let Some(plan) = headland {
                return plan.segments;
            }
            // Only fall back to a direct turn when the field has no
            // headlands at all.
            return direct_connection_plan(from, to, work_area, cfg)
                .map(|plan| plan.segments)
                .unwrap_or_default();
        }
    }
}

fn direct_connection_plan(
    from: &Swath,
    to: &Swath,
    work_area: Option<&Polygon>,
    cfg: &TurnPlannerConfig,
) -> Option<ConnectorPlan> {
    direct_connection_swath_points(
        from.tail(),
        heading_between(from.head(), from.tail()),
        to.head(),
        heading_between(to.head(), to.tail()),
        work_area,
        cfg,
    )
    .map(|swath| ConnectorPlan {
        segments: vec![swath],
    })
}

fn headland_connection_plan(
    from: &Swath,
    to: &Swath,
    headland_ring: &Polygon,
    work_area: Option<&Polygon>,
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
            work_area,
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
            work_area,
            cfg,
        ) {
            exit.r#type = SwathType::Connection;
            out.push(exit);
        }
    }

    Some(ConnectorPlan { segments: out })
}

fn direct_connection_swath_points(
    start_point: Point,
    start_yaw: f64,
    goal_point: Point,
    goal_yaw: f64,
    work_area: Option<&Polygon>,
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

    // Enumerate candidate (waypoints, reverse_flags) pairs. For
    // Dubins/Sharper everything is forward (reverse_flags = all false).
    // For Reeds-Shepp the planner reports per-waypoint reverse state.
    let candidates: Vec<(Vec<Point>, Vec<bool>)> = match cfg.model {
        TurnPlannerModel::Dubins => Dubins::new(cfg.min_turning_radius)
            .get_all_paths(start, goal, cfg.step_size)
            .into_iter()
            .filter(|path| !path.waypoints.is_empty())
            .map(|path| {
                let pts: Vec<Point> = path.waypoints.into_iter().map(|p| p.point).collect();
                let n = pts.len();
                (pts, vec![false; n])
            })
            .collect(),
        TurnPlannerModel::ReedsShepp => ReedsShepp::new(cfg.min_turning_radius)
            .get_all_paths(start, goal, cfg.step_size)
            .into_iter()
            .filter(|path| !path.waypoints.is_empty())
            .map(|path| {
                let pts: Vec<Point> = path.waypoints.into_iter().map(|p| p.point).collect();
                let mut rev = path.waypoint_reverse;
                if rev.len() != pts.len() {
                    rev.resize(pts.len(), false);
                }
                (pts, rev)
            })
            .collect(),
        TurnPlannerModel::Sharper => {
            let waypoints = Sharper::new(
                cfg.min_turning_radius,
                cfg.machine_length,
                cfg.machine_width,
            )
            .plan_sharp_turn(start, goal, &cfg.sharper_pattern)
            .waypoints;
            let pts: Vec<Point> = waypoints.into_iter().map(|p| p.point).collect();
            let n = pts.len();
            vec![(pts, vec![false; n])]
        }
        TurnPlannerModel::Auto => {
            if point_distance(start.point, goal.point) < cfg.min_turning_radius * 0.25 {
                let waypoints = Sharper::new(
                    cfg.min_turning_radius,
                    cfg.machine_length,
                    cfg.machine_width,
                )
                .plan_sharp_turn(start, goal, &cfg.sharper_pattern)
                .waypoints;
                let pts: Vec<Point> = waypoints.into_iter().map(|p| p.point).collect();
                let n = pts.len();
                vec![(pts, vec![false; n])]
            } else {
                ReedsShepp::new(cfg.min_turning_radius)
                    .get_all_paths(start, goal, cfg.step_size)
                    .into_iter()
                    .filter(|path| !path.waypoints.is_empty())
                    .map(|path| {
                        let pts: Vec<Point> = path.waypoints.into_iter().map(|p| p.point).collect();
                        let mut rev = path.waypoint_reverse;
                        if rev.len() != pts.len() {
                            rev.resize(pts.len(), false);
                        }
                        (pts, rev)
                    })
                    .collect()
            }
        }
    };

    let (polyline, reverse_flags) = select_best_path(candidates, work_area)?;
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
    swath.point_reverse = reverse_flags;
    swath.points = polyline.clone();
    swath.bounding_box = aabb_from_points(&polyline);
    Some(swath)
}

/// Pick the shortest candidate path that keeps its interior OUTSIDE the work
/// area. Start and end points are allowed to lie on the work-area boundary
/// (swaths start/end there). If no candidate is clean, fall back to the
/// shortest one so we always return something.
fn select_best_path(
    candidates: Vec<(Vec<Point>, Vec<bool>)>,
    work_area: Option<&Polygon>,
) -> Option<(Vec<Point>, Vec<bool>)> {
    if candidates.is_empty() {
        return None;
    }

    let mut clean: Vec<(Vec<Point>, Vec<bool>)> = Vec::new();
    let mut all: Vec<(Vec<Point>, Vec<bool>)> = Vec::new();
    for (points, reverse) in candidates {
        if points.len() < 2 {
            continue;
        }
        let clean_of_work_area = match work_area {
            Some(polygon) => path_stays_outside(&points, polygon),
            None => true,
        };
        if clean_of_work_area {
            clean.push((points.clone(), reverse.clone()));
        }
        all.push((points, reverse));
    }

    let pool = if clean.is_empty() { all } else { clean };
    pool.into_iter().min_by(|(a, _), (b, _)| {
        polyline_length(a)
            .partial_cmp(&polyline_length(b))
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

/// Check whether `path` stays out of `polygon` — tested along each segment,
/// not just at the waypoints. The caller's endpoints (first and last
/// points) may sit on the polygon boundary (swath endpoints sit on the
/// ring edge by construction) so those are skipped; everything between
/// is sampled at `SAMPLE_STEP` intervals and rejected if strictly inside.
fn path_stays_outside(path: &[Point], polygon: &Polygon) -> bool {
    if path.len() < 2 {
        return true;
    }
    const SAMPLE_STEP: f64 = 0.1;
    let last_idx = path.len() - 1;
    for (i, pair) in path.windows(2).enumerate() {
        let a = pair[0];
        let b = pair[1];
        let len = point_distance(a, b);
        let samples = ((len / SAMPLE_STEP).ceil() as usize).max(1);
        // Sample the interior of each segment (skip endpoints; the next
        // segment's endpoint check covers them).
        for s in 1..samples {
            let t = s as f64 / samples as f64;
            let p = point_xy(a.x() + t * (b.x() - a.x()), a.y() + t * (b.y() - a.y()));
            if point_strictly_inside(p, polygon) {
                return false;
            }
        }
        // Check `b` unless it's the final endpoint of the whole path.
        if i + 1 < last_idx && point_strictly_inside(b, polygon) {
            return false;
        }
    }
    true
}

/// Like `point_in_polygon` but rejects points exactly on the boundary so the
/// swath endpoints (which sit on the ring) don't count as inside.
fn point_strictly_inside(point: Point, polygon: &Polygon) -> bool {
    if !point_in_polygon(point, polygon) {
        return false;
    }
    // Reject boundary hits.
    let ring = polygon_open_vertices(polygon);
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        let seg = segment_new(a, b);
        if segment_distance_to_point(seg, point) < 1e-6 {
            return false;
        }
    }
    true
}

fn segment_distance_to_point(segment: Segment, point: Point) -> f64 {
    let a = segment_start(segment);
    let b = segment_end(segment);
    let abx = b.x() - a.x();
    let aby = b.y() - a.y();
    let denom = abx * abx + aby * aby;
    if denom <= 1e-12 {
        return point_distance(point, a);
    }
    let t = (((point.x() - a.x()) * abx) + ((point.y() - a.y()) * aby)) / denom;
    let t = t.clamp(0.0, 1.0);
    let projected = point_xy(a.x() + abx * t, a.y() + aby * t);
    point_distance(point, projected)
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

        let corner_in = point_xy(
            corner.x() + (prev.x() - corner.x()) * (trim / in_len),
            corner.y() + (prev.y() - corner.y()) * (trim / in_len),
        );
        let corner_out = point_xy(
            corner.x() + (next.x() - corner.x()) * (trim / out_len),
            corner.y() + (next.y() - corner.y()) * (trim / out_len),
        );

        if !points_equal(*out.last().unwrap(), corner_in, 1e-6) {
            out.push(corner_in);
        }
        if let Some(turn) = direct_connection_swath_points(
            corner_in,
            in_heading,
            corner_out,
            out_heading,
            None,
            cfg,
        ) {
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
        let projected = point_xy(ax + abx * t, ay + aby * t);
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

fn dedup_polyline(mut points: Vec<Point>) -> Vec<Point> {
    points.dedup_by(|a, b| point_distance(*a, *b) < 1e-9);
    points
}
