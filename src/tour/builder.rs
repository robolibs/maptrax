use crate::core::{
    Point, Point2Ext, Polygon, Segment, aabb_from_points, angle_difference, heading_between,
    point_distance, point_xy, points_equal, polygon_from_points, polygon_open_vertices, segment_end,
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

/// How much space a turn model needs to connect adjacent rows feasibly.
/// See PLAN.md "Required Turn Space Model". Conservative estimates — tune
/// against visual tests.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurnSpaceRequirement {
    /// Headland band depth (metres) needed for a U-turn between adjacent rows.
    pub min_headland_depth: f64,
    /// Lateral distance (metres) between consecutive passes for a comfortable turn.
    pub min_lateral_row_spacing: f64,
    /// Whether the model can use reverse segments to turn in tighter space.
    pub supports_reverse: bool,
}

/// Policy for reconciling a user's requested headland count with the count the
/// turn model actually needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeadlandSizingPolicy {
    /// Keep the user's count; error if it is below the required count.
    StrictUser,
    /// Keep the user's count, but surface a warning when it is too low.
    WarnOnly,
    /// Use `max(user, required)`.
    AutoIncrease,
}

impl Default for HeadlandSizingPolicy {
    fn default() -> Self {
        Self::WarnOnly
    }
}

/// Why a particular headland count / row-skip stride was chosen for a plan.
#[derive(Debug, Clone, PartialEq)]
pub struct TurnFeasibilityReport {
    pub requested_headland_count: usize,
    pub required_headland_count: usize,
    pub effective_headland_count: usize,
    pub row_skip_stride: usize,
    pub required_headland_depth: f64,
    pub required_lateral_row_spacing: f64,
    pub turn_model: TurnPlannerModel,
    pub warnings: Vec<String>,
}

impl TurnPlannerConfig {
    /// Conservative estimate of the turn space this configuration needs.
    pub fn required_turn_space(&self) -> TurnSpaceRequirement {
        let r = self.min_turning_radius.max(0.0);
        let half_w = 0.5 * self.machine_width.max(0.0);
        match self.model {
            TurnPlannerModel::Dubins => TurnSpaceRequirement {
                min_headland_depth: 2.0 * r + half_w,
                min_lateral_row_spacing: 2.0 * r,
                supports_reverse: false,
            },
            // `Auto` resolves to Reeds-Shepp in open space and Sharper when
            // tight; use the Reeds-Shepp estimate as the conservative middle.
            TurnPlannerModel::ReedsShepp | TurnPlannerModel::Auto => TurnSpaceRequirement {
                min_headland_depth: r + half_w,
                min_lateral_row_spacing: r,
                supports_reverse: true,
            },
            TurnPlannerModel::Sharper => TurnSpaceRequirement {
                min_headland_depth: self.machine_length.max(r) + half_w,
                min_lateral_row_spacing: self.machine_width.max(0.0),
                supports_reverse: true,
            },
        }
    }

    /// Headland rings needed so the U-turn fits inside the reserved band.
    pub fn required_headland_count(&self, swath_width: f64) -> usize {
        required_headland_count(swath_width, self)
    }

    /// Row-skip stride needed so consecutive passes are far enough apart.
    pub fn required_row_skip_stride(&self, swath_width: f64) -> usize {
        required_row_skip_stride(swath_width, self)
    }
}

/// `ceil(required_headland_depth / swath_width)`. Returns 0 for non-positive
/// swath width (nothing meaningful to size against).
pub fn required_headland_count(swath_width: f64, cfg: &TurnPlannerConfig) -> usize {
    if swath_width <= 0.0 {
        return 0;
    }
    let depth = cfg.required_turn_space().min_headland_depth;
    (depth / swath_width).ceil().max(0.0) as usize
}

/// `ceil(required_lateral_row_spacing / swath_width)`, at least 1.
pub fn required_row_skip_stride(swath_width: f64, cfg: &TurnPlannerConfig) -> usize {
    if swath_width <= 0.0 {
        return 1;
    }
    let spacing = cfg.required_turn_space().min_lateral_row_spacing;
    ((spacing / swath_width).ceil() as i64).max(1) as usize
}

/// Combine turn-space sizing with a user request and a policy into a report:
/// the effective headland count to use, the row-skip stride needed to recover
/// any lateral spacing the headland band cannot provide, and any warnings.
pub fn turn_feasibility_report(
    swath_width: f64,
    requested_headland_count: usize,
    cfg: &TurnPlannerConfig,
    policy: HeadlandSizingPolicy,
) -> TurnFeasibilityReport {
    let req = cfg.required_turn_space();
    let required = required_headland_count(swath_width, cfg);
    let mut warnings = Vec::new();

    let effective = match policy {
        HeadlandSizingPolicy::StrictUser => requested_headland_count,
        HeadlandSizingPolicy::WarnOnly => {
            if requested_headland_count < required {
                warnings.push(format!(
                    "requested {requested_headland_count} headlands but {:?} needs {required} \
                     for min_turning_radius={:.2}m (will lean on row skipping)",
                    cfg.model, cfg.min_turning_radius
                ));
            }
            requested_headland_count
        }
        HeadlandSizingPolicy::AutoIncrease => requested_headland_count.max(required),
    };

    // If the effective headland band is shallower than the turn needs, recover
    // the missing space laterally by skipping rows.
    let effective_depth = effective as f64 * swath_width;
    let row_skip_stride = if swath_width <= 0.0 || effective_depth + 1e-9 >= req.min_headland_depth {
        1
    } else {
        required_row_skip_stride(swath_width, cfg)
    };

    TurnFeasibilityReport {
        requested_headland_count,
        required_headland_count: required,
        effective_headland_count: effective,
        row_skip_stride,
        required_headland_depth: req.min_headland_depth,
        required_lateral_row_spacing: req.min_lateral_row_spacing,
        turn_model: cfg.model,
        warnings,
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
        // innermost headland ring (where swaths live) — nor swing outside the
        // field border.
        let work_area = innermost_headland_ring(part);
        let field = Some(&part.boundary.polygon);
        let mut out = Vec::with_capacity(ordered_swaths.len() * 2);

        for (index, swath) in ordered_swaths.iter().enumerate() {
            out.push(swath.clone());
            if let Some(next) = ordered_swaths.get(index + 1) {
                out.extend(connect_between_swaths(
                    swath,
                    next,
                    headland_ring,
                    work_area,
                    field,
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
        let field = Some(&part.boundary.polygon);
        let mut out: Vec<Swath> = Vec::new();

        for arc in headland_arcs {
            if arc.len() < 2 {
                continue;
            }

            if let Some(prev) = out.last() {
                if let Some(conn) = build_connector(prev, arc[0], arc[1], work_area, field, cfg) {
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
    field: Option<&Polygon>,
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
        field,
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
    field: Option<&Polygon>,
    cfg: &TurnPlannerConfig,
) -> Vec<Swath> {
    let headland = headland_connection_plan(from, to, field_ring, work_area, field, cfg);

    match cfg.connector_mode {
        ConnectorMode::Direct => {
            // Tight swath-to-swath turn at the row end (no headland detour).
            // Still prefer the headland route if the direct turn isn't viable.
            let direct = direct_connection_plan(from, to, work_area, field, cfg);
            direct.or(headland).map(|plan| plan.segments).unwrap_or_default()
        }
        ConnectorMode::Headland | ConnectorMode::Auto => {
            // Realistic farming rule: never cut across finished swaths.
            // Always route through the headland band.
            if let Some(plan) = headland {
                plan.segments
            } else {
                // Only fall back to a direct turn when the field has no
                // headlands at all.
                direct_connection_plan(from, to, work_area, field, cfg)
                    .map(|plan| plan.segments)
                    .unwrap_or_default()
            }
        }
    }
}

fn direct_connection_plan(
    from: &Swath,
    to: &Swath,
    work_area: Option<&Polygon>,
    field: Option<&Polygon>,
    cfg: &TurnPlannerConfig,
) -> Option<ConnectorPlan> {
    direct_connection_swath_points(
        from.tail(),
        heading_between(from.head(), from.tail()),
        to.head(),
        heading_between(to.head(), to.tail()),
        work_area,
        field,
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
    field: Option<&Polygon>,
    cfg: &TurnPlannerConfig,
) -> Option<ConnectorPlan> {
    let from_end = from.tail();
    let to_start = to.head();
    // Densify the ring into many interpolated nodes so the connector can attach
    // right next to the row end and follow the band smoothly (a bare polygon
    // ring has only its corner vertices — a rectangle ring = 4 points).
    let dense_ring = densify_polygon(headland_ring, RING_NODE_SPACING_M);
    let start_proj = project_to_ring(&dense_ring, from_end)?;
    let goal_proj = project_to_ring(&dense_ring, to_start)?;
    let ring_path = shorter_ring_path(&dense_ring, start_proj, goal_proj);
    if ring_path.len() < 2 {
        return None;
    }
    let ring_path = smooth_headland_path(&ring_path, cfg);

    let mut out = Vec::new();

    // ENTER: a TURNER-planned maneuver (Dubins / Reeds-Shepp / Sharper) from the
    // row end onto the ring. The turner respects the turning radius, so the turn
    // is drivable — never a hard corner.
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
            field,
            cfg,
        ) {
            enter.r#type = SwathType::Connection;
            out.push(enter);
        }
    }

    // FOLLOW: drive the dense headland ring between the two attach points.
    let mut ring_swath = create_swath(
        ring_path[0],
        *ring_path.last().unwrap(),
        SwathType::Connection,
        "",
    );
    ring_swath.points = ring_path.clone();
    ring_swath.bounding_box = aabb_from_points(&ring_path);
    out.push(ring_swath);

    // EXIT: TURNER-planned maneuver from the ring back into the next row.
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
            field,
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
    field: Option<&Polygon>,
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

    let (polyline, reverse_flags) = select_best_path(candidates, work_area, field)?;
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
    field: Option<&Polygon>,
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
        // A clean turn stays OUT of the work area (no cutting across finished
        // rows) AND INSIDE the field boundary (no swinging off the field — the
        // failure mode for forward-only Dubins turns near the edge).
        let clean_of_work_area = match work_area {
            Some(polygon) => path_stays_outside(&points, polygon),
            None => true,
        };
        let inside_field = match field {
            Some(polygon) => path_stays_inside(&points, polygon),
            None => true,
        };
        if clean_of_work_area && inside_field {
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

/// True if every sampled point of `path` lies inside (or within `tol` of) the
/// polygon boundary. Used to reject turn arcs that swing outside the field.
fn path_stays_inside(path: &[Point], polygon: &Polygon) -> bool {
    const SAMPLE_STEP: f64 = 0.2;
    const TOL: f64 = 0.05;
    for pair in path.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let len = point_distance(a, b);
        let samples = ((len / SAMPLE_STEP).ceil() as usize).max(1);
        for s in 0..=samples {
            let t = s as f64 / samples as f64;
            let p = point_xy(a.x() + t * (b.x() - a.x()), a.y() + t * (b.y() - a.y()));
            if point_outside_polygon(p, polygon, TOL) {
                return false;
            }
        }
    }
    true
}

/// A point is "outside" only if it is neither inside nor within `tol` of the
/// boundary (so points sitting on an edge by construction are not rejected).
fn point_outside_polygon(point: Point, polygon: &Polygon, tol: f64) -> bool {
    if point_in_polygon(point, polygon) {
        return false;
    }
    let ring = polygon_open_vertices(polygon);
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        if segment_distance_to_point(segment_new(a, b), point) <= tol {
            return false;
        }
    }
    true
}

/// Outcome of validating a tour's connector segments against the allowed
/// corridor (inside the field boundary, outside the work area / headland band).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TourValidation {
    /// Connector (turn) segments inspected.
    pub connector_segments: usize,
    /// Connectors with at least one point outside the field boundary.
    pub outside_boundary: usize,
    /// Connectors whose interior cuts through the work area (already-worked rows).
    pub through_work_area: usize,
}

impl TourValidation {
    /// True when no connector leaves the allowed corridor.
    pub fn ok(&self) -> bool {
        self.outside_boundary == 0 && self.through_work_area == 0
    }

    /// Total corridor violations.
    pub fn violations(&self) -> usize {
        self.outside_boundary + self.through_work_area
    }
}

/// Validate that every connector segment in `tour` stays inside `boundary` and
/// outside `work_area` (the headland-band corridor). Only `Connection`/`Around`
/// segments are checked — the work rows themselves are not connectors.
pub fn validate_tour(
    tour: &[Swath],
    boundary: &Polygon,
    work_area: Option<&Polygon>,
) -> TourValidation {
    let mut result = TourValidation::default();
    for swath in tour {
        if swath.r#type != SwathType::Connection && swath.r#type != SwathType::Around {
            continue;
        }
        result.connector_segments += 1;
        let points: Vec<Point> = if swath.points.len() >= 2 {
            swath.points.clone()
        } else {
            vec![swath.head(), swath.tail()]
        };
        // A point sitting on the boundary edge (e.g. clamped there) is on the
        // field, not outside it — only count points genuinely beyond the edge.
        if points.iter().any(|p| point_outside_polygon(*p, boundary, 0.06)) {
            result.outside_boundary += 1;
        }
        if let Some(area) = work_area {
            if !path_stays_outside(&points, area) {
                result.through_work_area += 1;
            }
        }
    }
    result
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

/// Spacing for interpolated headland nodes. Dense nodes give connectors many
/// attach points along the band so turns stay inside the field.
const RING_NODE_SPACING_M: f64 = 0.5;

/// Return a copy of `poly` with extra vertices interpolated along every edge,
/// no farther apart than `step`. Turns the sparse corner-only ring into a dense
/// point set the connectors can follow smoothly.
fn densify_polygon(poly: &Polygon, step: f64) -> Polygon {
    let verts = polygon_open_vertices(poly);
    if verts.len() < 2 || step <= 0.0 {
        return poly.clone();
    }
    let mut out: Vec<Point> = Vec::new();
    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        out.push(a);
        let dist = point_distance(a, b);
        let n = (dist / step).floor() as usize;
        for k in 1..n {
            let t = k as f64 / n as f64;
            out.push(point_xy(a.x() + (b.x() - a.x()) * t, a.y() + (b.y() - a.y()) * t));
        }
    }
    polygon_from_points(out)
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
        // Even-odd ray cast. The edge straddles the horizontal line at
        // `point.y()` exactly when `(pi.y() > y) != (pj.y() > y)`, which also
        // guarantees `pj.y() - pi.y() != 0`. The crossing x must use the
        // *signed* dy — taking its absolute value flips the result for
        // downward edges and misclassifies points.
        if (pi.y() > point.y()) != (pj.y() > point.y()) {
            let cross_x =
                (pj.x() - pi.x()) * (point.y() - pi.y()) / (pj.y() - pi.y()) + pi.x();
            if point.x() < cross_x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside || ring.iter().any(|vertex| points_equal(*vertex, point, 1e-8))
}

fn dedup_polyline(mut points: Vec<Point>) -> Vec<Point> {
    points.dedup_by(|a, b| point_distance(*a, *b) < 1e-9);
    points
}

#[cfg(test)]
mod turn_space_tests {
    use super::*;

    fn cfg(model: TurnPlannerModel, radius: f64, length: f64, width: f64) -> TurnPlannerConfig {
        TurnPlannerConfig {
            model,
            min_turning_radius: radius,
            machine_length: length,
            machine_width: width,
            ..TurnPlannerConfig::default()
        }
    }

    #[test]
    fn dubins_needs_the_most_headland() {
        // depth = 2r + w/2 = 16, ceil(16/3) = 6
        let c = cfg(TurnPlannerModel::Dubins, 8.0, 6.0, 0.0);
        assert_eq!(required_headland_count(3.0, &c), 6);
        // lateral spacing = 2r = 16, ceil(16/3) = 6
        assert_eq!(required_row_skip_stride(3.0, &c), 6);
        assert!(!c.required_turn_space().supports_reverse);
    }

    #[test]
    fn reeds_shepp_needs_less() {
        // depth = r = 8, ceil(8/3) = 3
        let c = cfg(TurnPlannerModel::ReedsShepp, 8.0, 6.0, 0.0);
        assert_eq!(required_headland_count(3.0, &c), 3);
        // lateral spacing = r = 9, ceil(9/3) = 3
        let c2 = cfg(TurnPlannerModel::ReedsShepp, 9.0, 6.0, 0.0);
        assert_eq!(required_row_skip_stride(3.0, &c2), 3);
        assert!(c.required_turn_space().supports_reverse);
    }

    #[test]
    fn sharper_uses_machine_length_and_width() {
        // depth = max(len=6, r=8) + w/2 = 9.5, ceil(9.5/3) = 4
        let c = cfg(TurnPlannerModel::Sharper, 8.0, 6.0, 3.0);
        assert_eq!(required_headland_count(3.0, &c), 4);
        // lateral spacing = width = 3, ceil(3/3) = 1
        assert_eq!(required_row_skip_stride(3.0, &c), 1);
    }

    #[test]
    fn auto_matches_reeds_shepp() {
        let auto = cfg(TurnPlannerModel::Auto, 8.0, 6.0, 0.0);
        let rs = cfg(TurnPlannerModel::ReedsShepp, 8.0, 6.0, 0.0);
        assert_eq!(auto.required_turn_space(), rs.required_turn_space());
    }

    #[test]
    fn warn_only_keeps_count_but_skips_rows() {
        let c = cfg(TurnPlannerModel::Dubins, 8.0, 6.0, 0.0);
        let report = turn_feasibility_report(3.0, 2, &c, HeadlandSizingPolicy::WarnOnly);
        assert_eq!(report.required_headland_count, 6);
        assert_eq!(report.effective_headland_count, 2);
        assert!(!report.warnings.is_empty());
        // 2 headlands (6 m) < 16 m needed → recover laterally via stride 6.
        assert_eq!(report.row_skip_stride, 6);
    }

    #[test]
    fn auto_increase_grows_headlands_and_drops_stride() {
        let c = cfg(TurnPlannerModel::Dubins, 8.0, 6.0, 0.0);
        let report = turn_feasibility_report(3.0, 2, &c, HeadlandSizingPolicy::AutoIncrease);
        assert_eq!(report.effective_headland_count, 6);
        // 6 headlands (18 m) >= 16 m needed → no row skipping required.
        assert_eq!(report.row_skip_stride, 1);
        assert!(report.warnings.is_empty());
    }

    #[test]
    fn strict_user_keeps_request_untouched() {
        let c = cfg(TurnPlannerModel::Dubins, 8.0, 6.0, 0.0);
        let report = turn_feasibility_report(3.0, 2, &c, HeadlandSizingPolicy::StrictUser);
        assert_eq!(report.effective_headland_count, 2);
    }
}
