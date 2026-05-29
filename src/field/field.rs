use crate::Geo;

use crate::core::{
    Aabb, MaptraxError, Point, Point2Ext, Polygon, Result, Segment, aabb_from_points, aabb_height,
    aabb_width, next_id, point_distance, points_equal, polygon_aabb, polygon_area,
    polygon_ensure_ccw, polygon_from_points, polygon_is_axis_aligned_rectangle,
    point_xy, polygon_open_vertices, polygon_shrink, remove_colinear_points, segment_end,
    segment_length, segment_new, segment_start,
};

#[derive(Debug, Clone, PartialEq)]
pub struct Ring {
    pub polygon: Polygon,
    pub uuid: String,
    pub finished: bool,
    pub bounding_box: Aabb,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwathType {
    Swath,
    Connection,
    Around,
    Headland,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Swath {
    pub line: Segment,
    pub uuid: String,
    pub r#type: SwathType,
    pub finished: bool,
    pub bounding_box: Aabb,
    pub id: i32,
    pub width: f64,
    pub points: Vec<Point>,
    /// Parallel to `points` when present; `true` means the machine is
    /// reversing at that waypoint. An empty vec means "all forward"
    /// (the common case — swaths, headlands, Dubins, Sharper, straight
    /// connectors are never reverse). Populated only by Reeds-Shepp
    /// connectors that contain reverse segments.
    pub point_reverse: Vec<bool>,
}

impl Swath {
    pub fn head(&self) -> Point {
        segment_start(self.line)
    }

    pub fn tail(&self) -> Point {
        segment_end(self.line)
    }

    pub fn swap_direction(&mut self) {
        self.line = segment_new(segment_end(self.line), segment_start(self.line));
        self.points.reverse();
    }

    pub fn with_swapped_direction(&self) -> Self {
        let mut swapped = self.clone();
        swapped.swap_direction();
        swapped
    }
}

/// Describes a synthetic split line that runs along one side of a Part's
/// boundary. Used by `AutoSplit` decomposition to record where a Part was
/// cut off from its neighbour; non-owning Parts skip the headland inset
/// along this edge so the neighbour's midline headland is shared.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SplitBoundary {
    /// Split line runs vertically at the given x.
    Vertical { x: f64 },
    /// Split line runs horizontally at the given y.
    Horizontal { y: f64 },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Part {
    pub boundary: Ring,
    pub swaths: Vec<Swath>,
    pub headlands: Vec<Ring>,
    /// Split boundaries bordering this Part where the midline headland is
    /// owned by the NEIGHBOUR (not this Part). When generating headlands
    /// and the swath interior, this Part does NOT inset along these
    /// boundaries — swaths extend all the way to the split line so the
    /// machine can turn using the neighbour's headland.
    pub non_owned_splits: Vec<SplitBoundary>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecompositionMode {
    /// Keep the field as one part.
    None,
    /// Cut once through the centroid (vertical axis).
    SimpleSplit,
    /// Cut once through the centroid along the short axis, but only if
    /// the field is concave.
    ConcaveSplit,
    /// Recursively bisect perpendicular to the longer AABB side until
    /// every resulting part's longer dimension is <= `max_side`. Triggers
    /// once the field exceeds the threshold; small fields stay as one part.
    AutoSplit { max_side: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SwathObjective {
    ApproxMinSwathCount,
    ExactSwathCount(usize),
    TotalSwathLength,
    OverlapPenalty,
    CoverageScore,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwathAngleSearchOptions {
    pub start_degrees: f64,
    pub end_degrees: f64,
    pub step_degrees: f64,
}

impl Default for SwathAngleSearchOptions {
    fn default() -> Self {
        Self {
            start_degrees: 1.0,
            end_degrees: 359.0,
            step_degrees: 1.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SwathAngleSearchResult {
    pub angle_degrees: f64,
    pub score: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    border: Polygon,
    parts: Vec<Part>,
    datum: Geo,
    overlap_threshold: f64,
    /// The decomposition that produced `parts`. When this is `AutoSplit`,
    /// headlands are generated once from `border` and shared across all
    /// parts so no internal border appears between the sub-fields.
    decomposition: DecompositionMode,
}

pub fn create_ring(poly: Polygon, uuid: impl Into<String>) -> Result<Ring> {
    let points = polygon_open_vertices(&poly);
    if points.is_empty() {
        return Err(MaptraxError::EmptyPolygon);
    }
    let polygon = polygon_ensure_ccw(&poly);
    let bounding_box = polygon_aabb(&polygon).ok_or(MaptraxError::InvalidPolygon(
        "ring requires at least 3 points",
    ))?;
    Ok(Ring {
        polygon,
        uuid: {
            let value = uuid.into();
            if value.is_empty() {
                next_id("ring")
            } else {
                value
            }
        },
        finished: false,
        bounding_box,
    })
}

pub fn create_swath(
    start: Point,
    end: Point,
    swath_type: SwathType,
    uuid: impl Into<String>,
) -> Swath {
    let points = vec![start, end];
    Swath {
        line: segment_new(start, end),
        uuid: {
            let value = uuid.into();
            if value.is_empty() {
                next_id("swath")
            } else {
                value
            }
        },
        r#type: swath_type,
        finished: false,
        bounding_box: aabb_from_points(&points),
        id: -1,
        width: 0.0,
        points,
        point_reverse: Vec::new(),
    }
}

impl Field {
    pub fn new(border: Polygon, datum: Geo) -> Result<Self> {
        Self::with_options(border, datum, true, 0.5, false)
    }

    pub fn with_options(
        border: Polygon,
        datum: Geo,
        _centred: bool,
        _area_threshold: f64,
        _use_equal_areas: bool,
    ) -> Result<Self> {
        let boundary = create_ring(border, "field_boundary")?;
        let border = boundary.polygon.clone();
        Ok(Self {
            border,
            parts: vec![Part {
                boundary,
                swaths: Vec::new(),
                headlands: Vec::new(),
                non_owned_splits: Vec::new(),
            }],
            datum,
            overlap_threshold: 0.7,
            decomposition: DecompositionMode::None,
        })
    }

    pub fn border(&self) -> &Polygon {
        &self.border
    }

    pub fn get_border(&self) -> &Polygon {
        self.border()
    }

    pub fn parts(&self) -> &[Part] {
        &self.parts
    }

    pub fn get_parts(&self) -> &[Part] {
        self.parts()
    }

    pub fn parts_mut(&mut self) -> &mut [Part] {
        &mut self.parts
    }

    pub fn datum(&self) -> Geo {
        self.datum
    }

    pub fn total_area(&self) -> f64 {
        polygon_area(&self.border).abs()
    }

    pub fn overlap_threshold(&self) -> f64 {
        self.overlap_threshold
    }

    pub fn part(&self, index: usize) -> Result<&Part> {
        self.parts
            .get(index)
            .ok_or(MaptraxError::MissingPart(index))
    }

    pub fn decompose(&mut self, mode: DecompositionMode) -> Result<usize> {
        self.parts = decompose_polygon_into_parts(&self.border, mode)?;
        self.decomposition = mode;
        Ok(self.parts.len())
    }

    pub fn generate(
        &mut self,
        swath_width: f64,
        angle_degrees: f64,
        headland_count: usize,
    ) -> Result<()> {
        self.generate_headlands(swath_width, headland_count)?;
        self.generate_swaths(swath_width, angle_degrees)?;
        Ok(())
    }

    pub fn generate_headlands(&mut self, swath_width: f64, headland_count: usize) -> Result<()> {
        if swath_width <= 0.0 {
            return Err(MaptraxError::InvalidPolygon("swath width must be positive"));
        }

        for part in &mut self.parts {
            part.headlands.clear();
            if headland_count > 0 {
                part.headlands = generate_headlands_with_non_owned_splits(
                    &part.boundary.polygon,
                    swath_width,
                    headland_count,
                    &part.non_owned_splits,
                );
            }
        }

        Ok(())
    }

    pub fn generate_swaths(&mut self, swath_width: f64, angle_degrees: f64) -> Result<()> {
        if swath_width <= 0.0 {
            return Err(MaptraxError::InvalidPolygon("swath width must be positive"));
        }

        for part in &mut self.parts {
            let interior = part
                .headlands
                .last()
                .map(|ring| &ring.polygon)
                .unwrap_or(&part.boundary.polygon);
            part.swaths = generate_swaths_for_polygon(swath_width, angle_degrees, interior);
        }

        Ok(())
    }

    pub fn generate_swaths_with_objective(
        &mut self,
        swath_width: f64,
        objective: SwathObjective,
        options: SwathAngleSearchOptions,
    ) -> Result<Vec<SwathAngleSearchResult>> {
        if swath_width <= 0.0 {
            return Err(MaptraxError::InvalidPolygon("swath width must be positive"));
        }

        let mut results = Vec::with_capacity(self.parts.len());
        for part in &mut self.parts {
            let interior = part
                .headlands
                .last()
                .map(|ring| &ring.polygon)
                .unwrap_or(&part.boundary.polygon);
            let search = search_swath_angle(swath_width, interior, objective, options);
            part.swaths = search.1;
            results.push(search.0);
        }

        Ok(results)
    }

    pub fn generate_with_objective(
        &mut self,
        swath_width: f64,
        objective: SwathObjective,
        options: SwathAngleSearchOptions,
        headland_count: usize,
    ) -> Result<Vec<SwathAngleSearchResult>> {
        self.generate_headlands(swath_width, headland_count)?;
        self.generate_swaths_with_objective(swath_width, objective, options)
    }

    pub fn gen_field(
        &mut self,
        swath_width: f64,
        angle_degrees: f64,
        headland_count: usize,
    ) -> Result<()> {
        self.generate(swath_width, angle_degrees, headland_count)
    }
}

/// Average tangent direction of a swath set (unit-length). Falls back to
/// (1, 0) if the swaths cancel out or are empty.
pub fn dominant_swath_tangent(swaths: &[Swath]) -> (f64, f64) {
    let mut tx = 0.0;
    let mut ty = 0.0;
    for swath in swaths {
        tx += swath.tail().x() - swath.head().x();
        ty += swath.tail().y() - swath.head().y();
    }
    let len = (tx * tx + ty * ty).sqrt();
    if len < 1e-9 {
        (1.0, 0.0)
    } else {
        (tx / len, ty / len)
    }
}

/// Return swath indices sorted in canonical row order. Primary key: lateral
/// offset (centre projected onto the dominant-tangent normal) — this is "which
/// row". Secondary key: along-tangent offset for swaths that share a row.
/// Tertiary key: original index (stable, deterministic).
pub fn canonical_swath_order(swaths: &[Swath]) -> Vec<usize> {
    let tangent = dominant_swath_tangent(swaths);
    let normal = (-tangent.1, tangent.0);
    let mut indexed: Vec<(usize, f64, f64)> = swaths
        .iter()
        .enumerate()
        .map(|(index, swath)| {
            let cx = (swath.head().x() + swath.tail().x()) * 0.5;
            let cy = (swath.head().y() + swath.tail().y()) * 0.5;
            let lateral = cx * normal.0 + cy * normal.1;
            let along = cx * tangent.0 + cy * tangent.1;
            (index, lateral, along)
        })
        .collect();
    indexed.sort_by(|a, b| {
        a.1.partial_cmp(&b.1)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
            .then_with(|| a.0.cmp(&b.0))
    });
    indexed.into_iter().map(|(i, _, _)| i).collect()
}

pub fn generate_headlands_for_polygon(
    border: &Polygon,
    swath_width: f64,
    headland_count: usize,
) -> Vec<Ring> {
    let mut rings = Vec::new();
    let mut current = border.clone();
    for i in 0..headland_count {
        let Some(inset) = inset_polygon(&current, swath_width) else {
            break;
        };
        let Ok(ring) = create_ring(inset.clone(), format!("headland_{}", i + 1)) else {
            break;
        };
        current = inset;
        rings.push(ring);
    }
    rings
}

/// Like `generate_headlands_for_polygon` but for each inset ring, vertices
/// that sit near a non-owned split line are snapped back onto the split.
/// Result: along the split edge the ring has zero inset (the adjacent Part
/// owns the midline headland and swaths here extend to the split line).
fn generate_headlands_with_non_owned_splits(
    border: &Polygon,
    swath_width: f64,
    headland_count: usize,
    non_owned: &[SplitBoundary],
) -> Vec<Ring> {
    let mut rings = Vec::new();
    let mut current = border.clone();
    for i in 0..headland_count {
        let Some(mut inset) = inset_polygon(&current, swath_width) else {
            break;
        };
        if !non_owned.is_empty() {
            let inset_distance = (i + 1) as f64 * swath_width;
            inset = snap_inset_vertices_to_splits(&inset, non_owned, inset_distance);
        }
        let Ok(ring) = create_ring(inset.clone(), format!("headland_{}", i + 1)) else {
            break;
        };
        current = inset;
        rings.push(ring);
    }
    rings
}

/// For each vertex of `inset_polygon`, if it is within ~`inset_distance`
/// (plus a small tolerance) of a non-owned split line, snap that vertex
/// onto the split. This widens the inset polygon so it touches the split
/// line directly — no gap, neighbour's headland is shared.
fn snap_inset_vertices_to_splits(
    inset_polygon: &Polygon,
    non_owned: &[SplitBoundary],
    inset_distance: f64,
) -> Polygon {
    let vertices = polygon_open_vertices(inset_polygon);
    let threshold = inset_distance * 1.2 + 1e-6;
    let snapped: Vec<Point> = vertices
        .into_iter()
        .map(|point| {
            let mut pt = point;
            for boundary in non_owned {
                match *boundary {
                    SplitBoundary::Vertical { x } => {
                        if (pt.x() - x).abs() < threshold {
                            pt = point_xy(x, pt.y());
                        }
                    }
                    SplitBoundary::Horizontal { y } => {
                        if (pt.y() - y).abs() < threshold {
                            pt = point_xy(pt.x(), y);
                        }
                    }
                }
            }
            pt
        })
        .collect();
    polygon_from_points(snapped)
}

fn inset_polygon(polygon: &Polygon, distance: f64) -> Option<Polygon> {
    let bb = polygon_aabb(polygon)?;
    if polygon_is_axis_aligned_rectangle(polygon) {
        let width = aabb_width(bb);
        let height = aabb_height(bb);
        if width <= 2.0 * distance || height <= 2.0 * distance {
            return None;
        }
    }
    let shrunk = polygon_shrink(polygon, distance)?;
    let simplified = remove_colinear_points(&shrunk, 1e-4);
    (polygon_area(&simplified) > 1e-6).then_some(simplified)
}

pub fn generate_swaths_for_polygon(
    swath_width: f64,
    angle_degrees: f64,
    polygon: &Polygon,
) -> Vec<Swath> {
    let Some(bb) = polygon_aabb(polygon) else {
        return Vec::new();
    };

    if angle_degrees == 0.0 {
        return search_swath_angle(
            swath_width,
            polygon,
            SwathObjective::ApproxMinSwathCount,
            SwathAngleSearchOptions::default(),
        )
        .1;
    }

    let angle = angle_degrees.to_radians();
    if aabb_width(bb) <= 1e-9 || aabb_height(bb) <= 1e-9 {
        return Vec::new();
    }

    let centroid = polygon_centroid(polygon);
    let cx = centroid.x();
    let cy = centroid.y();
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    let width = aabb_width(bb);
    let height = aabb_height(bb);
    let line_ext = width.max(height) * 1.5;
    let max_offset = width.max(height) * 0.75;
    let tangent = (cos_a, sin_a);

    let mut swaths = Vec::new();
    let mut swath_id: i32 = 0;
    let mut offset = -max_offset;
    while offset <= max_offset + 1e-9 {
        let x1 = cx + offset * sin_a - line_ext * cos_a;
        let y1 = cy - offset * cos_a - line_ext * sin_a;
        let x2 = cx + offset * sin_a + line_ext * cos_a;
        let y2 = cy - offset * cos_a + line_ext * sin_a;
        let ray = segment_new(point_xy(x1, y1), point_xy(x2, y2));

        for seg in clip_segment_to_polygon(ray, polygon) {
            let start = segment_start(seg);
            let end = segment_end(seg);
            if point_distance(start, end) < swath_width * 0.1 || points_equal(start, end, 1e-6) {
                continue;
            }
            let swath =
                create_indexed_swath(start, end, SwathType::Swath, swath_id, swath_width, tangent);
            swaths.push(swath);
            swath_id += 1;
        }
        offset += swath_width;
    }

    swaths
}

fn search_swath_angle(
    swath_width: f64,
    polygon: &Polygon,
    objective: SwathObjective,
    options: SwathAngleSearchOptions,
) -> (SwathAngleSearchResult, Vec<Swath>) {
    let mut angle = options.start_degrees;
    let mut best: Option<(SwathAngleSearchResult, Vec<Swath>)> = None;
    let step = options.step_degrees.abs().max(1e-6);

    while angle <= options.end_degrees + 1e-9 {
        let swaths = generate_swaths_with_explicit_angle(swath_width, angle, polygon);
        let score = evaluate_swath_objective(swath_width, polygon, &swaths, objective);
        let candidate = (
            SwathAngleSearchResult {
                angle_degrees: angle,
                score,
            },
            swaths,
        );
        let replace = best
            .as_ref()
            .map(|current| {
                objective_better(
                    objective,
                    score,
                    angle,
                    current.0.score,
                    current.0.angle_degrees,
                )
            })
            .unwrap_or(true);
        if replace {
            best = Some(candidate);
        }
        angle += step;
    }

    best.unwrap_or((
        SwathAngleSearchResult {
            angle_degrees: options.start_degrees,
            score: objective_default_score(objective),
        },
        Vec::new(),
    ))
}

fn objective_default_score(objective: SwathObjective) -> f64 {
    match objective {
        SwathObjective::CoverageScore => f64::NEG_INFINITY,
        _ => f64::INFINITY,
    }
}

fn objective_better(
    objective: SwathObjective,
    score: f64,
    angle: f64,
    best_score: f64,
    best_angle: f64,
) -> bool {
    let better = match objective {
        SwathObjective::CoverageScore => score > best_score + 1e-9,
        _ => score < best_score - 1e-9,
    };
    better || ((score - best_score).abs() <= 1e-9 && angle < best_angle)
}

fn evaluate_swath_objective(
    swath_width: f64,
    polygon: &Polygon,
    swaths: &[Swath],
    objective: SwathObjective,
) -> f64 {
    let total_length: f64 = swaths.iter().map(|swath| segment_length(swath.line)).sum();
    let approx_covered_area = total_length * swath_width;
    let polygon_area_abs = polygon_area(polygon).abs().max(1e-9);
    let overlap_penalty = (approx_covered_area - polygon_area_abs).abs() / polygon_area_abs;

    match objective {
        SwathObjective::ApproxMinSwathCount => swaths.len() as f64,
        SwathObjective::ExactSwathCount(target) => {
            (swaths.len() as isize - target as isize).abs() as f64
        }
        SwathObjective::TotalSwathLength => total_length,
        SwathObjective::OverlapPenalty => overlap_penalty,
        SwathObjective::CoverageScore => {
            let count_penalty = swaths.len() as f64 * 1e-3;
            (approx_covered_area / polygon_area_abs) - overlap_penalty - count_penalty
        }
    }
}

fn generate_swaths_with_explicit_angle(
    swath_width: f64,
    angle_degrees: f64,
    polygon: &Polygon,
) -> Vec<Swath> {
    let Some(bb) = polygon_aabb(polygon) else {
        return Vec::new();
    };

    let angle = angle_degrees.to_radians();
    if aabb_width(bb) <= 1e-9 || aabb_height(bb) <= 1e-9 {
        return Vec::new();
    }

    let centroid = polygon_centroid(polygon);
    let cx = centroid.x();
    let cy = centroid.y();
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    let width = aabb_width(bb);
    let height = aabb_height(bb);
    let line_ext = width.max(height) * 1.5;
    let max_offset = width.max(height) * 0.75;
    let tangent = (cos_a, sin_a);

    let mut swaths = Vec::new();
    let mut swath_id: i32 = 0;
    let mut offset = -max_offset;
    while offset <= max_offset + 1e-9 {
        let x1 = cx + offset * sin_a - line_ext * cos_a;
        let y1 = cy - offset * cos_a - line_ext * sin_a;
        let x2 = cx + offset * sin_a + line_ext * cos_a;
        let y2 = cy - offset * cos_a + line_ext * sin_a;
        let ray = segment_new(point_xy(x1, y1), point_xy(x2, y2));

        for seg in clip_segment_to_polygon(ray, polygon) {
            let start = segment_start(seg);
            let end = segment_end(seg);
            if point_distance(start, end) < swath_width * 0.1 || points_equal(start, end, 1e-6) {
                continue;
            }
            let swath =
                create_indexed_swath(start, end, SwathType::Swath, swath_id, swath_width, tangent);
            swaths.push(swath);
            swath_id += 1;
        }
        offset += swath_width;
    }

    swaths
}

fn create_indexed_swath(
    start: Point,
    end: Point,
    swath_type: SwathType,
    swath_id: i32,
    swath_width: f64,
    tangent: (f64, f64),
) -> Swath {
    let (start, end) = orient_segment_points(start, end, tangent);
    let points = vec![start, end];
    Swath {
        line: segment_new(start, end),
        uuid: format!("swath_{}", swath_id + 1),
        r#type: swath_type,
        finished: false,
        bounding_box: aabb_from_points(&points),
        id: swath_id,
        width: swath_width,
        points,
        point_reverse: Vec::new(),
    }
}

fn orient_segment_points(start: Point, end: Point, tangent: (f64, f64)) -> (Point, Point) {
    let dx = end.x() - start.x();
    let dy = end.y() - start.y();
    let dot = dx * tangent.0 + dy * tangent.1;
    if dot < 0.0 {
        (end, start)
    } else {
        (start, end)
    }
}

fn polygon_centroid(polygon: &Polygon) -> Point {
    let verts = polygon_open_vertices(polygon);
    if verts.is_empty() {
        return point_xy(0.0, 0.0);
    }
    if verts.len() == 1 {
        return verts[0];
    }
    if verts.len() == 2 {
        return point_xy(
            (verts[0].x() + verts[1].x()) * 0.5,
            (verts[0].y() + verts[1].y()) * 0.5,
        );
    }

    let mut signed_area = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    for i in 0..verts.len() {
        let j = (i + 1) % verts.len();
        let cross = verts[i].x() * verts[j].y() - verts[j].x() * verts[i].y();
        signed_area += cross;
        cx += (verts[i].x() + verts[j].x()) * cross;
        cy += (verts[i].y() + verts[j].y()) * cross;
    }

    signed_area *= 0.5;
    if signed_area.abs() < 1e-10 {
        let sx: f64 = verts.iter().map(|p| p.x()).sum();
        let sy: f64 = verts.iter().map(|p| p.y()).sum();
        return point_xy(sx / verts.len() as f64, sy / verts.len() as f64);
    }

    let factor = 1.0 / (6.0 * signed_area);
    point_xy(cx * factor, cy * factor)
}

fn clip_segment_to_polygon(segment: Segment, polygon: &Polygon) -> Vec<Segment> {
    let ring = polygon_open_vertices(polygon);
    if ring.len() < 3 {
        return Vec::new();
    }

    #[derive(Clone, Copy)]
    struct Intersection {
        t: f64,
        point: Point,
    }

    let seg_start = segment_start(segment);
    let seg_end = segment_end(segment);
    let seg_dir = point_xy(seg_end.x() - seg_start.x(), seg_end.y() - seg_start.y());
    let mut intersections = Vec::new();

    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        let d2x = b.x() - a.x();
        let d2y = b.y() - a.y();
        let cross = seg_dir.x() * d2y - seg_dir.y() * d2x;
        if cross.abs() < 1e-10 {
            continue;
        }

        let dx = a.x() - seg_start.x();
        let dy = a.y() - seg_start.y();
        let t = (dx * d2y - dy * d2x) / cross;
        let u = (dx * seg_dir.y() - dy * seg_dir.x()) / cross;
        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
            intersections.push(Intersection {
                t,
                point: point_xy(
                    seg_start.x() + t * seg_dir.x(),
                    seg_start.y() + t * seg_dir.y(),
                ),
            });
        }
    }

    intersections.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
    intersections.dedup_by(|a, b| (a.t - b.t).abs() < 1e-9 || points_equal(a.point, b.point, 1e-9));

    let mut result = Vec::new();
    let mut currently_inside = point_in_polygon(seg_start, polygon);
    let mut current_start = seg_start;
    for isect in intersections {
        if currently_inside {
            result.push(segment_new(current_start, isect.point));
            currently_inside = false;
        } else {
            current_start = isect.point;
            currently_inside = true;
        }
    }
    if currently_inside {
        result.push(segment_new(current_start, seg_end));
    }
    result
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

fn decompose_polygon_into_parts(border: &Polygon, mode: DecompositionMode) -> Result<Vec<Part>> {
    let border = polygon_ensure_ccw(border);
    match mode {
        DecompositionMode::None => Ok(vec![Part {
            boundary: create_ring(border, "field_boundary")?,
            swaths: Vec::new(),
            headlands: Vec::new(),
            non_owned_splits: Vec::new(),
        }]),
        DecompositionMode::SimpleSplit => {
            split_polygon_into_parts(&border, choose_split_axis(&border, false))
        }
        DecompositionMode::ConcaveSplit => {
            if polygon_is_concave(&border) {
                split_polygon_into_parts(&border, choose_split_axis(&border, true))
            } else {
                Ok(vec![Part {
                    boundary: create_ring(border, "field_boundary")?,
                    swaths: Vec::new(),
                    headlands: Vec::new(),
                    non_owned_splits: Vec::new(),
                }])
            }
        }
        DecompositionMode::AutoSplit { max_side } => {
            // Recurse with split-ownership tracking: each time we bisect,
            // the "lower" half (smaller coordinate along the split axis)
            // owns the midline, the "upper" half doesn't.
            let entries = auto_split_polygons_tracked(&border, max_side, Vec::new());
            entries
                .into_iter()
                .enumerate()
                .map(|(index, (polygon, non_owned))| {
                    Ok(Part {
                        boundary: create_ring(polygon, format!("part_{index}"))?,
                        swaths: Vec::new(),
                        headlands: Vec::new(),
                        non_owned_splits: non_owned,
                    })
                })
                .collect()
        }
    }
}

/// Recursively bisect `border` perpendicular to its longer AABB side until
/// every resulting polygon's longer side is <= `max_side`. Tracks per-
/// resulting-polygon split ownership: the half with the smaller coordinate
/// along the split axis OWNS the midline headland; the larger-coordinate
/// half records the axis as non-owned (swaths reach the split directly).
fn auto_split_polygons_tracked(
    border: &Polygon,
    max_side: f64,
    inherited_non_owned: Vec<SplitBoundary>,
) -> Vec<(Polygon, Vec<SplitBoundary>)> {
    let Some(bb) = polygon_aabb(border) else {
        return vec![(border.clone(), inherited_non_owned)];
    };
    let width = aabb_width(bb);
    let height = aabb_height(bb);
    let longer = width.max(height);
    if max_side <= 0.0 || longer <= max_side {
        return vec![(border.clone(), inherited_non_owned)];
    }

    let centroid = polygon_centroid(border);
    let axis = if width >= height {
        SplitAxis::Vertical(centroid.x())
    } else {
        SplitAxis::Horizontal(centroid.y())
    };

    let halves = bisect_polygon(border, axis);
    if halves.len() < 2 {
        return vec![(border.clone(), inherited_non_owned)];
    }

    // Sort halves by their centroid along the split axis so the "lower"
    // (smaller-coord) half comes first. That one owns the midline.
    let mut halves_sorted: Vec<Polygon> = halves;
    halves_sorted.sort_by(|a, b| {
        let (ac, bc) = (polygon_centroid(a), polygon_centroid(b));
        match axis {
            SplitAxis::Vertical(_) => ac
                .x()
                .partial_cmp(&bc.x())
                .unwrap_or(std::cmp::Ordering::Equal),
            SplitAxis::Horizontal(_) => ac
                .y()
                .partial_cmp(&bc.y())
                .unwrap_or(std::cmp::Ordering::Equal),
        }
    });

    let split_boundary = match axis {
        SplitAxis::Vertical(x) => SplitBoundary::Vertical { x },
        SplitAxis::Horizontal(y) => SplitBoundary::Horizontal { y },
    };

    let mut out = Vec::new();
    for (index, half) in halves_sorted.into_iter().enumerate() {
        let mut non_owned = inherited_non_owned.clone();
        if index > 0 {
            // Upper half: doesn't own this midline.
            non_owned.push(split_boundary);
        }
        out.extend(auto_split_polygons_tracked(&half, max_side, non_owned));
    }
    out
}

fn bisect_polygon(border: &Polygon, axis: SplitAxis) -> Vec<Polygon> {
    let (first, second) = match axis {
        SplitAxis::Vertical(x) => (
            clip_polygon_half_plane(border, |p| p.x() <= x + 1e-9, AxisBoundary::Vertical(x)),
            clip_polygon_half_plane(border, |p| p.x() >= x - 1e-9, AxisBoundary::Vertical(x)),
        ),
        SplitAxis::Horizontal(y) => (
            clip_polygon_half_plane(border, |p| p.y() <= y + 1e-9, AxisBoundary::Horizontal(y)),
            clip_polygon_half_plane(border, |p| p.y() >= y - 1e-9, AxisBoundary::Horizontal(y)),
        ),
    };
    [first, second]
        .into_iter()
        .flatten()
        .filter(|polygon| polygon_area(polygon).abs() > 1e-6)
        .collect()
}

#[derive(Clone, Copy)]
enum SplitAxis {
    Vertical(f64),
    Horizontal(f64),
}

fn choose_split_axis(polygon: &Polygon, prefer_shape: bool) -> SplitAxis {
    let centroid = polygon_centroid(polygon);
    let bb = polygon_aabb(polygon).expect("polygon with vertices");
    if prefer_shape && aabb_height(bb) > aabb_width(bb) {
        SplitAxis::Horizontal(centroid.y())
    } else {
        SplitAxis::Vertical(centroid.x())
    }
}

fn split_polygon_into_parts(border: &Polygon, axis: SplitAxis) -> Result<Vec<Part>> {
    let (first, second) = match axis {
        SplitAxis::Vertical(x) => (
            clip_polygon_half_plane(border, |p| p.x() <= x + 1e-9, AxisBoundary::Vertical(x)),
            clip_polygon_half_plane(border, |p| p.x() >= x - 1e-9, AxisBoundary::Vertical(x)),
        ),
        SplitAxis::Horizontal(y) => (
            clip_polygon_half_plane(border, |p| p.y() <= y + 1e-9, AxisBoundary::Horizontal(y)),
            clip_polygon_half_plane(border, |p| p.y() >= y - 1e-9, AxisBoundary::Horizontal(y)),
        ),
    };

    let mut polygons = Vec::new();
    for polygon in [first, second].into_iter().flatten() {
        if polygon_area(&polygon).abs() > 1e-6 {
            polygons.push(polygon);
        }
    }

    if polygons.len() < 2 {
        return Ok(vec![Part {
            boundary: create_ring(border.clone(), "field_boundary")?,
            swaths: Vec::new(),
            headlands: Vec::new(),
            non_owned_splits: Vec::new(),
        }]);
    }

    polygons
        .into_iter()
        .enumerate()
        .map(|(index, polygon)| {
            Ok(Part {
                boundary: create_ring(polygon, format!("part_{}", index))?,
                swaths: Vec::new(),
                headlands: Vec::new(),
                non_owned_splits: Vec::new(),
            })
        })
        .collect()
}

#[derive(Clone, Copy)]
enum AxisBoundary {
    Vertical(f64),
    Horizontal(f64),
}

fn clip_polygon_half_plane<F>(
    polygon: &Polygon,
    inside: F,
    boundary: AxisBoundary,
) -> Option<Polygon>
where
    F: Fn(Point) -> bool,
{
    let input = polygon_open_vertices(polygon);
    if input.len() < 3 {
        return None;
    }

    let mut output = Vec::new();
    let mut prev = *input.last().unwrap();
    let mut prev_inside = inside(prev);

    for &current in &input {
        let current_inside = inside(current);
        if current_inside {
            if !prev_inside {
                output.push(intersection_on_axis(prev, current, boundary));
            }
            output.push(current);
        } else if prev_inside {
            output.push(intersection_on_axis(prev, current, boundary));
        }
        prev = current;
        prev_inside = current_inside;
    }

    output = polygon_open_vertices(&remove_colinear_points(&polygon_from_points(output), 1e-6));
    if output.first() == output.last() {
        output.pop();
    }
    (output.len() >= 3).then(|| polygon_from_points(output))
}

fn intersection_on_axis(a: Point, b: Point, boundary: AxisBoundary) -> Point {
    match boundary {
        AxisBoundary::Vertical(x) => {
            let dx = b.x() - a.x();
            if dx.abs() <= 1e-12 {
                point_xy(x, a.y())
            } else {
                let t = (x - a.x()) / dx;
                point_xy(x, a.y() + t * (b.y() - a.y()))
            }
        }
        AxisBoundary::Horizontal(y) => {
            let dy = b.y() - a.y();
            if dy.abs() <= 1e-12 {
                point_xy(a.x(), y)
            } else {
                let t = (y - a.y()) / dy;
                point_xy(a.x() + t * (b.x() - a.x()), y)
            }
        }
    }
}

fn polygon_is_concave(polygon: &Polygon) -> bool {
    let ring = polygon_open_vertices(polygon);
    if ring.len() < 4 {
        return false;
    }

    let mut sign = 0.0;
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        let c = ring[(i + 2) % ring.len()];
        let cross = (b.x() - a.x()) * (c.y() - b.y()) - (b.y() - a.y()) * (c.x() - b.x());
        if cross.abs() <= 1e-9 {
            continue;
        }
        if sign == 0.0 {
            sign = cross.signum();
        } else if cross.signum() != sign {
            return true;
        }
    }
    false
}
