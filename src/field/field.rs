use concord::Geo;
use geo::{Point, Polygon};

use crate::core::{
    Aabb, MaptraxError, Result, Segment, aabb_from_points, aabb_height, aabb_width, next_id,
    point_distance, points_equal, polygon_aabb, polygon_area, polygon_ensure_ccw,
    polygon_from_points, polygon_is_axis_aligned_rectangle, polygon_open_vertices, polygon_shrink,
    remove_colinear_points, segment_end, segment_length, segment_new, segment_start,
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

#[derive(Debug, Clone, PartialEq)]
pub struct Part {
    pub boundary: Ring,
    pub swaths: Vec<Swath>,
    pub headlands: Vec<Ring>,
    pub transit_rings: Vec<Ring>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecompositionMode {
    None,
    SimpleSplit,
    ConcaveSplit,
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
                transit_rings: Vec::new(),
            }],
            datum,
            overlap_threshold: 0.7,
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
                part.headlands = generate_headlands_for_polygon(
                    &part.boundary.polygon,
                    swath_width,
                    headland_count,
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
        let ray = segment_new(Point::new(x1, y1), Point::new(x2, y2));

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
        let ray = segment_new(Point::new(x1, y1), Point::new(x2, y2));

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
        return Point::new(0.0, 0.0);
    }
    if verts.len() == 1 {
        return verts[0];
    }
    if verts.len() == 2 {
        return Point::new(
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
        return Point::new(sx / verts.len() as f64, sy / verts.len() as f64);
    }

    let factor = 1.0 / (6.0 * signed_area);
    Point::new(cx * factor, cy * factor)
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
    let seg_dir = Point::new(seg_end.x() - seg_start.x(), seg_end.y() - seg_start.y());
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
                point: Point::new(
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
            transit_rings: Vec::new(),
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
                    transit_rings: Vec::new(),
                }])
            }
        }
    }
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
            transit_rings: Vec::new(),
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
                transit_rings: Vec::new(),
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

    output = remove_colinear_points(&polygon_from_points(output), 1e-6)
        .exterior()
        .points()
        .map(|point| Point::new(point.x(), point.y()))
        .collect();
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
                Point::new(x, a.y())
            } else {
                let t = (x - a.x()) / dx;
                Point::new(x, a.y() + t * (b.y() - a.y()))
            }
        }
        AxisBoundary::Horizontal(y) => {
            let dy = b.y() - a.y();
            if dy.abs() <= 1e-12 {
                Point::new(a.x(), y)
            } else {
                let t = (y - a.y()) / dy;
                Point::new(a.x() + t * (b.x() - a.x()), y)
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
