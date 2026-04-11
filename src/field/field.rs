use concord::Geo;
use geo::{Point, Polygon};

use crate::core::{
    Aabb, MaptraxError, Result, Segment, aabb_from_points, aabb_height, aabb_width, next_id,
    point_distance, points_equal, polygon_aabb, polygon_area, polygon_ensure_ccw,
    polygon_is_axis_aligned_rectangle, polygon_open_vertices, polygon_shrink,
    remove_colinear_points, segment_end, segment_new, segment_start,
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
    let bounding_box = polygon_aabb(&polygon)
        .ok_or(MaptraxError::InvalidPolygon("ring requires at least 3 points"))?;
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
        let boundary = create_ring(border.clone(), "")?;
        Ok(Self {
            border,
            parts: vec![Part {
                boundary,
                swaths: Vec::new(),
                headlands: Vec::new(),
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

    pub fn generate(
        &mut self,
        swath_width: f64,
        angle_degrees: f64,
        headland_count: usize,
    ) -> Result<()> {
        if swath_width <= 0.0 {
            return Err(MaptraxError::InvalidPolygon("swath width must be positive"));
        }

        for part in &mut self.parts {
            part.headlands.clear();
            part.swaths.clear();
            if headland_count > 0 {
                part.headlands =
                    generate_headlands(&part.boundary.polygon, swath_width, headland_count);
            }
            let interior = part
                .headlands
                .last()
                .map(|ring| &ring.polygon)
                .unwrap_or(&part.boundary.polygon)
                .clone();
            part.swaths = generate_swaths(swath_width, angle_degrees, &interior);
        }

        Ok(())
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

fn generate_headlands(border: &Polygon, swath_width: f64, headland_count: usize) -> Vec<Ring> {
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

fn generate_swaths(swath_width: f64, angle_degrees: f64, polygon: &Polygon) -> Vec<Swath> {
    let Some(bb) = polygon_aabb(polygon) else {
        return Vec::new();
    };

    if angle_degrees == 0.0 {
        let mut best_out = Vec::new();
        let mut best_count = usize::MAX;
        for deg in 1..360 {
            let out = generate_swaths(swath_width, deg as f64, polygon);
            if out.len() < best_count {
                best_count = out.len();
                best_out = out;
            }
        }
        return best_out;
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
            if point_distance(start, end) < swath_width * 0.1
                || points_equal(start, end, 1e-6)
            {
                continue;
            }
            let mut swath = create_swath(start, end, SwathType::Swath, "");
            swath.id = swath_id;
            swath.width = swath_width;
            swath.points = vec![start, end];
            swaths.push(swath);
            swath_id += 1;
        }
        offset += swath_width;
    }

    swaths
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
                point: Point::new(seg_start.x() + t * seg_dir.x(), seg_start.y() + t * seg_dir.y()),
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
