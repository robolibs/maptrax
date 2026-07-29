//! GeoJSON export built on the sibling `vectory` crate.
//!
//! Maptrax plans in a local ENU frame anchored at the field datum. `vectory`
//! already owns the GeoJSON encoding and the ENU -> WGS84 conversion (through
//! `concord`), so this module only maps maptrax domain types onto a
//! [`vectory::Vector`] and hands it over.
//!
//! Every emitted element carries a `type` property (vectory's element kind):
//!
//! | kind            | geometry   | emitted for                              |
//! |-----------------|------------|------------------------------------------|
//! | `field`         | Polygon    | the field border (vectory writes this)   |
//! | `part_boundary` | Polygon    | each decomposed part                     |
//! | `headland`      | Polygon    | each headland ring of each part          |
//! | `swath`         | LineString | generated or ordered work rows           |
//! | `tour`          | LineString | a full drive path, connectors included   |
//! | `headland_arc`  | LineString | per-machine headland assignments         |

use std::collections::HashMap;
use std::path::Path;

use crate::core::{Point, Polygon, points_equal, polygon_exterior_points};
use crate::division::HeadlandArc;
use crate::facade::{PlannedField, PlannedFieldStages, PlannedMachines, tour_polyline};
use crate::field::{Field, Part, Ring, Swath, SwathType};
use crate::{MaptraxError, Result};

pub use vectory::{Crs, Element, Geometry, Vector};

/// Which layers land in the exported document.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GeoJsonOptions {
    /// Emit one polygon per decomposed part. Redundant when the field was
    /// never decomposed, since the single part matches the field border.
    pub include_part_boundaries: bool,
    /// Emit each headland ring as its own polygon.
    pub include_headlands: bool,
    /// Emit work rows as linestrings. For the planned exports these are the
    /// *ordered* rows and carry an `order` property.
    pub include_swaths: bool,
    /// Emit each tour as a single flattened drive path.
    pub include_tours: bool,
    /// Coordinate reference system to write. [`Crs::Wgs`] converts through the
    /// field datum; [`Crs::Enu`] writes the raw local metres.
    pub crs: Crs,
}

impl Default for GeoJsonOptions {
    fn default() -> Self {
        Self {
            include_part_boundaries: true,
            include_headlands: true,
            include_swaths: true,
            include_tours: true,
            crs: Crs::Wgs,
        }
    }
}

impl GeoJsonOptions {
    /// Border and headlands only — no rows, no drive path.
    pub fn geometry_only() -> Self {
        Self {
            include_swaths: false,
            include_tours: false,
            ..Self::default()
        }
    }

    /// Write local ENU metres instead of longitude/latitude.
    pub fn in_enu(mut self) -> Self {
        self.crs = Crs::Enu;
        self
    }
}

/// Field geometry alone: border, parts, headlands and the generated rows.
pub fn field_to_vector(field: &Field, options: &GeoJsonOptions) -> Vector {
    let mut vector = base_vector(field, options);
    if options.include_swaths {
        for (part_index, part) in field.parts().iter().enumerate() {
            push_swaths(&mut vector, part_index, &part.swaths, None);
        }
    }
    vector
}

/// Field geometry plus the ordered rows and tour of every planned part.
pub fn planned_field_to_vector(
    field: &Field,
    planned: &PlannedField,
    options: &GeoJsonOptions,
) -> Vector {
    let mut vector = base_vector(field, options);
    for part in &planned.parts {
        if options.include_swaths {
            push_swaths(&mut vector, part.part_index, &part.ordered_swaths, None);
        }
        if options.include_tours {
            push_tour(&mut vector, part.part_index, &part.tour, None);
        }
    }
    vector
}

/// Same as [`planned_field_to_vector`], for the staged planner output.
pub fn planned_stages_to_vector(
    field: &Field,
    planned: &PlannedFieldStages,
    options: &GeoJsonOptions,
) -> Vector {
    let mut vector = base_vector(field, options);
    for part in &planned.parts {
        if options.include_swaths {
            push_swaths(&mut vector, part.part_index, &part.ordered_swaths, None);
        }
        if options.include_tours {
            push_tour(&mut vector, part.part_index, &part.tour, None);
        }
    }
    vector
}

/// Field geometry plus per-machine rows, headland arcs and tours. Every
/// machine-owned element carries a `machine` property.
pub fn planned_machines_to_vector(
    field: &Field,
    planned: &PlannedMachines,
    options: &GeoJsonOptions,
) -> Vector {
    let mut vector = base_vector(field, options);
    vector.set_global_property("machine_count", planned.machines.len().to_string());
    if let Some(pattern) = planned.division.pattern_used {
        vector.set_global_property("division_pattern", format!("{pattern:?}"));
    }

    for machine in &planned.machines {
        let index = machine.machine_index;
        if options.include_swaths {
            push_swaths(
                &mut vector,
                planned.part_index,
                &machine.ordered_swaths,
                Some(index),
            );
        }
        if options.include_headlands {
            push_headland_arcs(
                &mut vector,
                planned.part_index,
                &machine.assigned_headland_arcs,
                index,
            );
        }
        if options.include_tours {
            push_tour(&mut vector, planned.part_index, &machine.tour, Some(index));
        }
    }
    vector
}

/// Write a vector to disk as GeoJSON.
pub fn write_vector(vector: &Vector, path: impl AsRef<Path>, crs: Crs) -> Result<()> {
    vectory::write(&to_collection(vector), path, crs)
        .map_err(|err| MaptraxError::Export(err.to_string()))
}

/// Serialize a vector to a GeoJSON string. Byte-for-byte what
/// [`write_vector`] would put on disk, minus the trailing newline.
pub fn to_json_string(vector: &Vector, crs: Crs) -> Result<String> {
    vectory::to_json_string(&to_collection(vector), crs)
        .map_err(|err| MaptraxError::Export(err.to_string()))
}

// --- internals ---------------------------------------------------------

/// Flatten a `Vector` into the feature collection that actually gets
/// serialized. This mirrors `vectory::Vector::to_file`, which builds the same
/// collection privately; doing it here lets the file and string paths share
/// one definition instead of drifting apart.
fn to_collection(vector: &Vector) -> vectory::FeatureCollection {
    let mut collection = vectory::FeatureCollection::new(vector.datum(), vector.heading());
    collection.global_properties = vector.global_properties().clone();

    // The boundary is a feature like any other; `type: field` is what marks it
    // as the one to restore as the boundary on read.
    let mut field_properties = vector.field_properties().clone();
    field_properties.insert("type".into(), "field".into());
    collection.features.push(vectory::Feature {
        geometry: Geometry::polygon(vector.field_boundary().to_vec()),
        properties: field_properties,
    });

    for element in vector.iter() {
        collection.features.push(vectory::Feature {
            geometry: element.geometry.clone(),
            properties: element.properties.clone(),
        });
    }
    collection
}

/// Border, parts and headlands — the layers every export shares. Rows and
/// tours differ per entry point, so they are appended by the callers.
fn base_vector(field: &Field, options: &GeoJsonOptions) -> Vector {
    let mut vector = Vector::new(
        closed_ring(field.border()),
        field.datum(),
        Default::default(),
        Crs::Enu,
    );

    vector.set_global_property("generator", "maptrax");
    vector.set_global_property("maptrax_version", env!("CARGO_PKG_VERSION"));
    vector.set_field_property("area", format!("{:.3}", field.total_area()));
    vector.set_field_property("part_count", field.parts().len().to_string());

    for (part_index, part) in field.parts().iter().enumerate() {
        if options.include_part_boundaries {
            push_part_boundary(&mut vector, part_index, part);
        }
        if options.include_headlands {
            push_headlands(&mut vector, part_index, &part.headlands);
        }
    }
    vector
}

fn push_part_boundary(vector: &mut Vector, part_index: usize, part: &Part) {
    let mut properties = HashMap::new();
    properties.insert("part".to_string(), part_index.to_string());
    vector.add_polygon(
        closed_ring(&part.boundary.polygon),
        "part_boundary",
        properties,
    );
}

fn push_headlands(vector: &mut Vector, part_index: usize, headlands: &[Ring]) {
    for (ring_index, ring) in headlands.iter().enumerate() {
        let mut properties = HashMap::new();
        properties.insert("part".to_string(), part_index.to_string());
        properties.insert("ring".to_string(), ring_index.to_string());
        vector.add_polygon(closed_ring(&ring.polygon), "headland", properties);
    }
}

/// Rows are emitted in the order given, and the index is recorded as `order`
/// so a consumer can replay the sequence the planner chose.
fn push_swaths(
    vector: &mut Vector,
    part_index: usize,
    swaths: &[Swath],
    machine_index: Option<usize>,
) {
    for (order, swath) in swaths.iter().enumerate() {
        let points = swath_points(swath);
        if points.len() < 2 {
            continue;
        }
        let mut properties = HashMap::new();
        properties.insert("part".to_string(), part_index.to_string());
        properties.insert("order".to_string(), order.to_string());
        properties.insert("swath_id".to_string(), swath.id.to_string());
        properties.insert(
            "swath_type".to_string(),
            swath_type_name(swath.r#type).to_string(),
        );
        properties.insert("width".to_string(), format!("{:.3}", swath.width));
        if let Some(machine) = machine_index {
            properties.insert("machine".to_string(), machine.to_string());
        }
        vector.add_path(points, "swath", properties);
    }
}

fn push_headland_arcs(
    vector: &mut Vector,
    part_index: usize,
    arcs: &[HeadlandArc],
    machine_index: usize,
) {
    for (arc_index, arc) in arcs.iter().enumerate() {
        if arc.len() < 2 {
            continue;
        }
        let mut properties = HashMap::new();
        properties.insert("part".to_string(), part_index.to_string());
        properties.insert("arc".to_string(), arc_index.to_string());
        properties.insert("machine".to_string(), machine_index.to_string());
        vector.add_path(arc.clone(), "headland_arc", properties);
    }
}

/// A tour is flattened into one continuous linestring, so turn arcs and
/// connectors stay attached to the rows they join.
fn push_tour(vector: &mut Vector, part_index: usize, tour: &[Swath], machine_index: Option<usize>) {
    let points = tour_polyline(tour);
    if points.len() < 2 {
        return;
    }
    let mut properties = HashMap::new();
    properties.insert("part".to_string(), part_index.to_string());
    properties.insert("segments".to_string(), tour.len().to_string());
    if let Some(machine) = machine_index {
        properties.insert("machine".to_string(), machine.to_string());
    }
    vector.add_path(points, "tour", properties);
}

/// GeoJSON linear rings must repeat the first vertex at the end; maptrax
/// stores rings open.
fn closed_ring(polygon: &Polygon) -> Vec<Point> {
    let mut points = polygon_exterior_points(polygon);
    if points.len() >= 3 {
        let first = points[0];
        let last = points[points.len() - 1];
        if !points_equal(first, last, 1e-9) {
            points.push(first);
        }
    }
    points
}

/// Prefer the densified drive path when a swath carries one (turn arcs,
/// Reeds-Shepp connectors); fall back to the straight head/tail line.
fn swath_points(swath: &Swath) -> Vec<Point> {
    if swath.points.len() >= 2 {
        swath.points.clone()
    } else {
        vec![swath.head(), swath.tail()]
    }
}

fn swath_type_name(kind: SwathType) -> &'static str {
    match kind {
        SwathType::Swath => "swath",
        SwathType::Connection => "connection",
        SwathType::Around => "around",
        SwathType::Headland => "headland",
    }
}
