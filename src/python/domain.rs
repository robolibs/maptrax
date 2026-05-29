//! Domain / result pyclasses.
//!
//! These wrap the crate's runtime value types (geometry, swaths, rings,
//! parts, ABLines) and the result types produced by the planner pipeline
//! (paths, segments, planned parts/machines, division results). They are
//! read-only from Python (getters only - no setters): they describe what
//! the planner produced, not configuration knobs.
//!
//! Conventions:
//! - Points / point sequences are exposed as `(x, y)` tuples / lists of
//!   tuples; no `Point2D` class.
//! - Poses are exposed as `(x, y, yaw)` 3-tuples; no `Pose2D` class.
//! - Tagged-enum variants follow the `.kind` SCREAMING_SNAKE convention.

use geo::Point;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate as mt;

use super::enums::{PyDubinsSegmentType, PyReedsSheppSegmentType, PySwathType};

fn pt(p: Point) -> (f64, f64) {
    (p.x(), p.y())
}

fn pose(p: mt::Pose2D) -> (f64, f64, f64) {
    (p.point.x(), p.point.y(), p.yaw)
}

fn poly_points(poly: &geo::Polygon<f64>) -> Vec<(f64, f64)> {
    poly.exterior().points().map(pt).collect()
}

// -----------------------------------------------------------------------------
// SplitBoundary (tagged: Vertical { x } | Horizontal { y })
// -----------------------------------------------------------------------------

#[pyclass(name = "SplitBoundary", eq, frozen)]
#[derive(Copy, Clone, PartialEq)]
pub struct PySplitBoundary {
    pub(crate) inner: mt::SplitBoundary,
}

#[pymethods]
impl PySplitBoundary {
    #[staticmethod]
    fn vertical(x: f64) -> Self {
        Self { inner: mt::SplitBoundary::Vertical { x } }
    }

    #[staticmethod]
    fn horizontal(y: f64) -> Self {
        Self { inner: mt::SplitBoundary::Horizontal { y } }
    }

    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            mt::SplitBoundary::Vertical { .. } => "VERTICAL",
            mt::SplitBoundary::Horizontal { .. } => "HORIZONTAL",
        }
    }

    #[getter]
    fn position(&self) -> f64 {
        match self.inner {
            mt::SplitBoundary::Vertical { x } => x,
            mt::SplitBoundary::Horizontal { y } => y,
        }
    }

    fn __repr__(&self) -> String {
        match self.inner {
            mt::SplitBoundary::Vertical { x } => format!("SplitBoundary.vertical(x={x})"),
            mt::SplitBoundary::Horizontal { y } => format!("SplitBoundary.horizontal(y={y})"),
        }
    }
}

// -----------------------------------------------------------------------------
// Swath
// -----------------------------------------------------------------------------

#[pyclass(name = "Swath", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PySwath {
    pub(crate) inner: mt::Swath,
}

#[pymethods]
impl PySwath {
    #[getter]
    fn r#type(&self) -> PySwathType {
        self.inner.r#type.into()
    }
    #[getter]
    fn id(&self) -> i32 {
        self.inner.id
    }
    #[getter]
    fn uuid(&self) -> &str {
        &self.inner.uuid
    }
    #[getter]
    fn width(&self) -> f64 {
        self.inner.width
    }
    #[getter]
    fn finished(&self) -> bool {
        self.inner.finished
    }
    #[getter]
    fn head(&self) -> (f64, f64) {
        pt(self.inner.head())
    }
    #[getter]
    fn tail(&self) -> (f64, f64) {
        pt(self.inner.tail())
    }
    #[getter]
    fn points(&self) -> Vec<(f64, f64)> {
        self.inner.points.iter().copied().map(pt).collect()
    }
    #[getter]
    fn point_reverse(&self) -> Vec<bool> {
        self.inner.point_reverse.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "Swath(id={}, type={:?}, width={}, points={})",
            self.inner.id,
            self.inner.r#type,
            self.inner.width,
            self.inner.points.len()
        )
    }
}

fn swath_list(swaths: &[mt::Swath]) -> Vec<PySwath> {
    swaths.iter().cloned().map(|inner| PySwath { inner }).collect()
}

// -----------------------------------------------------------------------------
// Ring
// -----------------------------------------------------------------------------

#[pyclass(name = "Ring", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyRing {
    pub(crate) inner: mt::Ring,
}

#[pymethods]
impl PyRing {
    #[getter]
    fn uuid(&self) -> &str {
        &self.inner.uuid
    }
    #[getter]
    fn finished(&self) -> bool {
        self.inner.finished
    }
    #[getter]
    fn points(&self) -> Vec<(f64, f64)> {
        poly_points(&self.inner.polygon)
    }

    fn __repr__(&self) -> String {
        format!(
            "Ring(uuid={}, finished={}, vertices={})",
            self.inner.uuid,
            self.inner.finished,
            self.inner.polygon.exterior().points().count()
        )
    }
}

fn ring_list(rings: &[mt::Ring]) -> Vec<PyRing> {
    rings.iter().cloned().map(|inner| PyRing { inner }).collect()
}

// -----------------------------------------------------------------------------
// Part
// -----------------------------------------------------------------------------

#[pyclass(name = "Part", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyPart {
    pub(crate) inner: mt::Part,
}

#[pymethods]
impl PyPart {
    #[getter]
    fn boundary(&self) -> PyRing {
        PyRing { inner: self.inner.boundary.clone() }
    }
    #[getter]
    fn swaths(&self) -> Vec<PySwath> {
        swath_list(&self.inner.swaths)
    }
    #[getter]
    fn headlands(&self) -> Vec<PyRing> {
        ring_list(&self.inner.headlands)
    }
    #[getter]
    fn non_owned_splits(&self) -> Vec<PySplitBoundary> {
        self.inner
            .non_owned_splits
            .iter()
            .map(|b| PySplitBoundary { inner: *b })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "Part(swaths={}, headlands={}, non_owned_splits={})",
            self.inner.swaths.len(),
            self.inner.headlands.len(),
            self.inner.non_owned_splits.len()
        )
    }
}

pub(crate) fn part_list(parts: &[mt::Part]) -> Vec<PyPart> {
    parts.iter().cloned().map(|inner| PyPart { inner }).collect()
}

// -----------------------------------------------------------------------------
// ABLine
// -----------------------------------------------------------------------------

#[pyclass(name = "ABLine", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyABLine {
    pub(crate) inner: mt::ABLine,
}

#[pymethods]
impl PyABLine {
    #[getter]
    fn a(&self) -> (f64, f64) {
        pt(self.inner.a)
    }
    #[getter]
    fn b(&self) -> (f64, f64) {
        pt(self.inner.b)
    }
    #[getter]
    fn uuid(&self) -> &str {
        &self.inner.uuid
    }
    #[getter]
    fn line_id(&self) -> usize {
        self.inner.line_id
    }
    #[getter]
    fn length(&self) -> f64 {
        self.inner.length()
    }

    fn __repr__(&self) -> String {
        format!(
            "ABLine(line_id={}, a={:?}, b={:?})",
            self.inner.line_id,
            pt(self.inner.a),
            pt(self.inner.b)
        )
    }
}

// -----------------------------------------------------------------------------
// SwathAngleSearchResult
// -----------------------------------------------------------------------------

#[pyclass(name = "SwathAngleSearchResult", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PySwathAngleSearchResult {
    pub(crate) inner: mt::SwathAngleSearchResult,
}

#[pymethods]
impl PySwathAngleSearchResult {
    #[getter]
    fn angle_degrees(&self) -> f64 {
        self.inner.angle_degrees
    }
    #[getter]
    fn score(&self) -> f64 {
        self.inner.score
    }

    fn __repr__(&self) -> String {
        format!(
            "SwathAngleSearchResult(angle_degrees={}, score={})",
            self.inner.angle_degrees, self.inner.score
        )
    }
}

fn objective_list(results: &[mt::SwathAngleSearchResult]) -> Vec<PySwathAngleSearchResult> {
    results
        .iter()
        .cloned()
        .map(|inner| PySwathAngleSearchResult { inner })
        .collect()
}

// -----------------------------------------------------------------------------
// DubinsSegment / DubinsPath
// -----------------------------------------------------------------------------

#[pyclass(name = "DubinsSegment", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyDubinsSegment {
    pub(crate) inner: mt::DubinsSegment,
}

#[pymethods]
impl PyDubinsSegment {
    #[getter]
    fn r#type(&self) -> PyDubinsSegmentType {
        self.inner.r#type.into()
    }
    #[getter]
    fn length(&self) -> f64 {
        self.inner.length
    }

    fn __repr__(&self) -> String {
        format!(
            "DubinsSegment(type={:?}, length={})",
            self.inner.r#type, self.inner.length
        )
    }
}

#[pyclass(name = "DubinsPath", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyDubinsPath {
    pub(crate) inner: mt::DubinsPath,
}

#[pymethods]
impl PyDubinsPath {
    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }
    #[getter]
    fn total_length(&self) -> f64 {
        self.inner.total_length
    }
    #[getter]
    fn segments(&self) -> Vec<PyDubinsSegment> {
        self.inner
            .segments
            .iter()
            .cloned()
            .map(|inner| PyDubinsSegment { inner })
            .collect()
    }
    #[getter]
    fn waypoints(&self) -> Vec<(f64, f64, f64)> {
        self.inner.waypoints.iter().copied().map(pose).collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "DubinsPath(name={}, total_length={}, waypoints={})",
            self.inner.name,
            self.inner.total_length,
            self.inner.waypoints.len()
        )
    }
}

// -----------------------------------------------------------------------------
// ReedsSheppSegment / ReedsSheppPath
// -----------------------------------------------------------------------------

#[pyclass(name = "ReedsSheppSegment", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyReedsSheppSegment {
    pub(crate) inner: mt::ReedsSheppSegment,
}

#[pymethods]
impl PyReedsSheppSegment {
    #[getter]
    fn r#type(&self) -> PyReedsSheppSegmentType {
        self.inner.r#type.into()
    }
    #[getter]
    fn length(&self) -> f64 {
        self.inner.length
    }
    #[getter]
    fn forward(&self) -> bool {
        self.inner.forward
    }

    fn __repr__(&self) -> String {
        format!(
            "ReedsSheppSegment(type={:?}, length={}, forward={})",
            self.inner.r#type, self.inner.length, self.inner.forward
        )
    }
}

#[pyclass(name = "ReedsSheppPath", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyReedsSheppPath {
    pub(crate) inner: mt::ReedsSheppPath,
}

#[pymethods]
impl PyReedsSheppPath {
    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }
    #[getter]
    fn total_length(&self) -> f64 {
        self.inner.total_length
    }
    #[getter]
    fn segments(&self) -> Vec<PyReedsSheppSegment> {
        self.inner
            .segments
            .iter()
            .cloned()
            .map(|inner| PyReedsSheppSegment { inner })
            .collect()
    }
    #[getter]
    fn waypoints(&self) -> Vec<(f64, f64, f64)> {
        self.inner.waypoints.iter().copied().map(pose).collect()
    }
    #[getter]
    fn waypoint_reverse(&self) -> Vec<bool> {
        self.inner.waypoint_reverse.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "ReedsSheppPath(name={}, total_length={}, waypoints={})",
            self.inner.name,
            self.inner.total_length,
            self.inner.waypoints.len()
        )
    }
}

// -----------------------------------------------------------------------------
// SharpTurnPath
// -----------------------------------------------------------------------------

#[pyclass(name = "SharpTurnPath", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PySharpTurnPath {
    pub(crate) inner: mt::SharpTurnPath,
}

#[pymethods]
impl PySharpTurnPath {
    #[getter]
    fn pattern_name(&self) -> &str {
        &self.inner.pattern_name
    }
    #[getter]
    fn total_length(&self) -> f64 {
        self.inner.total_length
    }
    #[getter]
    fn segment_types(&self) -> Vec<String> {
        self.inner.segment_types.clone()
    }
    #[getter]
    fn waypoints(&self) -> Vec<(f64, f64, f64)> {
        self.inner.waypoints.iter().copied().map(pose).collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "SharpTurnPath(pattern_name={}, total_length={}, waypoints={})",
            self.inner.pattern_name,
            self.inner.total_length,
            self.inner.waypoints.len()
        )
    }
}

// -----------------------------------------------------------------------------
// DivisionResult
// -----------------------------------------------------------------------------

#[pyclass(name = "DivisionResult", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyDivisionResult {
    pub(crate) inner: mt::DivisionResult,
}

#[pymethods]
impl PyDivisionResult {
    #[getter]
    fn pattern_used(&self) -> Option<String> {
        self.inner.pattern_used.map(|p| {
            super::tagged_enums::PyDivisionPattern { inner: p }
                .__repr__()
        })
    }
    #[getter]
    fn swaths_per_machine(&self) -> Vec<Vec<PySwath>> {
        self.inner
            .swaths_per_machine
            .iter()
            .map(|m| swath_list(m))
            .collect()
    }
    #[getter]
    fn headland_arcs_per_machine(&self) -> Vec<Vec<Vec<(f64, f64)>>> {
        self.inner
            .headland_arcs_per_machine
            .iter()
            .map(|m| {
                m.iter()
                    .map(|arc| arc.iter().copied().map(pt).collect())
                    .collect()
            })
            .collect()
    }
    #[getter]
    fn estimated_work_time(&self) -> Vec<f64> {
        self.inner.estimated_work_time.clone()
    }
    #[getter]
    fn estimated_transit(&self) -> Vec<f64> {
        self.inner.estimated_transit.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "DivisionResult(machines={}, pattern_used={:?})",
            self.inner.swaths_per_machine.len(),
            self.inner.pattern_used
        )
    }
}

// -----------------------------------------------------------------------------
// PlannedPart / PlannedField
// -----------------------------------------------------------------------------

#[pyclass(name = "PlannedPart", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyPlannedPart {
    pub(crate) inner: mt::PlannedPart,
}

#[pymethods]
impl PyPlannedPart {
    #[getter]
    fn part_index(&self) -> usize {
        self.inner.part_index
    }
    #[getter]
    fn ordered_swaths(&self) -> Vec<PySwath> {
        swath_list(&self.inner.ordered_swaths)
    }
    #[getter]
    fn tour(&self) -> Vec<PySwath> {
        swath_list(&self.inner.tour)
    }
    #[getter]
    fn tour_polyline(&self) -> Vec<(f64, f64)> {
        mt::tour_polyline(&self.inner.tour)
            .into_iter()
            .map(pt)
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "PlannedPart(part_index={}, ordered_swaths={}, tour={})",
            self.inner.part_index,
            self.inner.ordered_swaths.len(),
            self.inner.tour.len()
        )
    }
}

#[pyclass(name = "PlannedField", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyPlannedField {
    pub(crate) inner: mt::PlannedField,
}

#[pymethods]
impl PyPlannedField {
    #[getter]
    fn parts(&self) -> Vec<PyPlannedPart> {
        self.inner
            .parts
            .iter()
            .cloned()
            .map(|inner| PyPlannedPart { inner })
            .collect()
    }
    #[getter]
    fn objective_results(&self) -> Vec<PySwathAngleSearchResult> {
        objective_list(&self.inner.objective_results)
    }

    fn __repr__(&self) -> String {
        format!(
            "PlannedField(parts={}, objective_results={})",
            self.inner.parts.len(),
            self.inner.objective_results.len()
        )
    }
}

// -----------------------------------------------------------------------------
// PlannedPartStages / PlannedFieldStages
// -----------------------------------------------------------------------------

#[pyclass(name = "PlannedPartStages", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyPlannedPartStages {
    pub(crate) inner: mt::PlannedPartStages,
}

#[pymethods]
impl PyPlannedPartStages {
    #[getter]
    fn part_index(&self) -> usize {
        self.inner.part_index
    }
    #[getter]
    fn headlands(&self) -> Vec<PyRing> {
        ring_list(&self.inner.headlands)
    }
    #[getter]
    fn generated_swaths(&self) -> Vec<PySwath> {
        swath_list(&self.inner.generated_swaths)
    }
    #[getter]
    fn ordered_swaths(&self) -> Vec<PySwath> {
        swath_list(&self.inner.ordered_swaths)
    }
    #[getter]
    fn tour(&self) -> Vec<PySwath> {
        swath_list(&self.inner.tour)
    }
    #[getter]
    fn tour_polyline(&self) -> Vec<(f64, f64)> {
        mt::tour_polyline(&self.inner.tour)
            .into_iter()
            .map(pt)
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "PlannedPartStages(part_index={}, generated_swaths={}, tour={})",
            self.inner.part_index,
            self.inner.generated_swaths.len(),
            self.inner.tour.len()
        )
    }
}

#[pyclass(name = "PlannedFieldStages", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyPlannedFieldStages {
    pub(crate) inner: mt::PlannedFieldStages,
}

#[pymethods]
impl PyPlannedFieldStages {
    #[getter]
    fn parts(&self) -> Vec<PyPlannedPartStages> {
        self.inner
            .parts
            .iter()
            .cloned()
            .map(|inner| PyPlannedPartStages { inner })
            .collect()
    }
    #[getter]
    fn objective_results(&self) -> Vec<PySwathAngleSearchResult> {
        objective_list(&self.inner.objective_results)
    }

    fn __repr__(&self) -> String {
        format!(
            "PlannedFieldStages(parts={}, objective_results={})",
            self.inner.parts.len(),
            self.inner.objective_results.len()
        )
    }
}

// -----------------------------------------------------------------------------
// MachinePlannedPart / PlannedMachines
// -----------------------------------------------------------------------------

#[pyclass(name = "MachinePlannedPart", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyMachinePlannedPart {
    pub(crate) inner: mt::MachinePlannedPart,
}

#[pymethods]
impl PyMachinePlannedPart {
    #[getter]
    fn machine_index(&self) -> usize {
        self.inner.machine_index
    }
    #[getter]
    fn assigned_swaths(&self) -> Vec<PySwath> {
        swath_list(&self.inner.assigned_swaths)
    }
    #[getter]
    fn assigned_headland_arcs(&self) -> Vec<Vec<(f64, f64)>> {
        self.inner
            .assigned_headland_arcs
            .iter()
            .map(|arc| arc.iter().copied().map(pt).collect())
            .collect()
    }
    #[getter]
    fn ordered_swaths(&self) -> Vec<PySwath> {
        swath_list(&self.inner.ordered_swaths)
    }
    #[getter]
    fn tour(&self) -> Vec<PySwath> {
        swath_list(&self.inner.tour)
    }
    #[getter]
    fn tour_polyline(&self) -> Vec<(f64, f64)> {
        mt::tour_polyline(&self.inner.tour)
            .into_iter()
            .map(pt)
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "MachinePlannedPart(machine_index={}, assigned_swaths={}, tour={})",
            self.inner.machine_index,
            self.inner.assigned_swaths.len(),
            self.inner.tour.len()
        )
    }
}

#[pyclass(name = "PlannedMachines", eq, frozen)]
#[derive(Clone, PartialEq)]
pub struct PyPlannedMachines {
    pub(crate) inner: mt::PlannedMachines,
}

#[pymethods]
impl PyPlannedMachines {
    #[getter]
    fn part_index(&self) -> usize {
        self.inner.part_index
    }
    #[getter]
    fn division(&self) -> PyDivisionResult {
        PyDivisionResult { inner: self.inner.division.clone() }
    }
    #[getter]
    fn machines(&self) -> Vec<PyMachinePlannedPart> {
        self.inner
            .machines
            .iter()
            .cloned()
            .map(|inner| PyMachinePlannedPart { inner })
            .collect()
    }
    #[getter]
    fn estimated_work_time(&self) -> Vec<f64> {
        self.inner.division.estimated_work_time.clone()
    }
    #[getter]
    fn estimated_transit(&self) -> Vec<f64> {
        self.inner.division.estimated_transit.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "PlannedMachines(part_index={}, machines={})",
            self.inner.part_index,
            self.inner.machines.len()
        )
    }
}

// -----------------------------------------------------------------------------
// Registration
// -----------------------------------------------------------------------------

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PySplitBoundary>()?;
    module.add_class::<PySwath>()?;
    module.add_class::<PyRing>()?;
    module.add_class::<PyPart>()?;
    module.add_class::<PyABLine>()?;
    module.add_class::<PySwathAngleSearchResult>()?;
    module.add_class::<PyDubinsSegment>()?;
    module.add_class::<PyDubinsPath>()?;
    module.add_class::<PyReedsSheppSegment>()?;
    module.add_class::<PyReedsSheppPath>()?;
    module.add_class::<PySharpTurnPath>()?;
    module.add_class::<PyDivisionResult>()?;
    module.add_class::<PyPlannedPart>()?;
    module.add_class::<PyPlannedField>()?;
    module.add_class::<PyPlannedPartStages>()?;
    module.add_class::<PyPlannedFieldStages>()?;
    module.add_class::<PyMachinePlannedPart>()?;
    module.add_class::<PyPlannedMachines>()?;
    Ok(())
}
