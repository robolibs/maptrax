//! Standalone algorithm pyclasses.
//!
//! Each Rust planner / algorithm type is exposed as its own Python class so
//! Python callers can compose the pipeline manually:
//!
//! ```python
//! field = Field(border=[...], datum=(lat, lon, alt))
//! field.generate(swath_width=3.0, angle_degrees=0.0, headland_count=2)
//! part = field.get_part(0)
//!
//! division = Divy.plan(part, division_plan)
//! tour = TourBuilder.build(part, ordered, turn_config)
//!
//! nety = Nety(part.swaths)
//! ordered = nety.route(routing_options)
//!
//! dub = Dubins(min_turning_radius=2.5)
//! path = dub.plan((0,0,0), (10,5,1.57), step_size=0.2)
//! ```

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate as mt;
use crate::{Point2Ext, point_xy};

use super::domain::{
    PyABLine, PyDivisionResult, PyDubinsPath, PyPart, PyReedsSheppPath, PyRing, PySharpTurnPath,
    PySwath, part_list,
};
use super::options::{PyDivisionPlan, PyRoutingOptions, PyTurnPlannerConfig};
use super::tagged_enums::PyDecompositionMode;

fn py_err(err: mt::MaptraxError) -> PyErr {
    PyRuntimeError::new_err(err.to_string())
}

fn polygon_from_xy(points: Vec<(f64, f64)>) -> PyResult<mt::Polygon> {
    if points.len() < 3 {
        return Err(PyValueError::new_err("polygon requires at least 3 points"));
    }
    Ok(mt::polygon_from_points(
        points
            .into_iter()
            .map(|(x, y)| point_xy(x, y))
            .collect::<Vec<_>>(),
    ))
}

fn pose(t: (f64, f64, f64)) -> mt::Pose2D {
    mt::Pose2D::new(t.0, t.1, t.2)
}

// -----------------------------------------------------------------------------
// Dubins
// -----------------------------------------------------------------------------

#[pyclass(name = "Dubins")]
#[derive(Clone)]
pub struct PyDubins {
    inner: mt::Dubins,
}

#[pymethods]
impl PyDubins {
    #[new]
    #[pyo3(signature = (min_turning_radius))]
    fn new(min_turning_radius: f64) -> Self {
        Self {
            inner: mt::Dubins::new(min_turning_radius),
        }
    }

    #[pyo3(signature = (start, goal, step_size = 0.2))]
    fn plan(&self, start: (f64, f64, f64), goal: (f64, f64, f64), step_size: f64) -> PyDubinsPath {
        PyDubinsPath {
            inner: self.inner.plan_path(pose(start), pose(goal), step_size),
        }
    }

    #[pyo3(signature = (start, goal, step_size = 0.2))]
    fn all_paths(
        &self,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
        step_size: f64,
    ) -> Vec<PyDubinsPath> {
        self.inner
            .get_all_paths(pose(start), pose(goal), step_size)
            .into_iter()
            .map(|inner| PyDubinsPath { inner })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!("Dubins({:?})", self.inner)
    }
}

// -----------------------------------------------------------------------------
// ReedsShepp
// -----------------------------------------------------------------------------

#[pyclass(name = "ReedsShepp")]
#[derive(Clone)]
pub struct PyReedsShepp {
    inner: mt::ReedsShepp,
}

#[pymethods]
impl PyReedsShepp {
    #[new]
    #[pyo3(signature = (min_turning_radius))]
    fn new(min_turning_radius: f64) -> Self {
        Self {
            inner: mt::ReedsShepp::new(min_turning_radius),
        }
    }

    #[pyo3(signature = (start, goal, step_size = 0.2))]
    fn plan(
        &self,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
        step_size: f64,
    ) -> PyReedsSheppPath {
        PyReedsSheppPath {
            inner: self.inner.plan_path(pose(start), pose(goal), step_size),
        }
    }

    #[pyo3(signature = (start, goal, step_size = 0.2))]
    fn all_paths(
        &self,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
        step_size: f64,
    ) -> Vec<PyReedsSheppPath> {
        self.inner
            .get_all_paths(pose(start), pose(goal), step_size)
            .into_iter()
            .map(|inner| PyReedsSheppPath { inner })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!("ReedsShepp({:?})", self.inner)
    }
}

// -----------------------------------------------------------------------------
// Sharper
// -----------------------------------------------------------------------------

#[pyclass(name = "Sharper")]
#[derive(Clone)]
pub struct PySharper {
    inner: mt::Sharper,
}

#[pymethods]
impl PySharper {
    #[new]
    #[pyo3(signature = (min_turning_radius, machine_length, machine_width = 0.0))]
    fn new(min_turning_radius: f64, machine_length: f64, machine_width: f64) -> Self {
        Self {
            inner: mt::Sharper::new(min_turning_radius, machine_length, machine_width),
        }
    }

    #[pyo3(signature = (start, goal, pattern = "auto"))]
    fn plan(
        &self,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
        pattern: &str,
    ) -> PySharpTurnPath {
        PySharpTurnPath {
            inner: self.inner.plan_sharp_turn(pose(start), pose(goal), pattern),
        }
    }

    fn __repr__(&self) -> String {
        format!("Sharper({:?})", self.inner)
    }
}

// -----------------------------------------------------------------------------
// Nety
// -----------------------------------------------------------------------------

#[pyclass(name = "Nety")]
pub struct PyNety {
    inner: mt::Nety,
}

#[pymethods]
impl PyNety {
    #[new]
    #[pyo3(signature = (swaths))]
    fn new(swaths: Vec<PySwath>) -> Self {
        let raw: Vec<mt::Swath> = swaths.into_iter().map(|s| s.inner).collect();
        Self {
            inner: mt::Nety::new(&raw),
        }
    }

    #[getter]
    fn num_vertices(&self) -> usize {
        self.inner.num_vertices()
    }
    #[getter]
    fn num_edges(&self) -> usize {
        self.inner.num_edges()
    }
    #[getter]
    fn ab_lines(&self) -> Vec<PyABLine> {
        self.inner
            .ab_lines()
            .iter()
            .cloned()
            .map(|inner| PyABLine { inner })
            .collect()
    }
    #[getter]
    fn swaths(&self) -> Vec<PySwath> {
        self.inner
            .swaths()
            .iter()
            .cloned()
            .map(|inner| PySwath { inner })
            .collect()
    }

    /// Run routing in place. Returns the now-ordered swath list.
    #[pyo3(signature = (options = None, start = None))]
    fn route(
        &mut self,
        options: Option<PyRoutingOptions>,
        start: Option<(f64, f64)>,
    ) -> Vec<PySwath> {
        let opts: mt::RoutingOptions = options.as_ref().map(Into::into).unwrap_or_default();
        let start = start.map(|(x, y)| point_xy(x, y));
        self.inner.field_traversal_with_options(start, opts);
        self.swaths()
    }

    fn __repr__(&self) -> String {
        format!(
            "Nety(swaths={}, vertices={}, edges={})",
            self.inner.swaths().len(),
            self.inner.num_vertices(),
            self.inner.num_edges()
        )
    }
}

// -----------------------------------------------------------------------------
// Divy
// -----------------------------------------------------------------------------

#[pyclass(name = "Divy")]
pub struct PyDivy;

#[pymethods]
impl PyDivy {
    #[new]
    fn new() -> Self {
        Self
    }

    #[staticmethod]
    #[pyo3(signature = (part, plan))]
    fn plan(part: &PyPart, plan: &PyDivisionPlan) -> PyResult<PyDivisionResult> {
        let rust_plan: mt::DivisionPlan = plan.into();
        let result = mt::Divy::plan(&part.inner, &rust_plan).map_err(py_err)?;
        Ok(PyDivisionResult { inner: result })
    }

    fn __repr__(&self) -> &'static str {
        "Divy()"
    }
}

// -----------------------------------------------------------------------------
// TourBuilder
// -----------------------------------------------------------------------------

#[pyclass(name = "TourBuilder")]
pub struct PyTourBuilder;

#[pymethods]
impl PyTourBuilder {
    #[new]
    fn new() -> Self {
        Self
    }

    #[staticmethod]
    #[pyo3(signature = (part, ordered_swaths, config))]
    fn build(
        part: &PyPart,
        ordered_swaths: Vec<PySwath>,
        config: &PyTurnPlannerConfig,
    ) -> Vec<PySwath> {
        let raw: Vec<mt::Swath> = ordered_swaths.into_iter().map(|s| s.inner).collect();
        let rust_cfg: mt::TurnPlannerConfig = config.into();
        let tour = mt::TourBuilder::build(&part.inner, &raw, &rust_cfg);
        tour.into_iter().map(|inner| PySwath { inner }).collect()
    }

    fn __repr__(&self) -> &'static str {
        "TourBuilder()"
    }
}

// -----------------------------------------------------------------------------
// Field
// -----------------------------------------------------------------------------

#[pyclass(name = "Field")]
#[derive(Clone)]
pub struct PyField {
    pub(crate) inner: mt::Field,
}

#[pymethods]
impl PyField {
    #[new]
    #[pyo3(signature = (border, datum))]
    fn new(border: Vec<(f64, f64)>, datum: (f64, f64, f64)) -> PyResult<Self> {
        let polygon = polygon_from_xy(border)?;
        let geo = mt::Geo::new(datum.0, datum.1, datum.2);
        Ok(Self {
            inner: mt::Field::new(polygon, geo).map_err(py_err)?,
        })
    }

    #[getter]
    fn border(&self) -> Vec<(f64, f64)> {
        self.inner
            .border()
            .vertices
            .iter()
            .map(|p| (p.x(), p.y()))
            .collect()
    }

    #[getter]
    fn datum(&self) -> (f64, f64, f64) {
        let g = self.inner.datum();
        (g.latitude, g.longitude, g.altitude)
    }

    #[getter]
    fn total_area(&self) -> f64 {
        self.inner.total_area()
    }

    #[getter]
    fn part_count(&self) -> usize {
        self.inner.parts().len()
    }

    #[getter]
    fn parts(&self) -> Vec<PyPart> {
        part_list(self.inner.parts())
    }

    fn get_part(&self, index: usize) -> PyResult<PyPart> {
        let p = self.inner.part(index).map_err(py_err)?.clone();
        Ok(PyPart { inner: p })
    }

    fn get_boundary(&self) -> PyRing {
        // The first part's boundary is the field boundary ring with uuid + bbox.
        let part = &self.inner.parts()[0];
        PyRing {
            inner: part.boundary.clone(),
        }
    }

    #[pyo3(signature = (mode))]
    fn decompose(&mut self, mode: PyDecompositionMode) -> PyResult<usize> {
        self.inner.decompose(mode.into()).map_err(py_err)
    }

    #[pyo3(signature = (swath_width, angle_degrees = 0.0, headland_count = 0))]
    fn generate(
        &mut self,
        swath_width: f64,
        angle_degrees: f64,
        headland_count: usize,
    ) -> PyResult<()> {
        self.inner
            .generate(swath_width, angle_degrees, headland_count)
            .map_err(py_err)
    }

    #[pyo3(signature = (swath_width, headland_count))]
    fn generate_headlands(&mut self, swath_width: f64, headland_count: usize) -> PyResult<()> {
        self.inner
            .generate_headlands(swath_width, headland_count)
            .map_err(py_err)
    }

    #[pyo3(signature = (swath_width, angle_degrees = 0.0))]
    fn generate_swaths(&mut self, swath_width: f64, angle_degrees: f64) -> PyResult<()> {
        self.inner
            .generate_swaths(swath_width, angle_degrees)
            .map_err(py_err)
    }

    /// Convert local ENU (x, y) to (latitude, longitude) using the field datum.
    fn enu_to_wgs(&self, x: f64, y: f64) -> (f64, f64) {
        let datum = self.inner.datum();
        let wgs = concord::to_wgs_from_enu(concord::Enu::new(x, y, 0.0, datum));
        (wgs.latitude, wgs.longitude)
    }

    /// Batch version of `enu_to_wgs`. Faster than calling in a loop.
    fn enu_to_wgs_batch(&self, points: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
        let datum = self.inner.datum();
        points
            .into_iter()
            .map(|(x, y)| {
                let wgs = concord::to_wgs_from_enu(concord::Enu::new(x, y, 0.0, datum));
                (wgs.latitude, wgs.longitude)
            })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "Field(parts={}, total_area={:.1})",
            self.inner.parts().len(),
            self.inner.total_area()
        )
    }
}

// -----------------------------------------------------------------------------
// Registration
// -----------------------------------------------------------------------------

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyDubins>()?;
    module.add_class::<PyReedsShepp>()?;
    module.add_class::<PySharper>()?;
    module.add_class::<PyNety>()?;
    module.add_class::<PyDivy>()?;
    module.add_class::<PyTourBuilder>()?;
    module.add_class::<PyField>()?;
    Ok(())
}
