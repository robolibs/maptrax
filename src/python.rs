use geo::Point;
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyModule};

use crate::{
    ConnectorMode, FieldGenerationMode, FieldGenerationOptions, Geo, Maptrax, Pose2D,
    RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel, polygon_from_points,
};

fn py_err(err: crate::MaptraxError) -> PyErr {
    PyRuntimeError::new_err(err.to_string())
}

fn parse_routing_strategy(name: &str) -> PyResult<RoutingStrategy> {
    match name {
        "greedy_nearest" | "greedy" => Ok(RoutingStrategy::GreedyNearest),
        "snake" => Ok(RoutingStrategy::Snake),
        "spiral" => Ok(RoutingStrategy::Spiral),
        other => Err(PyValueError::new_err(format!(
            "unknown routing strategy: {other}"
        ))),
    }
}

fn parse_turn_model(name: &str) -> PyResult<TurnPlannerModel> {
    match name {
        "auto" => Ok(TurnPlannerModel::Auto),
        "dubins" => Ok(TurnPlannerModel::Dubins),
        "reeds_shepp" | "reeds-shepp" => Ok(TurnPlannerModel::ReedsShepp),
        "sharper" => Ok(TurnPlannerModel::Sharper),
        other => Err(PyValueError::new_err(format!(
            "unknown turn model: {other}"
        ))),
    }
}

fn parse_connector_mode(name: &str) -> PyResult<ConnectorMode> {
    match name {
        "auto" => Ok(ConnectorMode::Auto),
        "direct" => Ok(ConnectorMode::Direct),
        "headland" => Ok(ConnectorMode::Headland),
        other => Err(PyValueError::new_err(format!(
            "unknown connector mode: {other}"
        ))),
    }
}

fn swath_to_dict<'py>(py: Python<'py>, swath: &crate::Swath) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("type", format!("{:?}", swath.r#type).to_lowercase())?;
    dict.set_item("id", swath.id)?;
    dict.set_item("width", swath.width)?;
    let points = swath
        .points
        .iter()
        .map(|point| (point.x(), point.y()))
        .collect::<Vec<_>>();
    dict.set_item("points", points)?;
    Ok(dict)
}

fn part_to_dict<'py>(
    py: Python<'py>,
    part_index: usize,
    ordered_swaths: &[crate::Swath],
    tour: &[crate::Swath],
) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("part_index", part_index)?;

    let ordered = PyList::empty(py);
    for swath in ordered_swaths {
        ordered.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("ordered_swaths", ordered)?;

    let tour_list = PyList::empty(py);
    for swath in tour {
        tour_list.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("tour", tour_list)?;
    Ok(dict)
}

fn pose_path_to_dict<'py>(
    py: Python<'py>,
    name: &str,
    total_length: f64,
    waypoints: &[Pose2D],
) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("name", name)?;
    dict.set_item("total_length", total_length)?;
    dict.set_item(
        "waypoints",
        waypoints
            .iter()
            .map(|pose| (pose.point.x(), pose.point.y(), pose.yaw))
            .collect::<Vec<_>>(),
    )?;
    Ok(dict)
}

#[pyclass(name = "Maptrax")]
pub struct PyMaptrax {
    inner: Maptrax,
}

#[pymethods]
impl PyMaptrax {
    #[new]
    fn new() -> Self {
        Self {
            inner: Maptrax::new(),
        }
    }

    fn set_field(&mut self, border: Vec<(f64, f64)>, datum: (f64, f64, f64)) -> PyResult<()> {
        if border.len() < 3 {
            return Err(PyValueError::new_err(
                "field border requires at least 3 points",
            ));
        }
        let polygon = polygon_from_points(
            border
                .into_iter()
                .map(|(x, y)| Point::new(x, y))
                .collect::<Vec<_>>(),
        );
        self.inner
            .set_field(polygon, Geo::new(datum.0, datum.1, datum.2))
            .map_err(py_err)
    }

    #[pyo3(signature = (swath_width, angle_degrees=0.0, headland_count=0))]
    fn generate_field(
        &mut self,
        swath_width: f64,
        angle_degrees: f64,
        headland_count: usize,
    ) -> PyResult<()> {
        self.inner
            .generate_field(swath_width, angle_degrees, headland_count)
            .map_err(py_err)
    }

    #[pyo3(signature = (
        part_index=0,
        routing_strategy="greedy_nearest",
        local_improvement_passes=0,
        turn_model="reeds_shepp",
        connector_mode="auto",
        min_turning_radius=2.0,
        step_size=0.2,
        machine_length=6.0,
        machine_width=3.0,
        swath_width=0.0
    ))]
    fn plan_part<'py>(
        &self,
        py: Python<'py>,
        part_index: usize,
        routing_strategy: &str,
        local_improvement_passes: usize,
        turn_model: &str,
        connector_mode: &str,
        min_turning_radius: f64,
        step_size: f64,
        machine_length: f64,
        machine_width: f64,
        swath_width: f64,
    ) -> PyResult<Bound<'py, PyDict>> {
        let planned = self
            .inner
            .plan_tour_for_part(
                part_index,
                RoutingOptions {
                    strategy: parse_routing_strategy(routing_strategy)?,
                    local_improvement_passes,
                },
                &crate::ObstaclePlanningOptions::default(),
                &TurnPlannerConfig {
                    model: parse_turn_model(turn_model)?,
                    connector_mode: parse_connector_mode(connector_mode)?,
                    min_turning_radius,
                    step_size,
                    machine_length,
                    machine_width,
                    swath_width,
                    ..TurnPlannerConfig::default()
                },
            )
            .map_err(py_err)?;
        part_to_dict(py, planned.part_index, &planned.ordered_swaths, &planned.tour)
    }

    #[pyo3(signature = (start, goal, min_turning_radius, step_size=0.2))]
    fn plan_dubins<'py>(
        &self,
        py: Python<'py>,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
        min_turning_radius: f64,
        step_size: f64,
    ) -> PyResult<Bound<'py, PyDict>> {
        let path = self.inner.plan_dubins(
            Pose2D::new(start.0, start.1, start.2),
            Pose2D::new(goal.0, goal.1, goal.2),
            min_turning_radius,
            step_size,
        );
        pose_path_to_dict(py, &path.name, path.total_length, &path.waypoints)
    }

    #[pyo3(signature = (start, goal, min_turning_radius, step_size=0.2))]
    fn plan_reeds_shepp<'py>(
        &self,
        py: Python<'py>,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
        min_turning_radius: f64,
        step_size: f64,
    ) -> PyResult<Bound<'py, PyDict>> {
        let path = self.inner.plan_reeds_shepp(
            Pose2D::new(start.0, start.1, start.2),
            Pose2D::new(goal.0, goal.1, goal.2),
            min_turning_radius,
            step_size,
        );
        pose_path_to_dict(py, &path.name, path.total_length, &path.waypoints)
    }

    #[pyo3(signature = (
        start,
        goal,
        min_turning_radius,
        machine_length=6.0,
        machine_width=3.0,
        pattern="auto"
    ))]
    fn plan_sharp_turn<'py>(
        &self,
        py: Python<'py>,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
        min_turning_radius: f64,
        machine_length: f64,
        machine_width: f64,
        pattern: &str,
    ) -> PyResult<Bound<'py, PyDict>> {
        let path = self.inner.plan_sharp_turn(
            Pose2D::new(start.0, start.1, start.2),
            Pose2D::new(goal.0, goal.1, goal.2),
            min_turning_radius,
            machine_length,
            machine_width,
            pattern,
        );
        pose_path_to_dict(py, &path.pattern_name, path.total_length, &path.waypoints)
    }

    fn __repr__(&self) -> &'static str {
        "Maptrax()"
    }
}

pub fn register_python_module(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyMaptrax>()?;
    module.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

#[pymodule]
fn maptrax(module: &Bound<'_, PyModule>) -> PyResult<()> {
    register_python_module(module)
}
