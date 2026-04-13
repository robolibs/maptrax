use geo::Point;
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyModule};

use crate::{
    ConnectorMode, DecompositionMode, DivisionType, Geo, MachinePlanningOptions, Maptrax,
    ObstaclePlanningOptions, Pose2D, RoutingOptions, RoutingStrategy, TurnPlannerConfig,
    TurnPlannerModel, polygon_from_points,
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

fn parse_division_type(name: &str) -> PyResult<DivisionType> {
    match name {
        "alternate" => Ok(DivisionType::Alternate),
        "block" => Ok(DivisionType::Block),
        "spatial_rtree" | "spatial-rtree" => Ok(DivisionType::SpatialRtree),
        "length_balanced" | "length-balanced" => Ok(DivisionType::LengthBalanced),
        other => Err(PyValueError::new_err(format!(
            "unknown division type: {other}"
        ))),
    }
}

fn parse_decomposition_mode(name: &str) -> PyResult<DecompositionMode> {
    match name {
        "none" => Ok(DecompositionMode::None),
        "simple_split" | "simple-split" => Ok(DecompositionMode::SimpleSplit),
        "concave_split" | "concave-split" => Ok(DecompositionMode::ConcaveSplit),
        other => Err(PyValueError::new_err(format!(
            "unknown decomposition mode: {other}"
        ))),
    }
}

fn polygon_from_xy(points: Vec<(f64, f64)>) -> crate::Result<geo::Polygon<f64>> {
    Ok(polygon_from_points(
        points
            .into_iter()
            .map(|(x, y)| Point::new(x, y))
            .collect::<Vec<_>>(),
    ))
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

fn ring_to_dict<'py>(py: Python<'py>, ring: &crate::Ring) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("uuid", ring.uuid.clone())?;
    dict.set_item("finished", ring.finished)?;
    dict.set_item(
        "points",
        ring.polygon
            .exterior()
            .points()
            .map(|point| (point.x(), point.y()))
            .collect::<Vec<_>>(),
    )?;
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

fn machine_part_to_dict<'py>(
    py: Python<'py>,
    machine_plan: &crate::MachinePlannedPart,
) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("machine_index", machine_plan.machine_index)?;

    let assigned = PyList::empty(py);
    for swath in &machine_plan.assigned_swaths {
        assigned.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("assigned_swaths", assigned)?;

    let avoided = PyList::empty(py);
    for swath in &machine_plan.avoided_swaths {
        avoided.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("avoided_swaths", avoided)?;

    let ordered = PyList::empty(py);
    for swath in &machine_plan.ordered_swaths {
        ordered.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("ordered_swaths", ordered)?;

    let tour = PyList::empty(py);
    for swath in &machine_plan.tour {
        tour.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("tour", tour)?;
    Ok(dict)
}

fn part_snapshot_to_dict<'py>(
    py: Python<'py>,
    index: usize,
    part: &crate::Part,
) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("part_index", index)?;
    dict.set_item("boundary", ring_to_dict(py, &part.boundary)?)?;

    let headlands = PyList::empty(py);
    for ring in &part.headlands {
        headlands.append(ring_to_dict(py, ring)?)?;
    }
    dict.set_item("headlands", headlands)?;

    let transit_rings = PyList::empty(py);
    for ring in &part.transit_rings {
        transit_rings.append(ring_to_dict(py, ring)?)?;
    }
    dict.set_item("transit_rings", transit_rings)?;

    let swaths = PyList::empty(py);
    for swath in &part.swaths {
        swaths.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("swaths", swaths)?;
    Ok(dict)
}

fn staged_part_to_dict<'py>(
    py: Python<'py>,
    planned: &crate::PlannedPartStages,
) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("part_index", planned.part_index)?;

    let headlands = PyList::empty(py);
    for ring in &planned.headlands {
        headlands.append(ring_to_dict(py, ring)?)?;
    }
    dict.set_item("headlands", headlands)?;

    let transit_rings = PyList::empty(py);
    for ring in &planned.transit_rings {
        transit_rings.append(ring_to_dict(py, ring)?)?;
    }
    dict.set_item("transit_rings", transit_rings)?;

    let generated = PyList::empty(py);
    for swath in &planned.generated_swaths {
        generated.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("generated_swaths", generated)?;

    let avoided = PyList::empty(py);
    for swath in &planned.avoided_swaths {
        avoided.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("avoided_swaths", avoided)?;

    let ordered = PyList::empty(py);
    for swath in &planned.ordered_swaths {
        ordered.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("ordered_swaths", ordered)?;

    let tour = PyList::empty(py);
    for swath in &planned.tour {
        tour.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("tour", tour)?;
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

    fn total_area(&self) -> PyResult<f64> {
        Ok(self.inner.field().map_err(py_err)?.total_area())
    }

    fn part_count(&self) -> PyResult<usize> {
        Ok(self.inner.field().map_err(py_err)?.parts().len())
    }

    fn get_part<'py>(&self, py: Python<'py>, part_index: usize) -> PyResult<Bound<'py, PyDict>> {
        let part = self.inner.field().map_err(py_err)?.part(part_index).map_err(py_err)?;
        part_snapshot_to_dict(py, part_index, part)
    }

    fn decompose_field(&mut self, mode: &str) -> PyResult<usize> {
        self.inner
            .decompose_field(parse_decomposition_mode(mode)?)
            .map_err(py_err)
    }

    #[pyo3(signature = (swath_width, headland_count))]
    fn generate_headlands(&mut self, swath_width: f64, headland_count: usize) -> PyResult<()> {
        self.inner
            .field_mut()
            .map_err(py_err)?
            .generate_headlands(swath_width, headland_count)
            .map_err(py_err)
    }

    #[pyo3(signature = (swath_width, angle_degrees=0.0))]
    fn generate_swaths(&mut self, swath_width: f64, angle_degrees: f64) -> PyResult<()> {
        self.inner
            .field_mut()
            .map_err(py_err)?
            .generate_swaths(swath_width, angle_degrees)
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

    #[pyo3(signature = (
        part_index=0,
        obstacle_polygons=Vec::<Vec<(f64, f64)>>::new(),
        inflation_distance=0.0
    ))]
    fn avoid_obstacles<'py>(
        &self,
        py: Python<'py>,
        part_index: usize,
        obstacle_polygons: Vec<Vec<(f64, f64)>>,
        inflation_distance: f64,
    ) -> PyResult<Bound<'py, PyList>> {
        let obstacles = obstacle_polygons
            .into_iter()
            .map(polygon_from_xy)
            .collect::<crate::Result<Vec<_>>>()
            .map_err(py_err)?;
        let swaths = self
            .inner
            .avoid_obstacles_for_part(obstacles, inflation_distance, part_index)
            .map_err(py_err)?;
        let list = PyList::empty(py);
        for swath in &swaths {
            list.append(swath_to_dict(py, swath)?)?;
        }
        Ok(list)
    }

    #[pyo3(signature = (
        part_index=0,
        obstacle_polygons=Vec::<Vec<(f64, f64)>>::new(),
        inflation_distance=0.0,
        routing_strategy="greedy_nearest",
        local_improvement_passes=0
    ))]
    fn route_part<'py>(
        &self,
        py: Python<'py>,
        part_index: usize,
        obstacle_polygons: Vec<Vec<(f64, f64)>>,
        inflation_distance: f64,
        routing_strategy: &str,
        local_improvement_passes: usize,
    ) -> PyResult<Bound<'py, PyList>> {
        let obstacles = obstacle_polygons
            .into_iter()
            .map(polygon_from_xy)
            .collect::<crate::Result<Vec<_>>>()
            .map_err(py_err)?;
        let ordered = self
            .inner
            .plan_ordered_swaths_for_part(
                part_index,
                RoutingOptions {
                    strategy: parse_routing_strategy(routing_strategy)?,
                    local_improvement_passes,
                },
                &ObstaclePlanningOptions {
                    obstacles,
                    inflation_distance,
                },
            )
            .map_err(py_err)?;
        let list = PyList::empty(py);
        for swath in &ordered {
            list.append(swath_to_dict(py, swath)?)?;
        }
        Ok(list)
    }

    #[pyo3(signature = (
        part_index=0,
        obstacle_polygons=Vec::<Vec<(f64, f64)>>::new(),
        inflation_distance=0.0,
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
    fn plan_stages<'py>(
        &self,
        py: Python<'py>,
        part_index: usize,
        obstacle_polygons: Vec<Vec<(f64, f64)>>,
        inflation_distance: f64,
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
        let obstacles = obstacle_polygons
            .into_iter()
            .map(polygon_from_xy)
            .collect::<crate::Result<Vec<_>>>()
            .map_err(py_err)?;
        let staged = self
            .inner
            .plan_stages_for_part(
                part_index,
                RoutingOptions {
                    strategy: parse_routing_strategy(routing_strategy)?,
                    local_improvement_passes,
                },
                &ObstaclePlanningOptions {
                    obstacles,
                    inflation_distance,
                },
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
        staged_part_to_dict(py, &staged)
    }

    #[pyo3(signature = (
        part_index=0,
        machines=1,
        division_type="alternate",
        obstacle_polygons=Vec::<Vec<(f64, f64)>>::new(),
        inflation_distance=0.0,
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
    fn plan_machines<'py>(
        &self,
        py: Python<'py>,
        part_index: usize,
        machines: usize,
        division_type: &str,
        obstacle_polygons: Vec<Vec<(f64, f64)>>,
        inflation_distance: f64,
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
        let obstacles = obstacle_polygons
            .into_iter()
            .map(polygon_from_xy)
            .collect::<crate::Result<Vec<_>>>()
            .map_err(py_err)?;
        let planned = self
            .inner
            .plan_machines_for_part(
                &MachinePlanningOptions {
                    machines,
                    division_type: parse_division_type(division_type)?,
                    part_index,
                },
                &ObstaclePlanningOptions {
                    obstacles,
                    inflation_distance,
                },
                RoutingOptions {
                    strategy: parse_routing_strategy(routing_strategy)?,
                    local_improvement_passes,
                },
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

        let dict = PyDict::new(py);
        dict.set_item("part_index", planned.part_index)?;
        let machine_list = PyList::empty(py);
        for machine in &planned.machines {
            machine_list.append(machine_part_to_dict(py, machine)?)?;
        }
        dict.set_item("machines", machine_list)?;
        Ok(dict)
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
    fn plan_all_dubins<'py>(
        &self,
        py: Python<'py>,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
        min_turning_radius: f64,
        step_size: f64,
    ) -> PyResult<Bound<'py, PyList>> {
        let paths = crate::Dubins::new(min_turning_radius).get_all_paths(
            Pose2D::new(start.0, start.1, start.2),
            Pose2D::new(goal.0, goal.1, goal.2),
            step_size,
        );
        let list = PyList::empty(py);
        for path in &paths {
            list.append(pose_path_to_dict(py, &path.name, path.total_length, &path.waypoints)?)?;
        }
        Ok(list)
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

    #[pyo3(signature = (start, goal, min_turning_radius, step_size=0.2))]
    fn plan_all_reeds_shepp<'py>(
        &self,
        py: Python<'py>,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
        min_turning_radius: f64,
        step_size: f64,
    ) -> PyResult<Bound<'py, PyList>> {
        let paths = crate::ReedsShepp::new(min_turning_radius).get_all_paths(
            Pose2D::new(start.0, start.1, start.2),
            Pose2D::new(goal.0, goal.1, goal.2),
            step_size,
        );
        let list = PyList::empty(py);
        for path in &paths {
            list.append(pose_path_to_dict(py, &path.name, path.total_length, &path.waypoints)?)?;
        }
        Ok(list)
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
