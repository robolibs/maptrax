use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyModule};

use crate::{
    Balance, ConnectorMode, DecompositionMode, DivisionPattern, DivisionPlan, Geo, HeadlandMode,
    MachinePlanningOptions, MachineProfile, Maptrax, OptimizeObjective, Point2Ext, Pose2D,
    RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel, point_xy,
    polygon_exterior_points, polygon_from_points,
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

fn parse_pattern(name: &str, stride: usize, bands: usize) -> PyResult<DivisionPattern> {
    match name {
        "stripe" | "alternate" => Ok(DivisionPattern::Stripe {
            stride: stride.max(1),
        }),
        "block" => Ok(DivisionPattern::Block),
        "banded_stripe" | "banded-stripe" | "banded" => Ok(DivisionPattern::BandedStripe {
            bands: bands.max(1),
        }),
        "optimized_makespan" | "optimized" => Ok(DivisionPattern::Optimized {
            objective: OptimizeObjective::Makespan,
        }),
        "optimized_transit" => Ok(DivisionPattern::Optimized {
            objective: OptimizeObjective::TotalTransit,
        }),
        other => Err(PyValueError::new_err(format!(
            "unknown division pattern: {other}"
        ))),
    }
}

fn parse_balance(name: &str) -> PyResult<Balance> {
    match name {
        "count" | "by_count" | "by-count" => Ok(Balance::ByCount),
        "length" | "by_length" | "by-length" => Ok(Balance::ByLength),
        other => Err(PyValueError::new_err(format!("unknown balance: {other}"))),
    }
}

fn build_machine_profiles(
    machines: usize,
    profiles: Option<Vec<(f64, f64)>>,
) -> PyResult<Vec<MachineProfile>> {
    if let Some(list) = profiles {
        if list.len() != machines {
            return Err(PyValueError::new_err(format!(
                "expected {} machine profiles, got {}",
                machines,
                list.len()
            )));
        }
        Ok(list
            .into_iter()
            .map(|(weight, speed)| MachineProfile {
                weight: if weight > 0.0 { weight } else { 1.0 },
                speed: if speed > 0.0 { speed } else { 1.0 },
            })
            .collect())
    } else {
        Ok(MachineProfile::uniform(machines))
    }
}

fn parse_decomposition_mode(name: &str) -> PyResult<DecompositionMode> {
    // Accept "auto_split:520" or "auto_split:520.0" to specify max_side.
    if let Some(rest) = name
        .strip_prefix("auto_split:")
        .or_else(|| name.strip_prefix("auto-split:"))
    {
        let max_side: f64 = rest.parse().map_err(|_| {
            PyValueError::new_err(format!("auto_split needs a numeric max_side, got: {rest}"))
        })?;
        return Ok(DecompositionMode::AutoSplit { max_side });
    }
    match name {
        "none" => Ok(DecompositionMode::None),
        "simple_split" | "simple-split" => Ok(DecompositionMode::SimpleSplit),
        "concave_split" | "concave-split" => Ok(DecompositionMode::ConcaveSplit),
        other => Err(PyValueError::new_err(format!(
            "unknown decomposition mode: {other}"
        ))),
    }
}

fn parse_headland_mode(name: &str, dedicated_machine: usize) -> PyResult<HeadlandMode> {
    match name {
        "one_per_machine" | "one-per-machine" | "round_robin" | "rotate" => {
            Ok(HeadlandMode::OnePerMachine)
        }
        "dedicated" => Ok(HeadlandMode::Dedicated {
            machine: dedicated_machine,
        }),
        "split_by_zone" | "split-by-zone" | "split" => Ok(HeadlandMode::SplitByZone),
        "none" | "skip" => Ok(HeadlandMode::None),
        other => Err(PyValueError::new_err(format!(
            "unknown headland mode: {other}"
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
    dict.set_item("point_reverse", swath.point_reverse.clone())?;
    Ok(dict)
}

fn ring_to_dict<'py>(py: Python<'py>, ring: &crate::Ring) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("uuid", ring.uuid.clone())?;
    dict.set_item("finished", ring.finished)?;
    dict.set_item(
        "points",
        polygon_exterior_points(&ring.polygon)
            .into_iter()
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

    let assigned_headland_arcs = PyList::empty(py);
    for arc in &machine_plan.assigned_headland_arcs {
        let coords: Vec<(f64, f64)> = arc.iter().map(|p| (p.x(), p.y())).collect();
        assigned_headland_arcs.append(coords)?;
    }
    dict.set_item("assigned_headland_arcs", assigned_headland_arcs)?;

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

    // Flat list of (x, y) points forming the whole drive path — one
    // continuous polyline you can send straight to a controller / plotter.
    let polyline: Vec<(f64, f64)> = crate::tour_polyline(&machine_plan.tour)
        .into_iter()
        .map(|p| (p.x(), p.y()))
        .collect();
    dict.set_item("tour_polyline", polyline)?;

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

    let swaths = PyList::empty(py);
    for swath in &part.swaths {
        swaths.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("swaths", swaths)?;

    // Split-ownership metadata: for AutoSplit parts, lists the split lines
    // along which this part is the NON-OWNER (its swaths extend to the
    // split, and the neighbour provides the midline headland).
    let non_owned = PyList::empty(py);
    for boundary in &part.non_owned_splits {
        let entry = PyDict::new(py);
        match *boundary {
            crate::SplitBoundary::Vertical { x } => {
                entry.set_item("axis", "vertical")?;
                entry.set_item("position", x)?;
            }
            crate::SplitBoundary::Horizontal { y } => {
                entry.set_item("axis", "horizontal")?;
                entry.set_item("position", y)?;
            }
        }
        non_owned.append(entry)?;
    }
    dict.set_item("non_owned_splits", non_owned)?;
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

    let generated = PyList::empty(py);
    for swath in &planned.generated_swaths {
        generated.append(swath_to_dict(py, swath)?)?;
    }
    dict.set_item("generated_swaths", generated)?;

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

fn machine_plan_to_dict<'py>(
    py: Python<'py>,
    planned: &crate::PlannedMachines,
) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("part_index", planned.part_index)?;
    dict.set_item(
        "estimated_work_time",
        planned.division.estimated_work_time.clone(),
    )?;
    dict.set_item(
        "estimated_transit",
        planned.division.estimated_transit.clone(),
    )?;
    if let Some(pattern) = planned.division.pattern_used {
        dict.set_item("pattern_used", format!("{:?}", pattern))?;
    }
    let machine_list = PyList::empty(py);
    for machine in &planned.machines {
        machine_list.append(machine_part_to_dict(py, machine)?)?;
    }
    dict.set_item("machines", machine_list)?;
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
                .map(|(x, y)| point_xy(x, y))
                .collect::<Vec<_>>(),
        );
        self.inner
            .set_field(polygon, Geo::new(datum.0, datum.1, datum.2))
            .map_err(py_err)
    }

    fn total_area(&self) -> PyResult<f64> {
        Ok(self.inner.field().map_err(py_err)?.total_area())
    }

    /// Convert a local ENU (x, y) coordinate to (latitude, longitude)
    /// using the field's datum. Use this to build geo polylines for
    /// Rerun's `GeoLineStrings` when drawing map views.
    fn enu_to_wgs(&self, x: f64, y: f64) -> PyResult<(f64, f64)> {
        let datum = self.inner.field().map_err(py_err)?.datum();
        let wgs = concord::to_wgs_from_enu(concord::Enu::new(x, y, 0.0, datum));
        Ok((wgs.latitude, wgs.longitude))
    }

    /// Batch version of `enu_to_wgs`. Accepts a list of (x, y) tuples;
    /// returns a list of (lat, lon) tuples. Faster than calling
    /// `enu_to_wgs` in a loop when you have a long polyline.
    fn enu_to_wgs_batch(&self, points: Vec<(f64, f64)>) -> PyResult<Vec<(f64, f64)>> {
        let datum = self.inner.field().map_err(py_err)?.datum();
        Ok(points
            .into_iter()
            .map(|(x, y)| {
                let wgs = concord::to_wgs_from_enu(concord::Enu::new(x, y, 0.0, datum));
                (wgs.latitude, wgs.longitude)
            })
            .collect())
    }

    fn part_count(&self) -> PyResult<usize> {
        Ok(self.inner.field().map_err(py_err)?.parts().len())
    }

    fn get_part<'py>(&self, py: Python<'py>, part_index: usize) -> PyResult<Bound<'py, PyDict>> {
        let part = self
            .inner
            .field()
            .map_err(py_err)?
            .part(part_index)
            .map_err(py_err)?;
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

    /// Combined decompose + generate step. Use `decomposition="auto_split:520"`
    /// (for example) to auto-split a large field into sub-fields before
    /// generating headlands + swaths per part. Returns the number of parts.
    #[pyo3(signature = (swath_width, angle_degrees=0.0, headland_count=0, decomposition="none"))]
    fn plan_field(
        &mut self,
        swath_width: f64,
        angle_degrees: f64,
        headland_count: usize,
        decomposition: &str,
    ) -> PyResult<usize> {
        let mode = parse_decomposition_mode(decomposition)?;
        self.inner.decompose_field(mode).map_err(py_err)?;
        self.inner
            .generate_field(swath_width, angle_degrees, headland_count)
            .map_err(py_err)?;
        self.inner
            .field()
            .map(|field| field.parts().len())
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
        part_to_dict(
            py,
            planned.part_index,
            &planned.ordered_swaths,
            &planned.tour,
        )
    }

    #[pyo3(signature = (
        part_index=0,
        routing_strategy="greedy_nearest",
        local_improvement_passes=0
    ))]
    fn route_part<'py>(
        &self,
        py: Python<'py>,
        part_index: usize,
        routing_strategy: &str,
        local_improvement_passes: usize,
    ) -> PyResult<Bound<'py, PyList>> {
        let ordered = self
            .inner
            .plan_ordered_swaths_for_part(
                part_index,
                RoutingOptions {
                    strategy: parse_routing_strategy(routing_strategy)?,
                    local_improvement_passes,
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
        let staged = self
            .inner
            .plan_stages_for_part(
                part_index,
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
        staged_part_to_dict(py, &staged)
    }

    #[pyo3(signature = (
        part_index=0,
        machines=1,
        pattern="block",
        balance="count",
        stride=1,
        bands=2,
        machine_profiles=None,
        headland_mode="one_per_machine",
        dedicated_machine=0,
        routing_strategy="greedy_nearest",
        local_improvement_passes=0,
        turn_model="reeds_shepp",
        connector_mode="headland",
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
        pattern: &str,
        balance: &str,
        stride: usize,
        bands: usize,
        machine_profiles: Option<Vec<(f64, f64)>>,
        headland_mode: &str,
        dedicated_machine: usize,
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
        let plan = DivisionPlan {
            pattern: parse_pattern(pattern, stride, bands)?,
            balance: parse_balance(balance)?,
            machines: build_machine_profiles(machines, machine_profiles)?,
            headlands: parse_headland_mode(headland_mode, dedicated_machine)?,
        };
        let planned = self
            .inner
            .plan_machines_for_part(
                &MachinePlanningOptions { plan, part_index },
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

        Ok(machine_plan_to_dict(py, &planned)?)
    }

    /// Plan machines for EVERY part in the field (auto-split sub-fields).
    /// The same DivisionPlan runs on each part with the full fleet; each
    /// physical machine does part 0 then part 1, etc.
    /// Returns a list of per-part plan dicts (same shape as plan_machines).
    #[pyo3(signature = (
        machines=1,
        pattern="block",
        balance="count",
        stride=1,
        bands=2,
        machine_profiles=None,
        headland_mode="one_per_machine",
        dedicated_machine=0,
        routing_strategy="greedy_nearest",
        local_improvement_passes=0,
        turn_model="reeds_shepp",
        connector_mode="headland",
        min_turning_radius=2.0,
        step_size=0.2,
        machine_length=6.0,
        machine_width=3.0,
        swath_width=0.0
    ))]
    fn plan_machines_all_parts<'py>(
        &self,
        py: Python<'py>,
        machines: usize,
        pattern: &str,
        balance: &str,
        stride: usize,
        bands: usize,
        machine_profiles: Option<Vec<(f64, f64)>>,
        headland_mode: &str,
        dedicated_machine: usize,
        routing_strategy: &str,
        local_improvement_passes: usize,
        turn_model: &str,
        connector_mode: &str,
        min_turning_radius: f64,
        step_size: f64,
        machine_length: f64,
        machine_width: f64,
        swath_width: f64,
    ) -> PyResult<Bound<'py, PyList>> {
        let plan = DivisionPlan {
            pattern: parse_pattern(pattern, stride, bands)?,
            balance: parse_balance(balance)?,
            machines: build_machine_profiles(machines, machine_profiles)?,
            headlands: parse_headland_mode(headland_mode, dedicated_machine)?,
        };
        let turn = TurnPlannerConfig {
            model: parse_turn_model(turn_model)?,
            connector_mode: parse_connector_mode(connector_mode)?,
            min_turning_radius,
            step_size,
            machine_length,
            machine_width,
            swath_width,
            ..TurnPlannerConfig::default()
        };
        let routing = RoutingOptions {
            strategy: parse_routing_strategy(routing_strategy)?,
            local_improvement_passes,
        };
        let all_plans = self
            .inner
            .plan_machines_for_all_parts(&plan, routing, &turn)
            .map_err(py_err)?;

        let out = PyList::empty(py);
        for planned in &all_plans {
            out.append(machine_plan_to_dict(py, planned)?)?;
        }
        Ok(out)
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
            list.append(pose_path_to_dict(
                py,
                &path.name,
                path.total_length,
                &path.waypoints,
            )?)?;
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
            list.append(pose_path_to_dict(
                py,
                &path.name,
                path.total_length,
                &path.waypoints,
            )?)?;
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

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyMaptrax>()?;
    Ok(())
}
