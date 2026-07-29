//! Option / configuration pyclasses.
//!
//! These mirror the Rust crate's option structs field-for-field. Fields are
//! `#[pyo3(get, set)]` so Python users can build them either via the
//! constructor (kwargs) or by attribute assignment after construction:
//!
//! ```python
//! cfg = TurnPlannerConfig(min_turning_radius=2.5)
//! cfg.step_size = 0.1   # equally valid
//! ```
//!
//! `From<&Py*>` / `From<mt::*>` conversions live next to each class so the
//! facade and standalone classes can convert at the boundary without
//! exposing pyo3 internals to the rest of the crate.

use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate as mt;

use super::enums::{PyBalance, PyConnectorMode, PyRoutingStrategy, PyTurnPlannerModel};
use super::tagged_enums::{
    PyDecompositionMode, PyDivisionPattern, PyHeadlandMode, PySwathObjective,
};

// -----------------------------------------------------------------------------
// MachineProfile
// -----------------------------------------------------------------------------

#[pyclass(name = "MachineProfile", eq)]
#[derive(Copy, Clone, PartialEq)]
pub struct PyMachineProfile {
    #[pyo3(get, set)]
    pub weight: f64,
    #[pyo3(get, set)]
    pub speed: f64,
}

#[pymethods]
impl PyMachineProfile {
    #[new]
    #[pyo3(signature = (weight = 1.0, speed = 1.0))]
    fn new(weight: f64, speed: f64) -> Self {
        Self { weight, speed }
    }

    #[staticmethod]
    #[pyo3(signature = (count))]
    fn uniform(count: usize) -> Vec<Self> {
        (0..count)
            .map(|_| Self {
                weight: 1.0,
                speed: 1.0,
            })
            .collect()
    }

    pub(crate) fn __repr__(&self) -> String {
        format!(
            "MachineProfile(weight={}, speed={})",
            self.weight, self.speed
        )
    }
}

impl From<&PyMachineProfile> for mt::MachineProfile {
    fn from(p: &PyMachineProfile) -> Self {
        Self {
            weight: p.weight,
            speed: p.speed,
        }
    }
}

impl From<mt::MachineProfile> for PyMachineProfile {
    fn from(r: mt::MachineProfile) -> Self {
        Self {
            weight: r.weight,
            speed: r.speed,
        }
    }
}

// -----------------------------------------------------------------------------
// SwathAngleSearchOptions
// -----------------------------------------------------------------------------

#[pyclass(name = "SwathAngleSearchOptions", eq)]
#[derive(Copy, Clone, PartialEq)]
pub struct PySwathAngleSearchOptions {
    #[pyo3(get, set)]
    pub start_degrees: f64,
    #[pyo3(get, set)]
    pub end_degrees: f64,
    #[pyo3(get, set)]
    pub step_degrees: f64,
}

#[pymethods]
impl PySwathAngleSearchOptions {
    #[new]
    #[pyo3(signature = (start_degrees = 1.0, end_degrees = 359.0, step_degrees = 1.0))]
    fn new(start_degrees: f64, end_degrees: f64, step_degrees: f64) -> Self {
        Self {
            start_degrees,
            end_degrees,
            step_degrees,
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        format!(
            "SwathAngleSearchOptions(start_degrees={}, end_degrees={}, step_degrees={})",
            self.start_degrees, self.end_degrees, self.step_degrees,
        )
    }
}

impl From<&PySwathAngleSearchOptions> for mt::SwathAngleSearchOptions {
    fn from(p: &PySwathAngleSearchOptions) -> Self {
        Self {
            start_degrees: p.start_degrees,
            end_degrees: p.end_degrees,
            step_degrees: p.step_degrees,
        }
    }
}

impl From<mt::SwathAngleSearchOptions> for PySwathAngleSearchOptions {
    fn from(r: mt::SwathAngleSearchOptions) -> Self {
        Self {
            start_degrees: r.start_degrees,
            end_degrees: r.end_degrees,
            step_degrees: r.step_degrees,
        }
    }
}

// -----------------------------------------------------------------------------
// FieldGenerationMode (tagged: ExplicitAngle(f64) | Objective { objective, options })
// -----------------------------------------------------------------------------

#[pyclass(name = "FieldGenerationMode", eq)]
#[derive(Clone, PartialEq)]
pub struct PyFieldGenerationMode {
    pub(crate) inner: mt::FieldGenerationMode,
}

#[pymethods]
impl PyFieldGenerationMode {
    #[staticmethod]
    #[pyo3(signature = (angle_degrees = 0.0))]
    fn explicit_angle(angle_degrees: f64) -> Self {
        Self {
            inner: mt::FieldGenerationMode::ExplicitAngle(angle_degrees),
        }
    }

    #[staticmethod]
    #[pyo3(signature = (objective, options = None))]
    fn objective_search(
        objective: PySwathObjective,
        options: Option<PySwathAngleSearchOptions>,
    ) -> Self {
        let options = options
            .map(|o| (&o).into())
            .unwrap_or_else(mt::SwathAngleSearchOptions::default);
        Self {
            inner: mt::FieldGenerationMode::Objective {
                objective: objective.inner,
                options,
            },
        }
    }

    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            mt::FieldGenerationMode::ExplicitAngle(_) => "EXPLICIT_ANGLE",
            mt::FieldGenerationMode::Objective { .. } => "OBJECTIVE",
        }
    }

    #[getter]
    fn angle_degrees(&self) -> Option<f64> {
        match self.inner {
            mt::FieldGenerationMode::ExplicitAngle(a) => Some(a),
            _ => None,
        }
    }

    #[getter]
    fn objective(&self) -> Option<PySwathObjective> {
        match &self.inner {
            mt::FieldGenerationMode::Objective { objective, .. } => {
                Some(PySwathObjective { inner: *objective })
            }
            _ => None,
        }
    }

    #[getter]
    fn options(&self) -> Option<PySwathAngleSearchOptions> {
        match &self.inner {
            mt::FieldGenerationMode::Objective { options, .. } => Some((*options).into()),
            _ => None,
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        match &self.inner {
            mt::FieldGenerationMode::ExplicitAngle(a) => {
                format!("FieldGenerationMode.explicit_angle(angle_degrees={a})")
            }
            mt::FieldGenerationMode::Objective { objective, options } => {
                let obj = PySwathObjective { inner: *objective }.__repr__();
                let opt: PySwathAngleSearchOptions = (*options).into();
                format!(
                    "FieldGenerationMode.objective_search(objective={obj}, options={})",
                    opt.__repr__()
                )
            }
        }
    }
}

impl From<PyFieldGenerationMode> for mt::FieldGenerationMode {
    fn from(v: PyFieldGenerationMode) -> Self {
        v.inner
    }
}

impl From<mt::FieldGenerationMode> for PyFieldGenerationMode {
    fn from(inner: mt::FieldGenerationMode) -> Self {
        Self { inner }
    }
}

// -----------------------------------------------------------------------------
// FieldGenerationOptions
// -----------------------------------------------------------------------------

#[pyclass(name = "FieldGenerationOptions", eq)]
#[derive(Clone, PartialEq)]
pub struct PyFieldGenerationOptions {
    #[pyo3(get, set)]
    pub swath_width: f64,
    #[pyo3(get, set)]
    pub headland_count: usize,
    #[pyo3(get, set)]
    pub decomposition: PyDecompositionMode,
    #[pyo3(get, set)]
    pub mode: PyFieldGenerationMode,
}

#[pymethods]
impl PyFieldGenerationOptions {
    #[new]
    #[pyo3(signature = (
        swath_width = 10.0,
        headland_count = 0,
        decomposition = PyDecompositionMode { inner: mt::DecompositionMode::None },
        mode = PyFieldGenerationMode { inner: mt::FieldGenerationMode::ExplicitAngle(0.0) },
    ))]
    fn new(
        swath_width: f64,
        headland_count: usize,
        decomposition: PyDecompositionMode,
        mode: PyFieldGenerationMode,
    ) -> Self {
        Self {
            swath_width,
            headland_count,
            decomposition,
            mode,
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        format!(
            "FieldGenerationOptions(swath_width={}, headland_count={}, decomposition={}, mode={})",
            self.swath_width,
            self.headland_count,
            self.decomposition.__repr__(),
            self.mode.__repr__(),
        )
    }
}

impl From<&PyFieldGenerationOptions> for mt::FieldGenerationOptions {
    fn from(p: &PyFieldGenerationOptions) -> Self {
        Self {
            swath_width: p.swath_width,
            headland_count: p.headland_count,
            decomposition: p.decomposition.inner,
            mode: p.mode.inner.clone(),
        }
    }
}

impl From<mt::FieldGenerationOptions> for PyFieldGenerationOptions {
    fn from(r: mt::FieldGenerationOptions) -> Self {
        Self {
            swath_width: r.swath_width,
            headland_count: r.headland_count,
            decomposition: PyDecompositionMode {
                inner: r.decomposition,
            },
            mode: PyFieldGenerationMode { inner: r.mode },
        }
    }
}

// -----------------------------------------------------------------------------
// RoutingOptions
// -----------------------------------------------------------------------------

#[pyclass(name = "RoutingOptions", eq)]
#[derive(Copy, Clone, PartialEq)]
pub struct PyRoutingOptions {
    #[pyo3(get, set)]
    pub strategy: PyRoutingStrategy,
    #[pyo3(get, set)]
    pub local_improvement_passes: usize,
}

#[pymethods]
impl PyRoutingOptions {
    #[new]
    #[pyo3(signature = (strategy = PyRoutingStrategy::GreedyNearest, local_improvement_passes = 0))]
    fn new(strategy: PyRoutingStrategy, local_improvement_passes: usize) -> Self {
        Self {
            strategy,
            local_improvement_passes,
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        format!(
            "RoutingOptions(strategy={:?}, local_improvement_passes={})",
            self.strategy, self.local_improvement_passes
        )
    }
}

impl From<&PyRoutingOptions> for mt::RoutingOptions {
    fn from(p: &PyRoutingOptions) -> Self {
        Self {
            strategy: p.strategy.into(),
            local_improvement_passes: p.local_improvement_passes,
        }
    }
}

impl From<mt::RoutingOptions> for PyRoutingOptions {
    fn from(r: mt::RoutingOptions) -> Self {
        Self {
            strategy: r.strategy.into(),
            local_improvement_passes: r.local_improvement_passes,
        }
    }
}

// -----------------------------------------------------------------------------
// TurnPlannerConfig
// -----------------------------------------------------------------------------

#[pyclass(name = "TurnPlannerConfig", eq)]
#[derive(Clone, PartialEq)]
pub struct PyTurnPlannerConfig {
    #[pyo3(get, set)]
    pub model: PyTurnPlannerModel,
    #[pyo3(get, set)]
    pub connector_mode: PyConnectorMode,
    #[pyo3(get, set)]
    pub min_turning_radius: f64,
    #[pyo3(get, set)]
    pub step_size: f64,
    #[pyo3(get, set)]
    pub machine_length: f64,
    #[pyo3(get, set)]
    pub machine_width: f64,
    #[pyo3(get, set)]
    pub sharper_pattern: String,
    #[pyo3(get, set)]
    pub swath_width: f64,
    #[pyo3(get, set)]
    pub headland_threshold_rows: f64,
}

#[pymethods]
impl PyTurnPlannerConfig {
    #[new]
    #[pyo3(signature = (
        model = PyTurnPlannerModel::Auto,
        connector_mode = PyConnectorMode::Headland,
        min_turning_radius = 2.0,
        step_size = 0.2,
        machine_length = 6.0,
        machine_width = 0.0,
        sharper_pattern = "auto".to_string(),
        swath_width = 0.0,
        headland_threshold_rows = 2.0,
    ))]
    fn new(
        model: PyTurnPlannerModel,
        connector_mode: PyConnectorMode,
        min_turning_radius: f64,
        step_size: f64,
        machine_length: f64,
        machine_width: f64,
        sharper_pattern: String,
        swath_width: f64,
        headland_threshold_rows: f64,
    ) -> Self {
        Self {
            model,
            connector_mode,
            min_turning_radius,
            step_size,
            machine_length,
            machine_width,
            sharper_pattern,
            swath_width,
            headland_threshold_rows,
        }
    }

    fn turning_envelope_radius(&self) -> f64 {
        let rust_cfg: mt::TurnPlannerConfig = self.into();
        rust_cfg.turning_envelope_radius()
    }

    fn required_row_skip_stride(&self) -> usize {
        let rust_cfg: mt::TurnPlannerConfig = self.into();
        rust_cfg.required_row_skip_stride(rust_cfg.swath_width)
    }

    pub(crate) fn __repr__(&self) -> String {
        format!(
            "TurnPlannerConfig(model={:?}, connector_mode={:?}, min_turning_radius={}, step_size={}, machine_length={}, machine_width={}, sharper_pattern={:?}, swath_width={}, headland_threshold_rows={})",
            self.model,
            self.connector_mode,
            self.min_turning_radius,
            self.step_size,
            self.machine_length,
            self.machine_width,
            self.sharper_pattern,
            self.swath_width,
            self.headland_threshold_rows,
        )
    }
}

impl From<&PyTurnPlannerConfig> for mt::TurnPlannerConfig {
    fn from(p: &PyTurnPlannerConfig) -> Self {
        Self {
            model: p.model.into(),
            connector_mode: p.connector_mode.into(),
            min_turning_radius: p.min_turning_radius,
            step_size: p.step_size,
            machine_length: p.machine_length,
            machine_width: p.machine_width,
            sharper_pattern: p.sharper_pattern.clone(),
            swath_width: p.swath_width,
            headland_threshold_rows: p.headland_threshold_rows,
        }
    }
}

impl From<mt::TurnPlannerConfig> for PyTurnPlannerConfig {
    fn from(r: mt::TurnPlannerConfig) -> Self {
        Self {
            model: r.model.into(),
            connector_mode: r.connector_mode.into(),
            min_turning_radius: r.min_turning_radius,
            step_size: r.step_size,
            machine_length: r.machine_length,
            machine_width: r.machine_width,
            sharper_pattern: r.sharper_pattern,
            swath_width: r.swath_width,
            headland_threshold_rows: r.headland_threshold_rows,
        }
    }
}

// -----------------------------------------------------------------------------
// DivisionPlan
// -----------------------------------------------------------------------------

#[pyclass(name = "DivisionPlan", eq)]
#[derive(Clone, PartialEq)]
pub struct PyDivisionPlan {
    #[pyo3(get, set)]
    pub pattern: PyDivisionPattern,
    #[pyo3(get, set)]
    pub balance: PyBalance,
    #[pyo3(get, set)]
    pub machines: Vec<PyMachineProfile>,
    #[pyo3(get, set)]
    pub headlands: PyHeadlandMode,
}

#[pymethods]
impl PyDivisionPlan {
    #[new]
    #[pyo3(signature = (
        pattern = PyDivisionPattern { inner: mt::DivisionPattern::Block },
        balance = PyBalance::ByCount,
        machines = Vec::new(),
        headlands = PyHeadlandMode { inner: mt::HeadlandMode::OnePerMachine },
    ))]
    fn new(
        pattern: PyDivisionPattern,
        balance: PyBalance,
        machines: Vec<PyMachineProfile>,
        headlands: PyHeadlandMode,
    ) -> Self {
        Self {
            pattern,
            balance,
            machines,
            headlands,
        }
    }

    #[staticmethod]
    #[pyo3(signature = (machines, pattern = PyDivisionPattern { inner: mt::DivisionPattern::Block }, balance = PyBalance::ByCount))]
    fn uniform(machines: usize, pattern: PyDivisionPattern, balance: PyBalance) -> Self {
        Self {
            pattern,
            balance,
            machines: PyMachineProfile::uniform(machines),
            headlands: PyHeadlandMode {
                inner: mt::HeadlandMode::OnePerMachine,
            },
        }
    }

    #[getter]
    fn machine_count(&self) -> usize {
        self.machines.len()
    }

    pub(crate) fn __repr__(&self) -> String {
        format!(
            "DivisionPlan(pattern={}, balance={:?}, machines={:?}, headlands={})",
            self.pattern.__repr__(),
            self.balance,
            self.machines
                .iter()
                .map(|m| m.__repr__())
                .collect::<Vec<_>>(),
            self.headlands.__repr__(),
        )
    }
}

impl From<&PyDivisionPlan> for mt::DivisionPlan {
    fn from(p: &PyDivisionPlan) -> Self {
        Self {
            pattern: p.pattern.inner,
            balance: p.balance.into(),
            machines: p.machines.iter().map(Into::into).collect(),
            headlands: p.headlands.inner,
        }
    }
}

impl From<mt::DivisionPlan> for PyDivisionPlan {
    fn from(r: mt::DivisionPlan) -> Self {
        Self {
            pattern: PyDivisionPattern { inner: r.pattern },
            balance: r.balance.into(),
            machines: r.machines.into_iter().map(Into::into).collect(),
            headlands: PyHeadlandMode { inner: r.headlands },
        }
    }
}

// -----------------------------------------------------------------------------
// MachinePlanningOptions
// -----------------------------------------------------------------------------

#[pyclass(name = "MachinePlanningOptions", eq)]
#[derive(Clone, PartialEq)]
pub struct PyMachinePlanningOptions {
    #[pyo3(get, set)]
    pub plan: PyDivisionPlan,
    #[pyo3(get, set)]
    pub part_index: usize,
}

#[pymethods]
impl PyMachinePlanningOptions {
    #[new]
    #[pyo3(signature = (plan, part_index = 0))]
    fn new(plan: PyDivisionPlan, part_index: usize) -> Self {
        Self { plan, part_index }
    }

    pub(crate) fn __repr__(&self) -> String {
        format!(
            "MachinePlanningOptions(plan={}, part_index={})",
            self.plan.__repr__(),
            self.part_index
        )
    }
}

impl From<&PyMachinePlanningOptions> for mt::MachinePlanningOptions {
    fn from(p: &PyMachinePlanningOptions) -> Self {
        Self {
            plan: (&p.plan).into(),
            part_index: p.part_index,
        }
    }
}

impl From<mt::MachinePlanningOptions> for PyMachinePlanningOptions {
    fn from(r: mt::MachinePlanningOptions) -> Self {
        Self {
            plan: r.plan.into(),
            part_index: r.part_index,
        }
    }
}

// -----------------------------------------------------------------------------
// PlannerOptions (top-level bundle)
// -----------------------------------------------------------------------------

#[pyclass(name = "PlannerOptions", eq)]
#[derive(Clone, PartialEq)]
pub struct PyPlannerOptions {
    #[pyo3(get, set)]
    pub field: PyFieldGenerationOptions,
    #[pyo3(get, set)]
    pub routing: PyRoutingOptions,
    #[pyo3(get, set)]
    pub turn: PyTurnPlannerConfig,
    #[pyo3(get, set)]
    pub machines: PyMachinePlanningOptions,
}

#[pymethods]
impl PyPlannerOptions {
    #[new]
    #[pyo3(signature = (field = None, routing = None, turn = None, machines = None))]
    fn new(
        field: Option<PyFieldGenerationOptions>,
        routing: Option<PyRoutingOptions>,
        turn: Option<PyTurnPlannerConfig>,
        machines: Option<PyMachinePlanningOptions>,
    ) -> Self {
        Self {
            field: field.unwrap_or_else(|| mt::FieldGenerationOptions::default().into()),
            routing: routing.unwrap_or_else(|| mt::RoutingOptions::default().into()),
            turn: turn.unwrap_or_else(|| mt::TurnPlannerConfig::default().into()),
            machines: machines.unwrap_or_else(|| mt::MachinePlanningOptions::default().into()),
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        format!(
            "PlannerOptions(field={}, routing={}, turn={}, machines={})",
            self.field.__repr__(),
            self.routing.__repr__(),
            self.turn.__repr__(),
            self.machines.__repr__(),
        )
    }
}

impl From<&PyPlannerOptions> for mt::PlannerOptions {
    fn from(p: &PyPlannerOptions) -> Self {
        Self {
            field: (&p.field).into(),
            routing: (&p.routing).into(),
            turn: (&p.turn).into(),
            machines: (&p.machines).into(),
        }
    }
}

impl From<mt::PlannerOptions> for PyPlannerOptions {
    fn from(r: mt::PlannerOptions) -> Self {
        Self {
            field: r.field.into(),
            routing: r.routing.into(),
            turn: r.turn.into(),
            machines: r.machines.into(),
        }
    }
}

// -----------------------------------------------------------------------------
// Registration
// -----------------------------------------------------------------------------

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyMachineProfile>()?;
    module.add_class::<PySwathAngleSearchOptions>()?;
    module.add_class::<PyFieldGenerationMode>()?;
    module.add_class::<PyFieldGenerationOptions>()?;
    module.add_class::<PyRoutingOptions>()?;
    module.add_class::<PyTurnPlannerConfig>()?;
    module.add_class::<PyDivisionPlan>()?;
    module.add_class::<PyMachinePlanningOptions>()?;
    module.add_class::<PyPlannerOptions>()?;
    Ok(())
}
