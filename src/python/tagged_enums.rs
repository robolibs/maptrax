//! Tagged-union pyclasses (Rust enums with payload variants).
//!
//! Each Rust enum with a data-carrying variant is wrapped as a single
//! `#[pyclass]`. Variants are constructed via static methods - this mirrors
//! the Rust source while staying idiomatic in Python:
//!
//! ```python
//! DecompositionMode.auto_split(max_side=520.0)
//! DivisionPattern.stripe(stride=1)
//! HeadlandMode.dedicated(machine_index=0)
//! SwathObjective.exact_count(n=42)
//! OptimizeObjective.weighted(makespan=1.0, transit=0.5)
//! ```
//!
//! Every instance exposes a `.kind` SCREAMING_SNAKE string for branching, plus
//! per-payload `Optional[T]` getters (`None` for variants that don't carry
//! that field). `__repr__` mirrors the Python construction call.

use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate as mt;

// -----------------------------------------------------------------------------
// DecompositionMode
// -----------------------------------------------------------------------------

#[pyclass(name = "DecompositionMode", eq, frozen)]
#[derive(Copy, Clone, PartialEq)]
pub struct PyDecompositionMode {
    pub(crate) inner: mt::DecompositionMode,
}

#[pymethods]
impl PyDecompositionMode {
    #[staticmethod]
    fn none() -> Self {
        Self {
            inner: mt::DecompositionMode::None,
        }
    }

    #[staticmethod]
    fn simple_split() -> Self {
        Self {
            inner: mt::DecompositionMode::SimpleSplit,
        }
    }

    #[staticmethod]
    fn concave_split() -> Self {
        Self {
            inner: mt::DecompositionMode::ConcaveSplit,
        }
    }

    #[staticmethod]
    #[pyo3(signature = (max_side))]
    fn auto_split(max_side: f64) -> Self {
        Self {
            inner: mt::DecompositionMode::AutoSplit { max_side },
        }
    }

    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            mt::DecompositionMode::None => "NONE",
            mt::DecompositionMode::SimpleSplit => "SIMPLE_SPLIT",
            mt::DecompositionMode::ConcaveSplit => "CONCAVE_SPLIT",
            mt::DecompositionMode::AutoSplit { .. } => "AUTO_SPLIT",
        }
    }

    #[getter]
    fn max_side(&self) -> Option<f64> {
        match self.inner {
            mt::DecompositionMode::AutoSplit { max_side } => Some(max_side),
            _ => None,
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        match self.inner {
            mt::DecompositionMode::None => "DecompositionMode.none()".into(),
            mt::DecompositionMode::SimpleSplit => "DecompositionMode.simple_split()".into(),
            mt::DecompositionMode::ConcaveSplit => "DecompositionMode.concave_split()".into(),
            mt::DecompositionMode::AutoSplit { max_side } => {
                format!("DecompositionMode.auto_split(max_side={max_side})")
            }
        }
    }
}

impl From<PyDecompositionMode> for mt::DecompositionMode {
    fn from(v: PyDecompositionMode) -> Self {
        v.inner
    }
}

impl From<mt::DecompositionMode> for PyDecompositionMode {
    fn from(inner: mt::DecompositionMode) -> Self {
        Self { inner }
    }
}

// -----------------------------------------------------------------------------
// HeadlandMode
// -----------------------------------------------------------------------------

#[pyclass(name = "HeadlandMode", eq, frozen)]
#[derive(Copy, Clone, PartialEq)]
pub struct PyHeadlandMode {
    pub(crate) inner: mt::HeadlandMode,
}

#[pymethods]
impl PyHeadlandMode {
    #[staticmethod]
    fn one_per_machine() -> Self {
        Self {
            inner: mt::HeadlandMode::OnePerMachine,
        }
    }

    #[staticmethod]
    #[pyo3(signature = (machine_index = 0))]
    fn dedicated(machine_index: usize) -> Self {
        Self {
            inner: mt::HeadlandMode::Dedicated {
                machine: machine_index,
            },
        }
    }

    #[staticmethod]
    fn split_by_zone() -> Self {
        Self {
            inner: mt::HeadlandMode::SplitByZone,
        }
    }

    #[staticmethod]
    fn none() -> Self {
        Self {
            inner: mt::HeadlandMode::None,
        }
    }

    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            mt::HeadlandMode::OnePerMachine => "ONE_PER_MACHINE",
            mt::HeadlandMode::Dedicated { .. } => "DEDICATED",
            mt::HeadlandMode::SplitByZone => "SPLIT_BY_ZONE",
            mt::HeadlandMode::None => "NONE",
        }
    }

    #[getter]
    fn machine_index(&self) -> Option<usize> {
        match self.inner {
            mt::HeadlandMode::Dedicated { machine } => Some(machine),
            _ => None,
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        match self.inner {
            mt::HeadlandMode::OnePerMachine => "HeadlandMode.one_per_machine()".into(),
            mt::HeadlandMode::Dedicated { machine } => {
                format!("HeadlandMode.dedicated(machine_index={machine})")
            }
            mt::HeadlandMode::SplitByZone => "HeadlandMode.split_by_zone()".into(),
            mt::HeadlandMode::None => "HeadlandMode.none()".into(),
        }
    }
}

impl From<PyHeadlandMode> for mt::HeadlandMode {
    fn from(v: PyHeadlandMode) -> Self {
        v.inner
    }
}

impl From<mt::HeadlandMode> for PyHeadlandMode {
    fn from(inner: mt::HeadlandMode) -> Self {
        Self { inner }
    }
}

// -----------------------------------------------------------------------------
// SwathObjective
// -----------------------------------------------------------------------------

#[pyclass(name = "SwathObjective", eq, frozen)]
#[derive(Copy, Clone, PartialEq)]
pub struct PySwathObjective {
    pub(crate) inner: mt::SwathObjective,
}

#[pymethods]
impl PySwathObjective {
    #[staticmethod]
    fn approx_min_count() -> Self {
        Self {
            inner: mt::SwathObjective::ApproxMinSwathCount,
        }
    }

    #[staticmethod]
    #[pyo3(signature = (n))]
    fn exact_count(n: usize) -> Self {
        Self {
            inner: mt::SwathObjective::ExactSwathCount(n),
        }
    }

    #[staticmethod]
    fn total_length() -> Self {
        Self {
            inner: mt::SwathObjective::TotalSwathLength,
        }
    }

    #[staticmethod]
    fn overlap_penalty() -> Self {
        Self {
            inner: mt::SwathObjective::OverlapPenalty,
        }
    }

    #[staticmethod]
    fn coverage_score() -> Self {
        Self {
            inner: mt::SwathObjective::CoverageScore,
        }
    }

    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            mt::SwathObjective::ApproxMinSwathCount => "APPROX_MIN_COUNT",
            mt::SwathObjective::ExactSwathCount(_) => "EXACT_COUNT",
            mt::SwathObjective::TotalSwathLength => "TOTAL_LENGTH",
            mt::SwathObjective::OverlapPenalty => "OVERLAP_PENALTY",
            mt::SwathObjective::CoverageScore => "COVERAGE_SCORE",
        }
    }

    #[getter]
    fn n(&self) -> Option<usize> {
        match self.inner {
            mt::SwathObjective::ExactSwathCount(n) => Some(n),
            _ => None,
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        match self.inner {
            mt::SwathObjective::ApproxMinSwathCount => "SwathObjective.approx_min_count()".into(),
            mt::SwathObjective::ExactSwathCount(n) => {
                format!("SwathObjective.exact_count(n={n})")
            }
            mt::SwathObjective::TotalSwathLength => "SwathObjective.total_length()".into(),
            mt::SwathObjective::OverlapPenalty => "SwathObjective.overlap_penalty()".into(),
            mt::SwathObjective::CoverageScore => "SwathObjective.coverage_score()".into(),
        }
    }
}

impl From<PySwathObjective> for mt::SwathObjective {
    fn from(v: PySwathObjective) -> Self {
        v.inner
    }
}

impl From<mt::SwathObjective> for PySwathObjective {
    fn from(inner: mt::SwathObjective) -> Self {
        Self { inner }
    }
}

// -----------------------------------------------------------------------------
// OptimizeObjective
// -----------------------------------------------------------------------------

#[pyclass(name = "OptimizeObjective", eq, frozen)]
#[derive(Copy, Clone, PartialEq)]
pub struct PyOptimizeObjective {
    pub(crate) inner: mt::OptimizeObjective,
}

#[pymethods]
impl PyOptimizeObjective {
    #[staticmethod]
    fn makespan() -> Self {
        Self {
            inner: mt::OptimizeObjective::Makespan,
        }
    }

    #[staticmethod]
    fn total_transit() -> Self {
        Self {
            inner: mt::OptimizeObjective::TotalTransit,
        }
    }

    #[staticmethod]
    #[pyo3(signature = (makespan = 1.0, transit = 1.0))]
    fn weighted(makespan: f64, transit: f64) -> Self {
        Self {
            inner: mt::OptimizeObjective::Weighted { makespan, transit },
        }
    }

    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            mt::OptimizeObjective::Makespan => "MAKESPAN",
            mt::OptimizeObjective::TotalTransit => "TOTAL_TRANSIT",
            mt::OptimizeObjective::Weighted { .. } => "WEIGHTED",
        }
    }

    #[getter]
    fn makespan_weight(&self) -> Option<f64> {
        match self.inner {
            mt::OptimizeObjective::Weighted { makespan, .. } => Some(makespan),
            _ => None,
        }
    }

    #[getter]
    fn transit_weight(&self) -> Option<f64> {
        match self.inner {
            mt::OptimizeObjective::Weighted { transit, .. } => Some(transit),
            _ => None,
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        match self.inner {
            mt::OptimizeObjective::Makespan => "OptimizeObjective.makespan()".into(),
            mt::OptimizeObjective::TotalTransit => "OptimizeObjective.total_transit()".into(),
            mt::OptimizeObjective::Weighted { makespan, transit } => {
                format!("OptimizeObjective.weighted(makespan={makespan}, transit={transit})")
            }
        }
    }
}

impl From<PyOptimizeObjective> for mt::OptimizeObjective {
    fn from(v: PyOptimizeObjective) -> Self {
        v.inner
    }
}

impl From<mt::OptimizeObjective> for PyOptimizeObjective {
    fn from(inner: mt::OptimizeObjective) -> Self {
        Self { inner }
    }
}

// -----------------------------------------------------------------------------
// DivisionPattern
// -----------------------------------------------------------------------------

#[pyclass(name = "DivisionPattern", eq, frozen)]
#[derive(Copy, Clone, PartialEq)]
pub struct PyDivisionPattern {
    pub(crate) inner: mt::DivisionPattern,
}

#[pymethods]
impl PyDivisionPattern {
    #[staticmethod]
    #[pyo3(signature = (stride = 1))]
    fn stripe(stride: usize) -> Self {
        Self {
            inner: mt::DivisionPattern::Stripe {
                stride: stride.max(1),
            },
        }
    }

    #[staticmethod]
    fn block() -> Self {
        Self {
            inner: mt::DivisionPattern::Block,
        }
    }

    #[staticmethod]
    #[pyo3(signature = (bands = 2))]
    fn banded_stripe(bands: usize) -> Self {
        Self {
            inner: mt::DivisionPattern::BandedStripe {
                bands: bands.max(1),
            },
        }
    }

    #[staticmethod]
    #[pyo3(signature = (objective))]
    fn optimized(objective: PyOptimizeObjective) -> Self {
        Self {
            inner: mt::DivisionPattern::Optimized {
                objective: objective.inner,
            },
        }
    }

    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            mt::DivisionPattern::Stripe { .. } => "STRIPE",
            mt::DivisionPattern::Block => "BLOCK",
            mt::DivisionPattern::BandedStripe { .. } => "BANDED_STRIPE",
            mt::DivisionPattern::Optimized { .. } => "OPTIMIZED",
        }
    }

    #[getter]
    fn stride(&self) -> Option<usize> {
        match self.inner {
            mt::DivisionPattern::Stripe { stride } => Some(stride),
            _ => None,
        }
    }

    #[getter]
    fn bands(&self) -> Option<usize> {
        match self.inner {
            mt::DivisionPattern::BandedStripe { bands } => Some(bands),
            _ => None,
        }
    }

    #[getter]
    fn objective(&self) -> Option<PyOptimizeObjective> {
        match self.inner {
            mt::DivisionPattern::Optimized { objective } => {
                Some(PyOptimizeObjective { inner: objective })
            }
            _ => None,
        }
    }

    pub(crate) fn __repr__(&self) -> String {
        match self.inner {
            mt::DivisionPattern::Stripe { stride } => {
                format!("DivisionPattern.stripe(stride={stride})")
            }
            mt::DivisionPattern::Block => "DivisionPattern.block()".into(),
            mt::DivisionPattern::BandedStripe { bands } => {
                format!("DivisionPattern.banded_stripe(bands={bands})")
            }
            mt::DivisionPattern::Optimized { objective } => {
                let inner = PyOptimizeObjective { inner: objective }.__repr__();
                format!("DivisionPattern.optimized(objective={inner})")
            }
        }
    }
}

impl From<PyDivisionPattern> for mt::DivisionPattern {
    fn from(v: PyDivisionPattern) -> Self {
        v.inner
    }
}

impl From<mt::DivisionPattern> for PyDivisionPattern {
    fn from(inner: mt::DivisionPattern) -> Self {
        Self { inner }
    }
}

// -----------------------------------------------------------------------------
// Registration
// -----------------------------------------------------------------------------

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyDecompositionMode>()?;
    module.add_class::<PyHeadlandMode>()?;
    module.add_class::<PySwathObjective>()?;
    module.add_class::<PyOptimizeObjective>()?;
    module.add_class::<PyDivisionPattern>()?;
    Ok(())
}
