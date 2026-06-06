//! Plain (unit-variant) pyclass enums.
//!
//! These mirror the corresponding Rust crate enums and convert losslessly via
//! `From`. Member names follow PEP 8 (`SCREAMING_SNAKE_CASE`); Rust-side variant
//! names stay CamelCase. Each enum supports `==`, ordering against ints
//! (`eq_int`), and hashing.
//!
//! Tagged-union enums (variants carrying data, e.g. `DivisionPattern::Stripe
//! { stride }`) are handled separately - they need classmethod constructors and
//! land in a follow-up.

use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate as mt;

// -----------------------------------------------------------------------------
// Balance
// -----------------------------------------------------------------------------

#[pyclass(name = "Balance", eq, eq_int, frozen, hash)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PyBalance {
    #[pyo3(name = "BY_COUNT")]
    ByCount,
    #[pyo3(name = "BY_LENGTH")]
    ByLength,
}

impl From<PyBalance> for mt::Balance {
    fn from(v: PyBalance) -> Self {
        match v {
            PyBalance::ByCount => mt::Balance::ByCount,
            PyBalance::ByLength => mt::Balance::ByLength,
        }
    }
}

impl From<mt::Balance> for PyBalance {
    fn from(v: mt::Balance) -> Self {
        match v {
            mt::Balance::ByCount => PyBalance::ByCount,
            mt::Balance::ByLength => PyBalance::ByLength,
        }
    }
}

// -----------------------------------------------------------------------------
// RoutingStrategy
// -----------------------------------------------------------------------------

#[pyclass(name = "RoutingStrategy", eq, eq_int, frozen, hash)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PyRoutingStrategy {
    #[pyo3(name = "GREEDY_NEAREST")]
    GreedyNearest,
    #[pyo3(name = "SNAKE")]
    Snake,
    #[pyo3(name = "SPIRAL")]
    Spiral,
    /// Skip-row ordering. As a plain enum this carries no stride (defaults to
    /// 1); use the `routing_strategy="skip_rows"` + `stride=` keyword on
    /// `plan_machines` to set a concrete stride.
    #[pyo3(name = "SKIP_ROWS")]
    SkipRows,
    #[pyo3(name = "TURN_RADIUS_AWARE")]
    TurnRadiusAware,
}

impl From<PyRoutingStrategy> for mt::RoutingStrategy {
    fn from(v: PyRoutingStrategy) -> Self {
        match v {
            PyRoutingStrategy::GreedyNearest => mt::RoutingStrategy::GreedyNearest,
            PyRoutingStrategy::Snake => mt::RoutingStrategy::Snake,
            PyRoutingStrategy::Spiral => mt::RoutingStrategy::Spiral,
            PyRoutingStrategy::SkipRows => mt::RoutingStrategy::SkipRows { stride: 1 },
            PyRoutingStrategy::TurnRadiusAware => mt::RoutingStrategy::TurnRadiusAware,
        }
    }
}

impl From<mt::RoutingStrategy> for PyRoutingStrategy {
    fn from(v: mt::RoutingStrategy) -> Self {
        match v {
            mt::RoutingStrategy::GreedyNearest => PyRoutingStrategy::GreedyNearest,
            mt::RoutingStrategy::Snake => PyRoutingStrategy::Snake,
            mt::RoutingStrategy::Spiral => PyRoutingStrategy::Spiral,
            mt::RoutingStrategy::SkipRows { .. } => PyRoutingStrategy::SkipRows,
            mt::RoutingStrategy::TurnRadiusAware => PyRoutingStrategy::TurnRadiusAware,
        }
    }
}

// -----------------------------------------------------------------------------
// TurnPlannerModel
// -----------------------------------------------------------------------------

#[pyclass(name = "TurnPlannerModel", eq, eq_int, frozen, hash)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PyTurnPlannerModel {
    #[pyo3(name = "AUTO")]
    Auto,
    #[pyo3(name = "DUBINS")]
    Dubins,
    #[pyo3(name = "REEDS_SHEPP")]
    ReedsShepp,
    #[pyo3(name = "SHARPER")]
    Sharper,
}

impl From<PyTurnPlannerModel> for mt::TurnPlannerModel {
    fn from(v: PyTurnPlannerModel) -> Self {
        match v {
            PyTurnPlannerModel::Auto => mt::TurnPlannerModel::Auto,
            PyTurnPlannerModel::Dubins => mt::TurnPlannerModel::Dubins,
            PyTurnPlannerModel::ReedsShepp => mt::TurnPlannerModel::ReedsShepp,
            PyTurnPlannerModel::Sharper => mt::TurnPlannerModel::Sharper,
        }
    }
}

impl From<mt::TurnPlannerModel> for PyTurnPlannerModel {
    fn from(v: mt::TurnPlannerModel) -> Self {
        match v {
            mt::TurnPlannerModel::Auto => PyTurnPlannerModel::Auto,
            mt::TurnPlannerModel::Dubins => PyTurnPlannerModel::Dubins,
            mt::TurnPlannerModel::ReedsShepp => PyTurnPlannerModel::ReedsShepp,
            mt::TurnPlannerModel::Sharper => PyTurnPlannerModel::Sharper,
        }
    }
}

// -----------------------------------------------------------------------------
// ConnectorMode
// -----------------------------------------------------------------------------

#[pyclass(name = "ConnectorMode", eq, eq_int, frozen, hash)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PyConnectorMode {
    #[pyo3(name = "AUTO")]
    Auto,
    #[pyo3(name = "DIRECT")]
    Direct,
    #[pyo3(name = "HEADLAND")]
    Headland,
}

impl From<PyConnectorMode> for mt::ConnectorMode {
    fn from(v: PyConnectorMode) -> Self {
        match v {
            PyConnectorMode::Auto => mt::ConnectorMode::Auto,
            PyConnectorMode::Direct => mt::ConnectorMode::Direct,
            PyConnectorMode::Headland => mt::ConnectorMode::Headland,
        }
    }
}

impl From<mt::ConnectorMode> for PyConnectorMode {
    fn from(v: mt::ConnectorMode) -> Self {
        match v {
            mt::ConnectorMode::Auto => PyConnectorMode::Auto,
            mt::ConnectorMode::Direct => PyConnectorMode::Direct,
            mt::ConnectorMode::Headland => PyConnectorMode::Headland,
        }
    }
}

// -----------------------------------------------------------------------------
// SwathType
// -----------------------------------------------------------------------------

#[pyclass(name = "SwathType", eq, eq_int, frozen, hash)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PySwathType {
    #[pyo3(name = "SWATH")]
    Swath,
    #[pyo3(name = "CONNECTION")]
    Connection,
    #[pyo3(name = "AROUND")]
    Around,
    #[pyo3(name = "HEADLAND")]
    Headland,
}

impl From<PySwathType> for mt::SwathType {
    fn from(v: PySwathType) -> Self {
        match v {
            PySwathType::Swath => mt::SwathType::Swath,
            PySwathType::Connection => mt::SwathType::Connection,
            PySwathType::Around => mt::SwathType::Around,
            PySwathType::Headland => mt::SwathType::Headland,
        }
    }
}

impl From<mt::SwathType> for PySwathType {
    fn from(v: mt::SwathType) -> Self {
        match v {
            mt::SwathType::Swath => PySwathType::Swath,
            mt::SwathType::Connection => PySwathType::Connection,
            mt::SwathType::Around => PySwathType::Around,
            mt::SwathType::Headland => PySwathType::Headland,
        }
    }
}

// -----------------------------------------------------------------------------
// DubinsSegmentType
// -----------------------------------------------------------------------------

#[pyclass(name = "DubinsSegmentType", eq, eq_int, frozen, hash)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PyDubinsSegmentType {
    #[pyo3(name = "LEFT")]
    Left,
    #[pyo3(name = "STRAIGHT")]
    Straight,
    #[pyo3(name = "RIGHT")]
    Right,
}

impl From<PyDubinsSegmentType> for mt::DubinsSegmentType {
    fn from(v: PyDubinsSegmentType) -> Self {
        match v {
            PyDubinsSegmentType::Left => mt::DubinsSegmentType::Left,
            PyDubinsSegmentType::Straight => mt::DubinsSegmentType::Straight,
            PyDubinsSegmentType::Right => mt::DubinsSegmentType::Right,
        }
    }
}

impl From<mt::DubinsSegmentType> for PyDubinsSegmentType {
    fn from(v: mt::DubinsSegmentType) -> Self {
        match v {
            mt::DubinsSegmentType::Left => PyDubinsSegmentType::Left,
            mt::DubinsSegmentType::Straight => PyDubinsSegmentType::Straight,
            mt::DubinsSegmentType::Right => PyDubinsSegmentType::Right,
        }
    }
}

// -----------------------------------------------------------------------------
// ReedsSheppSegmentType
// -----------------------------------------------------------------------------

#[pyclass(name = "ReedsSheppSegmentType", eq, eq_int, frozen, hash)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PyReedsSheppSegmentType {
    #[pyo3(name = "LEFT_FORWARD")]
    LeftForward,
    #[pyo3(name = "STRAIGHT_FORWARD")]
    StraightForward,
    #[pyo3(name = "RIGHT_FORWARD")]
    RightForward,
    #[pyo3(name = "LEFT_BACKWARD")]
    LeftBackward,
    #[pyo3(name = "STRAIGHT_BACKWARD")]
    StraightBackward,
    #[pyo3(name = "RIGHT_BACKWARD")]
    RightBackward,
}

impl From<PyReedsSheppSegmentType> for mt::ReedsSheppSegmentType {
    fn from(v: PyReedsSheppSegmentType) -> Self {
        match v {
            PyReedsSheppSegmentType::LeftForward => mt::ReedsSheppSegmentType::LeftForward,
            PyReedsSheppSegmentType::StraightForward => mt::ReedsSheppSegmentType::StraightForward,
            PyReedsSheppSegmentType::RightForward => mt::ReedsSheppSegmentType::RightForward,
            PyReedsSheppSegmentType::LeftBackward => mt::ReedsSheppSegmentType::LeftBackward,
            PyReedsSheppSegmentType::StraightBackward => {
                mt::ReedsSheppSegmentType::StraightBackward
            }
            PyReedsSheppSegmentType::RightBackward => mt::ReedsSheppSegmentType::RightBackward,
        }
    }
}

impl From<mt::ReedsSheppSegmentType> for PyReedsSheppSegmentType {
    fn from(v: mt::ReedsSheppSegmentType) -> Self {
        match v {
            mt::ReedsSheppSegmentType::LeftForward => PyReedsSheppSegmentType::LeftForward,
            mt::ReedsSheppSegmentType::StraightForward => PyReedsSheppSegmentType::StraightForward,
            mt::ReedsSheppSegmentType::RightForward => PyReedsSheppSegmentType::RightForward,
            mt::ReedsSheppSegmentType::LeftBackward => PyReedsSheppSegmentType::LeftBackward,
            mt::ReedsSheppSegmentType::StraightBackward => {
                PyReedsSheppSegmentType::StraightBackward
            }
            mt::ReedsSheppSegmentType::RightBackward => PyReedsSheppSegmentType::RightBackward,
        }
    }
}

// -----------------------------------------------------------------------------
// Registration
// -----------------------------------------------------------------------------

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyBalance>()?;
    module.add_class::<PyRoutingStrategy>()?;
    module.add_class::<PyTurnPlannerModel>()?;
    module.add_class::<PyConnectorMode>()?;
    module.add_class::<PySwathType>()?;
    module.add_class::<PyDubinsSegmentType>()?;
    module.add_class::<PyReedsSheppSegmentType>()?;
    Ok(())
}
