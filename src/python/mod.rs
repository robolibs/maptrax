//! Python bindings for maptrax.
//!
//! Layout (one submodule per concern, mirrors the Rust crate layout):
//!
//! - `enums`   - plain pyclass enums (SCREAMING_SNAKE_CASE members)
//! - `facade`  - the `Maptrax` planner facade
//!
//! The rest of the planned surface (typed option/result classes, standalone
//! `Dubins`/`ReedsShepp`/`Sharper`/`Nety`/`Divy` classes, numpy interop) is
//! introduced module-by-module as the refactor lands.

mod algorithms;
mod domain;
mod enums;
mod facade;
mod options;
mod tagged_enums;

use pyo3::prelude::*;
use pyo3::types::PyModule;

pub fn register_python_module(module: &Bound<'_, PyModule>) -> PyResult<()> {
    enums::register(module)?;
    tagged_enums::register(module)?;
    options::register(module)?;
    domain::register(module)?;
    algorithms::register(module)?;
    facade::register(module)?;
    module.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

#[pymodule]
fn maptrax(module: &Bound<'_, PyModule>) -> PyResult<()> {
    register_python_module(module)
}
