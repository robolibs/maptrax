#![cfg(feature = "python")]

use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule};

#[test]
fn python_module_exposes_maptrax_planning_surface() {
    Python::with_gil(|py| {
        let module = PyModule::new(py, "maptrax").expect("module");
        maptrax::python::register_python_module(&module).expect("register");

        let cls = module.getattr("Maptrax").expect("class");
        let planner = cls.call0().expect("planner");
        planner
            .call_method1(
                "set_field",
                (
                    vec![(0.0_f64, 0.0_f64), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)],
                    (51.0_f64, 5.0_f64, 0.0_f64),
                ),
            )
            .expect("set_field");
        planner
            .call_method1("generate_field", (10.0_f64, 90.0_f64, 1_usize))
            .expect("generate");

        let plan = planner
            .call_method0("plan_part")
            .expect("plan_part")
            .downcast_into::<PyDict>()
            .expect("dict");
        let ordered = plan
            .get_item("ordered_swaths")
            .expect("ordered")
            .expect("ordered value");
        let tour = plan.get_item("tour").expect("tour").expect("tour value");

        let ordered_len: usize = ordered.len().expect("ordered len");
        let tour_len: usize = tour.len().expect("tour len");
        assert!(ordered_len > 0);
        assert!(tour_len >= ordered_len);

        let rs = planner
            .call_method1(
                "plan_reeds_shepp",
                ((0.0_f64, 0.0_f64, 0.0_f64), (0.0_f64, 18.0_f64, std::f64::consts::PI), 4.0_f64),
            )
            .expect("reeds_shepp")
            .downcast_into::<PyDict>()
            .expect("dict");
        let total_length: f64 = rs
            .get_item("total_length")
            .expect("len item")
            .expect("len value")
            .extract()
            .expect("extract");
        assert!(total_length > 0.0);
    });
}
