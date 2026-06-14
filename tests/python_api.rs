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

        let part_count: usize = planner
            .call_method0("part_count")
            .expect("part_count")
            .extract()
            .expect("extract");
        assert_eq!(part_count, 1);

        let area: f64 = planner
            .call_method0("total_area")
            .expect("total_area")
            .extract()
            .expect("extract");
        assert!(area > 0.0);

        let part = planner
            .call_method1("get_part", (0_usize,))
            .expect("get_part")
            .downcast_into::<PyDict>()
            .expect("dict");
        let boundary = part
            .get_item("boundary")
            .expect("boundary")
            .expect("boundary value");
        let swaths = part
            .get_item("swaths")
            .expect("swaths")
            .expect("swaths value");
        assert!(boundary.is_instance_of::<PyDict>());
        assert!(swaths.len().expect("swaths len") > 0);

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

        let staged = planner
            .call_method0("plan_stages")
            .expect("plan_stages")
            .downcast_into::<PyDict>()
            .expect("dict");
        let headlands = staged
            .get_item("headlands")
            .expect("headlands")
            .expect("headlands value");
        let generated = staged
            .get_item("generated_swaths")
            .expect("generated")
            .expect("generated value");
        let routed = planner
            .call_method0("route_part")
            .expect("route_part")
            .downcast_into::<pyo3::types::PyList>()
            .expect("list");
        assert!(headlands.len().expect("headlands len") > 0);
        assert!(generated.len().expect("generated len") > 0);
        assert!(routed.len() > 0);

        let rs = planner
            .call_method1(
                "plan_reeds_shepp",
                (
                    (0.0_f64, 0.0_f64, 0.0_f64),
                    (0.0_f64, 18.0_f64, std::f64::consts::PI),
                    4.0_f64,
                ),
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

        let dubins_paths = planner
            .call_method1(
                "plan_all_dubins",
                (
                    (0.0_f64, 0.0_f64, 0.0_f64),
                    (10.0_f64, 0.0_f64, 0.0_f64),
                    2.0_f64,
                ),
            )
            .expect("plan_all_dubins")
            .downcast_into::<pyo3::types::PyList>()
            .expect("list");
        let reeds_paths = planner
            .call_method1(
                "plan_all_reeds_shepp",
                (
                    (0.0_f64, 0.0_f64, 0.0_f64),
                    (0.0_f64, 18.0_f64, std::f64::consts::PI),
                    4.0_f64,
                ),
            )
            .expect("plan_all_reeds_shepp")
            .downcast_into::<pyo3::types::PyList>()
            .expect("list");
        assert!(dubins_paths.len() > 0);
        assert!(reeds_paths.len() > 0);

        let turn_cls = module.getattr("TurnPlannerConfig").expect("turn config");
        let turn_kwargs = PyDict::new(py);
        turn_kwargs
            .set_item("machine_length", 6.0_f64)
            .expect("set length");
        turn_kwargs
            .set_item("machine_width", 8.0_f64)
            .expect("set width");
        let turn_cfg = turn_cls.call((), Some(&turn_kwargs)).expect("turn cfg");
        let envelope: f64 = turn_cfg
            .call_method0("turning_envelope_radius")
            .expect("turning envelope")
            .extract()
            .expect("extract");
        assert!((envelope - 5.0).abs() < 1e-9);

        let stride_kwargs = PyDict::new(py);
        let turn_model = module.getattr("TurnPlannerModel").expect("turn model");
        stride_kwargs
            .set_item("model", turn_model.getattr("DUBINS").expect("dubins"))
            .expect("set model");
        stride_kwargs
            .set_item("min_turning_radius", 8.0_f64)
            .expect("set radius");
        stride_kwargs
            .set_item("swath_width", 6.0_f64)
            .expect("set swath width");
        let stride_cfg = turn_cls.call((), Some(&stride_kwargs)).expect("stride cfg");
        let stride: usize = stride_cfg
            .call_method0("required_row_skip_stride")
            .expect("row skip stride")
            .extract()
            .expect("extract stride");
        assert_eq!(stride, 3);
    });
}
