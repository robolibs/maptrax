// The extension-module build resolves Python symbols from the host
// interpreter, so a test binary cannot link it. Skip there.
#![cfg(all(
    feature = "python",
    feature = "geojson",
    not(feature = "python-extension")
))]

use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule};

fn planner<'py>(py: Python<'py>) -> Bound<'py, PyAny> {
    let module = PyModule::new(py, "maptrax").expect("module");
    maptrax::python::register_python_module(&module).expect("register");
    let cls = module.getattr("Maptrax").expect("class");
    let planner = cls.call0().expect("planner");
    planner
        .call_method1(
            "set_field",
            (
                vec![
                    (0.0_f64, 0.0_f64),
                    (200.0, 0.0),
                    (200.0, 100.0),
                    (0.0, 100.0),
                ],
                (51.0_f64, 5.0_f64, 0.0_f64),
            ),
        )
        .expect("set_field");
    planner
        .call_method1("generate_field", (10.0_f64, 90.0_f64, 2_usize))
        .expect("generate");
    planner
}

#[test]
fn to_geojson_returns_a_feature_collection() {
    Python::with_gil(|py| {
        let planner = planner(py);
        let text: String = planner
            .call_method0("to_geojson")
            .expect("to_geojson")
            .extract()
            .expect("string");

        assert!(text.contains("\"FeatureCollection\""));
        assert!(text.contains("\"crs\":\"EPSG:4326\""));
        assert!(text.contains("\"type\":\"field\""));
        assert!(text.contains("\"type\":\"headland\""));
    });
}

#[test]
fn crs_keyword_selects_the_frame() {
    Python::with_gil(|py| {
        let planner = planner(py);

        let kwargs = PyDict::new(py);
        kwargs.set_item("crs", "enu").expect("kwarg");
        let text: String = planner
            .call_method("to_geojson", (), Some(&kwargs))
            .expect("to_geojson")
            .extract()
            .expect("string");
        assert!(text.contains("\"crs\":\"ENU\""));
    });
}

#[test]
fn unknown_crs_is_a_value_error() {
    Python::with_gil(|py| {
        let planner = planner(py);
        let kwargs = PyDict::new(py);
        kwargs.set_item("crs", "mercator").expect("kwarg");

        let err = planner
            .call_method("to_geojson", (), Some(&kwargs))
            .expect_err("unknown crs must raise");
        assert!(err.to_string().contains("unknown crs"));
    });
}

#[test]
fn layer_toggles_drop_features() {
    Python::with_gil(|py| {
        let planner = planner(py);

        let kwargs = PyDict::new(py);
        kwargs.set_item("swaths", false).expect("kwarg");
        let text: String = planner
            .call_method("to_geojson", (), Some(&kwargs))
            .expect("to_geojson")
            .extract()
            .expect("string");

        assert!(text.contains("\"type\":\"headland\""));
        assert!(!text.contains("\"type\":\"swath\""));
    });
}

#[test]
fn planned_export_includes_the_tour() {
    Python::with_gil(|py| {
        let planner = planner(py);
        let text: String = planner
            .call_method0("planned_to_geojson")
            .expect("planned_to_geojson")
            .extract()
            .expect("string");

        assert!(text.contains("\"type\":\"tour\""));
        assert!(text.contains("\"order\":"));
    });
}

#[test]
fn machines_export_tags_each_machine() {
    Python::with_gil(|py| {
        let module = PyModule::new(py, "maptrax").expect("module");
        maptrax::python::register_python_module(&module).expect("register");

        let planner = planner(py);

        // Build PlannerOptions(machines=MachinePlanningOptions(plan=...)) so
        // the export plans a real three-way split. `machines` on the
        // constructor is a profile sequence, so go through `uniform`.
        let plan = module
            .getattr("DivisionPlan")
            .expect("DivisionPlan")
            .call_method1("uniform", (3_usize,))
            .expect("plan");

        let machine_cls = module
            .getattr("MachinePlanningOptions")
            .expect("MachinePlanningOptions");
        let machine_kwargs = PyDict::new(py);
        machine_kwargs.set_item("plan", plan).expect("kwarg");
        let machines = machine_cls
            .call((), Some(&machine_kwargs))
            .expect("machine options");

        let options_cls = module.getattr("PlannerOptions").expect("PlannerOptions");
        let options_kwargs = PyDict::new(py);
        options_kwargs
            .set_item("machines", machines)
            .expect("kwarg");
        let options = options_cls
            .call((), Some(&options_kwargs))
            .expect("planner options");

        let call_kwargs = PyDict::new(py);
        call_kwargs.set_item("options", options).expect("kwarg");
        let text: String = planner
            .call_method("machines_to_geojson", (), Some(&call_kwargs))
            .expect("machines_to_geojson")
            .extract()
            .expect("string");

        assert!(text.contains("\"machine_count\":\"3\""));
        for index in 0..3 {
            assert!(
                text.contains(&format!("\"machine\":\"{index}\"")),
                "machine {index} missing"
            );
        }
    });
}

#[test]
fn export_geojson_writes_a_file() {
    Python::with_gil(|py| {
        let planner = planner(py);
        let path = std::env::temp_dir().join("maptrax_python_export.geojson");
        let _ = std::fs::remove_file(&path);

        planner
            .call_method1("export_geojson", (path.to_str().expect("utf-8"),))
            .expect("export_geojson");

        let text = std::fs::read_to_string(&path).expect("written file");
        assert!(text.contains("\"FeatureCollection\""));

        let _ = std::fs::remove_file(&path);
    });
}
