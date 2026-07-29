#![cfg(feature = "geojson")]

use std::fs;
use std::path::PathBuf;

use maptrax::{
    FieldGenerationMode, FieldGenerationOptions, Geo, GeoJsonOptions, Geometry, Maptrax,
    PlannerOptions, Vector, point_xy, polygon_from_points,
};

const DATUM_LAT: f64 = 51.0;
const DATUM_LON: f64 = 5.0;

fn scratch_path(name: &str) -> PathBuf {
    let path = std::env::temp_dir().join(format!("maptrax_geojson_{name}.geojson"));
    let _ = fs::remove_file(&path);
    path
}

/// A 200x100 rectangle with 10 m rows and two headland rings.
fn planned_planner() -> Maptrax {
    let border = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(200.0, 0.0),
        point_xy(200.0, 100.0),
        point_xy(0.0, 100.0),
    ]);
    let mut planner = Maptrax::new();
    planner
        .set_field(border, Geo::new(DATUM_LAT, DATUM_LON, 0.0))
        .expect("field");
    planner.generate_field(10.0, 90.0, 2).expect("generate");
    planner
}

#[test]
fn field_export_writes_a_feature_collection() {
    let planner = planned_planner();
    let path = scratch_path("field");

    planner
        .export_geojson(&path, &GeoJsonOptions::default())
        .expect("export");

    let text = fs::read_to_string(&path).expect("read back");
    assert!(text.contains("\"FeatureCollection\""));
    assert!(text.contains("\"crs\":\"EPSG:4326\""));
    assert!(text.contains("\"type\":\"field\""));
    assert!(text.contains("\"type\":\"headland\""));
    assert!(text.contains("\"type\":\"swath\""));

    let _ = fs::remove_file(&path);
}

#[test]
fn geometry_only_omits_rows_and_tours() {
    let planner = planned_planner();
    let path = scratch_path("geometry_only");

    planner
        .export_geojson(&path, &GeoJsonOptions::geometry_only())
        .expect("export");

    let text = fs::read_to_string(&path).expect("read back");
    assert!(text.contains("\"type\":\"headland\""));
    assert!(!text.contains("\"type\":\"swath\""));
    assert!(!text.contains("\"type\":\"tour\""));

    let _ = fs::remove_file(&path);
}

#[test]
fn wgs_export_lands_near_the_datum() {
    let planner = planned_planner();
    let path = scratch_path("wgs");

    planner
        .export_geojson(&path, &GeoJsonOptions::geometry_only())
        .expect("export");

    // Reading back converts WGS -> ENU, so the boundary must return to the
    // local metres it started from.
    let loaded = Vector::from_file(&path).expect("round trip");
    let boundary = loaded.field_boundary();
    assert!(boundary.len() >= 4);
    for point in boundary {
        assert!(
            point.x >= -1.0 && point.x <= 201.0,
            "x out of range after round trip: {}",
            point.x
        );
        assert!(
            point.y >= -1.0 && point.y <= 101.0,
            "y out of range after round trip: {}",
            point.y
        );
    }

    let _ = fs::remove_file(&path);
}

#[test]
fn enu_export_keeps_local_metres() {
    let planner = planned_planner();
    let path = scratch_path("enu");

    planner
        .export_geojson(&path, &GeoJsonOptions::geometry_only().in_enu())
        .expect("export");

    let text = fs::read_to_string(&path).expect("read back");
    assert!(text.contains("\"crs\":\"ENU\""));
    // Local metres, not degrees — a longitude near 5.0 would mean the ENU
    // request silently converted.
    assert!(text.contains("200"));

    let _ = fs::remove_file(&path);
}

#[test]
fn ring_polygons_are_closed() {
    let planner = planned_planner();
    let vector = planner
        .to_vector(&GeoJsonOptions::geometry_only())
        .expect("vector");

    let boundary = vector.field_boundary();
    let first = boundary.first().expect("boundary points");
    let last = boundary.last().expect("boundary points");
    assert_eq!(first, last, "field boundary ring must be closed");

    for element in vector.polygons() {
        if let Geometry::Polygon(poly) = &element.geometry {
            let first = poly.vertices.first().expect("ring points");
            let last = poly.vertices.last().expect("ring points");
            assert_eq!(first, last, "ring {} must be closed", element.kind);
        }
    }
}

#[test]
fn planned_export_carries_tour_and_order() {
    let mut planner = planned_planner();
    let planned = planner
        .plan_all(&PlannerOptions {
            field: FieldGenerationOptions {
                swath_width: 10.0,
                headland_count: 2,
                mode: FieldGenerationMode::ExplicitAngle(90.0),
                ..FieldGenerationOptions::default()
            },
            ..PlannerOptions::default()
        })
        .expect("plan");

    let path = scratch_path("planned");
    planner
        .export_planned_geojson(&planned, &path, &GeoJsonOptions::default())
        .expect("export");

    let text = fs::read_to_string(&path).expect("read back");
    assert!(text.contains("\"type\":\"tour\""));
    assert!(text.contains("\"order\":"));
    assert!(text.contains("\"segments\":"));

    let loaded = Vector::from_file(&path).expect("round trip");
    assert_eq!(loaded.elements_by_type("tour").len(), planned.parts.len());

    let _ = fs::remove_file(&path);
}

#[test]
fn export_error_surfaces_as_maptrax_error() {
    let planner = planned_planner();
    let bad = PathBuf::from("/nonexistent-directory-maptrax/out.geojson");

    let err = planner
        .export_geojson(&bad, &GeoJsonOptions::default())
        .expect_err("write to a missing directory must fail");

    assert!(matches!(err, maptrax::MaptraxError::Export(_)));
}

#[test]
fn export_without_a_field_is_an_error() {
    let planner = Maptrax::new();
    let err = planner
        .to_vector(&GeoJsonOptions::default())
        .expect_err("no field set");
    assert!(matches!(err, maptrax::MaptraxError::MissingField));
}

#[test]
fn machine_export_tags_every_machine() {
    use maptrax::{Balance, DivisionPattern, DivisionPlan, MachinePlanningOptions};

    let planner = planned_planner();
    let plan = DivisionPlan::uniform(3, DivisionPattern::Stripe { stride: 1 }, Balance::ByCount);
    let planned = planner
        .plan_machines_for_part(
            &MachinePlanningOptions {
                plan,
                part_index: 0,
            },
            Default::default(),
            &Default::default(),
        )
        .expect("machines");

    let path = scratch_path("machines");
    planner
        .export_machines_geojson(&planned, &path, &GeoJsonOptions::default())
        .expect("export");

    let text = fs::read_to_string(&path).expect("read back");
    assert!(text.contains("\"machine_count\":\"3\""));
    for index in 0..3 {
        assert!(
            text.contains(&format!("\"machine\":\"{index}\"")),
            "machine {index} missing from export"
        );
    }

    let _ = fs::remove_file(&path);
}
