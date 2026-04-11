use concord::Geo;
use geo::Point;
use maptrax::{DivisionType, Divy, Field, polygon_from_points};

fn build_field() -> Field {
    let polygon = polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ]);
    let mut field = Field::new(polygon, Geo::new(51.0, 5.0, 0.0)).expect("field");
    field.gen_field(10.0, 90.0, 0).expect("generated");
    field
}

#[test]
fn divy_rejects_zero_machines() {
    let field = build_field();
    let error = Divy::from_field(&field, DivisionType::Alternate, 0).unwrap_err();
    assert_eq!(error.to_string(), "machine count must be greater than zero");
}

#[test]
fn alternate_division_preserves_total_swath_count() {
    let field = build_field();
    let mut divy = Divy::from_field(&field, DivisionType::Alternate, 2).expect("divy");
    divy.compute_division();

    let assigned: usize = divy.result().swaths_per_machine.iter().map(Vec::len).sum();
    assert_eq!(divy.result().swaths_per_machine.len(), 2);
    assert_eq!(assigned, field.get_parts()[0].swaths.len());
}

#[test]
fn block_division_clusters_spatially() {
    let field = build_field();
    let mut divy = Divy::from_field(&field, DivisionType::Block, 2).expect("divy");
    divy.compute_division();

    let left = &divy.result().swaths_per_machine[0];
    let right = &divy.result().swaths_per_machine[1];
    assert!(!left.is_empty());
    assert!(!right.is_empty());

    let avg_left = left.iter().map(|swath| swath.line.start.x).sum::<f64>() / left.len() as f64;
    let avg_right = right.iter().map(|swath| swath.line.start.x).sum::<f64>() / right.len() as f64;
    assert!((avg_left - avg_right).abs() > 5.0);
}

#[test]
fn changing_machine_count_recomputes_result() {
    let field = build_field();
    let mut divy = Divy::from_field(&field, DivisionType::Alternate, 2).expect("divy");
    divy.compute_division();
    assert_eq!(divy.result().swaths_per_machine.len(), 2);

    divy.set_machine_count(3).expect("machine count");
    divy.compute_division();
    assert_eq!(divy.result().swaths_per_machine.len(), 3);

    let assigned: usize = divy.result().swaths_per_machine.iter().map(Vec::len).sum();
    assert_eq!(assigned, field.get_parts()[0].swaths.len());
}

#[test]
fn length_balanced_and_spatial_divisions_assign_everything() {
    let field = build_field();

    let mut balanced = Divy::from_field(&field, DivisionType::LengthBalanced, 3).expect("balanced");
    balanced.compute_division();
    let balanced_total: usize = balanced
        .result()
        .swaths_per_machine
        .iter()
        .map(Vec::len)
        .sum();
    assert_eq!(balanced_total, field.get_parts()[0].swaths.len());

    let mut spatial = Divy::from_field(&field, DivisionType::SpatialRtree, 3).expect("spatial");
    spatial.compute_division();
    let spatial_total: usize = spatial
        .result()
        .swaths_per_machine
        .iter()
        .map(Vec::len)
        .sum();
    assert_eq!(spatial_total, field.get_parts()[0].swaths.len());
}
