use concord::{Geo, Wgs, to_enu};
use maptrax::{
    DecompositionMode, Field, SwathAngleSearchOptions, SwathObjective,
    generate_headlands_for_polygon, generate_swaths_for_polygon, polygon_area, polygon_from_points,
    segment_length,
};
use maptrax::{Point2Ext, Polygon, point_xy};

fn test_polygon() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(100.0, 0.0),
        point_xy(100.0, 50.0),
        point_xy(0.0, 50.0),
    ])
}

fn irregular_fixture_polygon() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(120.0, 10.0),
        point_xy(140.0, 60.0),
        point_xy(90.0, 95.0),
        point_xy(30.0, 85.0),
        point_xy(-10.0, 40.0),
    ])
}

fn concave_fixture_polygon() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(120.0, 0.0),
        point_xy(120.0, 30.0),
        point_xy(70.0, 30.0),
        point_xy(70.0, 90.0),
        point_xy(0.0, 90.0),
    ])
}

#[test]
fn field_constructor_and_area_work() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let field = Field::new(test_polygon(), datum).expect("field");

    assert_eq!(field.get_parts().len(), 1);
    assert_eq!(field.datum(), datum);
    assert!((field.total_area() - 5_000.0).abs() < 1e-9);
    assert!(polygon_area(field.get_border()) > 0.0);
    assert_eq!(field.get_border(), &field.get_parts()[0].boundary.polygon);
}

#[test]
fn field_generation_creates_swaths_for_rectangles() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(test_polygon(), datum).expect("field");
    field.gen_field(10.0, 90.0, 0).expect("generated");

    let part = &field.get_parts()[0];
    assert!(part.headlands.is_empty());
    assert_eq!(part.swaths.len(), 10);
    assert!(part.swaths.iter().all(|swath| swath.width == 10.0));
    assert!(
        part.swaths
            .iter()
            .all(|swath| (swath.line.start.x - swath.line.end.x).abs() < 1e-9)
    );
}

#[test]
fn field_generation_supports_non_orthogonal_angles() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(test_polygon(), datum).expect("field");
    field.gen_field(10.0, 45.0, 0).expect("generated");

    let part = &field.get_parts()[0];
    assert!(!part.swaths.is_empty());
    assert!(
        part.swaths
            .iter()
            .all(|swath| segment_length(swath.line) > 0.0)
    );
}

#[test]
fn headlands_are_generated_for_rectangular_fields() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(test_polygon(), datum).expect("field");
    field.gen_field(10.0, 90.0, 1).expect("generated");

    let part = &field.get_parts()[0];
    assert_eq!(part.headlands.len(), 1);
    assert!(polygon_area(&part.headlands[0].polygon).abs() < field.total_area());
    assert!(!part.swaths.is_empty());
}

#[test]
fn headland_generation_can_be_run_as_a_separate_stage() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(test_polygon(), datum).expect("field");

    field.generate_headlands(10.0, 2).expect("headlands");
    let part = &field.get_parts()[0];
    assert_eq!(part.headlands.len(), 2);
    assert!(part.swaths.is_empty());

    field.generate_swaths(10.0, 90.0).expect("swaths");
    let part = &field.get_parts()[0];
    assert!(!part.swaths.is_empty());
    assert!(part.swaths.iter().all(|swath| swath.width == 10.0));
}

#[test]
fn standalone_generation_helpers_are_deterministic() {
    let headlands_a = generate_headlands_for_polygon(&test_polygon(), 10.0, 2);
    let headlands_b = generate_headlands_for_polygon(&test_polygon(), 10.0, 2);
    assert_eq!(headlands_a, headlands_b);

    let swaths_a = generate_swaths_for_polygon(10.0, 90.0, &test_polygon());
    let swaths_b = generate_swaths_for_polygon(10.0, 90.0, &test_polygon());
    assert_eq!(swaths_a, swaths_b);
    assert_eq!(
        swaths_a
            .iter()
            .map(|swath| swath.uuid.clone())
            .collect::<Vec<_>>(),
        vec![
            "swath_1", "swath_2", "swath_3", "swath_4", "swath_5", "swath_6", "swath_7", "swath_8",
            "swath_9", "swath_10",
        ]
    );
}

#[test]
fn swath_points_and_bounding_boxes_match_generated_geometry() {
    let swaths = generate_swaths_for_polygon(10.0, 45.0, &test_polygon());
    assert!(!swaths.is_empty());

    for swath in swaths {
        assert_eq!(swath.points.len(), 2);
        assert_eq!(swath.points[0], swath.head());
        assert_eq!(swath.points[1], swath.tail());
        assert!(swath.bounding_box.min_point.x <= swath.head().x());
        assert!(swath.bounding_box.max_point.x >= swath.tail().x());
        assert!(swath.bounding_box.min_point.y <= swath.head().y());
        assert!(swath.bounding_box.max_point.y >= swath.tail().y());
        let dx = swath.tail().x() - swath.head().x();
        let dy = swath.tail().y() - swath.head().y();
        assert!(dx * 45f64.to_radians().cos() + dy * 45f64.to_radians().sin() >= -1e-9);
    }
}

#[test]
fn headlands_are_generated_for_irregular_upstream_shape() {
    let datum = Geo::new(51.98954034749562, 5.6584737410504715, 53.801823);
    let polygon = polygon_from_points(
        [
            Wgs::new(51.98765392402663, 5.660072928621929, 0.0),
            Wgs::new(51.98816428304869, 5.661754957062072, 0.0),
            Wgs::new(51.989850316694316, 5.660416700858434, 0.0),
            Wgs::new(51.990417354104295, 5.662166255987472, 0.0),
            Wgs::new(51.991078888673854, 5.660969191951295, 0.0),
            Wgs::new(51.989479848375254, 5.656874619070777, 0.0),
            Wgs::new(51.988156722216644, 5.657715633290422, 0.0),
            Wgs::new(51.98765392402663, 5.660072928621929, 0.0),
        ]
        .into_iter()
        .map(|wgs| {
            let enu = to_enu(datum, wgs);
            point_xy(enu.east(), enu.north())
        })
        .collect(),
    );

    let mut field = Field::new(polygon, datum).expect("field");
    field.gen_field(4.0, 0.0, 3).expect("generated");

    let part = &field.get_parts()[0];
    assert_eq!(part.headlands.len(), 3);
    assert!(part.swaths.len() > 50);

    let areas: Vec<f64> = part
        .headlands
        .iter()
        .map(|ring| polygon_area(&ring.polygon).abs())
        .collect();
    assert!(areas.windows(2).all(|pair| pair[1] < pair[0]));
}

#[test]
fn irregular_fixture_counts_are_stable() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(irregular_fixture_polygon(), datum).expect("field");
    field.gen_field(8.0, 0.0, 2).expect("generated");

    let part = &field.get_parts()[0];
    assert_eq!(part.headlands.len(), 2);
    assert_eq!(part.swaths.len(), 7);

    let areas: Vec<f64> = part
        .headlands
        .iter()
        .map(|ring| polygon_area(&ring.polygon).abs())
        .collect();
    assert!(areas.windows(2).all(|pair| pair[1] < pair[0]));
}

#[test]
fn min_swath_count_objective_chooses_expected_rectangle_angle() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(test_polygon(), datum).expect("field");

    let result = field
        .generate_with_objective(
            10.0,
            SwathObjective::ApproxMinSwathCount,
            SwathAngleSearchOptions {
                start_degrees: 0.0,
                end_degrees: 180.0,
                step_degrees: 90.0,
            },
            0,
        )
        .expect("objective");

    assert_eq!(result.len(), 1);
    assert_eq!(result[0].angle_degrees, 0.0);
    assert_eq!(field.get_parts()[0].swaths.len(), 6);
}

#[test]
fn exact_count_and_length_objectives_diverge_on_irregular_polygon() {
    let datum = Geo::new(51.0, 5.0, 0.0);

    let mut exact_field = Field::new(irregular_fixture_polygon(), datum).expect("field");
    let exact = exact_field
        .generate_with_objective(
            8.0,
            SwathObjective::ExactSwathCount(8),
            SwathAngleSearchOptions {
                start_degrees: 0.0,
                end_degrees: 170.0,
                step_degrees: 10.0,
            },
            0,
        )
        .expect("objective");

    let mut length_field = Field::new(irregular_fixture_polygon(), datum).expect("field");
    let length = length_field
        .generate_with_objective(
            8.0,
            SwathObjective::TotalSwathLength,
            SwathAngleSearchOptions {
                start_degrees: 0.0,
                end_degrees: 170.0,
                step_degrees: 10.0,
            },
            0,
        )
        .expect("objective");

    assert_eq!(exact.len(), 1);
    assert_eq!(length.len(), 1);
    assert_ne!(exact[0].angle_degrees, length[0].angle_degrees);
    assert_eq!(exact[0].angle_degrees, 10.0);
    assert_eq!(length[0].angle_degrees, 150.0);
    assert_eq!(exact_field.get_parts()[0].swaths.len(), 11);
}

#[test]
fn objective_search_matches_compatibility_auto_on_upstream_fixture() {
    let datum = Geo::new(51.98954034749562, 5.6584737410504715, 53.801823);
    let polygon = polygon_from_points(
        [
            Wgs::new(51.98765392402663, 5.660072928621929, 0.0),
            Wgs::new(51.98816428304869, 5.661754957062072, 0.0),
            Wgs::new(51.989850316694316, 5.660416700858434, 0.0),
            Wgs::new(51.990417354104295, 5.662166255987472, 0.0),
            Wgs::new(51.991078888673854, 5.660969191951295, 0.0),
            Wgs::new(51.989479848375254, 5.656874619070777, 0.0),
            Wgs::new(51.988156722216644, 5.657715633290422, 0.0),
            Wgs::new(51.98765392402663, 5.660072928621929, 0.0),
        ]
        .into_iter()
        .map(|wgs| {
            let enu = to_enu(datum, wgs);
            point_xy(enu.east(), enu.north())
        })
        .collect(),
    );

    let mut auto_field = Field::new(polygon.clone(), datum).expect("field");
    auto_field.gen_field(4.0, 0.0, 3).expect("generated");
    let auto_count = auto_field.get_parts()[0].swaths.len();

    let mut objective_field = Field::new(polygon, datum).expect("field");
    let result = objective_field
        .generate_with_objective(
            4.0,
            SwathObjective::ApproxMinSwathCount,
            SwathAngleSearchOptions::default(),
            3,
        )
        .expect("objective");

    assert_eq!(result.len(), 1);
    assert_eq!(objective_field.get_parts()[0].swaths.len(), auto_count);
}

#[test]
fn concave_field_decomposition_splits_into_multiple_parts() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(concave_fixture_polygon(), datum).expect("field");

    let part_count = field
        .decompose(DecompositionMode::ConcaveSplit)
        .expect("decompose");

    assert_eq!(part_count, 2);
    assert_eq!(field.get_parts().len(), 2);

    let total_area: f64 = field
        .get_parts()
        .iter()
        .map(|part| polygon_area(&part.boundary.polygon).abs())
        .sum();
    assert!((total_area - field.total_area()).abs() < 1e-6);
}

#[test]
fn decomposition_allows_swath_generation_per_part() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(concave_fixture_polygon(), datum).expect("field");
    field
        .decompose(DecompositionMode::ConcaveSplit)
        .expect("decompose");
    field.gen_field(10.0, 90.0, 0).expect("generated");

    assert_eq!(field.get_parts().len(), 2);
    assert!(field.get_parts().iter().all(|part| !part.swaths.is_empty()));
}
