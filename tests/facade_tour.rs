use concord::Geo;
use geo::{Point, Polygon};
use maptrax::{
    DivisionType, Maptrax, SwathType, TourBuilder, TurnPlannerConfig, polygon_from_points,
};

fn test_polygon() -> Polygon {
    polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ])
}

#[test]
fn tour_builder_inserts_connection_swaths() {
    let mut field = maptrax::Field::new(test_polygon(), Geo::new(51.0, 5.0, 0.0)).expect("field");
    field.gen_field(10.0, 90.0, 1).expect("generated");
    let part = &field.get_parts()[0];

    let ordered = part.swaths[..3].to_vec();
    let tour = TourBuilder::build(part, &ordered, &TurnPlannerConfig::default());

    assert!(tour.len() >= ordered.len());
    assert!(
        tour.iter()
            .any(|swath| swath.r#type == SwathType::Connection)
    );
    assert!(
        tour.iter()
            .any(|swath| swath.r#type == SwathType::Connection && swath.points.len() >= 2)
    );
}

#[test]
fn facade_end_to_end_flow_works() {
    let mut mt = Maptrax::new();
    mt.set_field(test_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");
    assert!(mt.has_field());

    mt.generate_field(10.0, 90.0, 1).expect("generate field");
    assert!(!mt.field().unwrap().get_parts().is_empty());
    assert!(!mt.field().unwrap().get_parts()[0].swaths.is_empty());

    let mut divy = mt
        .make_divy(DivisionType::Alternate, 2, 0)
        .expect("make divy");
    divy.compute_division();
    assert!(!divy.result().swaths_per_machine.is_empty());

    let obstacle = polygon_from_points(vec![
        Point::new(45.0, 20.0),
        Point::new(55.0, 20.0),
        Point::new(55.0, 30.0),
        Point::new(45.0, 30.0),
    ]);
    let avoided = mt
        .avoid_obstacles_for_part(vec![obstacle], 2.0, 0)
        .expect("avoid");
    assert!(!avoided.is_empty());

    let nety = mt.make_nety_from_part(0).expect("nety");
    assert!(!nety.get_swaths().is_empty());

    let cfg = TurnPlannerConfig {
        swath_width: 10.0,
        min_turning_radius: 2.0,
        step_size: 0.2,
        headland_threshold_rows: 2.0,
        ..TurnPlannerConfig::default()
    };
    let tour = mt.build_tour(0, nety.get_swaths(), &cfg).expect("tour");
    assert!(tour.len() >= nety.get_swaths().len());
    assert!(
        tour.iter()
            .any(|swath| swath.r#type == SwathType::Connection)
    );
}
