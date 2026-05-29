use concord::{Geo, Wgs, to_enu};
use maptrax::{Point, Point2Ext, Polygon, point_xy};
use maptrax::{
    Balance, DivisionPattern, DivisionPlan, Divy, Dubins, Field, Maptrax, Nety, Pose2D, ReedsShepp,
    Sharper, SwathType, TourBuilder, TurnPlannerConfig, polygon_from_points,
};

fn stripe_plan(machines: usize) -> DivisionPlan {
    DivisionPlan::uniform(
        machines,
        DivisionPattern::Stripe { stride: 1 },
        Balance::ByCount,
    )
}

fn rect() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(100.0, 0.0),
        point_xy(100.0, 50.0),
        point_xy(0.0, 50.0),
    ])
}

fn upstream_polygon(datum: Geo) -> Polygon {
    polygon_from_points(
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
    )
}

#[test]
fn upstream_test_surface_has_rust_parity_coverage() {
    let datum = Geo::new(51.0, 5.0, 0.0);

    let mut field = Field::with_options(rect(), datum, true, 1000.0, false).expect("field");
    assert_eq!(field.get_parts().len(), 1);
    assert!(field.total_area() > 0.0);
    field.gen_field(10.0, 90.0, 1).expect("generated");
    assert!(!field.get_parts()[0].swaths.is_empty());

    let result = Divy::plan(&field.get_parts()[0], &stripe_plan(2)).expect("divy");
    assert_eq!(
        result
            .swaths_per_machine
            .iter()
            .map(Vec::len)
            .sum::<usize>(),
        field.get_parts()[0].swaths.len()
    );

    let mut nety = Nety::new(&field.get_parts()[0].swaths);
    nety.field_traversal(None);
    assert_eq!(
        nety.get_swaths()
            .iter()
            .filter(|swath| swath.r#type == SwathType::Swath)
            .count(),
        field.get_parts()[0].swaths.len()
    );

    let start = Pose2D::new(0.0, 0.0, 0.0);
    let goal = Pose2D::new(5.0, 2.0, 1.0);
    assert!(Dubins::new(1.0).plan_path(start, goal, 0.2).total_length > 0.0);
    assert!(
        ReedsShepp::new(1.0)
            .plan_path(start, goal, 0.2)
            .total_length
            > 0.0
    );
    assert!(
        !Sharper::new(1.0, 4.0, 2.0)
            .plan_sharp_turn(start, Pose2D::new(0.0, 0.0, std::f64::consts::PI), "auto")
            .waypoints
            .is_empty()
    );

    let tour = TourBuilder::build(
        &field.get_parts()[0],
        nety.get_swaths(),
        &TurnPlannerConfig::default(),
    );
    assert!(tour.len() >= nety.get_swaths().len());

    let mut facade = Maptrax::new();
    facade.set_field(rect(), datum).expect("facade field");
    facade
        .generate_field(10.0, 90.0, 1)
        .expect("facade generate");
    let facade_nety = facade.make_nety_from_part(0).expect("facade net");
    let facade_tour = facade
        .build_tour(0, facade_nety.get_swaths(), &TurnPlannerConfig::default())
        .expect("facade tour");
    assert!(facade_tour.len() >= facade_nety.get_swaths().len());
}

#[test]
fn upstream_scene_matches_farmtrax_probe_counts() {
    let datum = Geo::new(51.98954034749562, 5.6584737410504715, 53.801823);
    let polygon = upstream_polygon(datum);

    let mut field = Field::new(polygon, datum).expect("field");
    field.gen_field(4.0, 0.0, 3).expect("generated");
    let part = &field.get_parts()[0];
    assert_eq!(part.headlands.len(), 3);
    assert_eq!(part.swaths.len(), 71);

    let mut nety = Nety::new(&part.swaths);
    nety.field_traversal(None);
    assert_eq!(nety.get_swaths().len(), 141);
    let ordered = nety
        .get_swaths()
        .iter()
        .filter(|swath| swath.r#type == SwathType::Swath)
        .cloned()
        .collect::<Vec<_>>();
    assert_eq!(ordered.len(), 71);

    let tour = TourBuilder::build(part, &ordered, &TurnPlannerConfig::default());
    assert!(tour.len() > ordered.len());
    assert!(
        tour.iter()
            .any(|swath| swath.r#type == SwathType::Connection)
    );
}

#[test]
fn upstream_main_cpp_flow_matches_probe_counts() {
    let datum = Geo::new(51.98954034749562, 5.6584737410504715, 53.801823);
    let polygon = upstream_polygon(datum);

    let mut field = Field::new(polygon, datum).expect("field");
    field.gen_field(4.0, 0.0, 3).expect("generated");

    let division = Divy::plan(&field.get_parts()[0], &stripe_plan(4)).expect("divy");
    let assigned: Vec<usize> = division.swaths_per_machine.iter().map(Vec::len).collect();
    assert_eq!(assigned, vec![18, 18, 18, 17]);

    let mut nety = Nety::new(&field.get_parts()[0].swaths);
    nety.field_traversal(None);
    assert_eq!(nety.get_swaths().len(), 141);
}
