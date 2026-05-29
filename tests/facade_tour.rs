use concord::Geo;
use maptrax::{Point, Point2Ext, Polygon, point_xy};
use maptrax::{
    Balance, ConnectorMode, DecompositionMode, DivisionPattern, DivisionPlan, FieldGenerationMode,
    FieldGenerationOptions, Maptrax, Nety, Part, PlannerOptions, RoutingOptions, RoutingStrategy,
    SwathAngleSearchOptions, SwathObjective, SwathType, TourBuilder, TurnPlannerConfig,
    TurnPlannerModel, create_ring, create_swath, polygon_from_points,
};

fn test_polygon() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(100.0, 0.0),
        point_xy(100.0, 50.0),
        point_xy(0.0, 50.0),
    ])
}

fn concave_polygon() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(120.0, 0.0),
        point_xy(120.0, 30.0),
        point_xy(70.0, 30.0),
        point_xy(70.0, 90.0),
        point_xy(0.0, 90.0),
    ])
}

fn upstream_fixture_polygon() -> Polygon {
    polygon_from_points(vec![
        point_xy(85.7, -209.6),
        point_xy(201.6, -152.9),
        point_xy(109.3, 34.4),
        point_xy(230.0, 97.4),
        point_xy(121.5, 170.9),
        point_xy(-249.8, -6.7),
        point_xy(-173.5, -153.4),
    ])
}

fn simple_part_with_headland() -> Part {
    let boundary_poly = test_polygon();
    let headland_poly = polygon_from_points(vec![
        point_xy(10.0, 10.0),
        point_xy(90.0, 10.0),
        point_xy(90.0, 40.0),
        point_xy(10.0, 40.0),
    ]);
    Part {
        boundary: create_ring(boundary_poly, "boundary").expect("ring"),
        swaths: Vec::new(),
        headlands: vec![create_ring(headland_poly, "headland").expect("ring")],
        non_owned_splits: Vec::new(),
    }
}

fn irregular_part_with_headland() -> Part {
    let boundary = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(120.0, 0.0),
        point_xy(120.0, 70.0),
        point_xy(70.0, 90.0),
        point_xy(0.0, 80.0),
    ]);
    let headland = polygon_from_points(vec![
        point_xy(10.0, 10.0),
        point_xy(105.0, 12.0),
        point_xy(102.0, 58.0),
        point_xy(62.0, 74.0),
        point_xy(12.0, 68.0),
    ]);
    Part {
        boundary: create_ring(boundary, "boundary").expect("ring"),
        swaths: Vec::new(),
        headlands: vec![create_ring(headland, "headland").expect("ring")],
        non_owned_splits: Vec::new(),
    }
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

    let division = mt
        .divide_part(
            0,
            &DivisionPlan::uniform(2, DivisionPattern::Stripe { stride: 1 }, Balance::ByCount),
        )
        .expect("divide part");
    assert!(!division.swaths_per_machine.is_empty());

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

#[test]
fn staged_planning_exposes_intermediate_outputs() {
    let mut mt = Maptrax::new();
    mt.set_field(test_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");

    let staged = mt
        .plan_stages(&PlannerOptions {
            field: FieldGenerationOptions {
                swath_width: 10.0,
                headland_count: 1,
                mode: FieldGenerationMode::ExplicitAngle(90.0),
                ..FieldGenerationOptions::default()
            },
            ..PlannerOptions::default()
        })
        .expect("staged plan");

    assert_eq!(staged.parts.len(), 1);
    let part = &staged.parts[0];
    assert_eq!(part.headlands.len(), 1);
    assert!(!part.generated_swaths.is_empty());
    assert!(!part.ordered_swaths.is_empty());
    assert!(!part.tour.is_empty());
    assert!(
        part.ordered_swaths
            .iter()
            .all(|swath| swath.r#type != SwathType::Connection)
    );
}

#[test]
fn staged_tour_replaces_graph_connections_with_turner_geometry() {
    let mut mt = Maptrax::new();
    mt.set_field(upstream_fixture_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");

    let staged = mt
        .plan_stages(&PlannerOptions {
            field: FieldGenerationOptions {
                swath_width: 8.0,
                headland_count: 2,
                mode: FieldGenerationMode::ExplicitAngle(0.0),
                ..FieldGenerationOptions::default()
            },
            routing: RoutingOptions {
                strategy: RoutingStrategy::GreedyNearest,
                local_improvement_passes: 1,
            },
            turn: TurnPlannerConfig {
                model: TurnPlannerModel::Sharper,
                connector_mode: ConnectorMode::Headland,
                swath_width: 4.0,
                min_turning_radius: 2.0,
                machine_length: 6.0,
                machine_width: 3.0,
                ..TurnPlannerConfig::default()
            },
            ..PlannerOptions::default()
        })
        .expect("staged");

    let part = &staged.parts[0];
    assert!(
        part.ordered_swaths
            .iter()
            .all(|swath| swath.r#type == SwathType::Swath)
    );
    let turner_segments = part
        .tour
        .iter()
        .filter(|swath| swath.r#type == SwathType::Connection)
        .collect::<Vec<_>>();
    assert!(!turner_segments.is_empty());
    assert!(
        turner_segments.iter().any(|swath| swath.points.len() > 2),
        "tour still contains only straight placeholder connectors"
    );
    assert!(part.tour.len() > part.ordered_swaths.len());
}

#[test]
fn staged_and_one_shot_facade_outputs_match() {
    let mut staged_mt = Maptrax::new();
    staged_mt
        .set_field(upstream_fixture_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("field");
    let options = PlannerOptions {
        field: FieldGenerationOptions {
            swath_width: 4.0,
            headland_count: 3,
            mode: FieldGenerationMode::ExplicitAngle(0.0),
            ..FieldGenerationOptions::default()
        },
        routing: RoutingOptions {
            strategy: RoutingStrategy::GreedyNearest,
            ..RoutingOptions::default()
        },
        turn: TurnPlannerConfig::default(),
        ..PlannerOptions::default()
    };

    let staged = staged_mt.plan_stages(&options).expect("staged");

    let mut one_shot_mt = Maptrax::new();
    one_shot_mt
        .set_field(upstream_fixture_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("field");
    let one_shot = one_shot_mt.plan_all(&options).expect("one shot");

    assert_eq!(staged.parts.len(), one_shot.parts.len());
    for (staged_part, one_shot_part) in staged.parts.iter().zip(one_shot.parts.iter()) {
        assert_eq!(staged_part.part_index, one_shot_part.part_index);
        assert_eq!(
            staged_part.ordered_swaths.len(),
            one_shot_part.ordered_swaths.len()
        );
        assert_eq!(staged_part.tour.len(), one_shot_part.tour.len());
    }
}

#[test]
fn facade_supports_objective_driven_field_generation() {
    let mut mt = Maptrax::new();
    mt.set_field(test_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");

    let result = mt
        .generate_field_with_objective(
            10.0,
            SwathObjective::ApproxMinSwathCount,
            SwathAngleSearchOptions {
                start_degrees: 0.0,
                end_degrees: 180.0,
                step_degrees: 90.0,
            },
            1,
        )
        .expect("objective field");

    assert_eq!(result.len(), 1);
    assert_eq!(result[0].angle_degrees, 0.0);
    assert_eq!(mt.field().unwrap().get_parts()[0].headlands.len(), 1);
    assert_eq!(mt.field().unwrap().get_parts()[0].swaths.len(), 3);
}

#[test]
fn decomposed_parts_can_be_routed_and_toured() {
    let mut mt = Maptrax::new();
    mt.set_field(concave_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");
    assert_eq!(
        mt.decompose_field(DecompositionMode::ConcaveSplit)
            .expect("decompose"),
        2
    );

    mt.generate_field(10.0, 90.0, 0).expect("generate");
    let field = mt.field().expect("field");
    assert_eq!(field.get_parts().len(), 2);

    let cfg = TurnPlannerConfig {
        swath_width: 10.0,
        min_turning_radius: 2.0,
        step_size: 0.2,
        headland_threshold_rows: 2.0,
        ..TurnPlannerConfig::default()
    };

    for part in field.get_parts() {
        assert!(!part.swaths.is_empty());
        let mut nety = Nety::new(&part.swaths);
        nety.field_traversal(None);
        let ordered = nety.get_swaths().to_vec();
        let tour = TourBuilder::build(part, &ordered, &cfg);
        assert!(!tour.is_empty());
        assert!(
            tour.iter()
                .any(|swath| matches!(swath.r#type, SwathType::Swath | SwathType::Connection))
        );
    }
}

#[test]
fn direct_turn_is_preferred_for_close_swaths() {
    let part = simple_part_with_headland();
    let mut from = create_swath(
        point_xy(20.0, 10.0),
        point_xy(20.0, 40.0),
        SwathType::Swath,
        "a",
    );
    from.width = 10.0;
    let mut to = create_swath(
        point_xy(30.0, 40.0),
        point_xy(30.0, 10.0),
        SwathType::Swath,
        "b",
    );
    to.width = 10.0;
    to.swap_direction();

    let tour = TourBuilder::build(
        &part,
        &[from, to],
        &TurnPlannerConfig {
            swath_width: 10.0,
            min_turning_radius: 25.0,
            headland_threshold_rows: 2.0,
            ..TurnPlannerConfig::default()
        },
    );

    let connections = tour
        .iter()
        .filter(|swath| swath.r#type == SwathType::Connection)
        .collect::<Vec<_>>();
    assert_eq!(connections.len(), 1);
    assert!(connections[0].points.len() >= 2);
}

#[test]
fn headland_route_is_preferred_for_far_swaths() {
    let part = simple_part_with_headland();
    let mut from = create_swath(
        point_xy(20.0, 10.0),
        point_xy(20.0, 40.0),
        SwathType::Swath,
        "a",
    );
    from.width = 10.0;
    let to = create_swath(
        point_xy(80.0, 10.0),
        point_xy(80.0, 40.0),
        SwathType::Swath,
        "b",
    );

    let tour = TourBuilder::build(
        &part,
        &[from, to],
        &TurnPlannerConfig {
            connector_mode: ConnectorMode::Headland,
            swath_width: 10.0,
            headland_threshold_rows: 2.0,
            ..TurnPlannerConfig::default()
        },
    );

    let connections = tour
        .iter()
        .filter(|swath| swath.r#type == SwathType::Connection)
        .collect::<Vec<_>>();
    assert!(!connections.is_empty());
    assert!(connections.len() >= 2 || connections.iter().any(|swath| swath.points.len() > 2));
    assert!(connections.iter().any(|swath| swath.points.len() > 2));
}

#[test]
fn forced_headland_mode_always_uses_headland_route_when_available() {
    let part = simple_part_with_headland();
    let mut from = create_swath(
        point_xy(20.0, 10.0),
        point_xy(20.0, 40.0),
        SwathType::Swath,
        "a",
    );
    from.width = 10.0;
    let to = create_swath(
        point_xy(80.0, 10.0),
        point_xy(80.0, 40.0),
        SwathType::Swath,
        "b",
    );

    let tour = TourBuilder::build(
        &part,
        &[from, to],
        &TurnPlannerConfig {
            connector_mode: ConnectorMode::Headland,
            model: TurnPlannerModel::Sharper,
            swath_width: 10.0,
            min_turning_radius: 2.0,
            machine_length: 6.0,
            machine_width: 3.0,
            ..TurnPlannerConfig::default()
        },
    );

    let headland_like = tour
        .iter()
        .filter(|swath| swath.r#type == SwathType::Connection)
        .filter(|swath| swath.points.len() > 2)
        .count();
    assert!(headland_like >= 1);
}

#[test]
fn irregular_headland_ring_routing_uses_polyline_path() {
    let part = irregular_part_with_headland();
    let mut from = create_swath(
        point_xy(20.0, 12.0),
        point_xy(20.0, 60.0),
        SwathType::Swath,
        "a",
    );
    from.width = 8.0;
    let to = create_swath(
        point_xy(92.0, 16.0),
        point_xy(92.0, 55.0),
        SwathType::Swath,
        "b",
    );

    let tour = TourBuilder::build(
        &part,
        &[from, to],
        &TurnPlannerConfig {
            swath_width: 8.0,
            headland_threshold_rows: 2.0,
            ..TurnPlannerConfig::default()
        },
    );

    let ring_like = tour
        .iter()
        .filter(|swath| swath.r#type == SwathType::Connection && swath.points.len() > 2)
        .collect::<Vec<_>>();
    assert!(!ring_like.is_empty());
    assert!(
        ring_like
            .iter()
            .any(|swath| swath.bounding_box.max_point.x > swath.bounding_box.min_point.x)
    );
}

#[test]
fn tiny_headland_entry_exit_hops_do_not_create_triangle_loops() {
    let part = simple_part_with_headland();
    let mut from = create_swath(
        point_xy(20.0, 10.0),
        point_xy(20.0, 40.0),
        SwathType::Swath,
        "a",
    );
    from.width = 10.0;
    let mut to = create_swath(
        point_xy(22.0, 40.0),
        point_xy(22.0, 10.0),
        SwathType::Swath,
        "b",
    );
    to.width = 10.0;

    let tour = TourBuilder::build(
        &part,
        &[from, to],
        &TurnPlannerConfig {
            model: TurnPlannerModel::Sharper,
            swath_width: 10.0,
            min_turning_radius: 2.0,
            machine_length: 6.0,
            machine_width: 3.0,
            headland_threshold_rows: 0.0,
            ..TurnPlannerConfig::default()
        },
    );

    let short_connections = tour
        .iter()
        .filter(|swath| swath.r#type == SwathType::Connection)
        .filter(|swath| {
            let dx = swath.tail().x() - swath.head().x();
            let dy = swath.tail().y() - swath.head().y();
            (dx * dx + dy * dy).sqrt() <= 3.0
        })
        .collect::<Vec<_>>();

    assert!(!short_connections.is_empty());
    assert!(
        short_connections
            .iter()
            .all(|swath| swath.points.len() == 2)
    );
}

#[test]
fn planner_options_drive_end_to_end_facade_flow() {
    let mut mt = Maptrax::new();
    mt.set_field(test_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");

    let planned = mt
        .plan_all(&PlannerOptions {
            field: FieldGenerationOptions {
                swath_width: 10.0,
                headland_count: 1,
                mode: FieldGenerationMode::ExplicitAngle(90.0),
                ..FieldGenerationOptions::default()
            },
            routing: RoutingOptions {
                strategy: RoutingStrategy::Snake,
                local_improvement_passes: 1,
            },
            turn: TurnPlannerConfig {
                swath_width: 10.0,
                ..TurnPlannerConfig::default()
            },
            ..PlannerOptions::default()
        })
        .expect("plan");

    assert_eq!(planned.parts.len(), 1);
    assert!(!planned.parts[0].ordered_swaths.is_empty());
    assert!(planned.parts[0].tour.len() >= planned.parts[0].ordered_swaths.len());
}

#[test]
fn planner_options_work_on_upstream_shaped_fixture() {
    let mut mt = Maptrax::new();
    mt.set_field(
        upstream_fixture_polygon(),
        Geo::new(51.98954034749562, 5.6584737410504715, 53.801823),
    )
    .expect("set field");

    let planned = mt
        .plan_all(&PlannerOptions {
            field: FieldGenerationOptions {
                swath_width: 4.0,
                headland_count: 3,
                mode: FieldGenerationMode::Objective {
                    objective: SwathObjective::ApproxMinSwathCount,
                    options: SwathAngleSearchOptions::default(),
                },
                ..FieldGenerationOptions::default()
            },
            routing: RoutingOptions {
                strategy: RoutingStrategy::GreedyNearest,
                local_improvement_passes: 0,
            },
            turn: TurnPlannerConfig {
                swath_width: 4.0,
                min_turning_radius: 2.0,
                ..TurnPlannerConfig::default()
            },
            ..PlannerOptions::default()
        })
        .expect("plan");

    assert_eq!(planned.parts.len(), 1);
    assert!(!planned.objective_results.is_empty());
    assert!(!planned.parts[0].tour.is_empty());
}

#[test]
fn facade_strategy_selection_changes_ordered_swaths() {
    let mut mt = Maptrax::new();
    mt.set_field(test_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");
    mt.generate_field(10.0, 90.0, 0).expect("generate");

    let snake = mt
        .plan_ordered_swaths_for_part(
            0,
            RoutingOptions {
                strategy: RoutingStrategy::Snake,
                local_improvement_passes: 0,
            },
        )
        .expect("snake");
    let spiral = mt
        .plan_ordered_swaths_for_part(
            0,
            RoutingOptions {
                strategy: RoutingStrategy::Spiral,
                local_improvement_passes: 0,
            },
        )
        .expect("spiral");

    let snake_ids = snake
        .iter()
        .filter(|swath| swath.r#type == SwathType::Swath)
        .map(|swath| swath.uuid.clone())
        .collect::<Vec<_>>();
    let spiral_ids = spiral
        .iter()
        .filter(|swath| swath.r#type == SwathType::Swath)
        .map(|swath| swath.uuid.clone())
        .collect::<Vec<_>>();

    assert_ne!(snake_ids, spiral_ids);
}
