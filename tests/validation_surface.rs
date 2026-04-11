use concord::{Geo, Wgs, to_enu};
use geo::{Point, Polygon};
use maptrax::{
    DecompositionMode, DivisionType, FieldGenerationMode, FieldGenerationOptions,
    MachinePlanningOptions, Maptrax, ObstaclePlanningOptions, PlannerOptions, RoutingOptions,
    RoutingStrategy, Swath, SwathType, TurnPlannerConfig, point_distance, polygon_from_points,
};

fn rectangle_polygon() -> Polygon {
    polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ])
}

fn irregular_convex_polygon() -> Polygon {
    polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(120.0, 10.0),
        Point::new(140.0, 60.0),
        Point::new(90.0, 95.0),
        Point::new(30.0, 85.0),
        Point::new(-10.0, 40.0),
    ])
}

fn concave_polygon() -> Polygon {
    polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(120.0, 0.0),
        Point::new(120.0, 30.0),
        Point::new(70.0, 30.0),
        Point::new(70.0, 90.0),
        Point::new(0.0, 90.0),
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
            Point::new(enu.east(), enu.north())
        })
        .collect(),
    )
}

fn centered_obstacle(border: &Polygon, half_size: f64) -> Polygon {
    let vertices = border.exterior().points().collect::<Vec<_>>();
    let (min_x, max_x) = vertices
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |acc, p| {
            (acc.0.min(p.x()), acc.1.max(p.x()))
        });
    let (min_y, max_y) = vertices
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |acc, p| {
            (acc.0.min(p.y()), acc.1.max(p.y()))
        });
    let center_x = (min_x + max_x) * 0.5;
    let center_y = (min_y + max_y) * 0.5;
    polygon_from_points(vec![
        Point::new(center_x - half_size, center_y - half_size),
        Point::new(center_x + half_size, center_y - half_size),
        Point::new(center_x + half_size, center_y + half_size),
        Point::new(center_x - half_size, center_y + half_size),
    ])
}

fn shifted_obstacle(center_x: f64, center_y: f64, half_size: f64) -> Polygon {
    polygon_from_points(vec![
        Point::new(center_x - half_size, center_y - half_size),
        Point::new(center_x + half_size, center_y - half_size),
        Point::new(center_x + half_size, center_y + half_size),
        Point::new(center_x - half_size, center_y + half_size),
    ])
}

fn assert_swaths_are_geometrically_sane(swaths: &[Swath]) {
    assert!(!swaths.is_empty());
    for swath in swaths {
        assert!(swath.bounding_box.min().x.is_finite());
        assert!(swath.bounding_box.min().y.is_finite());
        assert!(swath.bounding_box.max().x.is_finite());
        assert!(swath.bounding_box.max().y.is_finite());
        assert!(swath.bounding_box.min().x <= swath.bounding_box.max().x);
        assert!(swath.bounding_box.min().y <= swath.bounding_box.max().y);
        assert!(!swath.points.is_empty());
        assert!(
            swath
                .points
                .iter()
                .all(|point| point.x().is_finite() && point.y().is_finite())
        );
    }
}

fn assert_tour_is_connected(tour: &[Swath]) {
    assert!(!tour.is_empty());
    for pair in tour.windows(2) {
        let gap = point_distance(pair[0].tail(), pair[1].head());
        assert!(
            gap <= 1e-6,
            "tour gap too large between {} and {}: {gap}",
            pair[0].uuid,
            pair[1].uuid
        );
    }
}

#[test]
fn staged_planning_covers_fixture_matrix() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let upstream_datum = Geo::new(51.98954034749562, 5.6584737410504715, 53.801823);

    let scenarios = vec![
        (
            "rectangle",
            rectangle_polygon(),
            FieldGenerationOptions {
                swath_width: 10.0,
                headland_count: 1,
                decomposition: DecompositionMode::None,
                mode: FieldGenerationMode::ExplicitAngle(90.0),
            },
            ObstaclePlanningOptions::default(),
            datum,
        ),
        (
            "irregular-convex",
            irregular_convex_polygon(),
            FieldGenerationOptions {
                swath_width: 8.0,
                headland_count: 2,
                decomposition: DecompositionMode::None,
                mode: FieldGenerationMode::ExplicitAngle(0.0),
            },
            ObstaclePlanningOptions::default(),
            datum,
        ),
        (
            "concave-with-center-obstacle",
            concave_polygon(),
            FieldGenerationOptions {
                swath_width: 10.0,
                headland_count: 1,
                decomposition: DecompositionMode::ConcaveSplit,
                mode: FieldGenerationMode::ExplicitAngle(90.0),
            },
            ObstaclePlanningOptions {
                obstacles: vec![centered_obstacle(&concave_polygon(), 8.0)],
                inflation_distance: 2.0,
            },
            datum,
        ),
        (
            "upstream-wgs",
            upstream_polygon(upstream_datum),
            FieldGenerationOptions {
                swath_width: 4.0,
                headland_count: 3,
                decomposition: DecompositionMode::None,
                mode: FieldGenerationMode::ExplicitAngle(0.0),
            },
            ObstaclePlanningOptions {
                obstacles: vec![centered_obstacle(&upstream_polygon(upstream_datum), 25.0)],
                inflation_distance: 2.0,
            },
            upstream_datum,
        ),
    ];

    for (name, polygon, field_options, obstacle_options, scenario_datum) in scenarios {
        let mut planner = Maptrax::new();
        planner.set_field(polygon, scenario_datum).expect("field");
        let planned = planner
            .plan_stages(&PlannerOptions {
                field: field_options,
                routing: RoutingOptions {
                    strategy: RoutingStrategy::GreedyNearest,
                    local_improvement_passes: 1,
                },
                obstacles: obstacle_options,
                turn: TurnPlannerConfig::default(),
                ..PlannerOptions::default()
            })
            .expect("planned");

        assert!(!planned.parts.is_empty(), "{name}: no parts");
        for part in &planned.parts {
            assert_swaths_are_geometrically_sane(&part.generated_swaths);
            assert_swaths_are_geometrically_sane(&part.avoided_swaths);
            assert_swaths_are_geometrically_sane(&part.ordered_swaths);
            assert_swaths_are_geometrically_sane(&part.tour);
            assert!(!part.tour.is_empty(), "{name}: empty tour");
            assert_tour_is_connected(&part.tour);
        }
    }
}

#[test]
fn routing_strategies_preserve_work_across_fixture_matrix() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let fixtures = vec![
        ("rectangle", rectangle_polygon(), 10.0, 90.0, 1),
        ("irregular", irregular_convex_polygon(), 8.0, 0.0, 2),
        ("concave", concave_polygon(), 10.0, 90.0, 1),
    ];

    for (name, polygon, swath_width, angle, headlands) in fixtures {
        let mut planner = Maptrax::new();
        planner.set_field(polygon, datum).expect("field");
        planner
            .generate_field(swath_width, angle, headlands)
            .expect("generate");
        let expected_count = planner.field().unwrap().get_parts()[0].swaths.len();

        for strategy in [
            RoutingStrategy::GreedyNearest,
            RoutingStrategy::Snake,
            RoutingStrategy::Spiral,
        ] {
            let ordered = planner
                .plan_ordered_swaths_for_part(
                    0,
                    RoutingOptions {
                        strategy,
                        local_improvement_passes: 1,
                    },
                    &ObstaclePlanningOptions::default(),
                )
                .expect("ordered");
            assert_eq!(
                ordered
                    .iter()
                    .filter(|swath| swath.r#type == SwathType::Swath)
                    .count(),
                expected_count,
                "{name}: strategy lost work"
            );
            assert_swaths_are_geometrically_sane(&ordered);
        }
    }
}

#[test]
fn multiple_obstacles_expand_avoidance_surface() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let polygon = irregular_convex_polygon();
    let mut planner = Maptrax::new();
    planner.set_field(polygon, datum).expect("field");
    planner.generate_field(8.0, 0.0, 2).expect("generate");

    let single = planner
        .plan_stages_for_part(
            0,
            RoutingOptions::default(),
            &ObstaclePlanningOptions {
                obstacles: vec![shifted_obstacle(55.0, 45.0, 8.0)],
                inflation_distance: 2.0,
            },
            &TurnPlannerConfig::default(),
        )
        .expect("single");
    let multiple = planner
        .plan_stages_for_part(
            0,
            RoutingOptions::default(),
            &ObstaclePlanningOptions {
                obstacles: vec![
                    shifted_obstacle(40.0, 35.0, 8.0),
                    shifted_obstacle(85.0, 55.0, 8.0),
                ],
                inflation_distance: 2.0,
            },
            &TurnPlannerConfig::default(),
        )
        .expect("multiple");

    assert!(multiple.transit_rings.len() >= single.transit_rings.len());
    assert_eq!(single.transit_rings.len(), 1);
    assert_eq!(multiple.transit_rings.len(), 2);
    assert_tour_is_connected(&multiple.tour);
}

#[test]
fn machine_planning_covers_all_division_modes() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let polygon = irregular_convex_polygon();
    let obstacle = centered_obstacle(&polygon, 10.0);
    let mut planner = Maptrax::new();
    planner.set_field(polygon, datum).expect("field");
    planner.generate_field(8.0, 0.0, 2).expect("generate");
    let original_swaths = planner.field().unwrap().get_parts()[0].swaths.len();

    for division_type in [
        DivisionType::Alternate,
        DivisionType::Block,
        DivisionType::SpatialRtree,
        DivisionType::LengthBalanced,
    ] {
        let planned = planner
            .plan_machines_for_part(
                &MachinePlanningOptions {
                    machines: 3,
                    division_type,
                    part_index: 0,
                },
                &ObstaclePlanningOptions {
                    obstacles: vec![obstacle.clone()],
                    inflation_distance: 2.0,
                },
                RoutingOptions {
                    strategy: RoutingStrategy::GreedyNearest,
                    local_improvement_passes: 1,
                },
                &TurnPlannerConfig {
                    swath_width: 8.0,
                    ..TurnPlannerConfig::default()
                },
            )
            .expect("machines");

        assert_eq!(
            planned
                .machines
                .iter()
                .map(|machine| machine.assigned_swaths.len())
                .sum::<usize>(),
            original_swaths
        );
        assert_eq!(planned.machines.len(), 3);
        for machine in &planned.machines {
            assert_swaths_are_geometrically_sane(&machine.ordered_swaths);
            assert_swaths_are_geometrically_sane(&machine.tour);
            assert_tour_is_connected(&machine.tour);
        }
    }
}

#[test]
fn upstream_example_surface_remains_visually_sane() {
    let datum = Geo::new(51.98954034749562, 5.6584737410504715, 53.801823);
    let polygon = upstream_polygon(datum);
    let obstacle = centered_obstacle(&polygon, 25.0);

    let mut planner = Maptrax::new();
    planner.set_field(polygon, datum).expect("field");
    let machine_plan = planner
        .plan_machines_for_part(
            &MachinePlanningOptions {
                machines: 4,
                division_type: DivisionType::Alternate,
                part_index: 0,
            },
            &ObstaclePlanningOptions {
                obstacles: vec![obstacle],
                inflation_distance: 2.0,
            },
            RoutingOptions {
                strategy: RoutingStrategy::GreedyNearest,
                local_improvement_passes: 0,
            },
            &TurnPlannerConfig {
                swath_width: 4.0,
                min_turning_radius: 2.0,
                ..TurnPlannerConfig::default()
            },
        )
        .expect("pre-generation machine plan");

    assert_eq!(machine_plan.machines.len(), 4);
    assert!(
        machine_plan
            .machines
            .iter()
            .all(|machine| machine.assigned_swaths.is_empty()
                && machine.avoided_swaths.is_empty()
                && machine.ordered_swaths.is_empty()
                && machine.tour.is_empty())
    );

    planner.generate_field(4.0, 0.0, 3).expect("generate");
    let machine_plan = planner
        .plan_machines_for_part(
            &MachinePlanningOptions {
                machines: 4,
                division_type: DivisionType::Alternate,
                part_index: 0,
            },
            &ObstaclePlanningOptions {
                obstacles: vec![centered_obstacle(planner.field().unwrap().border(), 25.0)],
                inflation_distance: 2.0,
            },
            RoutingOptions {
                strategy: RoutingStrategy::GreedyNearest,
                local_improvement_passes: 0,
            },
            &TurnPlannerConfig {
                swath_width: 4.0,
                min_turning_radius: 2.0,
                ..TurnPlannerConfig::default()
            },
        )
        .expect("machines");

    let ordered_counts = machine_plan
        .machines
        .iter()
        .map(|machine| machine.ordered_swaths.len())
        .collect::<Vec<_>>();
    assert_eq!(ordered_counts, vec![23, 23, 23, 22]);
    assert!(
        machine_plan
            .machines
            .iter()
            .all(|machine| machine
                .ordered_swaths
                .iter()
                .all(|swath| swath.r#type == SwathType::Swath))
    );
    assert!(
        machine_plan
            .machines
            .iter()
            .all(|machine| machine.tour.len() > machine.ordered_swaths.len())
    );
}
