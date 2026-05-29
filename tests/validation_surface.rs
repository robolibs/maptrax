use concord::{Geo, Wgs, to_enu};
use maptrax::{Point, Point2Ext, Polygon, point_xy};
use maptrax::{
    Balance, DecompositionMode, DivisionPattern, DivisionPlan, FieldGenerationMode,
    FieldGenerationOptions, MachinePlanningOptions, Maptrax, OptimizeObjective, PlannerOptions,
    RoutingOptions, RoutingStrategy, Swath, SwathType, TurnPlannerConfig, point_distance,
    polygon_from_points,
};

fn rectangle_polygon() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(100.0, 0.0),
        point_xy(100.0, 50.0),
        point_xy(0.0, 50.0),
    ])
}

fn irregular_convex_polygon() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(120.0, 10.0),
        point_xy(140.0, 60.0),
        point_xy(90.0, 95.0),
        point_xy(30.0, 85.0),
        point_xy(-10.0, 40.0),
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

fn assert_swaths_are_geometrically_sane(swaths: &[Swath]) {
    assert!(!swaths.is_empty());
    for swath in swaths {
        assert!(swath.bounding_box.min_point.x.is_finite());
        assert!(swath.bounding_box.min_point.y.is_finite());
        assert!(swath.bounding_box.max_point.x.is_finite());
        assert!(swath.bounding_box.max_point.y.is_finite());
        assert!(swath.bounding_box.min_point.x <= swath.bounding_box.max_point.x);
        assert!(swath.bounding_box.min_point.y <= swath.bounding_box.max_point.y);
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
            datum,
        ),
        (
            "concave-split",
            concave_polygon(),
            FieldGenerationOptions {
                swath_width: 10.0,
                headland_count: 1,
                decomposition: DecompositionMode::ConcaveSplit,
                mode: FieldGenerationMode::ExplicitAngle(90.0),
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
            upstream_datum,
        ),
    ];

    for (name, polygon, field_options, scenario_datum) in scenarios {
        let mut planner = Maptrax::new();
        planner.set_field(polygon, scenario_datum).expect("field");
        let planned = planner
            .plan_stages(&PlannerOptions {
                field: field_options,
                routing: RoutingOptions {
                    strategy: RoutingStrategy::GreedyNearest,
                    local_improvement_passes: 1,
                },
                turn: TurnPlannerConfig::default(),
                ..PlannerOptions::default()
            })
            .expect("planned");

        assert!(!planned.parts.is_empty(), "{name}: no parts");
        for part in &planned.parts {
            assert_swaths_are_geometrically_sane(&part.generated_swaths);
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
fn machine_planning_covers_all_division_modes() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let polygon = irregular_convex_polygon();
    let mut planner = Maptrax::new();
    planner.set_field(polygon, datum).expect("field");
    planner.generate_field(8.0, 0.0, 2).expect("generate");
    let original_swaths = planner.field().unwrap().get_parts()[0].swaths.len();

    let plans = [
        DivisionPlan::uniform(3, DivisionPattern::Stripe { stride: 1 }, Balance::ByCount),
        DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount),
        DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByLength),
        DivisionPlan::uniform(
            3,
            DivisionPattern::BandedStripe { bands: 2 },
            Balance::ByLength,
        ),
        DivisionPlan::uniform(
            3,
            DivisionPattern::Optimized {
                objective: OptimizeObjective::Makespan,
            },
            Balance::ByLength,
        ),
    ];
    for plan in plans {
        let planned = planner
            .plan_machines_for_part(
                &MachinePlanningOptions {
                    plan,
                    part_index: 0,
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

    let mut planner = Maptrax::new();
    planner.set_field(polygon, datum).expect("field");
    let machine_plan = planner
        .plan_machines_for_part(
            &MachinePlanningOptions {
                plan: DivisionPlan::uniform(
                    4,
                    DivisionPattern::Stripe { stride: 1 },
                    Balance::ByCount,
                ),
                part_index: 0,
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
                && machine.ordered_swaths.is_empty()
                && machine.tour.is_empty())
    );

    planner.generate_field(4.0, 0.0, 3).expect("generate");
    let machine_plan = planner
        .plan_machines_for_part(
            &MachinePlanningOptions {
                plan: DivisionPlan::uniform(
                    4,
                    DivisionPattern::Stripe { stride: 1 },
                    Balance::ByCount,
                ),
                part_index: 0,
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
    assert_eq!(ordered_counts, vec![18, 18, 18, 17]);
    assert!(machine_plan.machines.iter().all(|machine| {
        machine
            .ordered_swaths
            .iter()
            .all(|swath| swath.r#type == SwathType::Swath)
    }));
    assert!(
        machine_plan
            .machines
            .iter()
            .all(|machine| machine.tour.len() > machine.ordered_swaths.len())
    );
}
