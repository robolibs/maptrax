use concord::{Geo, Wgs, to_enu};
use maptrax::point_xy;
use maptrax::{
    Balance, DivisionPattern, DivisionPlan, MachinePlanningOptions, Maptrax, RoutingOptions,
    RoutingStrategy, SwathType, TurnPlannerConfig, polygon_from_points,
};

fn stripe_1_by_count(machines: usize) -> DivisionPlan {
    DivisionPlan::uniform(
        machines,
        DivisionPattern::Stripe { stride: 1 },
        Balance::ByCount,
    )
}

fn rectangular_polygon() -> maptrax::Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(100.0, 0.0),
        point_xy(100.0, 50.0),
        point_xy(0.0, 50.0),
    ])
}

fn upstream_fixture_polygon() -> maptrax::Polygon {
    let datum = Geo::new(51.98954034749562, 5.6584737410504715, 53.801823);
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
fn machine_planning_preserves_assigned_swath_totals() {
    let mut mt = Maptrax::new();
    mt.set_field(rectangular_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");
    mt.generate_field(10.0, 90.0, 1).expect("generate");

    let planned = mt
        .plan_machines_for_part(
            &MachinePlanningOptions {
                plan: stripe_1_by_count(3),
                part_index: 0,
            },
            RoutingOptions::default(),
            &TurnPlannerConfig {
                swath_width: 10.0,
                ..TurnPlannerConfig::default()
            },
        )
        .expect("machines");

    let assigned_total: usize = planned
        .machines
        .iter()
        .map(|machine| machine.assigned_swaths.len())
        .sum();
    assert_eq!(
        assigned_total,
        mt.field().unwrap().get_parts()[0].swaths.len()
    );
    assert_eq!(planned.machines.len(), 3);
}

#[test]
fn machine_planning_tracks_assigned_routed_flow() {
    let mut mt = Maptrax::new();
    mt.set_field(rectangular_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");
    mt.generate_field(10.0, 90.0, 1).expect("generate");

    let planned = mt
        .plan_machines_for_part(
            &MachinePlanningOptions {
                plan: stripe_1_by_count(2),
                part_index: 0,
            },
            RoutingOptions {
                strategy: RoutingStrategy::Snake,
                local_improvement_passes: 1,
            },
            &TurnPlannerConfig {
                swath_width: 10.0,
                ..TurnPlannerConfig::default()
            },
        )
        .expect("machines");

    for machine in &planned.machines {
        assert!(!machine.assigned_swaths.is_empty());
        assert!(!machine.ordered_swaths.is_empty());
        assert!(!machine.tour.is_empty());
        assert!(
            machine
                .ordered_swaths
                .iter()
                .any(|swath| matches!(swath.r#type, SwathType::Swath | SwathType::Connection))
        );
    }
}

#[test]
fn upstream_machine_fixture_counts_remain_stable() {
    let mut mt = Maptrax::new();
    mt.set_field(
        upstream_fixture_polygon(),
        Geo::new(51.98954034749562, 5.6584737410504715, 53.801823),
    )
    .expect("set field");
    mt.generate_field(4.0, 0.0, 3).expect("generate");

    let planned = mt
        .plan_machines_for_part(
            &MachinePlanningOptions {
                plan: stripe_1_by_count(4),
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

    let assigned_counts = planned
        .machines
        .iter()
        .map(|machine| machine.assigned_swaths.len())
        .collect::<Vec<_>>();
    let ordered_counts = planned
        .machines
        .iter()
        .map(|machine| machine.ordered_swaths.len())
        .collect::<Vec<_>>();

    assert_eq!(assigned_counts, vec![18, 18, 18, 17]);
    assert_eq!(ordered_counts, assigned_counts);
    for machine in &planned.machines {
        assert!(
            machine
                .ordered_swaths
                .iter()
                .all(|swath| swath.r#type == SwathType::Swath)
        );
        assert!(machine.tour.len() > machine.ordered_swaths.len());
        assert!(
            machine
                .tour
                .iter()
                .any(|swath| swath.r#type == SwathType::Connection)
        );
    }
}
