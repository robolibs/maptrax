use concord::{Geo, Wgs, to_enu};
use geo::Point;
use maptrax::{
    DivisionType, MachinePlanningOptions, Maptrax, ObstaclePlanningOptions, RoutingOptions,
    RoutingStrategy, SwathType, TurnPlannerConfig, polygon_from_points,
};

fn rectangular_polygon() -> geo::Polygon<f64> {
    polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ])
}

fn upstream_fixture_polygon() -> geo::Polygon<f64> {
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
            Point::new(enu.east(), enu.north())
        })
        .collect(),
    )
}

fn centered_obstacle(border: &geo::Polygon<f64>, half_size: f64) -> geo::Polygon<f64> {
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

#[test]
fn machine_planning_preserves_assigned_swath_totals() {
    let mut mt = Maptrax::new();
    mt.set_field(rectangular_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");
    mt.generate_field(10.0, 90.0, 1).expect("generate");

    let planned = mt
        .plan_machines_for_part(
            &MachinePlanningOptions {
                machines: 3,
                division_type: DivisionType::Alternate,
                part_index: 0,
            },
            &ObstaclePlanningOptions::default(),
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
fn machine_planning_tracks_assigned_avoided_routed_flow() {
    let mut mt = Maptrax::new();
    mt.set_field(rectangular_polygon(), Geo::new(51.0, 5.0, 0.0))
        .expect("set field");
    mt.generate_field(10.0, 90.0, 1).expect("generate");

    let obstacle = polygon_from_points(vec![
        Point::new(45.0, 15.0),
        Point::new(55.0, 15.0),
        Point::new(55.0, 35.0),
        Point::new(45.0, 35.0),
    ]);

    let planned = mt
        .plan_machines_for_part(
            &MachinePlanningOptions {
                machines: 2,
                division_type: DivisionType::Alternate,
                part_index: 0,
            },
            &ObstaclePlanningOptions {
                obstacles: vec![obstacle],
                inflation_distance: 2.0,
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
        assert!(!machine.avoided_swaths.is_empty());
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
    let border = mt.field().unwrap().border().clone();

    let obstacle = centered_obstacle(&border, 25.0);

    let planned = mt
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
        .expect("machines");

    let assigned_counts = planned
        .machines
        .iter()
        .map(|machine| machine.assigned_swaths.len())
        .collect::<Vec<_>>();
    let avoided_counts = planned
        .machines
        .iter()
        .map(|machine| machine.avoided_swaths.len())
        .collect::<Vec<_>>();
    let ordered_counts = planned
        .machines
        .iter()
        .map(|machine| machine.ordered_swaths.len())
        .collect::<Vec<_>>();

    assert_eq!(assigned_counts, vec![18, 18, 18, 17]);
    assert_eq!(avoided_counts, vec![28, 28, 28, 27]);
    assert_eq!(ordered_counts, vec![23, 23, 23, 22]);
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
