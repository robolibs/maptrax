//! Integration coverage for the turn-radius-aware planning API (PLAN.md):
//! turn-space sizing, the headland sizing policy, turn-radius-aware routing,
//! and connector corridor validation.

use concord::Geo;
use maptrax::{
    Balance, DivisionPattern, DivisionPlan, HeadlandSizingPolicy, MachinePlanningOptions, Maptrax,
    Polygon, RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel, point_xy,
    polygon_from_points, required_headland_count, required_row_skip_stride,
};

fn datum() -> Geo {
    Geo::new(51.0, 5.0, 0.0)
}

fn rect(w: f64, h: f64) -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(w, 0.0),
        point_xy(w, h),
        point_xy(0.0, h),
    ])
}

fn dubins(radius: f64, swath_width: f64) -> TurnPlannerConfig {
    TurnPlannerConfig {
        model: TurnPlannerModel::Dubins,
        min_turning_radius: radius,
        machine_length: 6.0,
        machine_width: 0.0,
        swath_width,
        ..TurnPlannerConfig::default()
    }
}

#[test]
fn required_counts_scale_with_radius() {
    let cfg = dubins(8.0, 3.0);
    // Dubins depth = 2r = 16 → ceil(16/3) = 6; lateral = 16 → stride 6.
    assert_eq!(required_headland_count(3.0, &cfg), 6);
    assert_eq!(required_row_skip_stride(3.0, &cfg), 6);
}

#[test]
fn strict_policy_rejects_too_few_headlands() {
    let mut planner = Maptrax::new();
    planner.set_field(rect(120.0, 80.0), datum()).unwrap();
    let cfg = dubins(8.0, 3.0);
    let err = planner.generate_field_feasible(3.0, 0.0, 2, &cfg, HeadlandSizingPolicy::StrictUser);
    assert!(err.is_err(), "strict policy must reject 2 < 6 headlands");
}

#[test]
fn auto_increase_grows_headland_count() {
    let mut planner = Maptrax::new();
    planner.set_field(rect(120.0, 80.0), datum()).unwrap();
    let cfg = dubins(8.0, 3.0);
    let report = planner
        .generate_field_feasible(3.0, 0.0, 2, &cfg, HeadlandSizingPolicy::AutoIncrease)
        .unwrap();
    assert_eq!(report.effective_headland_count, 6);
    assert_eq!(report.row_skip_stride, 1);
    // The field actually has 6 headland rings now.
    let part = planner.field().unwrap().part(0).unwrap();
    assert_eq!(part.headlands.len(), 6);
}

#[test]
fn warn_only_keeps_count_and_reports_stride() {
    let planner = Maptrax::new();
    let cfg = dubins(8.0, 3.0);
    let report = planner.turn_feasibility(3.0, 2, &cfg, HeadlandSizingPolicy::WarnOnly);
    assert_eq!(report.effective_headland_count, 2);
    assert_eq!(report.row_skip_stride, 6);
    assert!(!report.warnings.is_empty());
}

#[test]
fn turn_radius_aware_routing_runs_and_validates() {
    let mut planner = Maptrax::new();
    planner.set_field(rect(120.0, 80.0), datum()).unwrap();
    // Reeds-Shepp with a modest radius + enough headlands: a feasible plan.
    let cfg = TurnPlannerConfig {
        model: TurnPlannerModel::ReedsShepp,
        min_turning_radius: 3.0,
        machine_length: 6.0,
        machine_width: 3.0,
        swath_width: 3.0,
        ..TurnPlannerConfig::default()
    };
    planner
        .generate_field_feasible(3.0, 0.0, 3, &cfg, HeadlandSizingPolicy::AutoIncrease)
        .unwrap();

    let planned = planner
        .plan_machines_for_part(
            &MachinePlanningOptions {
                plan: DivisionPlan::uniform(2, DivisionPattern::Block, Balance::ByLength),
                part_index: 0,
            },
            RoutingOptions {
                strategy: RoutingStrategy::TurnRadiusAware,
                local_improvement_passes: 0,
            },
            &cfg,
        )
        .unwrap();

    assert_eq!(planned.machines.len(), 2);
    // Every machine produced a non-empty tour, and validation runs cleanly.
    for machine in &planned.machines {
        assert!(!machine.tour.is_empty());
        let validation = planner.validate_part_tour(0, &machine.tour).unwrap();
        // Connectors exist and the validator counts them.
        assert!(validation.connector_segments >= machine.ordered_swaths.len().saturating_sub(1));
    }
}

#[test]
fn auto_planner_returns_a_plan() {
    let mut planner = Maptrax::new();
    planner.set_field(rect(120.0, 80.0), datum()).unwrap();
    let cfg = dubins(8.0, 3.0);
    planner
        .generate_field_feasible(3.0, 0.0, 2, &cfg, HeadlandSizingPolicy::WarnOnly)
        .unwrap();

    let (planned, _warnings) = planner
        .plan_machines_auto(
            &MachinePlanningOptions {
                plan: DivisionPlan::uniform(2, DivisionPattern::Block, Balance::ByLength),
                part_index: 0,
            },
            RoutingOptions {
                strategy: RoutingStrategy::TurnRadiusAware,
                local_improvement_passes: 0,
            },
            &cfg,
        )
        .unwrap();
    assert_eq!(planned.machines.len(), 2);
}
