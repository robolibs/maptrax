use maptrax::{
    DecompositionMode, FieldGenerationMode, FieldGenerationOptions, Geo, Maptrax, PlannerOptions,
    RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel, point_xy,
    polygon_from_points,
};

fn main() {
    let polygon = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(120.0, 0.0),
        point_xy(120.0, 30.0),
        point_xy(70.0, 30.0),
        point_xy(70.0, 90.0),
        point_xy(0.0, 90.0),
    ]);

    let mut planner = Maptrax::new();
    planner
        .set_field(polygon, Geo::new(51.0, 5.0, 0.0))
        .expect("field");

    let planned = planner
        .plan_stages(&PlannerOptions {
            field: FieldGenerationOptions {
                swath_width: 10.0,
                headland_count: 1,
                decomposition: DecompositionMode::ConcaveSplit,
                mode: FieldGenerationMode::ExplicitAngle(90.0),
            },
            routing: RoutingOptions {
                strategy: RoutingStrategy::Snake,
                local_improvement_passes: 1,
            },
            turn: TurnPlannerConfig {
                model: TurnPlannerModel::ReedsShepp,
                ..TurnPlannerConfig::default()
            },
            ..PlannerOptions::default()
        })
        .expect("plan");

    println!("objective evaluations: {}", planned.objective_results.len());
    println!("parts: {}", planned.parts.len());
    for part in &planned.parts {
        println!(
            "part {}: headlands={}, generated={}, ordered={}, tour={}",
            part.part_index,
            part.headlands.len(),
            part.generated_swaths.len(),
            part.ordered_swaths.len(),
            part.tour.len()
        );
    }
}
