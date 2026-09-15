use maptrax::{
    FieldGenerationMode, FieldGenerationOptions, Geo, Maptrax, PlannerOptions, TurnPlannerConfig,
    TurnPlannerModel, point_xy, polygon_from_points,
};

fn main() {
    let polygon = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(100.0, 0.0),
        point_xy(100.0, 50.0),
        point_xy(0.0, 50.0),
    ]);

    let mut maptrax = Maptrax::new();
    maptrax
        .set_field(polygon, Geo::new(51.0, 5.0, 0.0))
        .expect("field");

    let planned = maptrax
        .plan_all(&PlannerOptions {
            field: FieldGenerationOptions {
                swath_width: 10.0,
                headland_count: 1,
                mode: FieldGenerationMode::ExplicitAngle(90.0),
                ..FieldGenerationOptions::default()
            },
            turn: TurnPlannerConfig {
                model: TurnPlannerModel::ReedsShepp,
                ..TurnPlannerConfig::default()
            },
            ..PlannerOptions::default()
        })
        .expect("plan");

    println!(
        "facade flow: {} parts, {} ordered swaths, {} tour segments",
        planned.parts.len(),
        planned.parts[0].ordered_swaths.len(),
        planned.parts[0].tour.len()
    );

    let staged = maptrax
        .plan_stages(&PlannerOptions {
            field: FieldGenerationOptions {
                swath_width: 10.0,
                headland_count: 1,
                mode: FieldGenerationMode::ExplicitAngle(90.0),
                ..FieldGenerationOptions::default()
            },
            turn: TurnPlannerConfig {
                model: TurnPlannerModel::ReedsShepp,
                ..TurnPlannerConfig::default()
            },
            ..PlannerOptions::default()
        })
        .expect("staged plan");

    println!(
        "staged flow: headlands={}, generated={}, ordered={}, tour={}",
        staged.parts[0].headlands.len(),
        staged.parts[0].generated_swaths.len(),
        staged.parts[0].ordered_swaths.len(),
        staged.parts[0].tour.len()
    );
}
