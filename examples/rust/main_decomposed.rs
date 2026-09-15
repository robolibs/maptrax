#[path = "support/example_scenes.rs"]
mod example_scenes;
#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    ConnectorMode, DecompositionMode, FieldGenerationMode, FieldGenerationOptions, Maptrax,
    PlannerOptions, RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel,
};
use rerun::Color;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_decomposed_scene")?;
    let datum = example_scenes::upstream_datum();
    let border = example_scenes::concave_demo_polygon();

    let mut planner = Maptrax::new();
    planner.set_field(border.clone(), datum)?;
    let planned = planner.plan_stages(&PlannerOptions {
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
            swath_width: 10.0,
            min_turning_radius: 2.0,
            model: TurnPlannerModel::ReedsShepp,
            connector_mode: ConnectorMode::Auto,
            machine_length: 6.0,
            machine_width: 3.0,
            ..TurnPlannerConfig::default()
        },
        ..PlannerOptions::default()
    })?;

    rerun_viz::log_polygon(
        &rec,
        "enu/field/border",
        &border,
        Color::from_rgb(120, 70, 70),
    )?;
    rerun_viz::log_polygon_geo(
        &rec,
        "geo/field/border",
        &border,
        datum,
        Color::from_rgb(120, 70, 70),
    )?;

    for part in &planned.parts {
        let color = rerun_viz::machine_color(part.part_index);
        let layers = example_scenes::split_tour_layers(&part.tour);
        rerun_viz::log_polygon(
            &rec,
            &format!("enu/main/parts/part_{}", part.part_index),
            &planner.field()?.get_parts()[part.part_index]
                .boundary
                .polygon,
            Color::from_rgb(color.0, color.1, color.2),
        )?;
        rerun_viz::log_polygon_geo(
            &rec,
            &format!("geo/main/parts/part_{}", part.part_index),
            &planner.field()?.get_parts()[part.part_index]
                .boundary
                .polygon,
            datum,
            Color::from_rgb(color.0, color.1, color.2),
        )?;
        for (index, headland) in part.headlands.iter().enumerate() {
            rerun_viz::log_polygon(
                &rec,
                &format!("enu/debug/headlands/part_{}/ring_{index}", part.part_index),
                &headland.polygon,
                Color::from_rgb(70, 120, 70),
            )?;
            rerun_viz::log_polygon_geo(
                &rec,
                &format!("geo/debug/headlands/part_{}/ring_{index}", part.part_index),
                &headland.polygon,
                datum,
                Color::from_rgb(70, 120, 70),
            )?;
        }
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/debug/generated/part_{}", part.part_index),
            &part.generated_swaths,
            color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/debug/generated/part_{}", part.part_index),
            &part.generated_swaths,
            datum,
            Some(color),
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/debug/ordered/part_{}", part.part_index),
            &part.ordered_swaths,
            color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/debug/ordered/part_{}", part.part_index),
            &part.ordered_swaths,
            datum,
            Some(color),
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/main/swaths/part_{}", part.part_index),
            &layers.work,
            color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/main/swaths/part_{}", part.part_index),
            &layers.work,
            datum,
            Some(color),
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/main/turners/row_to_headland/part_{}", part.part_index),
            &layers.row_to_headland,
            (220, 60, 60),
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/main/turners/row_to_headland/part_{}", part.part_index),
            &layers.row_to_headland,
            datum,
            Some((220, 60, 60)),
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/main/turners/headland_travel/part_{}", part.part_index),
            &layers.headland_travel,
            (255, 170, 0),
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/main/turners/headland_travel/part_{}", part.part_index),
            &layers.headland_travel,
            datum,
            Some((255, 170, 0)),
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/main/turners/headland_to_row/part_{}", part.part_index),
            &layers.headland_to_row,
            (70, 190, 90),
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/main/turners/headland_to_row/part_{}", part.part_index),
            &layers.headland_to_row,
            datum,
            Some((70, 190, 90)),
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/main/turners/direct/part_{}", part.part_index),
            &layers.direct,
            (120, 180, 220),
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/main/turners/direct/part_{}", part.part_index),
            &layers.direct,
            datum,
            Some((120, 180, 220)),
        )?;
    }

    rec.flush_blocking()?;
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
    Ok(())
}
