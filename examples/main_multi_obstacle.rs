#[path = "support/example_scenes.rs"]
mod example_scenes;
#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    ConnectorMode, FieldGenerationMode, FieldGenerationOptions, Maptrax, ObstaclePlanningOptions,
    PlannerOptions, RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel,
};
use rerun::Color;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_multi_obstacle_scene")?;
    let datum = example_scenes::upstream_datum();
    let border = example_scenes::upstream_field_polygon(datum);
    let obstacles = example_scenes::offset_multi_obstacles(&border);

    let mut planner = Maptrax::new();
    planner.set_field(border.clone(), datum)?;
    let planned = planner.plan_stages(&PlannerOptions {
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
        obstacles: ObstaclePlanningOptions {
            obstacles: obstacles.clone(),
            inflation_distance: 2.0,
        },
        turn: TurnPlannerConfig {
            swath_width: 4.0,
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
    for (index, polygon) in obstacles.iter().enumerate() {
        rerun_viz::log_polygon(
            &rec,
            &format!("enu/obstacles/{index}"),
            polygon,
            Color::from_rgb(220, 40, 40),
        )?;
        rerun_viz::log_polygon_geo(
            &rec,
            &format!("geo/obstacles/{index}"),
            polygon,
            datum,
            Color::from_rgb(220, 40, 40),
        )?;
    }

    for part in &planned.parts {
        let layers = example_scenes::split_tour_layers(&part.tour);
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
        for (index, ring) in part.transit_rings.iter().enumerate() {
            rerun_viz::log_polygon(
                &rec,
                &format!("enu/debug/obstacle_headlands/part_{}/ring_{index}", part.part_index),
                &ring.polygon,
                Color::from_rgb(240, 200, 30),
            )?;
            rerun_viz::log_polygon_geo(
                &rec,
                &format!("geo/debug/obstacle_headlands/part_{}/ring_{index}", part.part_index),
                &ring.polygon,
                datum,
                Color::from_rgb(240, 200, 30),
            )?;
        }
        rerun_viz::log_swaths(
            &rec,
            &format!("enu/debug/generated/part_{}", part.part_index),
            &part.generated_swaths,
        )?;
        rerun_viz::log_swaths_geo(
            &rec,
            &format!("geo/debug/generated/part_{}", part.part_index),
            &part.generated_swaths,
            datum,
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/debug/avoided/part_{}", part.part_index),
            &part.avoided_swaths,
            (29, 145, 192),
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/debug/avoided/part_{}", part.part_index),
            &part.avoided_swaths,
            datum,
            Some((29, 145, 192)),
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/main/swaths/part_{}", part.part_index),
            &layers.work,
            (90, 130, 90),
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/main/swaths/part_{}", part.part_index),
            &layers.work,
            datum,
            Some((90, 130, 90)),
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
            "part {}: headlands={}, obstacle_headlands={}, generated={}, avoided={}, ordered={}, tour={}",
            part.part_index,
            part.headlands.len(),
            part.transit_rings.len(),
            part.generated_swaths.len(),
            part.avoided_swaths.len(),
            part.ordered_swaths.len(),
            part.tour.len()
        );
    }
    Ok(())
}
