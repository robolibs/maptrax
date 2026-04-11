#[path = "support/example_scenes.rs"]
mod example_scenes;
#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    ConnectorMode, DivisionType, Field, MachinePlanningOptions, Maptrax, ObstaclePlanningOptions,
    RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel,
};
use rerun::Color;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_machine_modes_scene")?;
    let datum = example_scenes::upstream_datum();
    let border = example_scenes::upstream_field_polygon(datum);
    let obstacle = example_scenes::centered_obstacle(&border, 25.0);

    let mut field = Field::new(border.clone(), datum)?;
    field.gen_field(4.0, 0.0, 3)?;

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
    rerun_viz::log_polygon(
        &rec,
        "enu/field/obstacle",
        &obstacle,
        Color::from_rgb(220, 40, 40),
    )?;
    rerun_viz::log_polygon_geo(
        &rec,
        "geo/field/obstacle",
        &obstacle,
        datum,
        Color::from_rgb(220, 40, 40),
    )?;

    for division_type in [
        DivisionType::Alternate,
        DivisionType::Block,
        DivisionType::SpatialRtree,
        DivisionType::LengthBalanced,
    ]
    .into_iter()
    {
        let mut planner = Maptrax::new();
        planner.set_field_object(field.clone());
        let planned = planner.plan_machines_for_part(
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
                swath_width: 4.0,
                min_turning_radius: 2.0,
                model: TurnPlannerModel::ReedsShepp,
                connector_mode: ConnectorMode::Auto,
                machine_length: 6.0,
                machine_width: 3.0,
                ..TurnPlannerConfig::default()
            },
        )?;

        for machine in &planned.machines {
            let color = rerun_viz::machine_color(machine.machine_index);
            let mode_name = division_name(division_type);
            let layers = example_scenes::split_tour_layers(&machine.tour);
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!(
                    "enu/debug/{mode_name}/assigned/machine_{}",
                    machine.machine_index
                ),
                &machine.assigned_swaths,
                color,
            )?;
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!(
                    "enu/debug/{mode_name}/ordered/machine_{}",
                    machine.machine_index
                ),
                &machine.ordered_swaths,
                color,
            )?;
            rerun_viz::log_swaths_geo_tinted(
                &rec,
                &format!(
                    "geo/debug/{mode_name}/ordered/machine_{}",
                    machine.machine_index
                ),
                &machine.ordered_swaths,
                datum,
                Some(color),
            )?;
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!("enu/main/{mode_name}/swaths/machine_{}", machine.machine_index),
                &layers.work,
                color,
            )?;
            rerun_viz::log_swaths_geo_tinted(
                &rec,
                &format!("geo/main/{mode_name}/swaths/machine_{}", machine.machine_index),
                &layers.work,
                datum,
                Some(color),
            )?;
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!(
                    "enu/main/{mode_name}/turners/row_to_headland/machine_{}",
                    machine.machine_index
                ),
                &layers.row_to_headland,
                (220, 60, 60),
            )?;
            rerun_viz::log_swaths_geo_tinted(
                &rec,
                &format!(
                    "geo/main/{mode_name}/turners/row_to_headland/machine_{}",
                    machine.machine_index
                ),
                &layers.row_to_headland,
                datum,
                Some((220, 60, 60)),
            )?;
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!(
                    "enu/main/{mode_name}/turners/headland_travel/machine_{}",
                    machine.machine_index
                ),
                &layers.headland_travel,
                (255, 170, 0),
            )?;
            rerun_viz::log_swaths_geo_tinted(
                &rec,
                &format!(
                    "geo/main/{mode_name}/turners/headland_travel/machine_{}",
                    machine.machine_index
                ),
                &layers.headland_travel,
                datum,
                Some((255, 170, 0)),
            )?;
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!(
                    "enu/main/{mode_name}/turners/headland_to_row/machine_{}",
                    machine.machine_index
                ),
                &layers.headland_to_row,
                (70, 190, 90),
            )?;
            rerun_viz::log_swaths_geo_tinted(
                &rec,
                &format!(
                    "geo/main/{mode_name}/turners/headland_to_row/machine_{}",
                    machine.machine_index
                ),
                &layers.headland_to_row,
                datum,
                Some((70, 190, 90)),
            )?;
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!(
                    "enu/main/{mode_name}/turners/direct/machine_{}",
                    machine.machine_index
                ),
                &layers.direct,
                (120, 180, 220),
            )?;
            rerun_viz::log_swaths_geo_tinted(
                &rec,
                &format!(
                    "geo/main/{mode_name}/turners/direct/machine_{}",
                    machine.machine_index
                ),
                &layers.direct,
                datum,
                Some((120, 180, 220)),
            )?;
            println!(
                "{} machine {}: assigned={}, avoided={}, ordered={}, tour={}",
                mode_name,
                machine.machine_index,
                machine.assigned_swaths.len(),
                machine.avoided_swaths.len(),
                machine.ordered_swaths.len(),
                machine.tour.len()
            );
        }
    }

    rec.flush_blocking()?;
    Ok(())
}

fn division_name(division_type: DivisionType) -> &'static str {
    match division_type {
        DivisionType::Alternate => "alternate",
        DivisionType::Block => "block",
        DivisionType::SpatialRtree => "spatial_rtree",
        DivisionType::LengthBalanced => "length_balanced",
    }
}
