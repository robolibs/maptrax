#[path = "support/example_scenes.rs"]
mod example_scenes;
#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    Balance, ConnectorMode, DivisionPattern, DivisionPlan, Field, MachinePlanningOptions, Maptrax,
    OptimizeObjective, RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel,
};
use rerun::Color;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_machine_modes_scene")?;
    let datum = example_scenes::upstream_datum();
    let border = example_scenes::upstream_field_polygon(datum);

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

    let scenarios: Vec<(&'static str, DivisionPlan)> = vec![
        (
            "stripe_by_count",
            DivisionPlan::uniform(3, DivisionPattern::Stripe { stride: 1 }, Balance::ByCount),
        ),
        (
            "stripe_stride_2",
            DivisionPlan::uniform(3, DivisionPattern::Stripe { stride: 2 }, Balance::ByCount),
        ),
        (
            "block_by_count",
            DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount),
        ),
        (
            "block_by_length",
            DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByLength),
        ),
        (
            "banded_stripe_2",
            DivisionPlan::uniform(
                3,
                DivisionPattern::BandedStripe { bands: 2 },
                Balance::ByLength,
            ),
        ),
        (
            "optimized_makespan",
            DivisionPlan::uniform(
                3,
                DivisionPattern::Optimized {
                    objective: OptimizeObjective::Makespan,
                },
                Balance::ByLength,
            ),
        ),
    ];

    for (mode_name, plan) in scenarios {
        let mut planner = Maptrax::new();
        planner.set_field_object(field.clone());
        let planned = planner.plan_machines_for_part(
            &MachinePlanningOptions {
                plan,
                part_index: 0,
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

        if let Some(pattern) = planned.division.pattern_used {
            println!("{mode_name} resolved pattern: {:?}", pattern);
        }

        for machine in &planned.machines {
            let color = rerun_viz::machine_color(machine.machine_index);
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
                &format!(
                    "enu/main/{mode_name}/swaths/machine_{}",
                    machine.machine_index
                ),
                &layers.work,
                color,
            )?;
            rerun_viz::log_swaths_geo_tinted(
                &rec,
                &format!(
                    "geo/main/{mode_name}/swaths/machine_{}",
                    machine.machine_index
                ),
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
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!(
                    "enu/main/{mode_name}/turners/headland_travel/machine_{}",
                    machine.machine_index
                ),
                &layers.headland_travel,
                (255, 170, 0),
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
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!(
                    "enu/main/{mode_name}/turners/direct/machine_{}",
                    machine.machine_index
                ),
                &layers.direct,
                (120, 180, 220),
            )?;
            println!(
                "  {mode_name} machine {}: assigned={}, ordered={}, tour={}, work_s={:.1}, transit_m={:.1}",
                machine.machine_index,
                machine.assigned_swaths.len(),
                machine.ordered_swaths.len(),
                machine.tour.len(),
                planned
                    .division
                    .estimated_work_time
                    .get(machine.machine_index)
                    .copied()
                    .unwrap_or(0.0),
                planned
                    .division
                    .estimated_transit
                    .get(machine.machine_index)
                    .copied()
                    .unwrap_or(0.0),
            );
        }
    }

    rec.flush_blocking()?;
    Ok(())
}
