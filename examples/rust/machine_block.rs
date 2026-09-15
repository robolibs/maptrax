//! Demo of BLOCK multi-machine division: each machine gets one contiguous
//! chunk of the field. Uses Balance::ByLength so the split is placed where
//! the work is actually fair, not just where swath counts are equal.
//! This is the "big blocks divied smartly" mode.
//!
//! Run with:
//!   cargo run --example machine_block

#[path = "support/example_scenes.rs"]
mod example_scenes;
#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    Balance, DivisionPattern, DivisionPlan, Field, MachinePlanningOptions, Maptrax, RoutingOptions,
    RoutingStrategy, TurnPlannerConfig, TurnPlannerModel, segment_length,
};
use rerun::Color;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_block")?;
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

    let mut planner = Maptrax::new();
    planner.set_field_object(field.clone());
    let planned = planner.plan_machines_for_part(
        &MachinePlanningOptions {
            plan: DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByLength),
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
            machine_length: 6.0,
            machine_width: 3.0,
            ..TurnPlannerConfig::default()
        },
    )?;

    println!("=== Block + ByLength — 3 machines ===");
    println!("   Each machine owns one contiguous band of the field.");
    println!("   Boundaries are placed so each machine's total work length is balanced.\n");
    println!(
        "   {:<8} {:>8}  {:>10}  {:>10}  {:>10}",
        "machine", "swaths", "work_len", "work_s", "transit_m"
    );
    let mut max_time = 0.0f64;
    let mut total_transit = 0.0;
    for machine in &planned.machines {
        let color = rerun_viz::machine_color(machine.machine_index);
        let idx = machine.machine_index;

        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/swaths/machine_{idx}"),
            &machine.assigned_swaths,
            color,
        )?;
        rerun_viz::log_polylines(
            &rec,
            &format!("enu/headlands/machine_{idx}"),
            &machine.assigned_headland_arcs,
            color,
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/tour/machine_{idx}"),
            &machine.tour,
            color,
        )?;

        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/swaths/machine_{idx}"),
            &machine.assigned_swaths,
            datum,
            Some(color),
        )?;
        rerun_viz::log_polylines_geo(
            &rec,
            &format!("geo/headlands/machine_{idx}"),
            &machine.assigned_headland_arcs,
            datum,
            color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/tour/machine_{idx}"),
            &machine.tour,
            datum,
            Some(color),
        )?;

        let work_len: f64 = machine
            .assigned_swaths
            .iter()
            .map(|s| segment_length(s.line))
            .sum();
        let work_s = planned
            .division
            .estimated_work_time
            .get(machine.machine_index)
            .copied()
            .unwrap_or(0.0);
        let transit = planned
            .division
            .estimated_transit
            .get(machine.machine_index)
            .copied()
            .unwrap_or(0.0);
        println!(
            "   {:<8} {:>8}  {:>10.1}  {:>10.1}  {:>10.1}",
            machine.machine_index,
            machine.assigned_swaths.len(),
            work_len,
            work_s,
            transit,
        );
        max_time = max_time.max(work_s);
        total_transit += transit;
    }
    println!("\n   makespan: {max_time:.1}s   total transit: {total_transit:.1}m");
    println!("   Note: swath counts differ per machine because some rows are shorter than others.");

    rec.flush_blocking()?;
    Ok(())
}
