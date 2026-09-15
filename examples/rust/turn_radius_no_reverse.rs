//! Forward-only machine (Dubins): turns cannot back up. It skips rows
//! (turn-radius-aware) so consecutive passes are far enough apart to turn, and
//! routes through the densely-interpolated headland nodes. Two machines split
//! the field.
//!
//! Companion: `turn_radius_reverse.rs` (Reeds-Shepp, can reverse).
//!
//! Run a Rerun viewer (or `rerun --serve`) then:
//!   cargo run --example turn_radius_no_reverse

#[path = "support/example_scenes.rs"]
mod example_scenes;
#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    Balance, DivisionPattern, DivisionPlan, MachinePlanningOptions, Maptrax, RoutingOptions,
    RoutingStrategy, TurnPlannerConfig, TurnPlannerModel, segment_length,
};
use rerun::Color;

const SWATH_WIDTH: f64 = 3.0;
const HEADLANDS: usize = 3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_turn_no_reverse")?;
    let datum = example_scenes::upstream_datum();
    let border = example_scenes::upstream_field_polygon(datum);

    let cfg = TurnPlannerConfig {
        model: TurnPlannerModel::Dubins, // forward only — cannot reverse
        min_turning_radius: 6.0,
        machine_length: 8.0,
        machine_width: 4.0,
        swath_width: SWATH_WIDTH,
        ..TurnPlannerConfig::default()
    };

    let mut planner = Maptrax::new();
    planner.set_field(border.clone(), datum)?;
    planner.generate_field(SWATH_WIDTH, 0.0, HEADLANDS)?;

    let planned = planner.plan_machines_for_part(
        &MachinePlanningOptions {
            plan: DivisionPlan::uniform(2, DivisionPattern::Block, Balance::ByLength),
            part_index: 0,
        },
        // Skip rows so a forward U-turn has the lateral room it needs.
        RoutingOptions { strategy: RoutingStrategy::TurnRadiusAware, local_improvement_passes: 0 },
        &cfg,
    )?;

    rerun_viz::log_polygon(&rec, "enu/border", &border, Color::from_rgb(120, 70, 70))?;
    rerun_viz::log_polygon_geo(&rec, "geo/border", &border, datum, Color::from_rgb(120, 70, 70))?;
    for ring in &planner.field()?.part(0)?.headlands {
        rerun_viz::log_polygon(&rec, "enu/headland_band", &ring.polygon, Color::from_rgb(90, 90, 120))?;
        rerun_viz::log_polygon_geo(&rec, "geo/headland_band", &ring.polygon, datum, Color::from_rgb(90, 90, 120))?;
    }

    println!("=== Forward-only (Dubins), {HEADLANDS} headlands, swath {SWATH_WIDTH} m ===\n");
    println!("   {:<8} {:>8}  {:>10}  {:>11}  {:>10}", "machine", "swaths", "work_len", "over_border", "in_crop");
    for machine in &planned.machines {
        let idx = machine.machine_index;
        let color = rerun_viz::machine_color(idx);
        rerun_viz::log_swaths_tinted(&rec, &format!("enu/machine_{idx}/swaths"), &machine.assigned_swaths, color)?;
        rerun_viz::log_swaths_tinted(&rec, &format!("enu/machine_{idx}/tour"), &machine.tour, color)?;
        rerun_viz::log_polylines(&rec, &format!("enu/machine_{idx}/headlands"), &machine.assigned_headland_arcs, color)?;
        rerun_viz::log_swaths_geo_tinted(&rec, &format!("geo/machine_{idx}/swaths"), &machine.assigned_swaths, datum, Some(color))?;
        rerun_viz::log_swaths_geo_tinted(&rec, &format!("geo/machine_{idx}/tour"), &machine.tour, datum, Some(color))?;
        rerun_viz::log_polylines_geo(&rec, &format!("geo/machine_{idx}/headlands"), &machine.assigned_headland_arcs, datum, color)?;

        let v = planner.validate_part_tour(0, &machine.tour)?;
        let work_len: f64 = machine.assigned_swaths.iter().map(|s| segment_length(s.line)).sum();
        println!("   {:<8} {:>8}  {:>10.0}  {:>11}  {:>10}", idx, machine.assigned_swaths.len(), work_len, v.outside_boundary, v.through_work_area);
    }

    rec.flush_blocking()?;
    Ok(())
}
