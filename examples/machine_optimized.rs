//! Demo of OPTIMIZED multi-machine division: evaluates every candidate
//! pattern (Block, Stripe{1}, BandedStripe{2}, BandedStripe{3}) for the
//! given field and picks the one minimising the chosen objective.
//!
//! This example prints the score of EACH candidate so you can see the
//! optimiser's reasoning, then logs the winner to Rerun.
//!
//! Run with:
//!   cargo run --example machine_optimized

#[path = "support/example_scenes.rs"]
mod example_scenes;
#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    Balance, DivisionPattern, DivisionPlan, Field, MachinePlanningOptions, Maptrax,
    OptimizeObjective, RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel,
    segment_length,
};
use rerun::Color;

struct Scored {
    label: &'static str,
    pattern: DivisionPattern,
    makespan: f64,
    total_transit: f64,
    work_times: Vec<f64>,
    transits: Vec<f64>,
    work_lengths: Vec<f64>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_optimized")?;
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

    let candidates: Vec<(&'static str, DivisionPattern)> = vec![
        ("Block", DivisionPattern::Block),
        ("Stripe{1}", DivisionPattern::Stripe { stride: 1 }),
        ("BandedStripe{2}", DivisionPattern::BandedStripe { bands: 2 }),
        ("BandedStripe{3}", DivisionPattern::BandedStripe { bands: 3 }),
    ];

    let turn = TurnPlannerConfig {
        swath_width: 4.0,
        min_turning_radius: 2.0,
        model: TurnPlannerModel::ReedsShepp,
        machine_length: 6.0,
        machine_width: 3.0,
        ..TurnPlannerConfig::default()
    };
    let routing = RoutingOptions {
        strategy: RoutingStrategy::GreedyNearest,
        local_improvement_passes: 1,
    };

    // Evaluate every candidate by actually running it and reading the
    // reported work times + transit distances.
    let mut scored: Vec<Scored> = Vec::new();
    for (label, pattern) in &candidates {
        let mut planner = Maptrax::new();
        planner.set_field_object(field.clone());
        let planned = planner.plan_machines_for_part(
            &MachinePlanningOptions {
                plan: DivisionPlan::uniform(3, *pattern, Balance::ByLength),
                part_index: 0,
            },
            routing,
            &turn,
        )?;

        let work_times = planned.division.estimated_work_time.clone();
        let transits = planned.division.estimated_transit.clone();
        let makespan = work_times
            .iter()
            .zip(transits.iter())
            .map(|(w, t)| w + t)
            .fold(0.0f64, f64::max);
        let total_transit: f64 = transits.iter().sum();
        let work_lengths: Vec<f64> = planned
            .machines
            .iter()
            .map(|m| m.assigned_swaths.iter().map(|s| segment_length(s.line)).sum())
            .collect();

        scored.push(Scored {
            label,
            pattern: *pattern,
            makespan,
            total_transit,
            work_times,
            transits,
            work_lengths,
        });
    }

    // Now run the Optimized planner — it will pick one of the above.
    let mut planner = Maptrax::new();
    planner.set_field_object(field.clone());
    let winner_plan = planner.plan_machines_for_part(
        &MachinePlanningOptions {
            plan: DivisionPlan::uniform(
                3,
                DivisionPattern::Optimized {
                    objective: OptimizeObjective::Makespan,
                },
                Balance::ByLength,
            ),
            part_index: 0,
        },
        routing,
        &turn,
    )?;

    let winner_pattern = winner_plan
        .division
        .pattern_used
        .expect("optimized must resolve to a concrete pattern");

    println!("=== Optimized {{ Makespan }} + ByLength — 3 machines ===");
    println!("   Objective: minimise max(work_time + transit_time) across machines.\n");

    println!("   Candidate scores:");
    println!(
        "   {:<18}  {:>12}  {:>14}  {:>18}",
        "pattern", "makespan_s", "total_transit_m", "per-machine work_s"
    );
    for entry in &scored {
        let per_machine: Vec<String> = entry
            .work_times
            .iter()
            .zip(entry.transits.iter())
            .map(|(w, t)| format!("{:.0}+{:.0}", w, t))
            .collect();
        let marker = if entry.pattern == winner_pattern {
            " ← winner"
        } else {
            ""
        };
        println!(
            "   {:<18}  {:>12.1}  {:>14.1}  {:>18}{}",
            entry.label,
            entry.makespan,
            entry.total_transit,
            per_machine.join(", "),
            marker
        );
    }

    println!("\n   Winner: {:?}", winner_pattern);
    let winner_entry = scored
        .iter()
        .find(|entry| entry.pattern == winner_pattern)
        .expect("winner must be among candidates");
    println!(
        "   Winner makespan: {:.1}s   total transit: {:.1}m\n",
        winner_entry.makespan, winner_entry.total_transit
    );

    println!("   Per-machine breakdown (winner):");
    println!(
        "   {:<8} {:>8}  {:>10}  {:>10}  {:>10}",
        "machine", "swaths", "work_len", "work_s", "transit_m"
    );
    for (index, machine) in winner_plan.machines.iter().enumerate() {
        let color = rerun_viz::machine_color(machine.machine_index);
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/winner/swaths/machine_{}", machine.machine_index),
            &machine.assigned_swaths,
            color,
        )?;
        rerun_viz::log_polylines(
            &rec,
            &format!("enu/winner/headlands/machine_{}", machine.machine_index),
            &machine.assigned_headland_arcs,
            color,
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/winner/tour/machine_{}", machine.machine_index),
            &machine.tour,
            color,
        )?;
        println!(
            "   {:<8} {:>8}  {:>10.1}  {:>10.1}  {:>10.1}",
            machine.machine_index,
            machine.assigned_swaths.len(),
            winner_entry.work_lengths[index],
            winner_entry.work_times[index],
            winner_entry.transits[index],
        );
    }

    // Also push every candidate to Rerun under its own path so you can
    // visually compare what the OTHER options would have looked like.
    for (label, pattern) in &candidates {
        let mut planner = Maptrax::new();
        planner.set_field_object(field.clone());
        let planned = planner.plan_machines_for_part(
            &MachinePlanningOptions {
                plan: DivisionPlan::uniform(3, *pattern, Balance::ByLength),
                part_index: 0,
            },
            routing,
            &turn,
        )?;
        let slug = label
            .replace([' ', '{', '}'], "")
            .replace(':', "_")
            .to_lowercase();
        for machine in &planned.machines {
            let color = rerun_viz::machine_color(machine.machine_index);
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!("enu/candidates/{slug}/swaths/machine_{}", machine.machine_index),
                &machine.assigned_swaths,
                color,
            )?;
            rerun_viz::log_polylines(
                &rec,
                &format!(
                    "enu/candidates/{slug}/headlands/machine_{}",
                    machine.machine_index
                ),
                &machine.assigned_headland_arcs,
                color,
            )?;
        }
    }

    rec.flush_blocking()?;
    Ok(())
}
