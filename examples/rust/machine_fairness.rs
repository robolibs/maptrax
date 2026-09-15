//! Demonstrates the new multi-machine division modes side-by-side on an
//! irregular field with 3 machines. Prints work-time and transit estimates
//! per scenario so you can see how each strategy trades locality vs balance.
//!
//! Run with:
//!   cargo run --example machine_fairness

#[path = "support/example_scenes.rs"]
mod example_scenes;

use maptrax::{
    Balance, DivisionPattern, DivisionPlan, Field, MachinePlanningOptions, MachineProfile, Maptrax,
    OptimizeObjective, RoutingOptions, RoutingStrategy, TurnPlannerConfig, segment_length,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let datum = example_scenes::upstream_datum();
    let border = example_scenes::upstream_field_polygon(datum);

    let mut field = Field::new(border.clone(), datum)?;
    field.gen_field(4.0, 0.0, 3)?;

    let part = &field.get_parts()[0];
    let total_swaths = part.swaths.len();
    let total_length: f64 = part.swaths.iter().map(|s| segment_length(s.line)).sum();
    println!("Field: {total_swaths} swaths, total work length = {total_length:.1} m\n");

    let scenarios: Vec<(String, DivisionPlan)> = vec![
        (
            "Stripe{stride:1} + ByCount   (classic alternating)".into(),
            DivisionPlan::uniform(3, DivisionPattern::Stripe { stride: 1 }, Balance::ByCount),
        ),
        (
            "Stripe{stride:2} + ByCount   (pairs of rows)".into(),
            DivisionPlan::uniform(3, DivisionPattern::Stripe { stride: 2 }, Balance::ByCount),
        ),
        (
            "Block + ByCount              (contiguous, equal swath count)".into(),
            DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount),
        ),
        (
            "Block + ByLength             (contiguous, equal work — this is the \"big blocks divied smartly\" mode)".into(),
            DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByLength),
        ),
        (
            "BandedStripe{bands:2} + ByLength  (two regions, internally striped)".into(),
            DivisionPlan::uniform(
                3,
                DivisionPattern::BandedStripe { bands: 2 },
                Balance::ByLength,
            ),
        ),
        (
            "Weighted (2.0, 1.0, 1.0) Block + ByLength  (machine 0 is 2x faster)".into(),
            DivisionPlan {
                pattern: DivisionPattern::Block,
                balance: Balance::ByLength,
                machines: vec![
                    MachineProfile { weight: 2.0, speed: 1.0 },
                    MachineProfile { weight: 1.0, speed: 1.0 },
                    MachineProfile { weight: 1.0, speed: 1.0 },
                ],
                headlands: maptrax::HeadlandMode::default(),
            },
        ),
        (
            "Optimized{Makespan} + ByLength  (planner picks)".into(),
            DivisionPlan::uniform(
                3,
                DivisionPattern::Optimized {
                    objective: OptimizeObjective::Makespan,
                },
                Balance::ByLength,
            ),
        ),
    ];

    for (label, plan) in scenarios {
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
                ..TurnPlannerConfig::default()
            },
        )?;

        println!("=== {label} ===");
        if let Some(pattern) = planned.division.pattern_used {
            println!("   resolved: {:?}", pattern);
        }
        println!(
            "   {:<8} {:>8}  {:>10}  {:>10}  {:>10}",
            "machine", "swaths", "work_len(m)", "work_time(s)", "transit(m)"
        );
        let mut max_time = 0.0f64;
        for machine in &planned.machines {
            let work_len: f64 = machine
                .assigned_swaths
                .iter()
                .map(|s| segment_length(s.line))
                .sum();
            let work_time = planned
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
                work_time,
                transit,
            );
            max_time = max_time.max(work_time);
        }
        let total_transit: f64 = planned.division.estimated_transit.iter().sum();
        println!("   makespan(s): {max_time:.1}   total_transit(m): {total_transit:.1}\n");
    }

    Ok(())
}
