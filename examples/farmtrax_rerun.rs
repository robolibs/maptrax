#[path = "support/example_scenes.rs"]
mod example_scenes;
#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    Balance, DivisionPattern, DivisionPlan, Field, MachinePlanningOptions, Maptrax,
    ObstacleAvoider, RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel,
};
use rerun::Color;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let app_id = "maptrax_farmtrax_scene";
    let rec = rerun_viz::connect(app_id)?;
    let datum = example_scenes::upstream_datum();
    let border = example_scenes::upstream_field_polygon(datum);

    let mut field = Field::new(border.clone(), datum)?;
    field.gen_field(4.0, 0.0, 3)?;
    let part = &field.get_parts()[0];

    let obstacle = example_scenes::centered_obstacle(&border, 25.0);
    let mut avoider = ObstacleAvoider::new(vec![obstacle.clone()], datum);
    let avoided = avoider.avoid(&part.swaths, 2.0);

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

    for (index, headland) in part.headlands.iter().enumerate() {
        rerun_viz::log_polygon(
            &rec,
            &format!("enu/field/headland/{index}"),
            &headland.polygon,
            Color::from_rgb(70, 120, 70),
        )?;
        rerun_viz::log_polygon_geo(
            &rec,
            &format!("geo/field/headland/{index}"),
            &headland.polygon,
            datum,
            Color::from_rgb(70, 120, 70),
        )?;
    }

    rerun_viz::log_swaths(&rec, "enu/field/swaths", &part.swaths)?;
    rerun_viz::log_swaths_geo(&rec, "geo/field/swaths", &part.swaths, datum)?;
    rerun_viz::log_polygon(
        &rec,
        "enu/avoidance/obstacle",
        &obstacle,
        Color::from_rgb(220, 20, 20),
    )?;
    rerun_viz::log_polygon_geo(
        &rec,
        "geo/avoidance/obstacle",
        &obstacle,
        datum,
        Color::from_rgb(220, 20, 20),
    )?;
    rerun_viz::log_swaths(&rec, "enu/avoidance/avoided", &avoided)?;
    rerun_viz::log_swaths_geo(&rec, "geo/avoidance/avoided", &avoided, datum)?;

    let mut planner = Maptrax::new();
    planner.set_field_object(field.clone());
    let machine_plan = planner.plan_machines_for_part(
        &MachinePlanningOptions {
            plan: DivisionPlan::uniform(
                4,
                DivisionPattern::Stripe { stride: 1 },
                Balance::ByCount,
            ),
            part_index: 0,
        },
        &maptrax::ObstaclePlanningOptions {
            obstacles: vec![obstacle.clone()],
            inflation_distance: 2.0,
        },
        RoutingOptions {
            strategy: RoutingStrategy::GreedyNearest,
            local_improvement_passes: 0,
        },
        &TurnPlannerConfig {
            swath_width: 4.0,
            min_turning_radius: 2.0,
            model: TurnPlannerModel::ReedsShepp,
            ..TurnPlannerConfig::default()
        },
    )?;

    let mut machine_summaries = Vec::new();
    for machine_plan in &machine_plan.machines {
        let machine = machine_plan.machine_index;
        let machine_color = rerun_viz::machine_color(machine);
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/division/machine_{machine}"),
            &machine_plan.avoided_swaths,
            machine_color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/division/machine_{machine}"),
            &machine_plan.avoided_swaths,
            datum,
            Some(machine_color),
        )?;
        if machine_plan.assigned_swaths.is_empty() {
            machine_summaries.push((machine, 0_usize, 0_usize, 0_usize));
            continue;
        }

        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/main/machine_{machine}"),
            &machine_plan.ordered_swaths,
            machine_color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/main/machine_{machine}"),
            &machine_plan.ordered_swaths,
            datum,
            Some(machine_color),
        )?;
        machine_summaries.push((
            machine,
            machine_plan.assigned_swaths.len(),
            machine_plan.avoided_swaths.len(),
            machine_plan.ordered_swaths.len(),
        ));
    }

    rec.flush_blocking()?;
    println!("Field area: {:.1} m^2", field.total_area());
    println!(
        "Part 0: {} headlands, {} swaths, {} avoided segments",
        part.headlands.len(),
        part.swaths.len(),
        avoided.len(),
    );
    for (machine, assigned, assigned_avoided, nety_count) in machine_summaries {
        println!(
            "Machine {machine}: assigned={assigned}, assigned_avoided={assigned_avoided}, nety={nety_count}"
        );
    }
    println!(
        "Connected to Rerun at {}",
        std::env::var("RERUN_URL")
            .unwrap_or_else(|_| "rerun+http://0.0.0.0:9876/proxy".to_string())
    );
    Ok(())
}
