//! Demo of AUTO-SPLIT decomposition on a realistic irregular field.
//!
//! AutoSplit recursively bisects the field perpendicular to its longer
//! AABB side until every sub-field fits under `max_side`. Each sub-field
//! keeps its own headland rings (including along the shared split edge,
//! so there's proper turn space where swaths terminate at the split).
//!
//! A fixed fleet of N machines works EACH sub-field together (Block +
//! ByLength), then moves to the next. Each machine's tour is the
//! concatenation of its per-sub-field tours — per-part routing stays
//! intact, so no cross-field spaghetti.
//!
//! The strip between two sub-fields' innermost headland rings is the
//! shared midline turn band — drawn in cyan so you can see where
//! machines transition between halves.
//!
//! Run with:
//!   cargo run --example machine_split_field

#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    Balance, DecompositionMode, DivisionPattern, DivisionPlan, FieldGenerationMode,
    FieldGenerationOptions, Geo, Maptrax, Point, RoutingOptions, RoutingStrategy, Swath,
    TurnPlannerConfig, TurnPlannerModel, point_xy, polygon_from_points, segment_length,
};
use rerun::Color;

fn big_irregular_field() -> maptrax::Polygon {
    polygon_from_points(vec![
        point_xy(-40.0, 90.0),
        point_xy(180.0, 0.0),
        point_xy(620.0, 30.0),
        point_xy(890.0, 180.0),
        point_xy(860.0, 420.0),
        point_xy(520.0, 495.0),
        point_xy(150.0, 450.0),
        point_xy(-60.0, 310.0),
    ])
}

const FLEET_SIZE: usize = 3;
const MAX_SIDE: f64 = 520.0;
const SWATH_WIDTH: f64 = 8.0;
const HEADLAND_COUNT: usize = 2;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_split_field")?;
    let border = big_irregular_field();

    let datum = Geo::new(51.0, 5.0, 0.0);

    rerun_viz::log_polygon(
        &rec,
        "enu/field/border",
        &border,
        Color::from_rgb(160, 100, 100),
    )?;
    rerun_viz::log_polygon_geo(
        &rec,
        "geo/field/border",
        &border,
        datum,
        Color::from_rgb(160, 100, 100),
    )?;

    let mut planner = Maptrax::new();
    planner.set_field(border.clone(), datum).expect("field");

    planner.plan_field(&FieldGenerationOptions {
        swath_width: SWATH_WIDTH,
        headland_count: HEADLAND_COUNT,
        decomposition: DecompositionMode::AutoSplit { max_side: MAX_SIDE },
        mode: FieldGenerationMode::ExplicitAngle(20.0),
    })?;

    let part_count = planner.field()?.get_parts().len();
    println!("=== AutoSplit {{ max_side: {MAX_SIDE} }} + {FLEET_SIZE}-machine fleet ===");
    println!("   Field extent: ~950m x ~495m (irregular 8-vertex polygon)");
    println!(
        "   Swath angle 20°, swath width {}m, {} headland rings per sub-field",
        SWATH_WIDTH, HEADLAND_COUNT
    );
    println!("   Split into {part_count} sub-field(s).");
    println!();

    // Per-sub-field headland rings. On an AutoSplit shared edge, the
    // non-owned side keeps its outermost headland on the split line (no empty
    // seam) and offsets deeper rings normally (no collapsed/overlapping
    // multi-headland lines).
    for (part_index, part) in planner.field()?.get_parts().iter().enumerate() {
        for (ring_index, ring) in part.headlands.iter().enumerate() {
            rerun_viz::log_polygon(
                &rec,
                &format!("enu/parts/part_{part_index}/headland_{ring_index}"),
                &ring.polygon,
                Color::from_rgb(120, 120, 160),
            )?;
            rerun_viz::log_polygon_geo(
                &rec,
                &format!("geo/parts/part_{part_index}/headland_{ring_index}"),
                &ring.polygon,
                datum,
                Color::from_rgb(120, 120, 160),
            )?;
        }
    }

    let plan = DivisionPlan::uniform(FLEET_SIZE, DivisionPattern::Block, Balance::ByLength);
    let turn = TurnPlannerConfig {
        swath_width: SWATH_WIDTH,
        min_turning_radius: 3.0,
        model: TurnPlannerModel::ReedsShepp,
        machine_length: 6.0,
        machine_width: 3.0,
        ..TurnPlannerConfig::default()
    };
    let routing = RoutingOptions {
        strategy: RoutingStrategy::GreedyNearest,
        local_improvement_passes: 1,
    };

    let all_plans = planner.plan_machines_for_all_parts(&plan, routing, &turn)?;

    // For each physical machine, stitch together the per-part tours in part
    // order. Each per-part tour was already routed inside that part with
    // its own headlands — no re-routing, no cross-field jumps.
    let mut assigned_per_machine: Vec<Vec<Swath>> = vec![Vec::new(); FLEET_SIZE];
    let mut tour_per_machine: Vec<Vec<Swath>> = vec![Vec::new(); FLEET_SIZE];
    let mut arcs_per_machine: Vec<Vec<Vec<Point>>> = vec![Vec::new(); FLEET_SIZE];

    for (part_index, planned) in all_plans.iter().enumerate() {
        for machine in &planned.machines {
            let id = machine.machine_index;
            assigned_per_machine[id].extend(machine.assigned_swaths.iter().cloned());
            tour_per_machine[id].extend(machine.tour.iter().cloned());
            arcs_per_machine[id].extend(machine.assigned_headland_arcs.iter().cloned());

            let color = machine_color(id);
            rerun_viz::log_swaths_tinted(
                &rec,
                &format!("enu/parts/part_{part_index}/swaths/machine_{id}"),
                &machine.assigned_swaths,
                color,
            )?;
            rerun_viz::log_swaths_geo_tinted(
                &rec,
                &format!("geo/parts/part_{part_index}/swaths/machine_{id}"),
                &machine.assigned_swaths,
                datum,
                Some(color),
            )?;
        }
    }

    println!(
        "   {:<12} {:>12} {:>12} {:>12}",
        "machine", "swath_count", "work_len_m", "work_s"
    );
    let mut makespan = 0.0f64;
    let mut total_work_len = 0.0;
    for machine_index in 0..FLEET_SIZE {
        let color = machine_color(machine_index);
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/machines/m{machine_index}/swaths"),
            &assigned_per_machine[machine_index],
            color,
        )?;
        rerun_viz::log_polylines(
            &rec,
            &format!("enu/machines/m{machine_index}/headlands"),
            &arcs_per_machine[machine_index],
            color,
        )?;
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/machines/m{machine_index}/tour"),
            &tour_per_machine[machine_index],
            color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/machines/m{machine_index}/swaths"),
            &assigned_per_machine[machine_index],
            datum,
            Some(color),
        )?;
        rerun_viz::log_polylines_geo(
            &rec,
            &format!("geo/machines/m{machine_index}/headlands"),
            &arcs_per_machine[machine_index],
            datum,
            color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/machines/m{machine_index}/tour"),
            &tour_per_machine[machine_index],
            datum,
            Some(color),
        )?;

        let work_len: f64 = assigned_per_machine[machine_index]
            .iter()
            .map(|s| segment_length(s.line))
            .sum();
        let work_s = work_len;
        println!(
            "   m{:<11} {:>12} {:>12.1} {:>12.1}",
            machine_index,
            assigned_per_machine[machine_index].len(),
            work_len,
            work_s,
        );
        makespan = makespan.max(work_s);
        total_work_len += work_len;
    }

    println!();
    println!("   Total work length: {:.1} m", total_work_len);
    println!(
        "   Makespan (slowest machine wins): {:.1} s   ~{:.0}% improvement vs single-machine baseline",
        makespan,
        (1.0 - makespan / total_work_len) * 100.0
    );
    println!(
        "   Each machine's tour = concat(part_0_tour, part_1_tour, …) — no re-routing between parts."
    );

    rec.flush_blocking()?;
    Ok(())
}

fn machine_color(machine_id: usize) -> (u8, u8, u8) {
    match machine_id {
        0 => (230, 90, 90),
        1 => (70, 180, 120),
        2 => (90, 140, 230),
        3 => (220, 170, 60),
        _ => (160, 160, 160),
    }
}
