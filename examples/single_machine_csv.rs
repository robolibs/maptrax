//! Single-machine tour export + Rerun viz.
//!
//! Uses the caller-provided field coordinates, plans a tour for one
//! machine, then:
//!   * writes every point of the tour to `target/single_machine_path.csv`
//!   * logs the field, headlands, swaths, full tour and flat polyline to
//!     a Rerun recording named `maptrax_single_csv`
//!
//! Run with:
//!   cargo run --example single_machine_csv

#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use std::fs::{File, create_dir_all};
use std::io::{BufWriter, Write};

use concord::Geo;
use geo::Point;
use maptrax::{
    Balance, DivisionPattern, DivisionPlan, MachinePlanningOptions, Maptrax, RoutingOptions,
    RoutingStrategy, TurnPlannerConfig, TurnPlannerModel, polygon_from_points, tour_polyline,
};
use rerun::Color;

/// Field boundary supplied by the caller.
fn field_points() -> Vec<Point<f64>> {
    vec![
        Point::new(0.0, 0.0),
        Point::new(-8.795, -3.606),
        Point::new(-10.674, -10.385),
        Point::new(-5.401, -20.865),
        Point::new(7.202, -27.906),
        Point::new(50.837, -39.694),
        Point::new(160.405, -46.377),
        Point::new(161.651, 5.508),
        Point::new(91.502, 16.831),
    ]
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let border = polygon_from_points(field_points());

    // ~172 m wide x ~63 m tall irregular 9-point polygon.
    let swath_width = 3.0;
    let headland_count = 2;
    let angle_degrees = 0.0; // rows along x (east-west)

    let mut mt = Maptrax::new();
    mt.set_field(border.clone(), datum)?;
    mt.generate_field(swath_width, angle_degrees, headland_count)?;

    let swath_total = mt.field()?.get_parts()[0].swaths.len();
    println!("field swaths generated: {swath_total}");
    if swath_total == 0 {
        eprintln!(
            "warning: no swaths were generated — the supplied polygon is very thin \
            (y-range < swath_width * 2). Adjust the coordinates or lower swath_width."
        );
    }

    // One machine. Block + ByCount with a fleet of size 1 simply assigns
    // every swath to machine 0.
    let plan = DivisionPlan::uniform(1, DivisionPattern::Block, Balance::ByCount);
    // Snake routing: walk rows in strict lateral order (row 0, 1, 2, ...)
    // with direction flipping. No diagonal cross-field jumps — each turn
    // goes to the neighbouring row.
    let routing = RoutingOptions {
        strategy: RoutingStrategy::Snake,
        local_improvement_passes: 0,
    };
    let turn = TurnPlannerConfig {
        swath_width,
        min_turning_radius: 2.0,
        step_size: 0.2,
        // Reeds-Shepp = can reverse when that gives a shorter path.
        model: TurnPlannerModel::ReedsShepp,
        machine_length: 6.0,
        machine_width: 3.0,
        ..TurnPlannerConfig::default()
    };

    let planned = mt.plan_machines_for_part(
        &MachinePlanningOptions {
            plan,
            part_index: 0,
        },
        routing,
        &turn,
    )?;

    let machine = planned
        .machines
        .first()
        .ok_or("no machine in planned output")?;

    // `tour_polyline` flattens the whole drive path into one contiguous
    // polyline — straight swaths, turner curves, headland arcs, all
    // densely sampled at `step_size`.
    let polyline = tour_polyline(&machine.tour);
    println!(
        "machine 0: assigned={} swaths, tour segments={}, polyline points={}",
        machine.assigned_swaths.len(),
        machine.tour.len(),
        polyline.len(),
    );

    create_dir_all("target")?;
    let out_path = "target/single_machine_path.csv";
    let mut writer = BufWriter::new(File::create(out_path)?);
    writeln!(writer, "x,y")?;
    for point in &polyline {
        writeln!(writer, "{:.6},{:.6}", point.x(), point.y())?;
    }
    writer.flush()?;
    println!("wrote {} points to {out_path}", polyline.len());

    // ----- Rerun visualisation ----------------------------------------
    let rec = rerun_viz::connect("maptrax_single_csv")?;

    rerun_viz::log_polygon(
        &rec,
        "enu/field/border",
        &border,
        Color::from_rgb(180, 100, 100),
    )?;
    rerun_viz::log_polygon_geo(
        &rec,
        "geo/field/border",
        &border,
        datum,
        Color::from_rgb(180, 100, 100),
    )?;

    // Headland rings for context.
    for (index, ring) in mt.field()?.get_parts()[0].headlands.iter().enumerate() {
        rerun_viz::log_polygon(
            &rec,
            &format!("enu/field/headland_{index}"),
            &ring.polygon,
            Color::from_rgb(120, 120, 160),
        )?;
        rerun_viz::log_polygon_geo(
            &rec,
            &format!("geo/field/headland_{index}"),
            &ring.polygon,
            datum,
            Color::from_rgb(120, 120, 160),
        )?;
    }

    let color = (70, 170, 230);

    rerun_viz::log_swaths_tinted(&rec, "enu/swaths", &machine.assigned_swaths, color)?;
    rerun_viz::log_swaths_geo_tinted(
        &rec,
        "geo/swaths",
        &machine.assigned_swaths,
        datum,
        Some(color),
    )?;

    rerun_viz::log_swaths_tinted(&rec, "enu/tour", &machine.tour, color)?;
    rerun_viz::log_swaths_geo_tinted(&rec, "geo/tour", &machine.tour, datum, Some(color))?;

    rerun_viz::log_polylines(
        &rec,
        "enu/headlands_per_machine",
        &machine.assigned_headland_arcs,
        color,
    )?;
    rerun_viz::log_polylines_geo(
        &rec,
        "geo/headlands_per_machine",
        &machine.assigned_headland_arcs,
        datum,
        color,
    )?;

    // The flat polyline — exactly what went into the CSV — logged as one
    // continuous strip so you can scrub through it in Rerun.
    rerun_viz::log_polylines(
        &rec,
        "enu/route_polyline",
        &[polyline.clone()],
        (240, 200, 60),
    )?;
    rerun_viz::log_polylines_geo(
        &rec,
        "geo/route_polyline",
        &[polyline.clone()],
        datum,
        (240, 200, 60),
    )?;

    // Detect Reeds-Shepp reverse sections by analysing ONLY the
    // Connection segments in the tour — swaths and headland rings are
    // always driven forward by construction and must never be flagged
    // as reverse.
    let (forward_runs, reverse_runs) = split_forward_reverse_by_segment(&machine.tour);
    if !forward_runs.is_empty() {
        rerun_viz::log_polylines(&rec, "enu/route/forward", &forward_runs, (80, 200, 120))?;
        rerun_viz::log_polylines_geo(
            &rec,
            "geo/route/forward",
            &forward_runs,
            datum,
            (80, 200, 120),
        )?;
    }
    if !reverse_runs.is_empty() {
        rerun_viz::log_polylines(&rec, "enu/route/reverse", &reverse_runs, (230, 60, 60))?;
        rerun_viz::log_polylines_geo(
            &rec,
            "geo/route/reverse",
            &reverse_runs,
            datum,
            (230, 60, 60),
        )?;
        let reverse_points: usize = reverse_runs.iter().map(|run| run.len()).sum();
        println!(
            "reverse segments: {} runs, {} points total",
            reverse_runs.len(),
            reverse_points
        );
    } else {
        println!("no reverse segments detected in this tour");
    }

    // Mark individual waypoints so you can see how densely the turns are
    // sampled vs the straight swaths.
    let point_xys: Vec<[f32; 2]> = polyline
        .iter()
        .map(|p| [p.x() as f32, p.y() as f32])
        .collect();
    rec.log(
        "enu/waypoints",
        &rerun::Points2D::new(point_xys).with_radii([0.3]),
    )?;

    rec.flush_blocking()?;
    println!("rerun: recording 'maptrax_single_csv' flushed");

    Ok(())
}

/// Classify the driven path into forward and reverse chunks using the
/// GROUND TRUTH reverse flag produced by the Reeds-Shepp planner and
/// carried through the Swath's `point_reverse` field. Swaths, headlands
/// and non-RS connectors have `point_reverse` empty (== all forward);
/// RS connectors have one bool per waypoint telling whether the machine
/// is reversing at that point.
fn split_forward_reverse_by_segment(
    tour: &[maptrax::Swath],
) -> (Vec<Vec<Point<f64>>>, Vec<Vec<Point<f64>>>) {
    use maptrax::SwathType;

    let mut forward_runs: Vec<Vec<Point<f64>>> = Vec::new();
    let mut reverse_runs: Vec<Vec<Point<f64>>> = Vec::new();

    for segment in tour {
        if segment.points.len() < 2 {
            continue;
        }
        let pts = &segment.points;

        // Work segments are always forward by construction.
        let is_work = !matches!(segment.r#type, SwathType::Connection);
        if is_work || segment.point_reverse.is_empty() {
            forward_runs.push(pts.clone());
            continue;
        }

        // Connection with per-waypoint direction info. Split by
        // consecutive "reverse" flag.
        let n = pts.len();
        let rev = &segment.point_reverse;
        let mut run_start = 0usize;
        let mut i = 1usize;
        while i < n {
            // rev[i] may be out of bounds if the length mismatch; guard.
            let current_reverse = *rev.get(i).unwrap_or(&false);
            let prev_reverse = *rev.get(i - 1).unwrap_or(&false);
            if current_reverse != prev_reverse {
                // Direction change between pts[i-1] and pts[i].
                let run: Vec<Point<f64>> = pts[run_start..i].to_vec();
                if run.len() >= 2 {
                    if prev_reverse {
                        reverse_runs.push(run);
                    } else {
                        forward_runs.push(run);
                    }
                }
                run_start = i - 1; // include the transition point in the next run
            }
            i += 1;
        }
        let tail: Vec<Point<f64>> = pts[run_start..].to_vec();
        if tail.len() >= 2 {
            let last_rev = *rev.last().unwrap_or(&false);
            if last_rev {
                reverse_runs.push(tail);
            } else {
                forward_runs.push(tail);
            }
        }
    }

    (forward_runs, reverse_runs)
}
