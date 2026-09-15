//! Very small visual demo: one square field, one combine-harvester-sized
//! turning model, and visible turn connector points.
//!
//! Run a Rerun viewer (or `rerun --serve`) then:
//!   make run EXAMPLE=combine_square_turning

#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    FieldGenerationMode, FieldGenerationOptions, Geo, Maptrax, PlannerOptions, Point,
    RoutingOptions, RoutingStrategy, Swath, SwathType, TurnPlannerConfig, TurnPlannerModel,
    point_distance, point_xy, polygon_from_points,
};
use rerun::Color;

// Keep it intentionally boring and readable.
const FIELD_SIZE_M: f64 = 120.0;
const SWATH_WIDTH_M: f64 = 6.0;
const HEADLAND_RINGS: usize = 4;

// Generic large combine reference values. These are deliberately conservative,
// not brand-specific: long machine, wide body, forward-only turn model.
const COMBINE_LENGTH_M: f64 = 9.0;
const COMBINE_WIDTH_M: f64 = 4.0;
const COMBINE_MIN_TURN_RADIUS_M: f64 = 8.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_combine_square_turning")?;

    let datum = Geo::new(51.0, 5.0, 0.0);
    let square = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(FIELD_SIZE_M, 0.0),
        point_xy(FIELD_SIZE_M, FIELD_SIZE_M),
        point_xy(0.0, FIELD_SIZE_M),
    ]);

    let combine = TurnPlannerConfig {
        model: TurnPlannerModel::Dubins, // forward-only: do not rely on reversing
        min_turning_radius: COMBINE_MIN_TURN_RADIUS_M,
        machine_length: COMBINE_LENGTH_M,
        machine_width: COMBINE_WIDTH_M,
        swath_width: SWATH_WIDTH_M,
        ..TurnPlannerConfig::default()
    };

    let mut planner = Maptrax::new();
    planner.set_field(square.clone(), datum)?;
    let planned = planner.plan_stages(&PlannerOptions {
        field: FieldGenerationOptions {
            swath_width: SWATH_WIDTH_M,
            headland_count: HEADLAND_RINGS,
            mode: FieldGenerationMode::ExplicitAngle(90.0),
            ..FieldGenerationOptions::default()
        },
        // TurnRadiusAware resolves to a skip-row stride from the combine's turn
        // radius so adjacent turns are not forced when the combine is too long.
        routing: RoutingOptions {
            strategy: RoutingStrategy::TurnRadiusAware,
            local_improvement_passes: 0,
        },
        turn: combine.clone(),
        ..PlannerOptions::default()
    })?;

    let part = &planned.parts[0];
    let turn_points = collect_turn_points(&part.tour);

    rerun_viz::log_polygon(
        &rec,
        "enu/field_square",
        &square,
        Color::from_rgb(180, 90, 80),
    )?;
    for (index, headland) in part.headlands.iter().enumerate() {
        rerun_viz::log_polygon(
            &rec,
            &format!("enu/headlands/ring_{index}"),
            &headland.polygon,
            Color::from_rgb(100, 100, 150),
        )?;
    }
    rerun_viz::log_swaths_tinted(
        &rec,
        "enu/generated_swaths",
        &part.generated_swaths,
        (80, 120, 230),
    )?;
    rerun_viz::log_swaths_tinted(&rec, "enu/final_tour", &part.tour, (230, 120, 40))?;
    rerun_viz::log_polylines(
        &rec,
        "enu/turning_points_yellow_crosses",
        &crosses(&turn_points, 1.2),
        (255, 230, 50),
    )?;

    println!("=== basic square field + combine turning reference ===");
    println!("field: {FIELD_SIZE_M:.0}m x {FIELD_SIZE_M:.0}m");
    println!("swath width: {SWATH_WIDTH_M:.1}m");
    println!("headland rings: {HEADLAND_RINGS}");
    println!(
        "combine: length={COMBINE_LENGTH_M:.1}m width={COMBINE_WIDTH_M:.1}m min_turn_radius={COMBINE_MIN_TURN_RADIUS_M:.1}m"
    );
    println!(
        "turning envelope radius used by planner: {:.2}m",
        combine.turning_envelope_radius()
    );
    println!("generated swaths: {}", part.generated_swaths.len());
    println!("tour segments: {}", part.tour.len());
    println!("turning point markers: {}", turn_points.len());
    println!("Rerun layers:");
    println!("  enu/field_square");
    println!("  enu/headlands/*");
    println!("  enu/generated_swaths");
    println!("  enu/final_tour");
    println!("  enu/turning_points_yellow_crosses");

    rec.flush_blocking()?;
    Ok(())
}

fn collect_turn_points(tour: &[Swath]) -> Vec<Point> {
    let mut points = Vec::new();
    for swath in tour
        .iter()
        .filter(|swath| swath.r#type == SwathType::Connection)
    {
        let pts = if swath.points.len() >= 2 {
            swath.points.as_slice()
        } else {
            continue;
        };

        push_unique(&mut points, pts[0]);
        push_unique(&mut points, pts[pts.len() / 2]);
        push_unique(&mut points, *pts.last().expect("connection has points"));
    }
    points
}

fn push_unique(points: &mut Vec<Point>, point: Point) {
    if points
        .iter()
        .all(|existing| point_distance(*existing, point) > 0.5)
    {
        points.push(point);
    }
}

fn crosses(points: &[Point], radius: f64) -> Vec<Vec<Point>> {
    let mut out = Vec::with_capacity(points.len() * 2);
    for point in points {
        out.push(vec![
            point_xy(point.x - radius, point.y),
            point_xy(point.x + radius, point.y),
        ]);
        out.push(vec![
            point_xy(point.x, point.y - radius),
            point_xy(point.x, point.y + radius),
        ]);
    }
    out
}
