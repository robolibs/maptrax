//! Larger four-corner irregular field with the same combine turn reference.
//!
//! Run:
//!   make run EXAMPLE=combine_irregular_turning

#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use maptrax::{
    FieldGenerationMode, FieldGenerationOptions, Geo, Maptrax, PlannerOptions, Point,
    RoutingOptions, RoutingStrategy, Swath, SwathType, TurnPlannerConfig, TurnPlannerModel,
    point_distance, point_xy, polygon_from_points,
};
use rerun::Color;

const SWATH_WIDTH_M: f64 = 9.0;
const HEADLAND_RINGS: usize = 4;
const SWATH_ANGLE_DEG: f64 = 86.0;

const COMBINE_LENGTH_M: f64 = 9.0;
const COMBINE_WIDTH_M: f64 = 4.0;
const COMBINE_MIN_TURN_RADIUS_M: f64 = 8.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_combine_irregular_turning")?;

    let datum = Geo::new(51.0, 5.0, 0.0);
    let field = polygon_from_points(field_points());

    let combine = TurnPlannerConfig {
        // Reeds-Shepp keeps this irregular-field reference continuous. A
        // forward-only Dubins turn is stricter, but on this skewed field some
        // transitions are infeasible inside the safe headland corridor.
        model: TurnPlannerModel::ReedsShepp,
        min_turning_radius: COMBINE_MIN_TURN_RADIUS_M,
        machine_length: COMBINE_LENGTH_M,
        machine_width: COMBINE_WIDTH_M,
        swath_width: SWATH_WIDTH_M,
        ..TurnPlannerConfig::default()
    };

    let mut planner = Maptrax::new();
    planner.set_field(field.clone(), datum)?;
    let planned = planner.plan_stages(&PlannerOptions {
        field: FieldGenerationOptions {
            swath_width: SWATH_WIDTH_M,
            headland_count: HEADLAND_RINGS,
            mode: FieldGenerationMode::ExplicitAngle(SWATH_ANGLE_DEG),
            ..FieldGenerationOptions::default()
        },
        routing: RoutingOptions {
            strategy: RoutingStrategy::TurnRadiusAware,
            local_improvement_passes: 0,
        },
        turn: combine.clone(),
        ..PlannerOptions::default()
    })?;

    let part = &planned.parts[0];
    let turn_points = collect_turn_points(&part.tour);
    let (gap_count, max_gap) = count_tour_gaps(&part.tour);

    rerun_viz::log_polygon(
        &rec,
        "enu/field_irregular_four_point",
        &field,
        Color::from_rgb(180, 90, 80),
    )?;
    for (index, headland) in part.headlands.iter().enumerate() {
        rerun_viz::log_polygon(
            &rec,
            &format!("enu/headlands_irregular/ring_{index}"),
            &headland.polygon,
            Color::from_rgb(100, 100, 150),
        )?;
    }
    rerun_viz::log_swaths_tinted(
        &rec,
        "enu/generated_swaths_irregular",
        &part.generated_swaths,
        (80, 120, 230),
    )?;
    rerun_viz::log_swaths_tinted(&rec, "enu/final_tour_irregular", &part.tour, (230, 120, 40))?;
    rerun_viz::log_polylines(
        &rec,
        "enu/turning_points_yellow_crosses_irregular",
        &crosses(&turn_points, 2.0),
        (255, 230, 50),
    )?;

    println!("=== irregular 4-corner field + combine turning reference ===");
    println!("field corners:");
    for point in field_points() {
        println!("  ({:.1}, {:.1})", point.x, point.y);
    }
    println!("field scale: ~5.2x square demo area");
    println!("swath width: {SWATH_WIDTH_M:.1}m");
    println!("swath angle: {SWATH_ANGLE_DEG:.1}deg");
    println!("headland rings: {HEADLAND_RINGS}");
    println!("turn model: Reeds-Shepp");
    println!(
        "combine: length={COMBINE_LENGTH_M:.1}m width={COMBINE_WIDTH_M:.1}m min_turn_radius={COMBINE_MIN_TURN_RADIUS_M:.1}m"
    );
    println!(
        "turning envelope radius used by planner: {:.2}m",
        combine.turning_envelope_radius()
    );
    println!(
        "turn-radius-aware routing stride: {}",
        combine.required_row_skip_stride(SWATH_WIDTH_M)
    );
    println!("generated swaths: {}", part.generated_swaths.len());
    println!("tour segments: {}", part.tour.len());
    println!("tour continuity gaps: {gap_count} (max {max_gap:.3}m)");
    println!("turning point markers: {}", turn_points.len());
    println!("Rerun layers:");
    println!("  enu/field_irregular_four_point");
    println!("  enu/headlands_irregular/*");
    println!("  enu/generated_swaths_irregular");
    println!("  enu/final_tour_irregular");
    println!("  enu/turning_points_yellow_crosses_irregular");

    rec.flush_blocking()?;
    Ok(())
}

fn field_points() -> Vec<Point> {
    vec![
        point_xy(0.0, 0.0),
        point_xy(310.0, 25.0),
        point_xy(285.0, 265.0),
        point_xy(-35.0, 230.0),
    ]
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

fn count_tour_gaps(tour: &[Swath]) -> (usize, f64) {
    let mut gaps = 0;
    let mut max_gap = 0.0_f64;
    for pair in tour.windows(2) {
        let gap = point_distance(swath_tail(&pair[0]), swath_head(&pair[1]));
        max_gap = max_gap.max(gap);
        if gap > 1e-6 {
            gaps += 1;
        }
    }
    (gaps, max_gap)
}

fn swath_head(swath: &Swath) -> Point {
    swath
        .points
        .first()
        .copied()
        .unwrap_or_else(|| swath.head())
}

fn swath_tail(swath: &Swath) -> Point {
    swath.points.last().copied().unwrap_or_else(|| swath.tail())
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
