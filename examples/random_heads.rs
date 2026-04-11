#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use concord::Geo;
use maptrax::{Dubins, Pose2D, ReedsShepp};
use rerun::Color;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_random_heads")?;
    let datum = Geo::new(51.0, 5.0, 0.0);

    let (start, end) = sample_pose_pair();

    rerun_viz::log_pose(&rec, "enu/start", start, Color::from_rgb(0, 255, 0), 0.35)?;
    rerun_viz::log_pose(&rec, "enu/end", end, Color::from_rgb(255, 0, 0), 0.35)?;
    rerun_viz::log_pose_geo(
        &rec,
        "geo/start",
        start,
        datum,
        Color::from_rgb(0, 255, 0),
        0.35,
    )?;
    rerun_viz::log_pose_geo(
        &rec,
        "geo/end",
        end,
        datum,
        Color::from_rgb(255, 0, 0),
        0.35,
    )?;

    let dubins = Dubins::new(4.0);
    let dubins_paths = dubins.get_all_paths(start, end, 0.2);
    for path in &dubins_paths {
        rerun_viz::log_pose_path(
            &rec,
            &format!("enu/dubins/all/{}", path.name),
            &path.waypoints,
            Color::from_rgb(140, 140, 140),
        )?;
        rerun_viz::log_pose_path_geo(
            &rec,
            &format!("geo/dubins/all/{}", path.name),
            &path.waypoints,
            datum,
            Color::from_rgb(140, 140, 140),
        )?;
    }
    let best_dubins = dubins.plan_path(start, end, 0.2);
    rerun_viz::log_pose_path(
        &rec,
        "enu/dubins/best",
        &best_dubins.waypoints,
        Color::from_rgb(255, 255, 0),
    )?;
    rerun_viz::log_pose_path_geo(
        &rec,
        "geo/dubins/best",
        &best_dubins.waypoints,
        datum,
        Color::from_rgb(255, 255, 0),
    )?;

    let rs = ReedsShepp::new(4.0);
    let rs_paths = rs.get_all_paths(start, end, 0.2);
    for path in &rs_paths {
        rerun_viz::log_pose_path(
            &rec,
            &format!("enu/reeds_shepp/all/{}", path.name),
            &path.waypoints,
            Color::from_rgb(120, 120, 120),
        )?;
        rerun_viz::log_pose_path_geo(
            &rec,
            &format!("geo/reeds_shepp/all/{}", path.name),
            &path.waypoints,
            datum,
            Color::from_rgb(120, 120, 120),
        )?;
    }
    let best_rs = rs.plan_path(start, end, 0.2);
    rerun_viz::log_pose_path(
        &rec,
        "enu/reeds_shepp/best",
        &best_rs.waypoints,
        Color::from_rgb(255, 0, 255),
    )?;
    rerun_viz::log_pose_path_geo(
        &rec,
        "geo/reeds_shepp/best",
        &best_rs.waypoints,
        datum,
        Color::from_rgb(255, 0, 255),
    )?;

    rec.flush_blocking()?;
    println!(
        "random_heads: start=({:.3},{:.3},{:.3}) end=({:.3},{:.3},{:.3}) dubins_all={} rs_all={}",
        start.point.x(),
        start.point.y(),
        start.yaw,
        end.point.x(),
        end.point.y(),
        end.yaw,
        dubins_paths.len(),
        rs_paths.len()
    );
    Ok(())
}

fn sample_pose_pair() -> (Pose2D, Pose2D) {
    let seed = std::env::var("MAPTRAX_HEADS_SEED")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .unwrap_or(1337);
    let mut state = seed;
    let mut next_f64 = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let bits = (state >> 11) as f64;
        bits / ((1u64 << 53) as f64)
    };

    let start = Pose2D::new(
        1.0 + next_f64() * 2.0,
        1.0 + next_f64() * 2.0,
        next_f64() * 2.0 * std::f64::consts::PI - std::f64::consts::PI,
    );
    let end = Pose2D::new(
        start.point.x() + next_f64() * 60.0 - 30.0,
        start.point.y() + next_f64() * 60.0 - 30.0,
        next_f64() * 2.0 * std::f64::consts::PI - std::f64::consts::PI,
    );
    (start, end)
}
