#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use concord::Geo;
use maptrax::{Dubins, Pose2D, ReedsShepp};
use rerun::Color;
use std::f64::consts::PI;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_heads")?;
    let datum = Geo::new(51.0, 5.0, 0.0);

    let start = Pose2D::new(0.0, 0.0, 0.0);
    let end = Pose2D::new(0.0, 18.0, PI);

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
        "heads: dubins_all={}, dubins_best={}, rs_all={}, rs_best={}",
        dubins_paths.len(),
        best_dubins.name,
        rs_paths.len(),
        best_rs.name
    );
    Ok(())
}
