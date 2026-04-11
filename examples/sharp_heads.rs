#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use concord::Geo;
use maptrax::{Pose2D, Sharper};
use rerun::Color;
use std::f64::consts::PI;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun_viz::connect("maptrax_sharp_heads")?;
    let datum = Geo::new(51.0, 5.0, 0.0);

    let start = Pose2D::new(0.0, 0.0, 0.0);
    let end = Pose2D::new(0.0, 18.0, PI);
    let sharper = Sharper::new(4.0, 12.0, 4.0);

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

    for (pattern, color) in [
        ("three_point", Color::from_rgb(255, 255, 0)),
        ("bulb", Color::from_rgb(255, 120, 0)),
        ("fishtail", Color::from_rgb(0, 200, 255)),
        ("auto", Color::from_rgb(255, 0, 255)),
    ] {
        let path = sharper.plan_sharp_turn(start, end, pattern);
        rerun_viz::log_pose_path(
            &rec,
            &format!("enu/sharper/{pattern}"),
            &path.waypoints,
            color,
        )?;
        rerun_viz::log_pose_path_geo(
            &rec,
            &format!("geo/sharper/{pattern}"),
            &path.waypoints,
            datum,
            color,
        )?;
        println!(
            "sharper {}: pattern={}, waypoints={}, length={:.3}",
            pattern,
            path.pattern_name,
            path.waypoints.len(),
            path.total_length
        );
    }

    rec.flush_blocking()?;
    Ok(())
}
