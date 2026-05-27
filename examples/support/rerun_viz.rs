#![allow(dead_code)]

use std::error::Error;

use concord::{Geo, Wgs, to_wgs_from_enu};
use geo::{LineString, Point, Polygon};
use maptrax::{Pose2D, Swath, SwathType};
use rerun::{Color, GeoLineStrings, LineStrips2D, RecordingStream, RecordingStreamBuilder};

pub fn connect(app_id: &str) -> Result<RecordingStream, Box<dyn Error>> {
    let url = std::env::var("RERUN_URL")
        .unwrap_or_else(|_| "rerun+http://0.0.0.0:9876/proxy".to_string());
    let rec = RecordingStreamBuilder::new(app_id).connect_grpc_opts(url)?;
    Ok(rec)
}

pub fn log_polygon(
    rec: &RecordingStream,
    path: &str,
    polygon: &Polygon<f64>,
    color: Color,
) -> Result<(), Box<dyn Error>> {
    let strip = close_points(line_string_points(polygon.exterior()));
    rec.log(path, &LineStrips2D::new([strip]).with_colors([color]))?;
    Ok(())
}

pub fn log_swaths(
    rec: &RecordingStream,
    path: &str,
    swaths: &[Swath],
) -> Result<(), Box<dyn Error>> {
    log_swaths_with_palette(rec, path, swaths, None)
}

pub fn log_swaths_tinted(
    rec: &RecordingStream,
    path: &str,
    swaths: &[Swath],
    base_color: (u8, u8, u8),
) -> Result<(), Box<dyn Error>> {
    log_swaths_with_palette(rec, path, swaths, Some(base_color))
}

/// Log a list of open polylines (e.g. headland arcs) under one path with a
/// single colour.
pub fn log_polylines(
    rec: &RecordingStream,
    path: &str,
    polylines: &[Vec<Point<f64>>],
    color: (u8, u8, u8),
) -> Result<(), Box<dyn Error>> {
    let strips: Vec<Vec<[f32; 2]>> = polylines
        .iter()
        .filter(|arc| arc.len() >= 2)
        .map(|arc| arc.iter().copied().map(point2).collect())
        .collect();
    if strips.is_empty() {
        return Ok(());
    }
    let colors: Vec<Color> = strips
        .iter()
        .map(|_| Color::from_rgb(color.0, color.1, color.2))
        .collect();
    rec.log(path, &LineStrips2D::new(strips).with_colors(colors))?;
    Ok(())
}

/// Geo-coded counterpart to `log_polylines`. Each local-ENU polyline is
/// converted to lat/lon via the datum and logged as `GeoLineStrings`.
pub fn log_polylines_geo(
    rec: &RecordingStream,
    path: &str,
    polylines: &[Vec<Point<f64>>],
    datum: Geo,
    color: (u8, u8, u8),
) -> Result<(), Box<dyn Error>> {
    let strips: Vec<Vec<[f64; 2]>> = polylines
        .iter()
        .filter(|arc| arc.len() >= 2)
        .map(|arc| arc.iter().map(|point| point_geo(*point, datum)).collect())
        .collect();
    if strips.is_empty() {
        return Ok(());
    }
    let colors: Vec<Color> = strips
        .iter()
        .map(|_| Color::from_rgb(color.0, color.1, color.2))
        .collect();
    rec.log(
        path,
        &GeoLineStrings::from_lat_lon(strips).with_colors(colors),
    )?;
    Ok(())
}

fn log_swaths_with_palette(
    rec: &RecordingStream,
    path: &str,
    swaths: &[Swath],
    base_color: Option<(u8, u8, u8)>,
) -> Result<(), Box<dyn Error>> {
    let mut strips = Vec::new();
    let mut colors = Vec::new();

    for swath in swaths {
        let strip = if swath.points.len() >= 2 {
            swath.points.iter().copied().map(point2).collect::<Vec<_>>()
        } else {
            vec![point2(swath.head()), point2(swath.tail())]
        };
        if strip.len() < 2 {
            continue;
        }
        strips.push(strip);
        colors.push(swath_color(swath.r#type, base_color));
    }

    if !strips.is_empty() {
        rec.log(path, &LineStrips2D::new(strips).with_colors(colors))?;
    }

    Ok(())
}

pub fn log_polygon_geo(
    rec: &RecordingStream,
    path: &str,
    polygon: &Polygon<f64>,
    datum: Geo,
    color: Color,
) -> Result<(), Box<dyn Error>> {
    let strip = close_geo_points(line_string_geo_points(polygon.exterior(), datum));
    rec.log(
        path,
        &GeoLineStrings::from_lat_lon([strip]).with_colors([color]),
    )?;
    Ok(())
}

pub fn log_swaths_geo(
    rec: &RecordingStream,
    path: &str,
    swaths: &[Swath],
    datum: Geo,
) -> Result<(), Box<dyn Error>> {
    log_swaths_geo_tinted(rec, path, swaths, datum, None)
}

pub fn log_swaths_geo_tinted(
    rec: &RecordingStream,
    path: &str,
    swaths: &[Swath],
    datum: Geo,
    base_color: Option<(u8, u8, u8)>,
) -> Result<(), Box<dyn Error>> {
    let mut strips = Vec::new();
    let mut colors = Vec::new();

    for swath in swaths {
        let strip = if swath.points.len() >= 2 {
            swath
                .points
                .iter()
                .copied()
                .map(|point| point_geo(point, datum))
                .collect::<Vec<_>>()
        } else {
            vec![
                point_geo(swath.head(), datum),
                point_geo(swath.tail(), datum),
            ]
        };
        if strip.len() < 2 {
            continue;
        }
        strips.push(strip);
        colors.push(swath_color(swath.r#type, base_color));
    }

    if !strips.is_empty() {
        rec.log(
            path,
            &GeoLineStrings::from_lat_lon(strips).with_colors(colors),
        )?;
    }

    Ok(())
}

fn line_string_points(line: &LineString<f64>) -> Vec<[f32; 2]> {
    line.points().map(point2).collect()
}

fn line_string_geo_points(line: &LineString<f64>, datum: Geo) -> Vec<[f64; 2]> {
    line.points().map(|point| point_geo(point, datum)).collect()
}

fn close_points(mut points: Vec<[f32; 2]>) -> Vec<[f32; 2]> {
    if let Some(first) = points.first().copied() {
        if points.last().copied() != Some(first) {
            points.push(first);
        }
    }
    points
}

fn close_geo_points(mut points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    if let Some(first) = points.first().copied() {
        if points.last().copied() != Some(first) {
            points.push(first);
        }
    }
    points
}

fn point2(point: Point<f64>) -> [f32; 2] {
    [point.x() as f32, point.y() as f32]
}

pub fn log_pose(
    rec: &RecordingStream,
    path: &str,
    pose: Pose2D,
    color: Color,
    axis_length: f64,
) -> Result<(), Box<dyn Error>> {
    let head = pose.point;
    let tip = Point::new(
        head.x() + axis_length * pose.yaw.cos(),
        head.y() + axis_length * pose.yaw.sin(),
    );
    let left = Point::new(
        tip.x() - axis_length * 0.25 * (pose.yaw - 2.6).cos(),
        tip.y() - axis_length * 0.25 * (pose.yaw - 2.6).sin(),
    );
    let right = Point::new(
        tip.x() - axis_length * 0.25 * (pose.yaw + 2.6).cos(),
        tip.y() - axis_length * 0.25 * (pose.yaw + 2.6).sin(),
    );
    let strips = vec![
        vec![point2(head), point2(tip)],
        vec![point2(tip), point2(left)],
        vec![point2(tip), point2(right)],
    ];
    rec.log(
        path,
        &LineStrips2D::new(strips).with_colors([color, color, color]),
    )?;
    Ok(())
}

pub fn log_pose_geo(
    rec: &RecordingStream,
    path: &str,
    pose: Pose2D,
    datum: Geo,
    color: Color,
    axis_length: f64,
) -> Result<(), Box<dyn Error>> {
    let head = pose.point;
    let tip = Point::new(
        head.x() + axis_length * pose.yaw.cos(),
        head.y() + axis_length * pose.yaw.sin(),
    );
    let left = Point::new(
        tip.x() - axis_length * 0.25 * (pose.yaw - 2.6).cos(),
        tip.y() - axis_length * 0.25 * (pose.yaw - 2.6).sin(),
    );
    let right = Point::new(
        tip.x() - axis_length * 0.25 * (pose.yaw + 2.6).cos(),
        tip.y() - axis_length * 0.25 * (pose.yaw + 2.6).sin(),
    );
    let strips = vec![
        vec![point_geo(head, datum), point_geo(tip, datum)],
        vec![point_geo(tip, datum), point_geo(left, datum)],
        vec![point_geo(tip, datum), point_geo(right, datum)],
    ];
    rec.log(
        path,
        &GeoLineStrings::from_lat_lon(strips).with_colors([color, color, color]),
    )?;
    Ok(())
}

pub fn log_pose_path(
    rec: &RecordingStream,
    path: &str,
    poses: &[Pose2D],
    color: Color,
) -> Result<(), Box<dyn Error>> {
    let strip = poses
        .iter()
        .map(|pose| point2(pose.point))
        .collect::<Vec<_>>();
    if strip.len() >= 2 {
        rec.log(path, &LineStrips2D::new([strip]).with_colors([color]))?;
    }
    Ok(())
}

pub fn log_pose_path_geo(
    rec: &RecordingStream,
    path: &str,
    poses: &[Pose2D],
    datum: Geo,
    color: Color,
) -> Result<(), Box<dyn Error>> {
    let strip = poses
        .iter()
        .map(|pose| point_geo(pose.point, datum))
        .collect::<Vec<_>>();
    if strip.len() >= 2 {
        rec.log(
            path,
            &GeoLineStrings::from_lat_lon([strip]).with_colors([color]),
        )?;
    }
    Ok(())
}

fn point_geo(point: Point<f64>, datum: Geo) -> [f64; 2] {
    let wgs: Wgs = to_wgs_from_enu(concord::Enu::new(point.x(), point.y(), 0.0, datum));
    [wgs.latitude, wgs.longitude]
}

pub fn machine_color(index: usize) -> (u8, u8, u8) {
    const PALETTE: [(u8, u8, u8); 6] = [
        (230, 57, 70),
        (29, 145, 192),
        (67, 170, 139),
        (244, 162, 97),
        (106, 76, 147),
        (233, 196, 106),
    ];
    PALETTE[index % PALETTE.len()]
}

fn swath_color(swath_type: SwathType, base_color: Option<(u8, u8, u8)>) -> Color {
    if let Some(color) = base_color {
        return match swath_type {
            SwathType::Swath => rgb(color),
            SwathType::Connection => scale_color(color, 0.65),
            SwathType::Around => Color::from_rgb(220, 60, 60),
            SwathType::Headland => scale_color(color, 0.5),
        };
    }

    match swath_type {
        SwathType::Swath => Color::from_rgb(70, 70, 210),
        SwathType::Connection => Color::from_rgb(230, 160, 30),
        SwathType::Around => Color::from_rgb(220, 60, 60),
        SwathType::Headland => Color::from_rgb(70, 150, 70),
    }
}

fn rgb(color: (u8, u8, u8)) -> Color {
    Color::from_rgb(color.0, color.1, color.2)
}

fn scale_color(color: (u8, u8, u8), factor: f32) -> Color {
    Color::from_rgb(
        (color.0 as f32 * factor).round().clamp(0.0, 255.0) as u8,
        (color.1 as f32 * factor).round().clamp(0.0, 255.0) as u8,
        (color.2 as f32 * factor).round().clamp(0.0, 255.0) as u8,
    )
}
