//! Viewer output for this example. Everything goes through `log_static`, so
//! each entity holds only its latest state and the recording stays small no
//! matter how long the simulation runs.

use std::error::Error;

use concord::to_wgs_from_enu;
use maptrax::{Geo, Point, Point2Ext, Polygon, polygon_exterior_points};
use rerun::components::Radius;
use rerun::{
    AsComponents, Color, GeoLineStrings, LineStrips3D, RecordingStream, RecordingStreamBuilder,
};

pub fn connect(app_id: &str) -> Result<RecordingStream, Box<dyn Error>> {
    let url = std::env::var("RERUN_URL")
        .unwrap_or_else(|_| "rerun+http://0.0.0.0:9876/proxy".to_string());
    Ok(RecordingStreamBuilder::new(app_id).connect_grpc_opts(url)?)
}

pub fn log<A: AsComponents>(
    rec: &RecordingStream,
    path: &str,
    archetype: &A,
) -> Result<(), Box<dyn Error>> {
    rec.log_static(path, archetype)?;
    Ok(())
}

pub fn polygon_3d(
    rec: &RecordingStream,
    path: &str,
    polygon: &Polygon,
    colour: (u8, u8, u8),
    radius: f32,
) -> Result<(), Box<dyn Error>> {
    let mut ring = polygon_exterior_points(polygon);
    if let Some(first) = ring.first().copied() {
        ring.push(first);
    }
    polylines_3d(rec, path, &[ring], colour, radius)
}

pub fn polygon_geo(
    rec: &RecordingStream,
    path: &str,
    polygon: &Polygon,
    datum: Geo,
    colour: (u8, u8, u8),
) -> Result<(), Box<dyn Error>> {
    let mut ring = polygon_exterior_points(polygon);
    if let Some(first) = ring.first().copied() {
        ring.push(first);
    }
    polylines_geo(rec, path, &[ring], datum, colour, 1.5)
}

pub fn polylines_3d(
    rec: &RecordingStream,
    path: &str,
    polylines: &[Vec<Point>],
    colour: (u8, u8, u8),
    radius: f32,
) -> Result<(), Box<dyn Error>> {
    let strips: Vec<Vec<[f32; 3]>> = polylines
        .iter()
        .filter(|line| line.len() >= 2)
        .map(|line| {
            line.iter()
                .map(|p| [p.x() as f32, p.y() as f32, 0.0])
                .collect()
        })
        .collect();
    let count = strips.len();
    log(
        rec,
        path,
        &LineStrips3D::new(strips)
            .with_colors(vec![Color::from_rgb(colour.0, colour.1, colour.2); count])
            .with_radii(vec![radius; count]),
    )
}

/// `ui_radius` is in screen points, so rows stay hairlines at any zoom.
pub fn polylines_geo(
    rec: &RecordingStream,
    path: &str,
    polylines: &[Vec<Point>],
    datum: Geo,
    colour: (u8, u8, u8),
    ui_radius: f32,
) -> Result<(), Box<dyn Error>> {
    let strips: Vec<Vec<[f64; 2]>> = polylines
        .iter()
        .filter(|line| line.len() >= 2)
        .map(|line| line.iter().map(|p| lat_lon(*p, datum)).collect())
        .collect();
    let count = strips.len();
    log(
        rec,
        path,
        &GeoLineStrings::from_lat_lon(strips)
            .with_colors(vec![Color::from_rgb(colour.0, colour.1, colour.2); count])
            .with_radii(vec![Radius::new_ui_points(ui_radius); count]),
    )
}

pub fn lat_lon(point: Point, datum: Geo) -> [f64; 2] {
    let wgs = to_wgs_from_enu(concord::Enu::new(point.x(), point.y(), 0.0, datum));
    [wgs.latitude, wgs.longitude]
}
