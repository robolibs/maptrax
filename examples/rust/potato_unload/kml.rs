//! KML of the tractor's route only, fed by the same `Vector` the GeoJSON
//! export consumes. Each line the tractor drove is a folder holding two
//! placemarks: `load`, entry to bay, and `unload`, bay back to entry.
//! KML has no local frame, so an ENU vector is converted to WGS84 through
//! its datum.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt::Write as _;
use std::path::Path;

use concord::{Enu, Geo, to_wgs_from_enu};
use maptrax::{Crs, Element, Geometry, Point, Vector};

pub fn write_kml(vector: &Vector, path: impl AsRef<Path>) -> Result<(), Box<dyn Error>> {
    std::fs::write(path, to_kml_string(vector))?;
    Ok(())
}

pub fn to_kml_string(vector: &Vector) -> String {
    let datum = vector.datum();
    let crs = vector.crs();
    let mut out = String::new();
    out.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    out.push_str("<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n");
    let name = vector
        .global_properties()
        .get("name")
        .map(String::as_str)
        .unwrap_or("maptrax");
    let _ = writeln!(out, "<name>{}</name>", escape(name));
    for (leg, colour) in STYLES {
        let _ = writeln!(
            out,
            "<Style id=\"{leg}\"><LineStyle><color>{colour}</color><width>3</width></LineStyle></Style>"
        );
    }

    let mut lines: BTreeMap<usize, Vec<&Element>> = BTreeMap::new();
    for element in vector.elements_by_type("tour") {
        let line = element
            .properties
            .get("line")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        lines.entry(line).or_default().push(element);
    }

    for (line, legs) in lines {
        let _ = writeln!(out, "<Folder>\n<name>Line {line}</name>");
        for leg in ["load", "unload"] {
            if let Some(element) = legs
                .iter()
                .find(|e| e.properties.get("leg").map(String::as_str) == Some(leg))
            {
                placemark(&mut out, leg, element, datum, crs);
            }
        }
        out.push_str("</Folder>\n");
    }
    out.push_str("</Document>\n</kml>\n");
    out
}

/// Colours are KML `aabbggrr`.
const STYLES: [(&str, &str); 2] = [("load", "ffb87f2f"), ("unload", "ff2e3bb8")];

fn placemark(out: &mut String, leg: &str, element: &Element, datum: Geo, crs: Crs) {
    out.push_str("<Placemark>\n");
    let _ = writeln!(out, "<name>{leg}</name>");
    let _ = writeln!(out, "<styleUrl>#{leg}</styleUrl>");
    let mut keys: Vec<&String> = element.properties.keys().collect();
    keys.sort();
    out.push_str("<ExtendedData>\n");
    for key in keys {
        let _ = writeln!(
            out,
            "<Data name=\"{}\"><value>{}</value></Data>",
            escape(key),
            escape(&element.properties[key])
        );
    }
    out.push_str("</ExtendedData>\n");
    let points: Vec<Point> = match &element.geometry {
        Geometry::Path(path) => path.points.clone(),
        Geometry::Segment(segment) => vec![segment.start, segment.end],
        Geometry::Polygon(polygon) => polygon.vertices.clone(),
        Geometry::Point(point) => vec![*point],
    };
    out.push_str("<LineString><tessellate>1</tessellate><coordinates>\n");
    for point in &points {
        let _ = writeln!(out, "{}", coordinate(point, datum, crs));
    }
    out.push_str("</coordinates></LineString>\n</Placemark>\n");
}

/// KML orders a coordinate `lon,lat,alt`.
fn coordinate(point: &Point, datum: Geo, crs: Crs) -> String {
    match crs {
        Crs::Enu => {
            let wgs = to_wgs_from_enu(Enu::new(point.x, point.y, point.z, datum));
            format!(
                "{:.8},{:.8},{:.2}",
                wgs.longitude, wgs.latitude, wgs.altitude
            )
        }
        Crs::Wgs => format!("{:.8},{:.8},{:.2}", point.y, point.x, point.z),
    }
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}
