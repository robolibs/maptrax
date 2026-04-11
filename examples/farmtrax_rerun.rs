#[path = "support/rerun_viz.rs"]
mod rerun_viz;

use concord::{Geo, Wgs, to_enu};
use geo::Point;
use maptrax::{DivisionType, Divy, Field, Nety, ObstacleAvoider, polygon_from_points};
use rerun::Color;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let app_id = "maptrax_farmtrax_scene";
    let rec = rerun_viz::connect(app_id)?;
    let datum = Geo::new(51.98954034749562, 5.6584737410504715, 53.801823);
    let border = upstream_field_polygon(datum);

    let mut field = Field::new(border.clone(), datum)?;
    field.gen_field(4.0, 0.0, 3)?;
    let part = &field.get_parts()[0];

    let obstacle = centered_obstacle(&border, 25.0);
    let mut avoider = ObstacleAvoider::new(vec![obstacle.clone()], datum);
    let avoided = avoider.avoid(&part.swaths, 2.0);

    rerun_viz::log_polygon(&rec, "enu/field/border", &border, Color::from_rgb(120, 70, 70))?;
    rerun_viz::log_polygon_geo(
        &rec,
        "geo/field/border",
        &border,
        datum,
        Color::from_rgb(120, 70, 70),
    )?;

    for (index, headland) in part.headlands.iter().enumerate() {
        rerun_viz::log_polygon(
            &rec,
            &format!("enu/field/headland/{index}"),
            &headland.polygon,
            Color::from_rgb(70, 120, 70),
        )?;
        rerun_viz::log_polygon_geo(
            &rec,
            &format!("geo/field/headland/{index}"),
            &headland.polygon,
            datum,
            Color::from_rgb(70, 120, 70),
        )?;
    }

    rerun_viz::log_swaths(&rec, "enu/field/swaths", &part.swaths)?;
    rerun_viz::log_swaths_geo(&rec, "geo/field/swaths", &part.swaths, datum)?;
    rerun_viz::log_polygon(
        &rec,
        "enu/avoidance/obstacle",
        &obstacle,
        Color::from_rgb(220, 20, 20),
    )?;
    rerun_viz::log_polygon_geo(
        &rec,
        "geo/avoidance/obstacle",
        &obstacle,
        datum,
        Color::from_rgb(220, 20, 20),
    )?;
    rerun_viz::log_swaths(&rec, "enu/avoidance/avoided", &avoided)?;
    rerun_viz::log_swaths_geo(&rec, "geo/avoidance/avoided", &avoided, datum)?;

    let mut divy = Divy::from_field(&field, DivisionType::Alternate, 2)?;
    divy.compute_division();
    divy.set_machine_count(4)?;
    divy.compute_division();

    let mut machine_summaries = Vec::new();
    for (machine, swaths) in divy.result().swaths_per_machine.iter().enumerate() {
        let machine_color = rerun_viz::machine_color(machine);
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/division/machine_{machine}"),
            swaths,
            machine_color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/division/machine_{machine}"),
            swaths,
            datum,
            Some(machine_color),
        )?;
        if swaths.is_empty() {
            machine_summaries.push((machine, 0_usize, 0_usize));
            continue;
        }

        let mut nety = Nety::new(&avoided);
        nety.field_traversal(None);
        rerun_viz::log_swaths_tinted(
            &rec,
            &format!("enu/main/machine_{machine}"),
            nety.get_swaths(),
            machine_color,
        )?;
        rerun_viz::log_swaths_geo_tinted(
            &rec,
            &format!("geo/main/machine_{machine}"),
            nety.get_swaths(),
            datum,
            Some(machine_color),
        )?;
        machine_summaries.push((machine, swaths.len(), nety.get_swaths().len()));
    }

    rec.flush_blocking()?;
    println!("Field area: {:.1} m^2", field.total_area());
    println!(
        "Part 0: {} headlands, {} swaths, {} avoided segments",
        part.headlands.len(),
        part.swaths.len(),
        avoided.len(),
    );
    for (machine, assigned, nety_count) in machine_summaries {
        println!("Machine {machine}: assigned={assigned}, nety={nety_count}");
    }
    println!(
        "Connected to Rerun at {}",
        std::env::var("RERUN_URL")
            .unwrap_or_else(|_| "rerun+http://0.0.0.0:9876/proxy".to_string())
    );
    Ok(())
}

fn upstream_field_polygon(datum: Geo) -> geo::Polygon<f64> {
    let coords = [
        Wgs::new(51.98765392402663, 5.660072928621929, 0.0),
        Wgs::new(51.98816428304869, 5.661754957062072, 0.0),
        Wgs::new(51.989850316694316, 5.660416700858434, 0.0),
        Wgs::new(51.990417354104295, 5.662166255987472, 0.0),
        Wgs::new(51.991078888673854, 5.660969191951295, 0.0),
        Wgs::new(51.989479848375254, 5.656874619070777, 0.0),
        Wgs::new(51.988156722216644, 5.657715633290422, 0.0),
        Wgs::new(51.98765392402663, 5.660072928621929, 0.0),
    ];

    let points = coords
        .into_iter()
        .map(|wgs| {
            let enu = to_enu(datum, wgs);
            Point::new(enu.east(), enu.north())
        })
        .collect();
    polygon_from_points(points)
}

fn centered_obstacle(border: &geo::Polygon<f64>, half_size: f64) -> geo::Polygon<f64> {
    let vertices = border.exterior().points().collect::<Vec<_>>();
    let (min_x, max_x) = vertices.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |acc, p| {
        (acc.0.min(p.x()), acc.1.max(p.x()))
    });
    let (min_y, max_y) = vertices.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |acc, p| {
        (acc.0.min(p.y()), acc.1.max(p.y()))
    });

    let center_x = (min_x + max_x) * 0.5;
    let center_y = (min_y + max_y) * 0.5;
    polygon_from_points(vec![
        Point::new(center_x - half_size, center_y - half_size),
        Point::new(center_x + half_size, center_y - half_size),
        Point::new(center_x + half_size, center_y + half_size),
        Point::new(center_x - half_size, center_y + half_size),
    ])
}
