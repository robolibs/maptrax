#![allow(dead_code)]

use concord::{Wgs, to_enu};
use maptrax::{Geo, Point, Polygon, Swath, SwathType, point_xy, polygon_from_points};

pub fn upstream_datum() -> Geo {
    Geo::new(51.98954034749562, 5.6584737410504715, 53.801823)
}

pub fn upstream_field_polygon(datum: Geo) -> Polygon {
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
            point_xy(enu.east(), enu.north())
        })
        .collect();
    polygon_from_points(points)
}

pub fn concave_demo_polygon() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(140.0, 0.0),
        point_xy(140.0, 34.0),
        point_xy(82.0, 34.0),
        point_xy(82.0, 102.0),
        point_xy(0.0, 102.0),
    ])
}

pub struct TourConnectionLayers {
    pub work: Vec<Swath>,
    pub row_to_headland: Vec<Swath>,
    pub headland_travel: Vec<Swath>,
    pub headland_to_row: Vec<Swath>,
    pub direct: Vec<Swath>,
}

pub fn split_tour_layers(tour: &[Swath]) -> TourConnectionLayers {
    let mut layers = TourConnectionLayers {
        work: Vec::new(),
        row_to_headland: Vec::new(),
        headland_travel: Vec::new(),
        headland_to_row: Vec::new(),
        direct: Vec::new(),
    };

    for (index, swath) in tour.iter().enumerate() {
        match swath.r#type {
            SwathType::Swath => layers.work.push(swath.clone()),
            SwathType::Connection => {
                let prev = index.checked_sub(1).and_then(|i| tour.get(i));
                let next = tour.get(index + 1);
                let prev_is_work = prev.is_some_and(|item| item.r#type == SwathType::Swath);
                let next_is_work = next.is_some_and(|item| item.r#type == SwathType::Swath);
                let prev_is_connection =
                    prev.is_some_and(|item| item.r#type == SwathType::Connection);
                let next_is_connection =
                    next.is_some_and(|item| item.r#type == SwathType::Connection);

                if prev_is_work && next_is_connection {
                    layers.row_to_headland.push(swath.clone());
                } else if prev_is_connection && next_is_work {
                    layers.headland_to_row.push(swath.clone());
                } else if prev_is_connection || next_is_connection {
                    layers.headland_travel.push(swath.clone());
                } else {
                    layers.direct.push(swath.clone());
                }
            }
            SwathType::Around | SwathType::Headland => {}
        }
    }

    layers
}
