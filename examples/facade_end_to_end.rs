use concord::Geo;
use geo::Point;
use maptrax::{Maptrax, TurnPlannerConfig, polygon_from_points};

fn main() {
    let polygon = polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ]);

    let mut maptrax = Maptrax::new();
    maptrax
        .set_field(polygon, Geo::new(51.0, 5.0, 0.0))
        .expect("field");
    maptrax.generate_field(10.0, 90.0, 1).expect("generated");

    let nety = maptrax.make_nety_from_part(0).expect("nety");
    let tour = maptrax
        .build_tour(0, nety.get_swaths(), &TurnPlannerConfig::default())
        .expect("tour");

    println!(
        "facade flow: {} swaths -> {} tour segments",
        nety.get_swaths().len(),
        tour.len()
    );
}
