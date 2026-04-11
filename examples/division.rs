use concord::Geo;
use geo::Point;
use maptrax::{DivisionType, Divy, Field, polygon_from_points};

fn main() {
    let polygon = polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ]);

    let mut field = Field::new(polygon, Geo::new(51.0, 5.0, 0.0)).expect("field");
    field.gen_field(10.0, 90.0, 0).expect("generated");

    let mut divy = Divy::from_field(&field, DivisionType::Alternate, 2).expect("divy");
    divy.compute_division();

    for (machine, swaths) in divy.result().swaths_per_machine.iter().enumerate() {
        println!("machine {machine}: {} swaths", swaths.len());
    }
}
