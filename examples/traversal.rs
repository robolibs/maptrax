use concord::Geo;
use geo::Point;
use maptrax::{Field, Nety, polygon_from_points};

fn main() {
    let polygon = polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ]);

    let mut field = Field::new(polygon, Geo::new(51.0, 5.0, 0.0)).expect("field");
    field.gen_field(10.0, 90.0, 0).expect("generated");

    let mut nety = Nety::new(&field.get_parts()[0].swaths);
    nety.field_traversal(None);

    println!(
        "traversal graph: {} swaths, {} vertices, {} edges",
        nety.get_swaths().len(),
        nety.num_vertices(),
        nety.num_edges()
    );
}
