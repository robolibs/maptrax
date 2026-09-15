use maptrax::{Field, Geo, Nety, point_xy, polygon_from_points};

fn main() {
    let polygon = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(100.0, 0.0),
        point_xy(100.0, 50.0),
        point_xy(0.0, 50.0),
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
