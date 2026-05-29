use maptrax::{
    Field, Geo, SwathAngleSearchOptions, SwathObjective, point_xy, polygon_from_points,
};

fn main() {
    let polygon = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(100.0, 0.0),
        point_xy(100.0, 50.0),
        point_xy(0.0, 50.0),
    ]);

    let mut field = Field::new(polygon, Geo::new(51.0, 5.0, 0.0)).expect("field");
    field.gen_field(10.0, 90.0, 1).expect("generated");
    let objective = field
        .generate_with_objective(
            10.0,
            SwathObjective::ApproxMinSwathCount,
            SwathAngleSearchOptions::default(),
            1,
        )
        .expect("objective search");

    let part = &field.get_parts()[0];
    println!(
        "generated field: {} headlands, {} swaths, best angle {:.1}",
        part.headlands.len(),
        part.swaths.len(),
        objective[0].angle_degrees
    );
}
