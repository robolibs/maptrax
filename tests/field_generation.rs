use concord::{Geo, Wgs, to_enu};
use geo::{Point, Polygon};
use maptrax::{Field, polygon_area, polygon_from_points, segment_length};

fn test_polygon() -> Polygon {
    polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ])
}

#[test]
fn field_constructor_and_area_work() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let field = Field::new(test_polygon(), datum).expect("field");

    assert_eq!(field.get_parts().len(), 1);
    assert_eq!(field.datum(), datum);
    assert!((field.total_area() - 5_000.0).abs() < 1e-9);
}

#[test]
fn field_generation_creates_swaths_for_rectangles() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(test_polygon(), datum).expect("field");
    field.gen_field(10.0, 90.0, 0).expect("generated");

    let part = &field.get_parts()[0];
    assert!(part.headlands.is_empty());
    assert_eq!(part.swaths.len(), 10);
    assert!(part.swaths.iter().all(|swath| swath.width == 10.0));
    assert!(
        part.swaths
            .iter()
            .all(|swath| (swath.line.start.x - swath.line.end.x).abs() < 1e-9)
    );
}

#[test]
fn field_generation_supports_non_orthogonal_angles() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(test_polygon(), datum).expect("field");
    field.gen_field(10.0, 45.0, 0).expect("generated");

    let part = &field.get_parts()[0];
    assert!(!part.swaths.is_empty());
    assert!(part.swaths.iter().all(|swath| segment_length(swath.line) > 0.0));
}

#[test]
fn headlands_are_generated_for_rectangular_fields() {
    let datum = Geo::new(51.0, 5.0, 0.0);
    let mut field = Field::new(test_polygon(), datum).expect("field");
    field.gen_field(10.0, 90.0, 1).expect("generated");

    let part = &field.get_parts()[0];
    assert_eq!(part.headlands.len(), 1);
    assert!(polygon_area(&part.headlands[0].polygon).abs() < field.total_area());
    assert!(!part.swaths.is_empty());
}

#[test]
fn headlands_are_generated_for_irregular_upstream_shape() {
    let datum = Geo::new(51.98954034749562, 5.6584737410504715, 53.801823);
    let polygon = polygon_from_points(
        [
            Wgs::new(51.98765392402663, 5.660072928621929, 0.0),
            Wgs::new(51.98816428304869, 5.661754957062072, 0.0),
            Wgs::new(51.989850316694316, 5.660416700858434, 0.0),
            Wgs::new(51.990417354104295, 5.662166255987472, 0.0),
            Wgs::new(51.991078888673854, 5.660969191951295, 0.0),
            Wgs::new(51.989479848375254, 5.656874619070777, 0.0),
            Wgs::new(51.988156722216644, 5.657715633290422, 0.0),
            Wgs::new(51.98765392402663, 5.660072928621929, 0.0),
        ]
        .into_iter()
        .map(|wgs| {
            let enu = to_enu(datum, wgs);
            Point::new(enu.east(), enu.north())
        })
        .collect(),
    );

    let mut field = Field::new(polygon, datum).expect("field");
    field.gen_field(4.0, 0.0, 3).expect("generated");

    let part = &field.get_parts()[0];
    assert_eq!(part.headlands.len(), 3);
    assert!(part.swaths.len() > 50);

    let areas: Vec<f64> = part
        .headlands
        .iter()
        .map(|ring| polygon_area(&ring.polygon).abs())
        .collect();
    assert!(areas.windows(2).all(|pair| pair[1] < pair[0]));
}
