use concord::Geo;
use geo::Point;
use maptrax::{ObstacleAvoider, SwathType, create_swath, polygon_from_points};

fn main() {
    let obstacle = polygon_from_points(vec![
        Point::new(45.0, 20.0),
        Point::new(55.0, 20.0),
        Point::new(55.0, 30.0),
        Point::new(45.0, 30.0),
    ]);

    let input = vec![create_swath(
            Point::new(50.0, 0.0),
            Point::new(50.0, 50.0),
        SwathType::Swath,
        "row_0",
    )];

    let mut avoider = ObstacleAvoider::new(vec![obstacle], Geo::new(51.0, 5.0, 0.0));
    let avoided = avoider.avoid(&input, 2.0);

    println!("avoidance output: {} segments", avoided.len());
}
