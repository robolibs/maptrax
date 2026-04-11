use concord::Geo;
use geo::{Point, Polygon};
use maptrax::{ObstacleAvoider, Swath, SwathType, create_swath, polygon_from_points};

fn obstacle() -> Polygon {
    polygon_from_points(vec![
        Point::new(45.0, 20.0),
        Point::new(55.0, 20.0),
        Point::new(55.0, 30.0),
        Point::new(45.0, 30.0),
    ])
}

fn crossing_swaths(xs: &[f64]) -> Vec<Swath> {
    xs.iter()
        .map(|x| {
            let mut swath = create_swath(
                Point::new(*x, 0.0),
                Point::new(*x, 50.0),
                SwathType::Swath,
                format!("swath_{}", *x as i32),
            );
            swath.width = 5.0;
            swath
        })
        .collect()
}

#[test]
fn obstacle_avoider_constructs_and_tracks_obstacles() {
    let avoider = ObstacleAvoider::new(vec![obstacle()], Geo::new(51.0, 5.0, 0.0));
    assert_eq!(avoider.get_obstacles().len(), 1);
    assert_eq!(avoider.get_inflated_obstacles().len(), 0);
}

#[test]
fn avoidance_splits_crossing_swaths_and_creates_around_segments() {
    let mut avoider = ObstacleAvoider::new(vec![obstacle()], Geo::new(51.0, 5.0, 0.0));
    let input = crossing_swaths(&[40.0, 45.0, 50.0, 55.0]);
    let avoided = avoider.avoid(&input, 2.0);

    assert!(!avoided.is_empty());
    assert!(avoided.len() >= input.len());
    assert!(avoider.get_inflated_obstacles().len() == 1);
    assert!(
        avoided
            .iter()
            .any(|swath| swath.r#type == SwathType::Around)
    );
    assert!(avoided.iter().any(|swath| swath.uuid != input[0].uuid));
}

#[test]
fn avoidance_with_different_inflation_distances_keeps_output_non_empty() {
    let input = crossing_swaths(&[10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]);

    let mut no_inflation = ObstacleAvoider::new(vec![obstacle()], Geo::new(51.0, 5.0, 0.0));
    let avoided_none = no_inflation.avoid(&input, 0.0);
    assert!(avoided_none.len() >= input.len());

    let mut small = ObstacleAvoider::new(vec![obstacle()], Geo::new(51.0, 5.0, 0.0));
    let avoided_small = small.avoid(&input, 1.0);
    assert!(avoided_small.len() >= input.len());

    let mut large = ObstacleAvoider::new(vec![obstacle()], Geo::new(51.0, 5.0, 0.0));
    let avoided_large = large.avoid(&input, 5.0);
    assert!(avoided_large.len() >= input.len());
}
