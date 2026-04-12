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

fn rotated_obstacle() -> Polygon {
    polygon_from_points(vec![
        Point::new(50.0, 18.0),
        Point::new(58.0, 25.0),
        Point::new(50.0, 32.0),
        Point::new(42.0, 25.0),
    ])
}

fn shifted_obstacle(dx: f64, dy: f64) -> Polygon {
    polygon_from_points(vec![
        Point::new(45.0 + dx, 20.0 + dy),
        Point::new(55.0 + dx, 20.0 + dy),
        Point::new(55.0 + dx, 30.0 + dy),
        Point::new(45.0 + dx, 30.0 + dy),
    ])
}

fn field_boundary() -> Polygon {
    polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ])
}

fn point_in_polygon(point: Point, polygon: &Polygon) -> bool {
    let ring = polygon.exterior().points().collect::<Vec<_>>();
    if ring.len() < 3 {
        return false;
    }

    let mut inside = false;
    let mut j = ring.len() - 1;
    for i in 0..ring.len() {
        let pi = ring[i];
        let pj = ring[j];
        let intersects = ((pi.y() > point.y()) != (pj.y() > point.y()))
            && (point.x()
                < (pj.x() - pi.x()) * (point.y() - pi.y()) / ((pj.y() - pi.y()).abs().max(1e-12))
                    + pi.x());
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[test]
fn obstacle_avoider_constructs_and_tracks_obstacles() {
    let avoider = ObstacleAvoider::new(vec![obstacle()], Geo::new(51.0, 5.0, 0.0));
    assert_eq!(avoider.get_obstacles().len(), 1);
    assert_eq!(avoider.get_inflated_obstacles().len(), 0);
}

#[test]
fn avoidance_splits_crossing_swaths_into_work_segments() {
    let mut avoider = ObstacleAvoider::new(vec![obstacle()], Geo::new(51.0, 5.0, 0.0));
    let input = crossing_swaths(&[40.0, 45.0, 50.0, 55.0]);
    let avoided = avoider.avoid(&input, 2.0);

    assert!(!avoided.is_empty());
    assert!(avoided.len() >= input.len());
    assert!(avoider.get_inflated_obstacles().len() == 1);
    assert!(
        avoided
            .iter()
            .all(|swath| swath.r#type == SwathType::Swath)
    );
    assert!(
        avoided
            .iter()
            .all(|swath| swath.points.len() >= 2)
    );
    assert!(avoided.iter().any(|swath| swath.id == input[1].id));
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

#[test]
fn rotated_obstacle_inflation_remains_polygon_shaped() {
    let mut avoider = ObstacleAvoider::new(vec![rotated_obstacle()], Geo::new(51.0, 5.0, 0.0));
    let input = crossing_swaths(&[50.0]);
    let _ = avoider.avoid(&input, 2.0);

    let inflated = &avoider.get_inflated_obstacles()[0];
    let points = inflated.exterior().points().collect::<Vec<_>>();
    assert!(points.len() >= 4);
    let xs = points.iter().map(|p| p.x()).collect::<Vec<_>>();
    assert!(
        xs.iter()
            .any(|x| (*x - 40.0).abs() > 1.0 && (*x - 60.0).abs() > 1.0)
    );
}

#[test]
fn off_axis_obstacle_splits_swath_without_points_inside_obstacle() {
    let mut swath = create_swath(
        Point::new(35.0, 15.0),
        Point::new(65.0, 35.0),
        SwathType::Swath,
        "diag",
    );
    swath.width = 5.0;

    let mut avoider = ObstacleAvoider::new(vec![rotated_obstacle()], Geo::new(51.0, 5.0, 0.0));
    let avoided = avoider.avoid(&[swath], 1.5);

    assert!(avoided.len() >= 2);
    assert!(
        avoided
            .iter()
            .flat_map(|swath| swath.points.iter())
            .all(|point| !point_in_polygon(*point, &rotated_obstacle()))
    );
}

#[test]
fn two_obstacles_on_one_swath_create_multiple_cut_segments() {
    let mut swath = create_swath(
        Point::new(50.0, 0.0),
        Point::new(50.0, 80.0),
        SwathType::Swath,
        "double",
    );
    swath.width = 5.0;

    let obstacles = vec![shifted_obstacle(0.0, 0.0), shifted_obstacle(0.0, 30.0)];
    let mut avoider = ObstacleAvoider::new(obstacles, Geo::new(51.0, 5.0, 0.0));
    let avoided = avoider.avoid(&[swath], 1.5);

    assert!(avoided.len() >= 3);
    assert!(avoided.iter().all(|swath| swath.r#type == SwathType::Swath));
}

#[test]
fn boundary_connected_obstacles_are_skipped_from_obstacle_routing() {
    let touching_border = polygon_from_points(vec![
        Point::new(0.0, 18.0),
        Point::new(10.0, 18.0),
        Point::new(10.0, 30.0),
        Point::new(0.0, 30.0),
    ]);

    let mut avoider = ObstacleAvoider::new(
        vec![obstacle(), touching_border],
        Geo::new(51.0, 5.0, 0.0),
    );
    avoider.set_field_boundary(field_boundary());
    let input = crossing_swaths(&[0.0, 50.0]);
    let avoided = avoider.avoid(&input, 2.0);

    assert_eq!(avoider.get_inflated_obstacles().len(), 2);
    assert_eq!(avoider.transit_obstacles().len(), 1);
    assert!(avoided.iter().all(|swath| swath.r#type == SwathType::Swath));
    assert!(!avoided.is_empty());
}
