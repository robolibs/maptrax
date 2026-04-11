use geo::Point;
use maptrax::{Dubins, Pose2D, ReedsShepp, Sharper, point_distance};

#[test]
fn dubins_produces_a_path() {
    let start = Pose2D::new(0.0, 0.0, 0.0);
    let goal = Pose2D::new(5.0, 2.0, 1.0);

    let dubins = Dubins::new(1.0);
    let path = dubins.plan_path(start, goal, 0.2);

    assert!(path.total_length > 0.0);
    assert!(!path.waypoints.is_empty());
    assert!(!path.segments.is_empty());
}

#[test]
fn reeds_shepp_produces_a_path() {
    let start = Pose2D::new(0.0, 0.0, 0.0);
    let goal = Pose2D::new(3.0, -2.0, std::f64::consts::PI);

    let rs = ReedsShepp::new(1.0);
    let path = rs.plan_path(start, goal, 0.2);

    assert!(path.total_length > 0.0);
    assert!(!path.waypoints.is_empty());
    assert!(!path.segments.is_empty());
}

#[test]
fn sharper_produces_a_turn() {
    let start = Pose2D::from_point(Point::new(0.0, 0.0), 0.0);
    let goal = Pose2D::from_point(Point::new(0.0, 0.0), std::f64::consts::PI);

    let sharper = Sharper::new(1.0, 4.0, 2.0);
    let path = sharper.plan_sharp_turn(start, goal, "auto");

    assert!(path.total_length >= 0.0);
    assert!(!path.waypoints.is_empty());
    assert!(point_distance(path.waypoints.first().unwrap().point, start.point) < 1e-9);
    assert!(point_distance(path.waypoints.last().unwrap().point, goal.point) < 1e-9);
}
