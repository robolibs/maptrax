use maptrax::{
    angle_between, angle_difference, are_colinear, float_to_byte, float_to_byte_in_range,
    heading_between, normalize_angle, point_to_line_distance, point_xy, polygon_exterior_points,
    polygon_from_points,
    remove_colinear_points,
};

#[test]
fn float_to_byte_matches_upstream_behavior() {
    assert_eq!(float_to_byte(0.0), 0);
    assert_eq!(float_to_byte(1.0), 255);
    assert_eq!(float_to_byte(0.5), 128);
    assert_eq!(float_to_byte(-1.0), 0);
    assert_eq!(float_to_byte(2.0), 255);
    assert_eq!(float_to_byte_in_range(0.5, 100.0, 200.0), 128);
}

#[test]
fn geometry_helpers_match_expected_values() {
    let p1 = point_xy(0.0, 0.0);
    let p2 = point_xy(1.0, 1.0);
    let p3 = point_xy(2.0, 2.0);
    let p4 = point_xy(2.0, 1.0);

    assert!(are_colinear(p1, p2, p3, 1e-10));
    assert!(!are_colinear(p1, p2, p4, 1e-10));

    let distance = point_to_line_distance(
        point_xy(0.0, 5.0),
        point_xy(1.0, 0.0),
        point_xy(1.0, 10.0),
    );
    assert!((distance - 1.0).abs() < 1e-9);

    let angle = angle_between(point_xy(1.0, 0.0), point_xy(0.0, 1.0));
    assert!((angle - std::f64::consts::FRAC_PI_2).abs() < 1e-9);

    let heading = heading_between(point_xy(0.0, 0.0), point_xy(0.0, 1.0));
    assert!((heading - std::f64::consts::FRAC_PI_2).abs() < 1e-9);

    let normalized = normalize_angle(4.0);
    assert!(normalized <= std::f64::consts::PI);

    let diff = angle_difference(0.0, std::f64::consts::PI * 1.5);
    assert!((diff + std::f64::consts::FRAC_PI_2).abs() < 1e-9);
}

#[test]
fn remove_colinear_points_simplifies_closed_polygon() {
    let polygon = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(1.0, 0.0),
        point_xy(2.0, 0.0),
        point_xy(2.0, 1.0),
        point_xy(0.0, 1.0),
    ]);

    let simplified = remove_colinear_points(&polygon, 0.01);
    assert!(polygon_exterior_points(&simplified).len() < polygon_exterior_points(&polygon).len());
    let exterior = polygon_exterior_points(&simplified);
    assert_eq!(exterior.first(), exterior.last());
}
