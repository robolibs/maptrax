use concord::Geo;
use maptrax::core::{Segment, segment_end, segment_new, segment_start};
use maptrax::{
    Field, Point2Ext, Polygon, Swath, generate_swaths_from_line_for_polygon, point_xy,
    polygon_exterior_points, polygon_from_points,
};

/// A polder-shaped field: 300 m across the rows, 400 m along them.
fn polder() -> Polygon {
    polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(300.0, 0.0),
        point_xy(300.0, 400.0),
        point_xy(0.0, 400.0),
    ])
}

const RIDGE_WIDTH: f64 = 1.5;
const HEADLAND_WIDTH: f64 = 3.0;

/// Koen's first pass: centreline 0.75 m in from the western edge, running the
/// full length of the field.
fn seed_line() -> Segment {
    segment_new(point_xy(0.75, 0.0), point_xy(0.75, 400.0))
}

fn row_direction(swath: &Swath) -> (f64, f64) {
    let start = segment_start(swath.line);
    let end = segment_end(swath.line);
    let (dx, dy) = (end.x() - start.x(), end.y() - start.y());
    let len = (dx * dx + dy * dy).sqrt();
    (dx / len, dy / len)
}

/// Signed perpendicular offset of a swath from the seed line.
fn offset_from_seed(swath: &Swath) -> f64 {
    let anchor = segment_start(seed_line());
    let start = segment_start(swath.line);
    // Seed runs +y, so its left normal is (-1, 0).
    -(start.x() - anchor.x())
}

#[test]
fn rows_are_anchored_to_the_seed_line_not_the_centroid() {
    let rows = generate_swaths_from_line_for_polygon(RIDGE_WIDTH, seed_line(), &polder());

    let closest = rows
        .iter()
        .map(|row| offset_from_seed(row).abs())
        .fold(f64::INFINITY, f64::min);

    assert!(
        closest < 1e-9,
        "one row must sit exactly on the seed line, closest was {closest}"
    );
}

#[test]
fn rows_step_off_the_seed_by_exactly_one_working_width() {
    let rows = generate_swaths_from_line_for_polygon(RIDGE_WIDTH, seed_line(), &polder());

    for row in &rows {
        let steps = offset_from_seed(row) / RIDGE_WIDTH;
        assert!(
            (steps - steps.round()).abs() < 1e-9,
            "row at offset {} is not a whole number of widths from the seed",
            offset_from_seed(row)
        );
    }
}

#[test]
fn the_polder_takes_two_hundred_ridges() {
    let rows = generate_swaths_from_line_for_polygon(RIDGE_WIDTH, seed_line(), &polder());

    // x = 0.75 + k * 1.5 for k in 0..200 lands the last row at 299.25 m.
    assert_eq!(rows.len(), 200);

    for row in &rows {
        let x = segment_start(row.line).x();
        assert!(
            (0.75..=299.25).contains(&x),
            "row at x={x} fell outside the field"
        );
    }
}

#[test]
fn rows_inherit_the_seed_bearing_including_a_fraction_of_a_degree() {
    // The surveyed headland line in Koen's field runs 180.43 deg, not due
    // south. Driving a nominal angle instead would drift ~3 m over a 400 m
    // pass, so the generated rows must carry the seed's exact bearing.
    let skew = 0.43_f64.to_radians();
    let seed = segment_new(
        point_xy(0.75, 0.0),
        point_xy(0.75 + 400.0 * skew.sin(), 400.0 * skew.cos()),
    );
    let expected = {
        let start = segment_start(seed);
        let end = segment_end(seed);
        let (dx, dy) = (end.x() - start.x(), end.y() - start.y());
        let len = (dx * dx + dy * dy).sqrt();
        (dx / len, dy / len)
    };

    let rows = generate_swaths_from_line_for_polygon(RIDGE_WIDTH, seed, &polder());
    assert!(!rows.is_empty());

    for row in &rows {
        let (dx, dy) = row_direction(row);
        let cross = dx * expected.1 - dy * expected.0;
        assert!(
            cross.abs() < 1e-9,
            "row bearing drifted from the seed line by cross product {cross}"
        );
    }
}

#[test]
fn headland_depth_is_independent_of_working_width() {
    let mut field = Field::new(polder(), Geo::new(52.674, 5.762, 0.0)).expect("field");
    field
        .generate_headlands_with_width(HEADLAND_WIDTH, 1)
        .expect("headlands");

    let part = field.part(0).expect("part");
    assert_eq!(part.headlands.len(), 1);

    let ring = polygon_exterior_points(&part.headlands[0].polygon);
    let min_x = ring.iter().map(|p| p.x()).fold(f64::INFINITY, f64::min);
    let max_x = ring.iter().map(|p| p.x()).fold(f64::NEG_INFINITY, f64::max);

    // A 3 m turn lane around a field worked by a 1.5 m harvester.
    assert!((min_x - HEADLAND_WIDTH).abs() < 1e-6, "min_x was {min_x}");
    assert!(
        (max_x - (300.0 - HEADLAND_WIDTH)).abs() < 1e-6,
        "max_x was {max_x}"
    );
}

#[test]
fn rows_are_generated_inside_the_headland() {
    let mut field = Field::new(polder(), Geo::new(52.674, 5.762, 0.0)).expect("field");
    field
        .gen_field_from_line(RIDGE_WIDTH, seed_line(), HEADLAND_WIDTH, 1)
        .expect("generated");

    let part = field.part(0).expect("part");
    assert!(!part.swaths.is_empty());

    for row in part.work_swaths() {
        for point in [segment_start(row.line), segment_end(row.line)] {
            assert!(
                point.x() >= HEADLAND_WIDTH - 1e-6 && point.x() <= 300.0 - HEADLAND_WIDTH + 1e-6,
                "row endpoint {:?} escaped the headland",
                (point.x(), point.y())
            );
        }
    }
}

#[test]
fn marking_rows_finished_moves_them_between_the_two_counts() {
    let mut field = Field::new(polder(), Geo::new(52.674, 5.762, 0.0)).expect("field");
    field
        .gen_field_from_line(RIDGE_WIDTH, seed_line(), HEADLAND_WIDTH, 1)
        .expect("generated");

    let total = field.remaining_swath_count();
    assert!(total > 0);
    assert_eq!(field.finished_swath_count(), 0);

    for id in 0..10 {
        assert!(
            field.set_swath_finished(0, id, true).expect("part exists"),
            "no row carried id {id}"
        );
    }

    assert_eq!(field.finished_swath_count(), 10);
    assert_eq!(field.remaining_swath_count(), total - 10);

    field.set_swath_finished(0, 3, false).expect("part exists");
    assert_eq!(field.finished_swath_count(), 9);
}

#[test]
fn marking_an_unknown_row_reports_false_and_a_missing_part_errors() {
    let mut field = Field::new(polder(), Geo::new(52.674, 5.762, 0.0)).expect("field");
    field
        .gen_field_from_line(RIDGE_WIDTH, seed_line(), HEADLAND_WIDTH, 1)
        .expect("generated");

    assert!(!field.set_swath_finished(0, 9_999, true).expect("part"));
    assert!(field.set_swath_finished(7, 0, true).is_err());
}

#[test]
fn a_degenerate_seed_line_is_rejected() {
    let mut field = Field::new(polder(), Geo::new(52.674, 5.762, 0.0)).expect("field");
    let degenerate = segment_new(point_xy(10.0, 10.0), point_xy(10.0, 10.0));
    assert!(
        field
            .generate_swaths_from_line(RIDGE_WIDTH, degenerate)
            .is_err()
    );
    assert!(field.generate_swaths_from_line(0.0, seed_line()).is_err());
}
