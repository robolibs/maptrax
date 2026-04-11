use geo::Point;
use maptrax::{ABLine, Nety, Swath, SwathType, create_swath};

fn test_swaths() -> Vec<Swath> {
    (0..5)
        .map(|i| {
            let x = i as f64 * 10.0;
            let mut swath = create_swath(
                Point::new(x, 0.0),
                Point::new(x, 100.0),
                SwathType::Swath,
                format!("swath_{i}"),
            );
            swath.id = i;
            swath.width = 5.0;
            swath
        })
        .collect()
}

#[test]
fn ab_line_creation_and_properties_work() {
    let line = ABLine::new(
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        "test_line",
        1,
    );
    assert!((line.length() - 10.0).abs() < 1e-9);

    let swath = create_swath(
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        SwathType::Swath,
        "test_line",
    );
    assert!(line.equal(&swath));
}

#[test]
fn nety_construction_uses_all_swaths() {
    let swaths = test_swaths();
    let nety = Nety::new(&swaths);

    assert_eq!(nety.get_swaths().len(), swaths.len());
    assert_eq!(nety.get_ab_lines().len(), swaths.len());
    assert_eq!(nety.num_vertices(), swaths.len() * 2);
    assert!(nety.num_edges() >= swaths.len());
}

#[test]
fn field_traversal_preserves_all_working_swaths() {
    let swaths = test_swaths();
    let mut nety = Nety::new(&swaths);
    nety.field_traversal(None);

    let optimized = nety.get_swaths();
    assert_eq!(optimized.len(), swaths.len() * 2 - 1);
    assert_eq!(
        optimized
            .iter()
            .filter(|swath| swath.r#type == SwathType::Swath)
            .count(),
        swaths.len()
    );
    assert_eq!(
        optimized
            .iter()
            .filter(|swath| swath.r#type == SwathType::Connection)
            .count(),
        swaths.len() - 1
    );
    for original in &swaths {
        assert!(
            optimized
                .iter()
                .any(|candidate| candidate.uuid == original.uuid)
        );
    }
}

#[test]
fn shortest_path_reorders_swaths_along_the_graph() {
    let swaths = test_swaths();
    let mut nety = Nety::new(&swaths);
    nety.shortest_path(
        Some(Point::new(0.0, 0.0)),
        Some(Point::new(40.0, 100.0)),
    );

    let ordered = nety.get_swaths();
    assert!(!ordered.is_empty());
    assert_eq!(
        ordered.first().map(|swath| swath.uuid.as_str()),
        Some("swath_0")
    );
    assert_eq!(
        ordered.last().map(|swath| swath.uuid.as_str()),
        Some("swath_4")
    );
}
