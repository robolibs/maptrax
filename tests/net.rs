use maptrax::{ABLine, Nety, RoutingOptions, RoutingStrategy, Swath, SwathType, create_swath};
use maptrax::{Point, Point2Ext, point_xy};

fn test_swaths() -> Vec<Swath> {
    (0..5)
        .map(|i| {
            let x = i as f64 * 10.0;
            let mut swath = create_swath(
                point_xy(x, 0.0),
                point_xy(x, 100.0),
                SwathType::Swath,
                format!("swath_{i}"),
            );
            swath.id = i;
            swath.width = 5.0;
            swath
        })
        .collect()
}

fn shuffled_parallel_swaths() -> Vec<Swath> {
    let xs = [30.0, 0.0, 40.0, 10.0, 20.0];
    xs.iter()
        .enumerate()
        .map(|(i, x)| {
            let mut swath = create_swath(
                point_xy(*x, 0.0),
                point_xy(*x, 100.0),
                SwathType::Swath,
                format!("swath_{}", *x as i32),
            );
            swath.id = i as i32;
            swath.width = 5.0;
            swath
        })
        .collect()
}

fn synthetic_quality_swaths() -> Vec<Swath> {
    vec![
        create_swath(
            point_xy(0.0, 0.0),
            point_xy(0.0, 40.0),
            SwathType::Swath,
            "a",
        ),
        create_swath(
            point_xy(30.0, 40.0),
            point_xy(30.0, 0.0),
            SwathType::Swath,
            "b",
        ),
        create_swath(
            point_xy(60.0, 0.0),
            point_xy(60.0, 40.0),
            SwathType::Swath,
            "c",
        ),
        create_swath(
            point_xy(90.0, 40.0),
            point_xy(90.0, 0.0),
            SwathType::Swath,
            "d",
        ),
    ]
}

fn swath_order(nety: &Nety) -> Vec<String> {
    nety.get_swaths()
        .iter()
        .filter(|swath| swath.r#type == SwathType::Swath)
        .map(|swath| swath.uuid.clone())
        .collect()
}

fn deadhead_distance(nety: &Nety, start: Point) -> f64 {
    let ordered = nety
        .get_swaths()
        .iter()
        .filter(|swath| swath.r#type == SwathType::Swath)
        .collect::<Vec<_>>();
    if ordered.is_empty() {
        return 0.0;
    }
    let mut total = ((ordered[0].head().x() - start.x()).powi(2)
        + (ordered[0].head().y() - start.y()).powi(2))
    .sqrt();
    for pair in ordered.windows(2) {
        total += ((pair[0].tail().x() - pair[1].head().x()).powi(2)
            + (pair[0].tail().y() - pair[1].head().y()).powi(2))
        .sqrt();
    }
    total
}

#[test]
fn ab_line_creation_and_properties_work() {
    let line = ABLine::new(point_xy(0.0, 0.0), point_xy(10.0, 0.0), "test_line", 1);
    assert!((line.length() - 10.0).abs() < 1e-9);

    let swath = create_swath(
        point_xy(0.0, 0.0),
        point_xy(10.0, 0.0),
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
    nety.shortest_path(Some(point_xy(0.0, 0.0)), Some(point_xy(40.0, 100.0)));

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

#[test]
fn snake_strategy_is_deterministic_on_parallel_swaths() {
    let swaths = shuffled_parallel_swaths();
    let options = RoutingOptions {
        strategy: RoutingStrategy::Snake,
        local_improvement_passes: 0,
    };

    let mut a = Nety::new(&swaths);
    a.field_traversal_with_options(Some(point_xy(0.0, 0.0)), options);

    let mut b = Nety::new(&swaths);
    b.field_traversal_with_options(Some(point_xy(0.0, 0.0)), options);

    assert_eq!(swath_order(&a), swath_order(&b));
    assert_eq!(
        swath_order(&a),
        vec![
            "swath_0".to_string(),
            "swath_10".to_string(),
            "swath_20".to_string(),
            "swath_30".to_string(),
            "swath_40".to_string(),
        ]
    );
}

#[test]
fn spiral_strategy_walks_outer_rows_inward() {
    let swaths = shuffled_parallel_swaths();
    let mut nety = Nety::new(&swaths);
    nety.field_traversal_with_options(
        Some(point_xy(0.0, 0.0)),
        RoutingOptions {
            strategy: RoutingStrategy::Spiral,
            local_improvement_passes: 0,
        },
    );

    assert_eq!(
        swath_order(&nety),
        vec![
            "swath_0".to_string(),
            "swath_40".to_string(),
            "swath_10".to_string(),
            "swath_30".to_string(),
            "swath_20".to_string(),
        ]
    );
}

#[test]
fn local_improvement_does_not_worsen_deadhead_distance() {
    let swaths = synthetic_quality_swaths();
    let start = point_xy(0.0, 0.0);

    let mut base = Nety::new(&swaths);
    base.field_traversal_with_options(
        Some(start),
        RoutingOptions {
            strategy: RoutingStrategy::Snake,
            local_improvement_passes: 0,
        },
    );
    let base_cost = deadhead_distance(&base, start);

    let mut improved = Nety::new(&swaths);
    improved.field_traversal_with_options(
        Some(start),
        RoutingOptions {
            strategy: RoutingStrategy::Snake,
            local_improvement_passes: 4,
        },
    );
    let improved_cost = deadhead_distance(&improved, start);

    assert!(improved_cost <= base_cost + 1e-9);
}
