use concord::{Geo, Wgs, to_enu};
use geo::{Point, Polygon};
use maptrax::{
    ConnectorMode, Field, ObstacleAvoider, Pose2D, ReedsShepp, Ring, SwathType,
    TourBuilder, TurnPlannerConfig, TurnPlannerModel, create_ring, polygon_from_points,
};

fn main() {
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

    let mut field = Field::new(polygon.clone(), datum).expect("field");
    field.gen_field(4.0, 0.0, 3).expect("generated");
    let part = &field.get_parts()[0];

    let obstacle = centered_obstacle(&polygon, 25.0);
    let clearance = part
        .swaths
        .first()
        .map(|swath| swath.width)
        .unwrap_or(0.0);
    let mut avoider = ObstacleAvoider::new(vec![obstacle.clone()], datum);
    avoider.set_field_boundary(part.boundary.polygon.clone());
    let avoided = avoider.avoid(&part.swaths, clearance);
    let transit_rings = avoider
        .inflated_obstacles()
        .iter()
        .enumerate()
        .map(|(index, polygon)| create_ring(polygon.clone(), format!("obstacle_transit_{}", index)))
        .collect::<Result<Vec<Ring>, _>>()
        .expect("rings");

    println!("headlands: {}", part.headlands.len());
    println!("obstacle clearance distance: {:.1}", clearance);
    println!("field swaths: {}", part.swaths.len());
    println!("avoided swaths: {}", avoided.len());
    println!("inflated obstacles kept: {}", avoider.inflated_obstacles().len());
    for (index, ring) in transit_rings.iter().enumerate() {
        println!(
            "transit ring {index}: points={}, bbox=({:.1},{:.1}) -> ({:.1},{:.1})",
            ring.polygon.exterior().points().count(),
            ring.bounding_box.min().x,
            ring.bounding_box.min().y,
            ring.bounding_box.max().x,
            ring.bounding_box.max().y,
        );
    }

    let obstacle_center = Point::new(
        (obstacle.exterior().points().map(|p| p.x()).fold(f64::INFINITY, f64::min)
            + obstacle
                .exterior()
                .points()
                .map(|p| p.x())
                .fold(f64::NEG_INFINITY, f64::max))
            * 0.5,
        (obstacle.exterior().points().map(|p| p.y()).fold(f64::INFINITY, f64::min)
            + obstacle
                .exterior()
                .points()
                .map(|p| p.y())
                .fold(f64::NEG_INFINITY, f64::max))
            * 0.5,
    );
    println!(
        "obstacle center=({:.1},{:.1})",
        obstacle_center.x(),
        obstacle_center.y()
    );

    let ordered = avoided
        .iter()
        .filter(|swath| swath.r#type == SwathType::Swath)
        .filter(|swath| {
            let mx = (swath.head().x() + swath.tail().x()) * 0.5;
            let my = (swath.head().y() + swath.tail().y()) * 0.5;
            (mx - obstacle_center.x()).abs() < 40.0 || (my - obstacle_center.y()).abs() < 60.0
        })
        .take(8)
        .cloned()
        .collect::<Vec<_>>();
    println!("ordered sample swaths: {}", ordered.len());
    for (index, swath) in ordered.iter().enumerate() {
        println!(
            "sample swath {index}: head=({:.1},{:.1}) tail=({:.1},{:.1})",
            swath.head().x(),
            swath.head().y(),
            swath.tail().x(),
            swath.tail().y()
        );
    }

    let mut transit_part = part.clone();
    transit_part.transit_rings = transit_rings;
    let tour = TourBuilder::build(
        &transit_part,
        &ordered,
        &TurnPlannerConfig {
            model: TurnPlannerModel::ReedsShepp,
            connector_mode: ConnectorMode::Auto,
            swath_width: 4.0,
            min_turning_radius: 2.0,
            step_size: 0.2,
            machine_length: 6.0,
            machine_width: 3.0,
            ..TurnPlannerConfig::default()
        },
    );

    let connections = tour
        .iter()
        .filter(|swath| swath.r#type == SwathType::Connection)
        .collect::<Vec<_>>();
    println!("tour segments: {}", tour.len());
    println!("connection segments: {}", connections.len());
    for (index, swath) in connections.iter().take(12).enumerate() {
        println!(
            "conn {index}: points={}, start=({:.1},{:.1}) end=({:.1},{:.1})",
            swath.points.len(),
            swath.head().x(),
            swath.head().y(),
            swath.tail().x(),
            swath.tail().y(),
        );
        if swath.points.len() > 2 {
            for (j, point) in swath.points.iter().take(6).enumerate() {
                println!("  p{j}=({:.1},{:.1})", point.x(), point.y());
            }
        }
    }

    let start = Pose2D::from_point(Point::new(0.0, 0.0), 0.0);
    let end = Pose2D::from_point(Point::new(0.0, 18.0), std::f64::consts::PI);
    let rs = ReedsShepp::new(4.0).plan_path(start, end, 0.2);
    println!(
        "reeds_shepp sanity: waypoints={}, length={:.2}",
        rs.waypoints.len(),
        rs.total_length
    );
}

fn centered_obstacle(border: &Polygon, half_size: f64) -> Polygon {
    let vertices = border.exterior().points().collect::<Vec<_>>();
    let (min_x, max_x) = vertices
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |acc, p| {
            (acc.0.min(p.x()), acc.1.max(p.x()))
        });
    let (min_y, max_y) = vertices
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |acc, p| {
            (acc.0.min(p.y()), acc.1.max(p.y()))
        });
    let center_x = (min_x + max_x) * 0.5;
    let center_y = (min_y + max_y) * 0.5;
    polygon_from_points(vec![
        Point::new(center_x - half_size, center_y - half_size),
        Point::new(center_x + half_size, center_y - half_size),
        Point::new(center_x + half_size, center_y + half_size),
        Point::new(center_x - half_size, center_y + half_size),
    ])
}
