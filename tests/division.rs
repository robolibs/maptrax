use concord::{Geo, Wgs, to_enu};
use geo::Point;
use maptrax::{
    Balance, DivisionPattern, DivisionPlan, Divy, Field, MachineProfile, OptimizeObjective,
    polygon_from_points, segment_length,
};

fn rect_field(swath_width: f64) -> Field {
    let polygon = polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(100.0, 50.0),
        Point::new(0.0, 50.0),
    ]);
    let mut field = Field::new(polygon, Geo::new(51.0, 5.0, 0.0)).expect("field");
    field.gen_field(swath_width, 90.0, 0).expect("generated");
    field
}

fn upstream_field() -> Field {
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
        .collect::<Vec<Point>>(),
    );
    let mut field = Field::new(polygon, datum).expect("field");
    field.gen_field(4.0, 0.0, 3).expect("generated");
    field
}

fn total_length(swaths: &[maptrax::Swath]) -> f64 {
    swaths.iter().map(|swath| segment_length(swath.line)).sum()
}

fn total_count(result: &maptrax::DivisionResult) -> usize {
    result.swaths_per_machine.iter().map(Vec::len).sum()
}

#[test]
fn rejects_empty_machine_list() {
    let field = rect_field(10.0);
    let plan = DivisionPlan {
        pattern: DivisionPattern::Block,
        balance: Balance::ByCount,
        machines: Vec::new(),
    };
    let err = Divy::plan(&field.get_parts()[0], &plan).unwrap_err();
    assert_eq!(err.to_string(), "machine count must be greater than zero");
}

#[test]
fn stripe_1_by_count_preserves_total_and_assigns_fairly() {
    let field = rect_field(10.0);
    let plan = DivisionPlan::uniform(
        2,
        DivisionPattern::Stripe { stride: 1 },
        Balance::ByCount,
    );
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    assert_eq!(result.swaths_per_machine.len(), 2);
    assert_eq!(total_count(&result), field.get_parts()[0].swaths.len());
    let lens: Vec<usize> = result.swaths_per_machine.iter().map(Vec::len).collect();
    assert!(lens.iter().max().unwrap() - lens.iter().min().unwrap() <= 1);
}

#[test]
fn stripe_stride_2_interleaves_in_pairs() {
    // 12 rows, 2 machines, stride=2 → pattern: 0 0 1 1 0 0 1 1 0 0 1 1
    // Each machine owns 6 rows; rows 0–1, 4–5, 8–9 for machine 0.
    let polygon = polygon_from_points(vec![
        Point::new(0.0, 0.0),
        Point::new(120.0, 0.0),
        Point::new(120.0, 40.0),
        Point::new(0.0, 40.0),
    ]);
    let mut field = Field::new(polygon, Geo::new(51.0, 5.0, 0.0)).expect("field");
    field.gen_field(10.0, 90.0, 0).expect("generated");

    let plan = DivisionPlan::uniform(
        2,
        DivisionPattern::Stripe { stride: 2 },
        Balance::ByCount,
    );
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");
    let (m0, m1) = (&result.swaths_per_machine[0], &result.swaths_per_machine[1]);

    assert_eq!(m0.len() + m1.len(), field.get_parts()[0].swaths.len());
    assert!(m0.len() >= 4 && m1.len() >= 4);

    // Check that machine 0's first two swaths are adjacent rows (consecutive x
    // centers), same for machine 1's first two: proves stride=2 bookkeeping.
    let x_center = |s: &maptrax::Swath| (s.head().x() + s.tail().x()) * 0.5;
    let d0 = (x_center(&m0[0]) - x_center(&m0[1])).abs();
    let d1 = (x_center(&m1[0]) - x_center(&m1[1])).abs();
    assert!(d0 < 11.0, "m0 first pair should be adjacent rows (gap {d0})");
    assert!(d1 < 11.0, "m1 first pair should be adjacent rows (gap {d1})");
}

#[test]
fn block_by_count_clusters_spatially() {
    let field = rect_field(10.0);
    let plan =
        DivisionPlan::uniform(2, DivisionPattern::Block, Balance::ByCount);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    let left = &result.swaths_per_machine[0];
    let right = &result.swaths_per_machine[1];
    assert!(!left.is_empty() && !right.is_empty());
    let avg_left = left.iter().map(|s| s.line.start.x).sum::<f64>() / left.len() as f64;
    let avg_right = right.iter().map(|s| s.line.start.x).sum::<f64>() / right.len() as f64;
    assert!(
        (avg_left - avg_right).abs() > 5.0,
        "block should separate machines spatially"
    );
}

#[test]
fn block_by_length_balances_work_on_irregular_field() {
    let field = upstream_field();
    let plan = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByLength);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    assert_eq!(total_count(&result), field.get_parts()[0].swaths.len());

    let lengths: Vec<f64> = result
        .swaths_per_machine
        .iter()
        .map(|swaths| total_length(swaths))
        .collect();
    let total: f64 = lengths.iter().sum();
    let target = total / 3.0;
    for (index, length) in lengths.iter().enumerate() {
        let deviation = (length - target).abs() / target;
        assert!(
            deviation < 0.20,
            "machine {index} length {length:.1} differs from target {target:.1} by {:.1}%",
            deviation * 100.0,
        );
    }
}

#[test]
fn banded_stripe_2_splits_into_two_contiguous_bands_with_internal_stripe() {
    let field = rect_field(10.0); // 10 rows, rect width 100
    let plan = DivisionPlan::uniform(
        2,
        DivisionPattern::BandedStripe { bands: 2 },
        Balance::ByCount,
    );
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    assert_eq!(total_count(&result), field.get_parts()[0].swaths.len());
    assert_eq!(result.swaths_per_machine.len(), 2);
    // With bands=2 and 2 machines with rotation, each machine ends up
    // owning swaths from both bands. Sanity: nobody is empty.
    for (idx, swaths) in result.swaths_per_machine.iter().enumerate() {
        assert!(!swaths.is_empty(), "machine {idx} empty");
    }
}

#[test]
fn weights_bias_assignment_proportionally() {
    let field = rect_field(10.0);
    // 10 swaths, weights (3, 1) → machine 0 should get roughly 3x the count/length.
    let plan = DivisionPlan {
        pattern: DivisionPattern::Block,
        balance: Balance::ByCount,
        machines: vec![
            MachineProfile {
                weight: 3.0,
                speed: 1.0,
            },
            MachineProfile {
                weight: 1.0,
                speed: 1.0,
            },
        ],
    };
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");
    let (c0, c1) = (
        result.swaths_per_machine[0].len(),
        result.swaths_per_machine[1].len(),
    );
    assert_eq!(c0 + c1, field.get_parts()[0].swaths.len());
    assert!(
        c0 as f64 / c1 as f64 > 2.0,
        "weight 3:1 should give m0 > 2x m1 (got {c0}:{c1})"
    );
}

#[test]
fn optimized_makespan_picks_a_valid_candidate() {
    let field = upstream_field();
    let plan = DivisionPlan::uniform(
        3,
        DivisionPattern::Optimized {
            objective: OptimizeObjective::Makespan,
        },
        Balance::ByLength,
    );
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    assert_eq!(total_count(&result), field.get_parts()[0].swaths.len());
    // Optimized resolves to a concrete pattern — never leaves Optimized in the result.
    assert!(
        !matches!(
            result.pattern_used,
            Some(DivisionPattern::Optimized { .. }) | None
        ),
        "pattern_used should resolve to a concrete pattern, got {:?}",
        result.pattern_used
    );
}

#[test]
fn headlands_are_distributed_across_machines() {
    let field = upstream_field(); // 3 headland rings
    let plan = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    let counts: Vec<usize> = result.headlands_per_machine.iter().map(Vec::len).collect();
    assert_eq!(counts, vec![1, 1, 1]);
}

#[test]
fn estimated_times_and_transit_are_populated() {
    let field = upstream_field();
    let plan = DivisionPlan::uniform(2, DivisionPattern::Block, Balance::ByCount);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    assert_eq!(result.estimated_work_time.len(), 2);
    assert_eq!(result.estimated_transit.len(), 2);
    assert!(result.estimated_work_time.iter().all(|t| *t > 0.0));
    assert!(result.estimated_transit.iter().all(|t| *t >= 0.0));
}
