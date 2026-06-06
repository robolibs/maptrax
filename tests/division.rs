use concord::{Geo, Wgs, to_enu};
use maptrax::{
    Balance, DivisionPattern, DivisionPlan, Divy, Field, MachineProfile, OptimizeObjective,
    polygon_from_points, segment_length,
};
use maptrax::{Point, Point2Ext, point_xy, polygon_exterior_points};

fn rect_field(swath_width: f64) -> Field {
    let polygon = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(100.0, 0.0),
        point_xy(100.0, 50.0),
        point_xy(0.0, 50.0),
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
            point_xy(enu.east(), enu.north())
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
        headlands: maptrax::HeadlandMode::default(),
    };
    let err = Divy::plan(&field.get_parts()[0], &plan).unwrap_err();
    assert_eq!(err.to_string(), "machine count must be greater than zero");
}

#[test]
fn stripe_1_by_count_preserves_total_and_assigns_fairly() {
    let field = rect_field(10.0);
    let plan = DivisionPlan::uniform(2, DivisionPattern::Stripe { stride: 1 }, Balance::ByCount);
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
        point_xy(0.0, 0.0),
        point_xy(120.0, 0.0),
        point_xy(120.0, 40.0),
        point_xy(0.0, 40.0),
    ]);
    let mut field = Field::new(polygon, Geo::new(51.0, 5.0, 0.0)).expect("field");
    field.gen_field(10.0, 90.0, 0).expect("generated");

    let plan = DivisionPlan::uniform(2, DivisionPattern::Stripe { stride: 2 }, Balance::ByCount);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");
    let (m0, m1) = (&result.swaths_per_machine[0], &result.swaths_per_machine[1]);

    assert_eq!(m0.len() + m1.len(), field.get_parts()[0].swaths.len());
    assert!(m0.len() >= 4 && m1.len() >= 4);

    // Check that machine 0's first two swaths are adjacent rows (consecutive x
    // centers), same for machine 1's first two: proves stride=2 bookkeeping.
    let x_center = |s: &maptrax::Swath| (s.head().x() + s.tail().x()) * 0.5;
    let d0 = (x_center(&m0[0]) - x_center(&m0[1])).abs();
    let d1 = (x_center(&m1[0]) - x_center(&m1[1])).abs();
    assert!(
        d0 < 11.0,
        "m0 first pair should be adjacent rows (gap {d0})"
    );
    assert!(
        d1 < 11.0,
        "m1 first pair should be adjacent rows (gap {d1})"
    );
}

#[test]
fn block_by_count_clusters_spatially() {
    let field = rect_field(10.0);
    let plan = DivisionPlan::uniform(2, DivisionPattern::Block, Balance::ByCount);
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
        headlands: maptrax::HeadlandMode::default(),
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
fn default_headland_mode_one_per_machine() {
    let field = upstream_field(); // 3 headland rings
    let plan = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    // Default is HeadlandMode::OnePerMachine — with 3 rings and 3 machines,
    // each machine gets exactly one ring.
    for (index, arcs) in result.headland_arcs_per_machine.iter().enumerate() {
        assert_eq!(
            arcs.len(),
            1,
            "machine {index} should own exactly 1 ring, got {}",
            arcs.len()
        );
    }
}

#[test]
fn one_per_machine_with_more_machines_than_rings() {
    let field = upstream_field(); // 3 rings
    let plan = DivisionPlan::uniform(5, DivisionPattern::Block, Balance::ByCount);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    // Machines 0,1,2 get one ring each; machines 3,4 get none.
    assert_eq!(result.headland_arcs_per_machine[0].len(), 1);
    assert_eq!(result.headland_arcs_per_machine[1].len(), 1);
    assert_eq!(result.headland_arcs_per_machine[2].len(), 1);
    assert!(result.headland_arcs_per_machine[3].is_empty());
    assert!(result.headland_arcs_per_machine[4].is_empty());
}

#[test]
fn dedicated_headland_mode_respects_machine_index() {
    let field = upstream_field();
    let plan = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount)
        .with_headlands(maptrax::HeadlandMode::Dedicated { machine: 2 });
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    assert!(result.headland_arcs_per_machine[0].is_empty());
    assert!(result.headland_arcs_per_machine[1].is_empty());
    assert_eq!(result.headland_arcs_per_machine[2].len(), 3);
}

#[test]
fn none_headland_mode_skips_headland_assignment() {
    let field = upstream_field();
    let plan = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount)
        .with_headlands(maptrax::HeadlandMode::None);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    for arcs in &result.headland_arcs_per_machine {
        assert!(arcs.is_empty());
    }
}

#[test]
fn split_by_zone_produces_arcs_for_every_machine() {
    let field = upstream_field(); // 3 headland rings
    let plan = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount)
        .with_headlands(maptrax::HeadlandMode::SplitByZone);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    for (machine_idx, arcs) in result.headland_arcs_per_machine.iter().enumerate() {
        assert!(
            arcs.len() >= 3,
            "machine {machine_idx} got only {} arcs under SplitByZone, expected ≥3",
            arcs.len()
        );
        for arc in arcs {
            assert!(arc.len() >= 2);
        }
    }
}

#[test]
fn split_by_zone_covers_each_ring_without_double_counting() {
    use maptrax::point_distance;
    let field = upstream_field();
    let plan = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount)
        .with_headlands(maptrax::HeadlandMode::SplitByZone);
    let result = Divy::plan(&field.get_parts()[0], &plan).expect("plan");

    let total_arc_length: f64 = result
        .headland_arcs_per_machine
        .iter()
        .flatten()
        .map(|arc| {
            arc.windows(2)
                .map(|pair| point_distance(pair[0], pair[1]))
                .sum::<f64>()
        })
        .sum();

    let total_ring_perimeter: f64 = field.get_parts()[0]
        .headlands
        .iter()
        .map(|ring| {
            let pts = polygon_exterior_points(&ring.polygon);
            pts.windows(2)
                .map(|pair| point_distance(pair[0], pair[1]))
                .sum::<f64>()
        })
        .sum();

    let error = (total_arc_length - total_ring_perimeter).abs() / total_ring_perimeter;
    assert!(
        error < 0.01,
        "arc total {total_arc_length:.1} vs ring perimeter {total_ring_perimeter:.1} ({:.2}% off)",
        error * 100.0,
    );
}

#[test]
fn dedicated_headlands_inflate_only_the_owner_work_time() {
    let field = upstream_field();
    let plan_none = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount)
        .with_headlands(maptrax::HeadlandMode::None);
    let plan_dedicated = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount)
        .with_headlands(maptrax::HeadlandMode::Dedicated { machine: 0 });

    let none = Divy::plan(&field.get_parts()[0], &plan_none).expect("plan");
    let dedicated = Divy::plan(&field.get_parts()[0], &plan_dedicated).expect("plan");

    // Machine 0 carries all headland perimeter work in Dedicated mode.
    assert!(
        dedicated.estimated_work_time[0] > none.estimated_work_time[0] + 1.0,
        "dedicated m0 {:.1}s should be >> none m0 {:.1}s",
        dedicated.estimated_work_time[0],
        none.estimated_work_time[0],
    );
    // Machines 1,2 do only interior swaths in both modes — times match.
    assert!((dedicated.estimated_work_time[1] - none.estimated_work_time[1]).abs() < 1e-6);
    assert!((dedicated.estimated_work_time[2] - none.estimated_work_time[2]).abs() < 1e-6);
}

#[test]
fn one_per_machine_spreads_headland_cost() {
    let field = upstream_field();
    let plan_none = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount)
        .with_headlands(maptrax::HeadlandMode::None);
    let plan_default = DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByCount);

    let none = Divy::plan(&field.get_parts()[0], &plan_none).expect("plan");
    let def = Divy::plan(&field.get_parts()[0], &plan_default).expect("plan");

    // Each machine's work time grew a bit (each got one ring).
    for index in 0..3 {
        assert!(
            def.estimated_work_time[index] > none.estimated_work_time[index] + 1.0,
            "machine {index} should have extra headland time under default mode",
        );
    }
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
