use std::collections::HashSet;

use geo::Point;

use crate::core::{
    MaptraxError, Result, point_distance, points_equal, polygon_open_vertices, segment_length,
};
use crate::field::{Part, Ring, Swath, SwathType, canonical_swath_order, dominant_swath_tangent};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DivisionPattern {
    /// Interleave rows across machines. `stride` = consecutive rows per
    /// machine before rotating. `stride: 1` is the classic alternating split.
    Stripe { stride: usize },

    /// Contiguous chunks along the swath normal. Machine 0 owns the first
    /// band of rows, machine 1 the next, and so on.
    Block,

    /// Split rows into `bands` contiguous bands; within each band, stripe by 1.
    /// `bands: 1` is equivalent to `Block`; large `bands` approaches `Stripe`.
    BandedStripe { bands: usize },

    /// Search a fixed grid of (pattern, balance) pairs and keep the best under
    /// `objective`. The result's `pattern_used` reports what was chosen.
    Optimized { objective: OptimizeObjective },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Balance {
    /// Weighted-equal swath count.
    ByCount,
    /// Weighted-equal total segment length (better when rows have varying
    /// lengths on irregular fields).
    ByLength,
}

/// How headland rings are distributed across machines.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HeadlandMode {
    /// One ring per machine, round-robin. Machine i owns ring (i mod
    /// num_rings), so with 3 machines and 3 rings each machine gets one
    /// complete headland ring. If there are more machines than rings, extra
    /// machines get no headland work; if there are more rings than machines,
    /// rings wrap and a machine may own two.
    OnePerMachine,

    /// All headland rings go to one machine (the chosen one still receives
    /// its normal swath share on top of headland work).
    Dedicated { machine: usize },

    /// Split each ring into arcs by lateral zone along the swath normal.
    /// Each machine gets the portion of each ring near its own swath band.
    SplitByZone,

    /// No headland work assigned (assume it's handled elsewhere).
    None,
}

impl Default for HeadlandMode {
    fn default() -> Self {
        HeadlandMode::OnePerMachine
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OptimizeObjective {
    /// Minimise max(per-machine work time + intra-machine transit time).
    Makespan,
    /// Minimise total intra-machine transit (meters).
    TotalTransit,
    /// Weighted sum of makespan and transit. Weights are unnormalised.
    Weighted { makespan: f64, transit: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MachineProfile {
    /// Relative share of work. Default 1.0.
    pub weight: f64,
    /// Traverse speed in m/s. Used for makespan estimation. Default 1.0.
    pub speed: f64,
}

impl Default for MachineProfile {
    fn default() -> Self {
        Self {
            weight: 1.0,
            speed: 1.0,
        }
    }
}

impl MachineProfile {
    pub fn uniform(count: usize) -> Vec<Self> {
        vec![Self::default(); count]
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DivisionPlan {
    pub pattern: DivisionPattern,
    pub balance: Balance,
    pub machines: Vec<MachineProfile>,
    /// Defaults to `HeadlandMode::Dedicated { machine: 0 }` — the typical
    /// farm setup where one machine runs the entire perimeter.
    pub headlands: HeadlandMode,
}

impl DivisionPlan {
    /// Convenience: N machines with uniform profiles, default balance/pattern,
    /// and the default `Dedicated { machine: 0 }` headland mode.
    pub fn uniform(machines: usize, pattern: DivisionPattern, balance: Balance) -> Self {
        Self {
            pattern,
            balance,
            machines: MachineProfile::uniform(machines),
            headlands: HeadlandMode::default(),
        }
    }

    pub fn machine_count(&self) -> usize {
        self.machines.len()
    }

    /// Override the headland strategy, returning `self` for chaining.
    pub fn with_headlands(mut self, mode: HeadlandMode) -> Self {
        self.headlands = mode;
        self
    }
}

/// Open polyline representing one contiguous arc of a headland ring assigned
/// to a single machine. Arcs are NOT closed — they represent a portion of the
/// ring's perimeter lying within that machine's lateral zone.
pub type HeadlandArc = Vec<Point>;

#[derive(Debug, Clone, PartialEq, Default)]
pub struct DivisionResult {
    /// The pattern that was actually applied. Matches `plan.pattern` except
    /// for `Optimized`, where this reports the winning concrete pattern.
    pub pattern_used: Option<DivisionPattern>,
    pub swaths_per_machine: Vec<Vec<Swath>>,
    /// Per-machine list of headland arcs — portions of each headland ring
    /// that fall within the machine's lateral zone along the swath normal.
    /// A machine typically gets two arcs per ring (one on the top of the
    /// field, one on the bottom) unless it owns the leftmost or rightmost
    /// zone, in which case its arcs wrap around the end of the field.
    pub headland_arcs_per_machine: Vec<Vec<HeadlandArc>>,
    /// Estimated pure work time per machine (seconds). Computed as
    /// total assigned swath length / machine speed.
    pub estimated_work_time: Vec<f64>,
    /// Estimated intra-machine transit distance per machine (meters),
    /// computed via greedy-nearest ordering over the assigned swaths.
    pub estimated_transit: Vec<f64>,
}

/// Stateless planner. Takes a `Part` plus a `DivisionPlan`, returns a
/// fully-populated `DivisionResult`. There is no `compute_division()` step.
pub struct Divy;

impl Divy {
    pub fn plan(part: &Part, plan: &DivisionPlan) -> Result<DivisionResult> {
        if plan.machines.is_empty() {
            return Err(MaptraxError::InvalidMachineCount);
        }
        for profile in &plan.machines {
            if profile.weight <= 0.0 || profile.speed <= 0.0 {
                return Err(MaptraxError::InvalidMachineCount);
            }
        }

        let work_swaths: Vec<Swath> = part
            .swaths
            .iter()
            .filter(|swath| !swath.finished && swath.r#type == SwathType::Swath)
            .cloned()
            .collect();

        let canonical = canonical_swath_order(&work_swaths);
        let ordered: Vec<Swath> = canonical.iter().map(|&i| work_swaths[i].clone()).collect();

        let headlands: Vec<Ring> = part
            .headlands
            .iter()
            .filter(|ring| !ring.finished)
            .cloned()
            .collect();

        let (swaths_per_machine, pattern_used) = match plan.pattern {
            DivisionPattern::Optimized { objective } => optimize_pattern(&ordered, plan, objective),
            pattern => (apply_pattern(&ordered, plan, pattern), pattern),
        };

        let headland_arcs_per_machine =
            assign_headlands(&headlands, &swaths_per_machine, &ordered, plan.headlands);

        let estimated_work_time: Vec<f64> = swaths_per_machine
            .iter()
            .zip(plan.machines.iter())
            .enumerate()
            .map(|(index, (swaths, profile))| {
                let swath_len = total_length(swaths);
                let headland_len: f64 = headland_arcs_per_machine
                    .get(index)
                    .map(|arcs| arcs.iter().map(|arc| polyline_length(arc)).sum())
                    .unwrap_or(0.0);
                (swath_len + headland_len) / profile.speed
            })
            .collect();

        let estimated_transit = swaths_per_machine
            .iter()
            .map(|swaths| greedy_nearest_transit(swaths))
            .collect();

        Ok(DivisionResult {
            pattern_used: Some(pattern_used),
            swaths_per_machine,
            headland_arcs_per_machine,
            estimated_work_time,
            estimated_transit,
        })
    }
}

fn apply_pattern(
    ordered: &[Swath],
    plan: &DivisionPlan,
    pattern: DivisionPattern,
) -> Vec<Vec<Swath>> {
    match pattern {
        DivisionPattern::Stripe { stride } => stripe_assign(ordered, plan, stride.max(1)),
        DivisionPattern::Block => block_assign(ordered, plan),
        DivisionPattern::BandedStripe { bands } => {
            banded_stripe_assign(ordered, plan, bands.max(1))
        }
        DivisionPattern::Optimized { .. } => unreachable!("optimize_pattern unrolls this"),
    }
}

fn total_length(swaths: &[Swath]) -> f64 {
    swaths.iter().map(|swath| segment_length(swath.line)).sum()
}

fn weight_shares(plan: &DivisionPlan) -> Vec<f64> {
    let total: f64 = plan.machines.iter().map(|profile| profile.weight).sum();
    if total <= 0.0 {
        return vec![1.0 / plan.machines.len() as f64; plan.machines.len()];
    }
    plan.machines
        .iter()
        .map(|profile| profile.weight / total)
        .collect()
}

fn stripe_assign(ordered: &[Swath], plan: &DivisionPlan, stride: usize) -> Vec<Vec<Swath>> {
    let machine_count = plan.machine_count();
    let mut buckets: Vec<Vec<Swath>> = vec![Vec::new(); machine_count];

    match plan.balance {
        Balance::ByCount => {
            let shares = weight_shares(plan);
            let quotas = by_count_quotas(ordered.len(), &shares);
            stripe_with_quotas(ordered, &mut buckets, stride, |machine, taken| {
                taken[machine] < quotas[machine]
            });
        }
        Balance::ByLength => {
            let shares = weight_shares(plan);
            let total = total_length(ordered);
            let quotas: Vec<f64> = shares.iter().map(|share| share * total).collect();
            let mut taken = vec![0.0; machine_count];
            let mut eligible = (0..machine_count).collect::<Vec<_>>();
            let mut cursor = 0usize;
            let mut stride_left = stride;
            for swath in ordered {
                if eligible.is_empty() {
                    // should not happen with a valid plan, but keep stable
                    break;
                }
                let machine = eligible[cursor % eligible.len()];
                buckets[machine].push(swath.clone());
                taken[machine] += segment_length(swath.line);
                stride_left = stride_left.saturating_sub(1);
                if stride_left == 0 {
                    cursor = cursor.saturating_add(1);
                    stride_left = stride;
                    // Drop machines that have met or exceeded their quota.
                    eligible.retain(|&m| taken[m] + 1e-9 < quotas[m]);
                    if eligible.is_empty() {
                        // Fallback: keep going with all machines (slack absorption).
                        eligible = (0..machine_count).collect();
                    }
                }
            }
        }
    }

    buckets
}

fn stripe_with_quotas<F>(
    ordered: &[Swath],
    buckets: &mut [Vec<Swath>],
    stride: usize,
    mut can_take: F,
) where
    F: FnMut(usize, &[usize]) -> bool,
{
    let machine_count = buckets.len();
    let mut taken = vec![0usize; machine_count];
    let mut cursor = 0usize;
    let mut stride_left = stride;
    for swath in ordered {
        // advance to a machine that can still take
        let mut attempts = 0;
        while attempts < machine_count && !can_take(cursor % machine_count, &taken) {
            cursor = cursor.saturating_add(1);
            stride_left = stride;
            attempts += 1;
        }
        let machine = cursor % machine_count;
        buckets[machine].push(swath.clone());
        taken[machine] += 1;
        stride_left = stride_left.saturating_sub(1);
        if stride_left == 0 {
            cursor = cursor.saturating_add(1);
            stride_left = stride;
        }
    }
}

fn block_assign(ordered: &[Swath], plan: &DivisionPlan) -> Vec<Vec<Swath>> {
    let machine_count = plan.machine_count();
    let shares = weight_shares(plan);
    let mut buckets: Vec<Vec<Swath>> = vec![Vec::new(); machine_count];

    match plan.balance {
        Balance::ByCount => {
            let quotas = by_count_quotas(ordered.len(), &shares);
            let mut cursor = 0usize;
            for (machine, quota) in quotas.iter().enumerate() {
                for _ in 0..*quota {
                    if cursor < ordered.len() {
                        buckets[machine].push(ordered[cursor].clone());
                        cursor += 1;
                    }
                }
            }
            while cursor < ordered.len() {
                buckets
                    .last_mut()
                    .expect("at least one machine")
                    .push(ordered[cursor].clone());
                cursor += 1;
            }
        }
        Balance::ByLength => {
            let total = total_length(ordered);
            let mut boundaries: Vec<f64> = Vec::with_capacity(machine_count);
            let mut cumulative = 0.0;
            for share in shares.iter().take(machine_count - 1) {
                cumulative += share * total;
                boundaries.push(cumulative);
            }
            let mut machine = 0usize;
            let mut running = 0.0;
            for swath in ordered {
                let len = segment_length(swath.line);
                // Advance machine if crossing a boundary *after* placing the
                // current swath would overshoot more than placing and stopping.
                if machine < boundaries.len() {
                    let boundary = boundaries[machine];
                    if running >= boundary && !buckets[machine].is_empty() {
                        machine += 1;
                    } else if running + len > boundary + 1e-9 {
                        // Place the swath with whichever side minimises overshoot.
                        let overshoot_here = (running + len - boundary).abs();
                        let undershoot_here = (boundary - running).abs();
                        if overshoot_here < undershoot_here || buckets[machine].is_empty() {
                            buckets[machine].push(swath.clone());
                            running += len;
                            machine += 1;
                            continue;
                        } else {
                            machine += 1;
                            buckets[machine.min(plan.machine_count() - 1)].push(swath.clone());
                            running += len;
                            continue;
                        }
                    }
                }
                buckets[machine.min(plan.machine_count() - 1)].push(swath.clone());
                running += len;
            }
        }
    }

    buckets
}

fn banded_stripe_assign(ordered: &[Swath], plan: &DivisionPlan, bands: usize) -> Vec<Vec<Swath>> {
    let machine_count = plan.machine_count();
    if bands <= 1 {
        return block_assign(ordered, plan);
    }
    let mut buckets: Vec<Vec<Swath>> = vec![Vec::new(); machine_count];

    // Partition `ordered` into `bands` contiguous slices weighted by the sum
    // of machine shares (i.e., all bands are equal-share slices of the field).
    let band_bounds = equal_count_slices(ordered.len(), bands);

    // Within each band, stripe across machines using the chosen balance.
    for (band_index, (start, end)) in band_bounds.iter().enumerate() {
        let slice = &ordered[*start..*end];
        let local_buckets = stripe_assign(slice, plan, 1);
        // Rotate the stripe starting machine per band so a machine doesn't
        // always own the same strip within every band (keeps headland load
        // less lopsided). Offset = band_index.
        for (machine, swaths) in local_buckets.into_iter().enumerate() {
            let target = (machine + band_index) % machine_count;
            buckets[target].extend(swaths);
        }
    }

    buckets
}

fn equal_count_slices(total: usize, slices: usize) -> Vec<(usize, usize)> {
    if slices == 0 || total == 0 {
        return vec![(0, total)];
    }
    let base = total / slices;
    let rem = total % slices;
    let mut bounds = Vec::with_capacity(slices);
    let mut cursor = 0usize;
    for i in 0..slices {
        let count = base + usize::from(i < rem);
        bounds.push((cursor, cursor + count));
        cursor += count;
    }
    bounds
}

fn by_count_quotas(total: usize, shares: &[f64]) -> Vec<usize> {
    let mut quotas: Vec<usize> = shares
        .iter()
        .map(|share| (share * total as f64).floor() as usize)
        .collect();
    let mut assigned: usize = quotas.iter().sum();
    // Hand out the remainder to the largest fractional shares first.
    let mut fractional: Vec<(usize, f64)> = shares
        .iter()
        .enumerate()
        .map(|(index, share)| {
            let exact = share * total as f64;
            (index, exact - exact.floor())
        })
        .collect();
    fractional.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    let mut idx = 0;
    while assigned < total && !fractional.is_empty() {
        let machine = fractional[idx % fractional.len()].0;
        quotas[machine] += 1;
        assigned += 1;
        idx += 1;
    }
    quotas
}

fn polyline_length(points: &[Point]) -> f64 {
    points
        .windows(2)
        .map(|pair| point_distance(pair[0], pair[1]))
        .sum()
}

fn assign_headlands(
    headlands: &[Ring],
    swaths_per_machine: &[Vec<Swath>],
    all_work_swaths: &[Swath],
    mode: HeadlandMode,
) -> Vec<Vec<HeadlandArc>> {
    let machine_count = swaths_per_machine.len();
    let mut result: Vec<Vec<HeadlandArc>> = vec![Vec::new(); machine_count];

    if machine_count == 0 || headlands.is_empty() {
        return result;
    }

    match mode {
        HeadlandMode::None => result,
        HeadlandMode::OnePerMachine => {
            for (ring_idx, ring) in headlands.iter().enumerate() {
                let pts = polygon_open_vertices(&ring.polygon);
                if pts.len() < 2 {
                    continue;
                }
                let target = ring_idx % machine_count;
                let mut arc = pts.clone();
                arc.push(pts[0]);
                result[target].push(arc);
            }
            result
        }
        HeadlandMode::Dedicated { machine } => {
            let target = machine.min(machine_count - 1);
            for ring in headlands {
                let pts = polygon_open_vertices(&ring.polygon);
                if pts.len() < 2 {
                    continue;
                }
                let mut arc = pts.clone();
                arc.push(pts[0]);
                result[target].push(arc);
            }
            result
        }
        HeadlandMode::SplitByZone => {
            split_headlands_into_arcs(headlands, swaths_per_machine, all_work_swaths)
        }
    }
}

/// Split each headland ring into arcs belonging to each machine's lateral
/// zone. The zone for machine i is defined by the midpoints between i's mean
/// lateral position and its neighbours' lateral positions (along the swath
/// normal). A machine with no assigned swaths contributes no zone.
fn split_headlands_into_arcs(
    headlands: &[Ring],
    swaths_per_machine: &[Vec<Swath>],
    all_work_swaths: &[Swath],
) -> Vec<Vec<HeadlandArc>> {
    let machine_count = swaths_per_machine.len();
    let mut result: Vec<Vec<HeadlandArc>> = vec![Vec::new(); machine_count];

    if machine_count == 0 || headlands.is_empty() || all_work_swaths.is_empty() {
        return result;
    }

    let tangent = dominant_swath_tangent(all_work_swaths);
    let normal = (-tangent.1, tangent.0);

    // Mean lateral position per machine. None for empty machines.
    let lats: Vec<Option<f64>> = swaths_per_machine
        .iter()
        .map(|swaths| {
            if swaths.is_empty() {
                None
            } else {
                let sum: f64 = swaths
                    .iter()
                    .map(|s| lateral_of(s.head(), s.tail(), normal))
                    .sum();
                Some(sum / swaths.len() as f64)
            }
        })
        .collect();

    // Order active machines by lateral position.
    let mut ordered: Vec<(usize, f64)> = lats
        .iter()
        .enumerate()
        .filter_map(|(idx, lat)| lat.map(|l| (idx, l)))
        .collect();
    ordered.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    if ordered.is_empty() {
        return result;
    }

    // Single active machine: give them every ring whole.
    if ordered.len() == 1 {
        let m = ordered[0].0;
        for ring in headlands {
            let points = polygon_open_vertices(&ring.polygon);
            if !points.is_empty() {
                let mut arc = points.clone();
                arc.push(points[0]); // close for continuity
                result[m].push(arc);
            }
        }
        return result;
    }

    // Zone boundaries between consecutive machines (midpoints of lat).
    let boundaries: Vec<f64> = ordered
        .windows(2)
        .map(|pair| (pair[0].1 + pair[1].1) * 0.5)
        .collect();

    for ring in headlands {
        let zone_arcs = split_ring_into_zone_arcs(&ring.polygon, &boundaries, normal);
        for (zone_idx, arcs) in zone_arcs.into_iter().enumerate() {
            let machine_idx = ordered[zone_idx].0;
            result[machine_idx].extend(arcs);
        }
    }

    result
}

fn lateral_of(head: Point, tail: Point, normal: (f64, f64)) -> f64 {
    let cx = (head.x() + tail.x()) * 0.5;
    let cy = (head.y() + tail.y()) * 0.5;
    cx * normal.0 + cy * normal.1
}

/// Split one ring polygon into arcs grouped by lateral zone.
/// Zone index 0 corresponds to the lowest lateral range (left of the first
/// boundary), zone `boundaries.len()` is the highest (right of the last).
fn split_ring_into_zone_arcs(
    polygon: &geo::Polygon,
    boundaries: &[f64],
    normal: (f64, f64),
) -> Vec<Vec<HeadlandArc>> {
    let num_zones = boundaries.len() + 1;
    let mut zone_arcs: Vec<Vec<HeadlandArc>> = vec![Vec::new(); num_zones];

    let points = polygon_open_vertices(polygon);
    if points.len() < 2 {
        return zone_arcs;
    }

    // Step 1: subdivide ring edges at boundary crossings so every edge lies
    // entirely in one zone (by midpoint).
    let mut subdivided: Vec<Point> = Vec::new();
    let n = points.len();
    for i in 0..n {
        let a = points[i];
        let b = points[(i + 1) % n];
        let la = a.x() * normal.0 + a.y() * normal.1;
        let lb = b.x() * normal.0 + b.y() * normal.1;
        subdivided.push(a);

        let denom = lb - la;
        let mut crossings: Vec<(f64, Point)> = Vec::new();
        if denom.abs() > 1e-12 {
            for &boundary in boundaries {
                if (la - boundary) * (lb - boundary) < 0.0 {
                    let t = (boundary - la) / denom;
                    if t > 1e-9 && t < 1.0 - 1e-9 {
                        let cx = a.x() + t * (b.x() - a.x());
                        let cy = a.y() + t * (b.y() - a.y());
                        crossings.push((t, Point::new(cx, cy)));
                    }
                }
            }
        }
        crossings.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
        for (_, p) in crossings {
            subdivided.push(p);
        }
    }

    // Step 2: walk subdivided edges; accumulate into arcs per zone.
    let m = subdivided.len();
    if m < 2 {
        return zone_arcs;
    }
    let mut current_zone: Option<usize> = None;
    let mut current_arc: HeadlandArc = Vec::new();

    for i in 0..m {
        let a = subdivided[i];
        let b = subdivided[(i + 1) % m];
        let mid_lat = ((a.x() + b.x()) * 0.5) * normal.0 + ((a.y() + b.y()) * 0.5) * normal.1;
        let zone = boundaries.partition_point(|&bd| bd < mid_lat);

        match current_zone {
            Some(z) if z == zone => {
                if current_arc
                    .last()
                    .map_or(true, |p| !points_equal(*p, a, 1e-9))
                {
                    current_arc.push(a);
                }
                current_arc.push(b);
            }
            _ => {
                if let Some(z) = current_zone.take() {
                    if current_arc.len() >= 2 {
                        zone_arcs[z].push(std::mem::take(&mut current_arc));
                    } else {
                        current_arc.clear();
                    }
                }
                current_arc = vec![a, b];
                current_zone = Some(zone);
            }
        }
    }
    if let Some(z) = current_zone {
        if current_arc.len() >= 2 {
            zone_arcs[z].push(current_arc);
        }
    }

    // Merge wrap-around: if the first and last arcs of a zone meet at the
    // same endpoint, stitch them into one polyline.
    for arcs in zone_arcs.iter_mut() {
        if arcs.len() >= 2 {
            let first_start = arcs[0].first().copied();
            let last_end = arcs.last().and_then(|arc| arc.last().copied());
            if let (Some(start), Some(end)) = (first_start, last_end) {
                if points_equal(start, end, 1e-6) {
                    let first = arcs.remove(0);
                    let last = arcs.last_mut().unwrap();
                    last.extend(first.into_iter().skip(1));
                }
            }
        }
    }

    zone_arcs
}

fn greedy_nearest_transit(swaths: &[Swath]) -> f64 {
    if swaths.len() < 2 {
        return 0.0;
    }
    let mut remaining: HashSet<usize> = (1..swaths.len()).collect();
    let mut previous_tail: Point = swaths[0].tail();
    let mut total = 0.0;

    while !remaining.is_empty() {
        let (next, dist, flip) = remaining
            .iter()
            .copied()
            .map(|idx| {
                let swath = &swaths[idx];
                let to_head = point_distance(previous_tail, swath.head());
                let to_tail = point_distance(previous_tail, swath.tail());
                if to_head <= to_tail {
                    (idx, to_head, false)
                } else {
                    (idx, to_tail, true)
                }
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();
        total += dist;
        remaining.remove(&next);
        let swath = &swaths[next];
        previous_tail = if flip { swath.head() } else { swath.tail() };
    }
    total
}

fn optimize_pattern(
    ordered: &[Swath],
    plan: &DivisionPlan,
    objective: OptimizeObjective,
) -> (Vec<Vec<Swath>>, DivisionPattern) {
    let mut candidates: Vec<DivisionPattern> = vec![
        DivisionPattern::Block,
        DivisionPattern::Stripe { stride: 1 },
    ];
    if plan.machine_count() >= 2 {
        candidates.push(DivisionPattern::BandedStripe { bands: 2 });
    }
    if plan.machine_count() >= 3 {
        candidates.push(DivisionPattern::BandedStripe { bands: 3 });
    }

    // For each pattern candidate, try both balance modes — but honour the
    // user's plan.balance as the caller's preference: we use it directly.
    // (We don't flip balance during Optimized to keep the result predictable
    // with their stated preference.)
    let mut best: Option<(f64, Vec<Vec<Swath>>, DivisionPattern)> = None;
    for pattern in candidates {
        let buckets = apply_pattern(ordered, plan, pattern);
        let cost = cost_for_objective(&buckets, plan, objective);
        let is_better = match best.as_ref() {
            Some((best_cost, _, _)) => cost + 1e-9 < *best_cost,
            None => true,
        };
        if is_better {
            best = Some((cost, buckets, pattern));
        }
    }

    let (_, buckets, pattern) = best.expect("at least one candidate");
    (buckets, pattern)
}

fn cost_for_objective(
    buckets: &[Vec<Swath>],
    plan: &DivisionPlan,
    objective: OptimizeObjective,
) -> f64 {
    let work_times: Vec<f64> = buckets
        .iter()
        .zip(plan.machines.iter())
        .map(|(swaths, profile)| total_length(swaths) / profile.speed)
        .collect();
    let transits: Vec<f64> = buckets.iter().map(|s| greedy_nearest_transit(s)).collect();
    let transit_times: Vec<f64> = transits
        .iter()
        .zip(plan.machines.iter())
        .map(|(transit, profile)| transit / profile.speed)
        .collect();
    let per_machine_time: Vec<f64> = work_times
        .iter()
        .zip(transit_times.iter())
        .map(|(work, transit)| work + transit)
        .collect();
    let makespan = per_machine_time
        .iter()
        .copied()
        .fold(0.0f64, |acc, x| acc.max(x));
    let total_transit: f64 = transits.iter().sum();

    match objective {
        OptimizeObjective::Makespan => makespan,
        OptimizeObjective::TotalTransit => total_transit,
        OptimizeObjective::Weighted {
            makespan: mw,
            transit: tw,
        } => (mw * makespan) + (tw * total_transit),
    }
}
