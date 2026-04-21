use std::collections::HashSet;

use geo::Point;

use crate::core::{MaptraxError, Result, point_distance, segment_length};
use crate::field::{Part, Ring, Swath, SwathType, canonical_swath_order};

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
}

impl DivisionPlan {
    /// Convenience: N machines with uniform profiles, default balance/pattern.
    pub fn uniform(machines: usize, pattern: DivisionPattern, balance: Balance) -> Self {
        Self {
            pattern,
            balance,
            machines: MachineProfile::uniform(machines),
        }
    }

    pub fn machine_count(&self) -> usize {
        self.machines.len()
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct DivisionResult {
    /// The pattern that was actually applied. Matches `plan.pattern` except
    /// for `Optimized`, where this reports the winning concrete pattern.
    pub pattern_used: Option<DivisionPattern>,
    pub swaths_per_machine: Vec<Vec<Swath>>,
    pub headlands_per_machine: Vec<Vec<Ring>>,
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
            DivisionPattern::Optimized { objective } => {
                optimize_pattern(&ordered, plan, objective)
            }
            pattern => (apply_pattern(&ordered, plan, pattern), pattern),
        };

        let headlands_per_machine = split_headlands(&headlands, plan.machine_count());

        let estimated_work_time = swaths_per_machine
            .iter()
            .zip(plan.machines.iter())
            .map(|(swaths, profile)| total_length(swaths) / profile.speed)
            .collect();

        let estimated_transit = swaths_per_machine
            .iter()
            .map(|swaths| greedy_nearest_transit(swaths))
            .collect();

        Ok(DivisionResult {
            pattern_used: Some(pattern_used),
            swaths_per_machine,
            headlands_per_machine,
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
        DivisionPattern::BandedStripe { bands } => banded_stripe_assign(ordered, plan, bands.max(1)),
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

fn banded_stripe_assign(
    ordered: &[Swath],
    plan: &DivisionPlan,
    bands: usize,
) -> Vec<Vec<Swath>> {
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

fn split_headlands(headlands: &[Ring], machine_count: usize) -> Vec<Vec<Ring>> {
    let mut buckets = vec![Vec::new(); machine_count];
    for (index, ring) in headlands.iter().enumerate() {
        buckets[index % machine_count].push(ring.clone());
    }
    buckets
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
        OptimizeObjective::Weighted { makespan: mw, transit: tw } => {
            (mw * makespan) + (tw * total_transit)
        }
    }
}
