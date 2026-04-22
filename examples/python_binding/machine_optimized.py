"""Optimized division demo — planner enumerates candidates, picks the best.

Scores Block, Stripe{1}, BandedStripe{2}, BandedStripe{3} by makespan
(work + transit) and reports the winner. Logs only the winner to Rerun
plus all candidates under `enu/candidates/...`.

Logs to `maptrax_optimized_py`.
"""

import rerun as rr

import maptrax

from _common import (
    BIG_IRREGULAR_FIELD,
    DATUM,
    arc_polyline,
    machine_color,
    swath_line,
)


CANDIDATES = [
    ("Block", {"pattern": "block"}),
    ("Stripe{1}", {"pattern": "stripe", "stride": 1}),
    ("BandedStripe{2}", {"pattern": "banded_stripe", "bands": 2}),
    ("BandedStripe{3}", {"pattern": "banded_stripe", "bands": 3}),
]


def _plan_with(planner, extra_kwargs):
    return planner.plan_machines(
        part_index=0,
        machines=3,
        balance="length",
        routing_strategy="greedy_nearest",
        local_improvement_passes=1,
        turn_model="reeds_shepp",
        min_turning_radius=3.0,
        swath_width=8.0,
        machine_length=6.0,
        machine_width=3.0,
        **extra_kwargs,
    )


def main():
    rr.init("maptrax_optimized_py", spawn=True)
    rr.log("enu/field/border", rr.LineStrips2D([BIG_IRREGULAR_FIELD + [BIG_IRREGULAR_FIELD[0]]]))

    planner = maptrax.Maptrax()
    planner.set_field(BIG_IRREGULAR_FIELD, DATUM)
    planner.generate_field(8.0, 20.0, 3)

    # Score every candidate by running it explicitly.
    scored = []
    for label, kwargs in CANDIDATES:
        plan = _plan_with(planner, kwargs)
        work_times = plan["estimated_work_time"]
        transits = plan["estimated_transit"]
        makespan = max(w + t for w, t in zip(work_times, transits))
        total_transit = sum(transits)
        scored.append((label, kwargs, plan, makespan, total_transit))

    # Let Maptrax pick the winner under the Optimized pattern.
    winner_plan = _plan_with(planner, {"pattern": "optimized_makespan"})
    winner_pattern = winner_plan.get("pattern_used", "Unknown")

    print("=== Optimized { Makespan } + ByLength — 3 machines ===")
    print("   Objective: minimise max(work_time + transit_time) across machines.\n")
    print("   Candidate scores:")
    print("   {:<18} {:>12} {:>14}".format("pattern", "makespan_s", "total_transit_m"))
    for label, _kwargs, _plan, makespan, total_transit in scored:
        marker = " ← winner" if winner_pattern.startswith(label.split("{")[0].rstrip()) else ""
        print(
            "   {:<18} {:>12.1f} {:>14.1f}{}".format(label, makespan, total_transit, marker)
        )
    print(f"\n   Winner: {winner_pattern}")

    # Render the winner's per-machine swaths + tour.
    for machine in winner_plan["machines"]:
        color = machine_color(machine["machine_index"])
        rr.log(
            f"enu/winner/m{machine['machine_index']}/swaths",
            rr.LineStrips2D(
                [swath_line(s) for s in machine["assigned_swaths"]], colors=[color]
            ),
        )
        rr.log(
            f"enu/winner/m{machine['machine_index']}/tour",
            rr.LineStrips2D(
                [swath_line(s) for s in machine["tour"]], colors=[color]
            ),
        )
        rr.log(
            f"enu/winner/m{machine['machine_index']}/headlands",
            rr.LineStrips2D(
                [arc_polyline(a) for a in machine["assigned_headland_arcs"]],
                colors=[color],
            ),
        )

    # Also push every candidate under enu/candidates/... for side-by-side view.
    for label, _kwargs, plan, _makespan, _transit in scored:
        slug = label.lower().replace("{", "").replace("}", "").replace(":", "")
        for machine in plan["machines"]:
            color = machine_color(machine["machine_index"])
            rr.log(
                f"enu/candidates/{slug}/m{machine['machine_index']}/swaths",
                rr.LineStrips2D(
                    [swath_line(s) for s in machine["assigned_swaths"]],
                    colors=[color],
                ),
            )


if __name__ == "__main__":
    main()
