"""Optimized division demo — planner enumerates candidates, picks the best.

Scores Block, Stripe{1}, BandedStripe{2}, BandedStripe{3} by makespan
(work + transit) and reports the winner. Logs the winner + every
candidate to Rerun `maptrax_optimized_py` in both 2D and map views.
"""

import rerun as rr

import maptrax

from _common import (
    BIG_IRREGULAR_FIELD,
    DATUM,
    log_machine_views,
    machine_color,
    polygon_geo,
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

    planner = maptrax.Maptrax()
    planner.set_field(BIG_IRREGULAR_FIELD, DATUM)
    planner.generate_field(8.0, 20.0, 3)

    rr.log(
        "enu/field/border",
        rr.LineStrips2D([BIG_IRREGULAR_FIELD + [BIG_IRREGULAR_FIELD[0]]]),
    )
    rr.log(
        "geo/field/border",
        rr.GeoLineStrings(lat_lon=[polygon_geo(planner, BIG_IRREGULAR_FIELD)]),
    )

    scored = []
    for label, kwargs in CANDIDATES:
        plan = _plan_with(planner, kwargs)
        work_times = plan["estimated_work_time"]
        transits = plan["estimated_transit"]
        makespan = max(w + t for w, t in zip(work_times, transits))
        total_transit = sum(transits)
        scored.append((label, kwargs, plan, makespan, total_transit))

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

    for machine in winner_plan["machines"]:
        idx = machine["machine_index"]
        log_machine_views(rr, planner, f"winner/m{idx}", machine, machine_color(idx))

    for label, _kwargs, plan, _makespan, _transit in scored:
        slug = label.lower().replace("{", "").replace("}", "").replace(":", "")
        for machine in plan["machines"]:
            idx = machine["machine_index"]
            log_machine_views(
                rr,
                planner,
                f"candidates/{slug}/m{idx}",
                machine,
                machine_color(idx),
            )


if __name__ == "__main__":
    main()
