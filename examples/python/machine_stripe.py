"""Stripe division demo — 3 machines each take every 3rd row.

Logs to Rerun recording `maptrax_stripe_py` with both 2D (ENU) and
map (geo) views.
"""

import rerun as rr

import maptrax

from _common import (
    BIG_IRREGULAR_FIELD,
    DATUM,
    log_machine_views,
    machine_color,
    polygon_geo,
    summarise_machines,
)


def main():
    rr.init("maptrax_stripe_py", spawn=True)

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

    plan = planner.plan_machines(
        part_index=0,
        machines=3,
        pattern="stripe",
        balance="count",
        stride=1,
        routing_strategy="greedy_nearest",
        local_improvement_passes=1,
        turn_model="reeds_shepp",
        min_turning_radius=3.0,
        swath_width=8.0,
        machine_length=6.0,
        machine_width=3.0,
    )

    print("=== Stripe {stride: 1} + ByCount — 3 machines ===")
    print("   Each machine takes every 3rd row; work fair, transit high.\n")
    makespan, total = summarise_machines(plan)
    print(f"\n   makespan: {makespan:.1f}s   total_work: {total:.1f} m")

    for machine in plan["machines"]:
        idx = machine["machine_index"]
        log_machine_views(rr, planner, f"machines/m{idx}", machine, machine_color(idx))


if __name__ == "__main__":
    main()
