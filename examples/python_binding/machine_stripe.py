"""Stripe division demo — 3 machines each take every 3rd row.

Fair workload per machine but high total transit because each machine
criss-crosses the field. Logs to a Rerun recording named `maptrax_stripe_py`.
"""

import rerun as rr

import maptrax

from _common import (
    BIG_IRREGULAR_FIELD,
    DATUM,
    arc_polyline,
    machine_color,
    summarise_machines,
    swath_line,
)


def main():
    rr.init("maptrax_stripe_py", spawn=True)
    rr.log("enu/field/border", rr.LineStrips2D([BIG_IRREGULAR_FIELD + [BIG_IRREGULAR_FIELD[0]]]))

    planner = maptrax.Maptrax()
    planner.set_field(BIG_IRREGULAR_FIELD, DATUM)
    planner.generate_field(8.0, 20.0, 3)

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
        color = machine_color(machine["machine_index"])
        rr.log(
            f"enu/machines/m{machine['machine_index']}/swaths",
            rr.LineStrips2D(
                [swath_line(s) for s in machine["assigned_swaths"]], colors=[color]
            ),
        )
        rr.log(
            f"enu/machines/m{machine['machine_index']}/tour",
            rr.LineStrips2D(
                [swath_line(s) for s in machine["tour"]], colors=[color]
            ),
        )
        rr.log(
            f"enu/machines/m{machine['machine_index']}/headlands",
            rr.LineStrips2D(
                [arc_polyline(a) for a in machine["assigned_headland_arcs"]],
                colors=[color],
            ),
        )


if __name__ == "__main__":
    main()
