"""AutoSplit + 3-machine fleet demo.

The big irregular field is auto-split into sub-fields (longer AABB side
larger than max_side triggers a bisect). Each sub-field then runs the
same 3-machine Block+ByLength plan; each physical machine's tour is the
concatenation of its per-sub-field shares.

One sub-field OWNS the midline headland (its rings extend all the way to
the split). The other sub-field's headland rings skip the inset along
the split — its swaths reach the split line directly and use the
neighbour's headland for turns.

Logs to `maptrax_split_field_py`.
"""

import rerun as rr

import maptrax

from _common import (
    BIG_IRREGULAR_FIELD,
    DATUM,
    arc_polyline,
    machine_color,
    segment_length,
    swath_line,
)


FLEET_SIZE = 3
MAX_SIDE = 520.0
SWATH_WIDTH = 8.0
HEADLAND_COUNT = 2


def ring_polyline(ring):
    points = [[float(x), float(y)] for x, y in ring["points"]]
    if points and points[0] != points[-1]:
        points.append(points[0])
    return points


def main():
    rr.init("maptrax_split_field_py", spawn=True)
    rr.log("enu/field/border", rr.LineStrips2D([BIG_IRREGULAR_FIELD + [BIG_IRREGULAR_FIELD[0]]]))

    planner = maptrax.Maptrax()
    planner.set_field(BIG_IRREGULAR_FIELD, DATUM)
    part_count = planner.plan_field(
        swath_width=SWATH_WIDTH,
        angle_degrees=20.0,
        headland_count=HEADLAND_COUNT,
        decomposition=f"auto_split:{MAX_SIDE}",
    )

    print(f"=== AutoSplit {{ max_side: {MAX_SIDE} }} + {FLEET_SIZE}-machine fleet ===")
    print("   Field extent: ~950m x ~495m (irregular 8-vertex polygon)")
    print(f"   Split into {part_count} sub-field(s).\n")

    # Headland rings per sub-field. Owner rings close around the split;
    # non-owner rings have their split-side snapped to the split line.
    for part_index in range(part_count):
        part = planner.get_part(part_index)
        for ring_index, ring in enumerate(part["headlands"]):
            rr.log(
                f"enu/parts/part_{part_index}/headland_{ring_index}",
                rr.LineStrips2D([ring_polyline(ring)], colors=[(120, 120, 160)]),
            )
        non_owned = part["non_owned_splits"]
        if non_owned:
            print(
                f"   part {part_index} is NON-OWNER of {len(non_owned)} split(s) — "
                f"swaths reach: {', '.join(f'{s[\"axis\"]}={s[\"position\"]:.1f}' for s in non_owned)}"
            )
        else:
            print(f"   part {part_index} OWNS its splits (midline headlands inset fully)")

    all_plans = planner.plan_machines_all_parts(
        machines=FLEET_SIZE,
        pattern="block",
        balance="length",
        routing_strategy="greedy_nearest",
        local_improvement_passes=1,
        turn_model="reeds_shepp",
        min_turning_radius=3.0,
        swath_width=SWATH_WIDTH,
        machine_length=6.0,
        machine_width=3.0,
    )

    # Concat each physical machine's work across sub-fields.
    per_machine_swaths = [[] for _ in range(FLEET_SIZE)]
    per_machine_tour = [[] for _ in range(FLEET_SIZE)]
    per_machine_headlands = [[] for _ in range(FLEET_SIZE)]

    for part_index, part_plan in enumerate(all_plans):
        for machine in part_plan["machines"]:
            idx = machine["machine_index"]
            per_machine_swaths[idx].extend(machine["assigned_swaths"])
            per_machine_tour[idx].extend(machine["tour"])
            per_machine_headlands[idx].extend(machine["assigned_headland_arcs"])

            color = machine_color(idx)
            rr.log(
                f"enu/parts/part_{part_index}/m{idx}/swaths",
                rr.LineStrips2D(
                    [swath_line(s) for s in machine["assigned_swaths"]],
                    colors=[color],
                ),
            )

    print()
    print(
        "   {:<8} {:>8}  {:>10}  {:>10}".format(
            "machine", "swaths", "work_len", "work_s"
        )
    )
    makespan = 0.0
    total_work = 0.0
    for idx in range(FLEET_SIZE):
        work_len = sum(
            segment_length(s["points"][0], s["points"][-1])
            for s in per_machine_swaths[idx]
        )
        work_s = work_len
        print(
            "   {:<8} {:>8}  {:>10.1f}  {:>10.1f}".format(
                f"m{idx}", len(per_machine_swaths[idx]), work_len, work_s
            )
        )
        makespan = max(makespan, work_s)
        total_work += work_len

        color = machine_color(idx)
        rr.log(
            f"enu/machines/m{idx}/swaths",
            rr.LineStrips2D(
                [swath_line(s) for s in per_machine_swaths[idx]], colors=[color]
            ),
        )
        rr.log(
            f"enu/machines/m{idx}/tour",
            rr.LineStrips2D(
                [swath_line(s) for s in per_machine_tour[idx]], colors=[color]
            ),
        )
        rr.log(
            f"enu/machines/m{idx}/headlands",
            rr.LineStrips2D(
                [arc_polyline(a) for a in per_machine_headlands[idx]],
                colors=[color],
            ),
        )

    print(f"\n   Total work length: {total_work:.1f} m")
    print(f"   Makespan: {makespan:.1f}s   ({FLEET_SIZE} machines across {part_count} sub-field(s))")


if __name__ == "__main__":
    main()
