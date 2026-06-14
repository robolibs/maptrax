"""Large four-corner irregular-field combine turning demo.

Run with Rerun:

    make -C examples/python_binding combine-irregular-turning

Headless:

    MAPTRAX_NO_RERUN=1 make -C examples/python_binding combine-irregular-turning
"""

from __future__ import annotations

import os

import maptrax

# Reuse the small visualization helpers from the square demo. Importing that
# file is safe: it only runs its own demo under `if __name__ == "__main__"`.
from combine_square_turning import (
    closed_line,
    collect_turn_points,
    count_tour_gaps,
    swath_line,
    turn_point_crosses,
)


# This is still only four field corners, but it is not a square/rectangle.
# Area is about 75,000 m², roughly 5.2x the 120m x 120m square demo.
FIELD = [
    (0.0, 0.0),
    (310.0, 25.0),
    (285.0, 265.0),
    (-35.0, 230.0),
]

SWATH_WIDTH_M = 9.0
HEADLAND_RINGS = 3
SWATH_ANGLE_DEG = 86.0

COMBINE_LENGTH_M = 9.0
COMBINE_WIDTH_M = 4.0
COMBINE_MIN_TURN_RADIUS_M = 8.0

DATUM = (51.0, 5.0, 0.0)


def main() -> None:
    planner = maptrax.Maptrax()
    planner.set_field(FIELD, DATUM)
    planner.generate_field(SWATH_WIDTH_M, SWATH_ANGLE_DEG, HEADLAND_RINGS)

    planned = planner.plan_stages(
        part_index=0,
        routing_strategy="turn_radius_aware",
        local_improvement_passes=0,
        # Reeds-Shepp keeps this irregular-field reference continuous. A
        # forward-only Dubins turn is stricter, but on this skewed 4-corner
        # field some row transitions are simply infeasible inside the safe
        # headland corridor.
        turn_model="reeds_shepp",
        connector_mode="headland",
        min_turning_radius=COMBINE_MIN_TURN_RADIUS_M,
        machine_length=COMBINE_LENGTH_M,
        machine_width=COMBINE_WIDTH_M,
        swath_width=SWATH_WIDTH_M,
    )

    feasibility = planner.turn_feasibility(
        swath_width=SWATH_WIDTH_M,
        headland_count=HEADLAND_RINGS,
        turn_model="reeds_shepp",
        min_turning_radius=COMBINE_MIN_TURN_RADIUS_M,
        machine_length=COMBINE_LENGTH_M,
        machine_width=COMBINE_WIDTH_M,
    )

    combine = maptrax.TurnPlannerConfig(
        model=maptrax.TurnPlannerModel.REEDS_SHEPP,
        connector_mode=maptrax.ConnectorMode.HEADLAND,
        min_turning_radius=COMBINE_MIN_TURN_RADIUS_M,
        machine_length=COMBINE_LENGTH_M,
        machine_width=COMBINE_WIDTH_M,
        swath_width=SWATH_WIDTH_M,
    )

    turn_points = collect_turn_points(planned["tour"])
    gap_count, max_gap = count_tour_gaps(planned["tour"])

    maybe_log_rerun(planned, turn_points)

    print("=== python irregular 4-corner field + combine turning reference ===")
    print("field corners:")
    for x, y in FIELD:
        print(f"  ({x:.1f}, {y:.1f})")
    print("field scale: ~5.2x square demo area")
    print(f"swath width: {SWATH_WIDTH_M:.1f}m")
    print(f"swath angle: {SWATH_ANGLE_DEG:.1f}deg")
    print(f"headland rings: {HEADLAND_RINGS}")
    print("turn model: reeds_shepp")
    print(
        "combine: "
        f"length={COMBINE_LENGTH_M:.1f}m "
        f"width={COMBINE_WIDTH_M:.1f}m "
        f"min_turn_radius={COMBINE_MIN_TURN_RADIUS_M:.1f}m"
    )
    print(f"turning envelope radius: {combine.turning_envelope_radius():.2f}m")
    print(f"feasibility recovery stride: {feasibility['row_skip_stride']}")
    print(f"turn-radius-aware routing stride: {combine.required_row_skip_stride()}")
    print(f"generated swaths: {len(planned['generated_swaths'])}")
    print(f"tour segments: {len(planned['tour'])}")
    print(f"tour continuity gaps: {gap_count} (max {max_gap:.3f}m)")
    print(f"turning point markers: {len(turn_points)}")
    print("Rerun layers:")
    print("  enu/field_irregular_four_point")
    print("  enu/headlands_irregular")
    print("  enu/generated_swaths_irregular")
    print("  enu/final_tour_irregular")
    print("  enu/turning_points_yellow_crosses_irregular")


def maybe_log_rerun(planned: dict, turn_points: list[tuple[float, float]]) -> None:
    if os.environ.get("MAPTRAX_NO_RERUN") == "1":
        return

    import rerun as rr

    spawn = os.environ.get("MAPTRAX_RERUN_SPAWN", "1").lower() not in {
        "0",
        "false",
        "no",
    }
    rr.init("maptrax_python_combine_irregular_turning", spawn=spawn)

    rr.log(
        "enu/field_irregular_four_point",
        rr.LineStrips2D([closed_line(FIELD)], colors=[(180, 90, 80)]),
    )

    headland_lines = [closed_line(ring["points"]) for ring in planned["headlands"]]
    if headland_lines:
        rr.log(
            "enu/headlands_irregular",
            rr.LineStrips2D(
                headland_lines,
                colors=[(100, 100, 150)] * len(headland_lines),
            ),
        )

    generated = [swath_line(swath) for swath in planned["generated_swaths"]]
    if generated:
        rr.log(
            "enu/generated_swaths_irregular",
            rr.LineStrips2D(generated, colors=[(80, 120, 230)] * len(generated)),
        )

    tour = [swath_line(swath) for swath in planned["tour"]]
    if tour:
        rr.log(
            "enu/final_tour_irregular",
            rr.LineStrips2D(tour, colors=[(230, 120, 40)] * len(tour)),
        )

    crosses = turn_point_crosses(turn_points, radius=2.0)
    if crosses:
        rr.log(
            "enu/turning_points_yellow_crosses_irregular",
            rr.LineStrips2D(crosses, colors=[(255, 230, 50)] * len(crosses)),
        )


if __name__ == "__main__":
    main()
