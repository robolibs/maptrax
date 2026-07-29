"""Tiny square-field combine turning demo for the Python binding.

Default run opens/logs to Rerun so the turn pockets are visible:

    make -C examples/python_binding combine-square-turning

For headless validation without Rerun:

    MAPTRAX_NO_RERUN=1 make -C examples/python_binding combine-square-turning
"""

from __future__ import annotations

import os

# This imports the local PyO3 extension built by `maturin develop` in the
# Makefile. The example intentionally exercises the public Python API rather
# than calling Rust-only helpers.
import maptrax


# ---------------------------------------------------------------------------
# Field setup
# ---------------------------------------------------------------------------
#
# Keep the field deliberately boring: a 120m x 120m square in local ENU metres.
# That makes it easy to visually judge whether headlands and turn pockets are
# being placed where expected.
FIELD_SIZE_M = 120.0

# A 6m swath is a plausible combine/header width and keeps the example small
# enough to inspect without creating hundreds of lines.
SWATH_WIDTH_M = 6.0

# Four rings means the headland band is 24m deep. That gives this large combine
# room to pick a turning lane inside the headland band instead of right on the
# crop edge.
HEADLAND_RINGS = 3

# Generic large combine reference values. Conservative, not brand-specific.
# These values feed both routing and turn generation:
# - length/width define the cheap machine envelope,
# - min turn radius defines how much row spacing the forward-only turn needs.
COMBINE_LENGTH_M = 9.0
COMBINE_WIDTH_M = 4.0
COMBINE_MIN_TURN_RADIUS_M = 8.0

# Datum is only required because the maptrax facade models fields as ENU
# relative to a WGS84 origin. This demo only draws ENU, so the exact lat/lon is
# not important.
DATUM = (51.0, 5.0, 0.0)

# Counter-clockwise square boundary. The helper closes it for visualization.
FIELD = [
    (0.0, 0.0),
    (FIELD_SIZE_M, 0.0),
    (FIELD_SIZE_M, FIELD_SIZE_M),
    (0.0, FIELD_SIZE_M),
]


def main() -> None:
    # Create the Python facade and load the basic square field.
    planner = maptrax.Maptrax()
    planner.set_field(FIELD, DATUM)

    # Generate headlands and interior swaths. `90.0` means north/south rows in
    # this simple ENU square, so the final tour is easy to read.
    planner.generate_field(SWATH_WIDTH_M, 90.0, HEADLAND_RINGS)

    # Ask for the full staged plan so the example can visualize:
    # - generated headland rings,
    # - generated interior swaths,
    # - final ordered tour with connection/turn segments inserted.
    planned = planner.plan_stages(
        part_index=0,

        # This strategy uses the combine turn radius to decide whether rows
        # should be skipped instead of forcing impossible adjacent-row turns.
        routing_strategy="turn_radius_aware",
        local_improvement_passes=0,

        # Dubins = forward-only. That is the stricter combine-harvester
        # reference because it does not rely on reversing inside the headland.
        turn_model="dubins",
        connector_mode="headland",

        # The machine geometry here is the key part of this demo: longer/wider
        # machines get a larger envelope and therefore choose safer turn points.
        min_turning_radius=COMBINE_MIN_TURN_RADIUS_M,
        machine_length=COMBINE_LENGTH_M,
        machine_width=COMBINE_WIDTH_M,
        swath_width=SWATH_WIDTH_M,
    )

    # Feasibility is a quick diagnostic: it explains the row-skip/headland
    # requirement for this machine without changing the field.
    feasibility = planner.turn_feasibility(
        swath_width=SWATH_WIDTH_M,
        headland_count=HEADLAND_RINGS,
        turn_model="dubins",
        min_turning_radius=COMBINE_MIN_TURN_RADIUS_M,
        machine_length=COMBINE_LENGTH_M,
        machine_width=COMBINE_WIDTH_M,
    )

    # Build a Python TurnPlannerConfig only to expose the same envelope number
    # and row-skip stride the Rust planner uses internally. This is printed
    # below for inspection.
    combine = maptrax.TurnPlannerConfig(
        model=maptrax.TurnPlannerModel.DUBINS,
        connector_mode=maptrax.ConnectorMode.HEADLAND,
        min_turning_radius=COMBINE_MIN_TURN_RADIUS_M,
        machine_length=COMBINE_LENGTH_M,
        machine_width=COMBINE_WIDTH_M,
        swath_width=SWATH_WIDTH_M,
    )

    # Extract representative points from every connection segment. They are not
    # separate "official" planner outputs; they are visual markers that make the
    # turn-pocket positions obvious in Rerun.
    turn_points = collect_turn_points(planned["tour"])
    gap_count, max_gap = count_tour_gaps(planned["tour"])

    # If MAPTRAX_NO_RERUN=1 is set, skip visualization and only print counts.
    # This keeps CI/headless validation cheap.
    maybe_log_rerun(planned, turn_points)

    # Console summary mirrors the visible Rerun layers so this can be used even
    # when no viewer is available.
    print("=== python square field + combine turning reference ===")
    print(f"field: {FIELD_SIZE_M:.0f}m x {FIELD_SIZE_M:.0f}m")
    print(f"swath width: {SWATH_WIDTH_M:.1f}m")
    print(f"headland rings: {HEADLAND_RINGS}")
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
    print("  enu/field_square")
    print("  enu/headlands")
    print("  enu/generated_swaths")
    print("  enu/final_tour")
    print("  enu/turning_points_yellow_crosses")


def maybe_log_rerun(planned: dict, turn_points: list[tuple[float, float]]) -> None:
    """Send the plan to Rerun unless the caller explicitly disabled it."""

    # Useful for tests and remote shells where opening a viewer is not wanted.
    if os.environ.get("MAPTRAX_NO_RERUN") == "1":
        return

    # Import lazily so headless mode does not require NumPy/Rerun to import.
    import rerun as rr

    # Default to spawning/opening a viewer. Set MAPTRAX_RERUN_SPAWN=0 when a
    # viewer is already running and you only want to connect/log.
    spawn = os.environ.get("MAPTRAX_RERUN_SPAWN", "1").lower() not in {
        "0",
        "false",
        "no",
    }
    rr.init("maptrax_python_combine_square_turning", spawn=spawn)

    # Red square outline: the actual field boundary.
    rr.log(
        "enu/field_square",
        rr.LineStrips2D([closed_line(FIELD)], colors=[(180, 90, 80)]),
    )

    # Purple rings: generated headland lanes. These are the turning workspace.
    headland_lines = [closed_line(ring["points"]) for ring in planned["headlands"]]
    if headland_lines:
        rr.log(
            "enu/headlands",
            rr.LineStrips2D(
                headland_lines,
                colors=[(100, 100, 150)] * len(headland_lines),
            ),
        )

    # Blue lines: raw generated working swaths before routing inserts turns.
    generated = [swath_line(swath) for swath in planned["generated_swaths"]]
    if generated:
        rr.log(
            "enu/generated_swaths",
            rr.LineStrips2D(generated, colors=[(80, 120, 230)] * len(generated)),
        )

    # Orange lines: final tour, including both working swaths and connection
    # swaths produced by the turn planner.
    tour = [swath_line(swath) for swath in planned["tour"]]
    if tour:
        rr.log(
            "enu/final_tour",
            rr.LineStrips2D(tour, colors=[(230, 120, 40)] * len(tour)),
        )

    # Yellow crosses: sampled turn/connection points so the chosen pockets pop
    # visually on top of the final tour.
    crosses = turn_point_crosses(turn_points, radius=1.2)
    if crosses:
        rr.log(
            "enu/turning_points_yellow_crosses",
            rr.LineStrips2D(crosses, colors=[(255, 230, 50)] * len(crosses)),
        )


def collect_turn_points(tour: list[dict]) -> list[tuple[float, float]]:
    """Collect visible marker points from connection swaths in the final tour."""

    points: list[tuple[float, float]] = []
    for swath in tour:
        # `generated_swaths` are real field rows; `connection` segments are the
        # inserted turns/transits between rows.
        if swath["type"] != "connection":
            continue
        pts = swath["points"]
        if len(pts) < 2:
            continue

        # Mark start, middle, and end. This makes curved/longer connection
        # paths visible even if they overlap headland lines.
        push_unique(points, pts[0])
        push_unique(points, pts[len(pts) // 2])
        push_unique(points, pts[-1])
    return points


def count_tour_gaps(tour: list[dict]) -> tuple[int, float]:
    """Return how many consecutive tour segments fail to touch."""

    gaps = 0
    max_gap = 0.0
    for prev, nxt in zip(tour, tour[1:]):
        gap = distance(swath_tail(prev), swath_head(nxt))
        max_gap = max(max_gap, gap)
        if gap > 1e-6:
            gaps += 1
    return gaps, max_gap


def swath_head(swath: dict) -> tuple[float, float]:
    """First visible point of a swath/connection."""

    pts = swath.get("points") or []
    if pts:
        return (float(pts[0][0]), float(pts[0][1]))
    head = swath["head"]
    return (float(head[0]), float(head[1]))


def swath_tail(swath: dict) -> tuple[float, float]:
    """Last visible point of a swath/connection."""

    pts = swath.get("points") or []
    if pts:
        return (float(pts[-1][0]), float(pts[-1][1]))
    tail = swath["tail"]
    return (float(tail[0]), float(tail[1]))


def push_unique(points: list[tuple[float, float]], point: tuple[float, float]) -> None:
    """Append a marker unless another marker is already almost on top of it."""

    p = (float(point[0]), float(point[1]))

    # Many adjacent connection swaths share endpoints. Deduping keeps the Rerun
    # layer readable instead of drawing a pile of identical crosses.
    if all(distance(existing, p) > 0.5 for existing in points):
        points.append(p)


def turn_point_crosses(
    points: list[tuple[float, float]],
    radius: float,
) -> list[list[list[float]]]:
    """Convert each point into two small polylines that look like a cross."""

    out: list[list[list[float]]] = []
    for x, y in points:
        # Horizontal stroke.
        out.append([[x - radius, y], [x + radius, y]])

        # Vertical stroke.
        out.append([[x, y - radius], [x, y + radius]])
    return out


def closed_line(points: list[tuple[float, float]]) -> list[list[float]]:
    """Return a Rerun LineStrips2D-ready closed polyline."""

    line = [[float(x), float(y)] for x, y in points]

    # Rerun line strips are open by default, so explicitly repeat the first
    # point when drawing polygons/rings.
    if line and line[0] != line[-1]:
        line.append(line[0])
    return line


def swath_line(swath: dict) -> list[list[float]]:
    """Return a Rerun LineStrips2D-ready open polyline for one swath."""

    # Python bindings expose swaths as dictionaries with `points = [(x, y), ...]`.
    return [[float(x), float(y)] for x, y in swath["points"]]


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Euclidean distance in local ENU metres."""

    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


if __name__ == "__main__":
    # Keep import side effects minimal; only run the demo when invoked directly.
    main()
