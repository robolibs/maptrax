"""Oxbo pea harvester field example from editor/game coordinates.

Run:

    make -C examples/python_binding oxbo-pea-field

The example:

* uses the supplied local X/Z polygon as the field border,
* requests at least 3 headland rings,
* uses 3.0 m swaths for the Oxbo pea harvester working width,
* uses an 8.0 m minimum turning radius for the final route,
* uses the forward-only Dubins turner for the final route,
* writes a matplotlib overview to /tmp/field.png.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import maptrax

from combine_square_turning import (
    closed_line,
    collect_turn_points,
    count_tour_gaps,
    swath_line,
)


FIELD_EDITOR_ID = "us30"
FIELD_GAME_ID = "us2"

# The editor/world origin supplied with the field. Point 1 in FIELD_LOCAL_XZ is
# at this world X/Z. Maptrax plans in local metres, so the planner receives the
# local polygon while the summary also prints the translated world coordinates.
WORLD_ORIGIN_XZ = (-898.152, 304.127)

# Supplied point table, interpreted as local editor metres: x -> ENU x,
# z -> ENU y. The source used comma decimal separators; keep normal Python
# floats here so the example is runnable.
FIELD_LOCAL_XZ = [
    (0.000, 0.000),
    (49.139, -133.215),
    (47.742, -145.517),
    (40.790, -158.575),
    (28.694, -168.923),
    (9.992, -176.288),
    (-12.004, -180.854),
    (-71.666, -187.089),
    (-78.418, -184.486),
    (-81.270, -178.043),
    (-76.745, -58.889),
    (-74.087, -41.836),
    (-66.872, -27.341),
    (-56.061, -17.720),
]

# User-requested Oxbo pea harvester assumptions.
SWATH_WIDTH_M = 3.0
MACHINE_WIDTH_M = 3.0
MACHINE_LENGTH_M = 9.0
MIN_TURNING_RADIUS_M = 8.0
TURN_MODEL = "dubins"
DUBINS_STEP_SIZE_M = 0.75
DUBINS_ALLOWED_PATHS = {"LSL", "RSR", "LSR", "RSL"}

# "At least 3" headlands. With an 8 m turn radius the feasibility helper will
# auto-increase this to the safe effective count when needed.
REQUESTED_HEADLAND_RINGS = 3

# Close to the field's long axis, while still giving the forward-only Dubins
# turner enough skipped-row spacing for continuous 8 m-radius turns.
SWATH_ANGLE_DEG = 100.0

# The map/geodesy datum is required by the facade, but this plot is local X/Z.
DATUM = (51.0, 5.0, 0.0)
PLOT_PATH = Path("/tmp/field.png")
CSV_PATH = Path("/tmp/field.csv")


def main() -> None:
    planner = maptrax.Maptrax()
    planner.set_field(FIELD_LOCAL_XZ, DATUM)

    feasibility = planner.generate_field_feasible(
        swath_width=SWATH_WIDTH_M,
        angle_degrees=SWATH_ANGLE_DEG,
        headland_count=REQUESTED_HEADLAND_RINGS,
        turn_model=TURN_MODEL,
        min_turning_radius=MIN_TURNING_RADIUS_M,
        machine_length=MACHINE_LENGTH_M,
        machine_width=MACHINE_WIDTH_M,
        headland_policy="auto_increase",
    )

    part = planner.get_part(0)
    generated_swaths = list(part["swaths"])
    route, turn_path_names = build_forward_only_dubins_route(
        generated_swaths,
        row_skip_stride=max(1, int(feasibility["row_skip_stride"])),
    )
    planned = {
        "headlands": list(part["headlands"]),
        "generated_swaths": generated_swaths,
        "tour": route,
    }

    turn_points = collect_turn_points(planned["tour"])
    gap_count, max_gap = count_tour_gaps(planned["tour"])

    save_csv(planned, CSV_PATH)
    save_plot(planned, turn_points, feasibility, PLOT_PATH)
    print_summary(
        planner,
        planned,
        turn_points,
        gap_count,
        max_gap,
        feasibility,
        turn_path_names,
    )


def build_forward_only_dubins_route(
    swaths: list[dict],
    row_skip_stride: int,
) -> tuple[list[dict], list[str]]:
    """Order swaths with skip rows and connect them with the Dubins turner.

    This deliberately uses `maptrax.Dubins`, not Reeds-Shepp. Dubins has no
    reverse gear. We also prefer CSC paths (LSL/RSR/LSR/RSL) and avoid the
    loopier CCC Dubins families unless there is no alternative.
    """

    turner = maptrax.Dubins(MIN_TURNING_RADIUS_M)
    ordered_indices = skip_row_order(len(swaths), row_skip_stride)
    route: list[dict] = []
    turn_path_names: list[str] = []
    current_pose: tuple[float, float, float] | None = None

    for route_index, swath_index in enumerate(ordered_indices):
        candidates = []
        for reversed_direction in (False, True):
            oriented = orient_swath(swaths[swath_index], reversed_direction, route_index)
            start_pose = pose_at_start(oriented)
            if current_pose is None:
                candidates.append((0.0, None, oriented))
                continue

            paths = turner.all_paths(current_pose, start_pose, DUBINS_STEP_SIZE_M)
            allowed_paths = [path for path in paths if path.name in DUBINS_ALLOWED_PATHS]
            for path in allowed_paths or paths:
                candidates.append((path.total_length, path, oriented))

        candidates.sort(key=lambda item: item[0])
        _, turn_path, oriented_swath = candidates[0]

        if turn_path is not None:
            turn_path_names.append(turn_path.name)
            route.append(dubins_connection_to_swath(turn_path))

        route.append(oriented_swath)
        current_pose = pose_at_end(oriented_swath)

    return route, turn_path_names


def skip_row_order(swath_count: int, stride: int) -> list[int]:
    return [
        index
        for offset in range(stride)
        for index in range(offset, swath_count, stride)
    ]


def orient_swath(swath: dict, reversed_direction: bool, route_index: int) -> dict:
    points = swath_line(swath)
    if reversed_direction:
        points = list(reversed(points))
    return {
        **swath,
        "id": route_index,
        "type": "swath",
        "points": points,
        "point_reverse": [False] * len(points),
    }


def pose_at_start(swath: dict) -> tuple[float, float, float]:
    points = swath["points"]
    return (float(points[0][0]), float(points[0][1]), heading(points))


def pose_at_end(swath: dict) -> tuple[float, float, float]:
    points = swath["points"]
    return (float(points[-1][0]), float(points[-1][1]), heading(points))


def heading(points: list[list[float]]) -> float:
    start = points[0]
    end = points[-1]
    return math.atan2(float(end[1]) - float(start[1]), float(end[0]) - float(start[0]))


def dubins_connection_to_swath(path) -> dict:
    points = [[float(x), float(y)] for x, y, _yaw in path.waypoints]
    return {
        "type": "connection",
        "id": -1,
        "width": SWATH_WIDTH_M,
        "points": points,
        "point_reverse": [False] * len(points),
        "turner": "dubins",
        "turn_path": path.name,
    }


def save_csv(
    planned: dict,
    output_path: Path,
) -> None:
    """Write the full route as exactly two CSV columns: x,y."""

    with output_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["x", "y"])
        for point in full_route_points(planned["tour"]):
            writer.writerow([f"{point[0]:.6f}", f"{point[1]:.6f}"])


def full_route_points(tour: list[dict]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for segment in tour:
        for x, y in segment["points"]:
            point = (float(x), float(y))
            if not points or route_point_distance(points[-1], point) > 1e-9:
                points.append(point)
    return points


def route_point_distance(
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def save_plot(
    planned: dict,
    turn_points: list[tuple[float, float]],
    feasibility: dict,
    output_path: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9.0, 10.5))

    boundary = closed_line(FIELD_LOCAL_XZ)
    bx = [p[0] for p in boundary]
    bz = [p[1] for p in boundary]
    ax.fill(bx, bz, color="#d7efd0", alpha=0.55, label="field")
    ax.plot(bx, bz, color="#3b2a1a", linewidth=2.2)

    for index, ring in enumerate(planned["headlands"]):
        line = closed_line(ring["points"])
        xs = [p[0] for p in line]
        zs = [p[1] for p in line]
        ax.plot(
            xs,
            zs,
            color="#6f56b3",
            linewidth=1.4,
            alpha=0.9,
            label="headlands" if index == 0 else None,
        )

    for index, swath in enumerate(planned["generated_swaths"]):
        line = swath_line(swath)
        xs = [p[0] for p in line]
        zs = [p[1] for p in line]
        ax.plot(
            xs,
            zs,
            color="#7c8798",
            linewidth=0.8,
            alpha=0.55,
            label="AB swaths, 3 m" if index == 0 else None,
        )

    for index, segment in enumerate(planned["tour"]):
        line = swath_line(segment)
        xs = [p[0] for p in line]
        zs = [p[1] for p in line]
        is_connection = segment["type"] == "connection"
        ax.plot(
            xs,
            zs,
            color="#d9480f" if is_connection else "#0b7285",
            linewidth=1.2 if is_connection else 1.0,
            alpha=0.82,
            label=(
                "Dubins 8 m turns, no reverse"
                if is_connection and not any(s["type"] == "connection" for s in planned["tour"][:index])
                else ("full route swaths" if index == 0 else None)
            ),
        )

    if turn_points:
        ax.scatter(
            [p[0] for p in turn_points],
            [p[1] for p in turn_points],
            marker="x",
            s=28,
            color="#f08c00",
            linewidths=1.0,
            label="turn markers",
        )

    start = planned["tour"][0]["points"][0]
    end = planned["tour"][-1]["points"][-1]
    ax.scatter([start[0]], [start[1]], color="#2f9e44", s=65, zorder=5, label="start")
    ax.scatter([end[0]], [end[1]], color="#c92a2a", s=65, zorder=5, label="end")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#d0d0d0", linewidth=0.6, alpha=0.7)
    ax.set_xlabel("local/editor x (m)")
    ax.set_ylabel("local/editor z (m)")
    ax.set_title(
        "Oxbo pea field route, Dubins no-reverse turner"
        f" — {SWATH_WIDTH_M:.0f} m swaths,"
        f" {feasibility['effective_headland_count']} headlands,"
        f" {MIN_TURNING_RADIUS_M:.0f} m min turn"
    )
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def print_summary(
    planner: maptrax.Maptrax,
    planned: dict,
    turn_points: list[tuple[float, float]],
    gap_count: int,
    max_gap: float,
    feasibility: dict,
    turn_path_names: list[str],
) -> None:
    print("=== Oxbo pea harvester field example ===")
    print(f"field in editor: {FIELD_EDITOR_ID}")
    print(f"field in game: {FIELD_GAME_ID}")
    print(
        "world/editor origin for local point 1: "
        f"x={WORLD_ORIGIN_XZ[0]:.3f}, z={WORLD_ORIGIN_XZ[1]:.3f}"
    )
    print(f"local polygon points: {len(FIELD_LOCAL_XZ)}")
    print(f"field area: {planner.total_area():.1f} m²")
    print(f"swath/working width: {SWATH_WIDTH_M:.1f} m")
    print(f"swath angle: {SWATH_ANGLE_DEG:.1f} deg")
    print(f"requested headlands: {REQUESTED_HEADLAND_RINGS}")
    print(f"effective headlands: {feasibility['effective_headland_count']}")
    print(f"required headlands for 8 m turn: {feasibility['required_headland_count']}")
    print("turner: Dubins (forward-only, no reverse)")
    print(
        "machine: "
        f"width={MACHINE_WIDTH_M:.1f} m, "
        f"length={MACHINE_LENGTH_M:.1f} m, "
        f"min_turn_radius={MIN_TURNING_RADIUS_M:.1f} m"
    )
    print(f"AB swaths generated: {len(planned['generated_swaths'])}")
    print(f"full-route segments: {len(planned['tour'])}")
    print(f"turning point markers: {len(turn_points)}")
    print(f"reverse route waypoints: {count_reverse_waypoints(planned['tour'])}")
    print(f"Dubins turn segments: {len(turn_path_names)}")
    print(f"Dubins loop-family turns (RLR/LRL): {count_loop_family_turns(turn_path_names)}")
    print(f"tour continuity gaps: {gap_count} (max {max_gap:.3f} m)")
    print(f"plot: {PLOT_PATH}")
    print(f"csv: {CSV_PATH}")

    if feasibility["warnings"]:
        print("feasibility warnings:")
        for warning in feasibility["warnings"]:
            print(f"  - {warning}")

    print("first/last world X/Z:")
    first = to_world(FIELD_LOCAL_XZ[0])
    last = to_world(FIELD_LOCAL_XZ[-1])
    print(f"  first: x={first[0]:.3f}, z={first[1]:.3f}")
    print(f"  last:  x={last[0]:.3f}, z={last[1]:.3f}")


def to_world(local_xz: tuple[float, float]) -> tuple[float, float]:
    return (
        WORLD_ORIGIN_XZ[0] + local_xz[0],
        WORLD_ORIGIN_XZ[1] + local_xz[1],
    )


def count_reverse_waypoints(tour: list[dict]) -> int:
    return sum(sum(1 for value in swath.get("point_reverse", []) if value) for swath in tour)


def count_loop_family_turns(turn_path_names: list[str]) -> int:
    return sum(1 for name in turn_path_names if name in {"RLR", "LRL"})


if __name__ == "__main__":
    main()
