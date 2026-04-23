"""Shared helpers for the Python multi-machine demos.

All demos use the same irregular ~950m x 495m polygon that the Rust
examples use, so their output can be compared 1:1.
"""

DATUM = (51.98954034749562, 5.6584737410504715, 53.801823)

BIG_IRREGULAR_FIELD = [
    (-40.0, 90.0),
    (180.0, 0.0),
    (620.0, 30.0),
    (890.0, 180.0),
    (860.0, 420.0),
    (520.0, 495.0),
    (150.0, 450.0),
    (-60.0, 310.0),
]


def swath_line(swath):
    """Extract a Rerun-friendly [[x, y], ...] polyline from a swath dict."""
    return [[float(x), float(y)] for x, y in swath["points"]]


def arc_polyline(arc):
    return [[float(x), float(y)] for x, y in arc]


def swath_line_geo(planner, swath):
    """Convert a swath's points from ENU to (lat, lon) tuples for
    `rr.GeoLineStrings`."""
    latlon = planner.enu_to_wgs_batch([(float(x), float(y)) for x, y in swath["points"]])
    return [[lat, lon] for (lat, lon) in latlon]


def arc_polyline_geo(planner, arc):
    latlon = planner.enu_to_wgs_batch([(float(x), float(y)) for x, y in arc])
    return [[lat, lon] for (lat, lon) in latlon]


def polygon_geo(planner, xy_points):
    """Convert a closed polygon (list of (x, y)) into a lat/lon ring
    suitable for `rr.GeoLineStrings`."""
    closed = list(xy_points) + [xy_points[0]] if xy_points and xy_points[0] != xy_points[-1] else list(xy_points)
    latlon = planner.enu_to_wgs_batch([(float(x), float(y)) for x, y in closed])
    return [[lat, lon] for (lat, lon) in latlon]


def log_machine_views(rr, planner, prefix, machine, color):
    """Log a single machine's swaths / tour / headlands in BOTH 2D (ENU)
    and map (geo) views. `prefix` is a path prefix like "machines/m0"."""
    swaths = machine["assigned_swaths"]
    tour = machine["tour"]
    arcs = machine["assigned_headland_arcs"]

    if swaths:
        rr.log(
            f"enu/{prefix}/swaths",
            rr.LineStrips2D([swath_line(s) for s in swaths], colors=[color]),
        )
        rr.log(
            f"geo/{prefix}/swaths",
            rr.GeoLineStrings(
                lat_lon=[swath_line_geo(planner, s) for s in swaths],
                colors=[color] * len(swaths),
            ),
        )
    if tour:
        rr.log(
            f"enu/{prefix}/tour",
            rr.LineStrips2D([swath_line(s) for s in tour], colors=[color]),
        )
        rr.log(
            f"geo/{prefix}/tour",
            rr.GeoLineStrings(
                lat_lon=[swath_line_geo(planner, s) for s in tour],
                colors=[color] * len(tour),
            ),
        )
    if arcs:
        rr.log(
            f"enu/{prefix}/headlands",
            rr.LineStrips2D([arc_polyline(a) for a in arcs], colors=[color]),
        )
        rr.log(
            f"geo/{prefix}/headlands",
            rr.GeoLineStrings(
                lat_lon=[arc_polyline_geo(planner, a) for a in arcs],
                colors=[color] * len(arcs),
            ),
        )


MACHINE_PALETTE = [
    (230, 90, 90),
    (70, 180, 120),
    (90, 140, 230),
    (220, 170, 60),
    (200, 100, 200),
    (160, 160, 160),
]


def machine_color(index: int):
    return MACHINE_PALETTE[index % len(MACHINE_PALETTE)]


def summarise_machines(plan):
    """Pretty-print the per-machine line of a plan_machines dict."""
    print(
        "   {:<8} {:>8}  {:>10}  {:>10}  {:>10}".format(
            "machine", "swaths", "work_len", "work_s", "transit_m"
        )
    )
    makespan = 0.0
    total_work = 0.0
    for index, machine in enumerate(plan["machines"]):
        work_len = sum(
            segment_length(swath["points"][0], swath["points"][-1])
            for swath in machine["assigned_swaths"]
        )
        work_s = plan["estimated_work_time"][index]
        transit = plan["estimated_transit"][index]
        print(
            "   {:<8} {:>8}  {:>10.1f}  {:>10.1f}  {:>10.1f}".format(
                f"m{machine['machine_index']}",
                len(machine["assigned_swaths"]),
                work_len,
                work_s,
                transit,
            )
        )
        makespan = max(makespan, work_s)
        total_work += work_len
    return makespan, total_work


def segment_length(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
