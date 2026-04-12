import math

import maptrax
import rerun as rr


def line_strip(swath):
    return [[float(x), float(y)] for x, y in swath["points"]]


planner = maptrax.Maptrax()
planner.set_field(
    [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)],
    (51.0, 5.0, 0.0),
)
planner.generate_field(10.0, 90.0, 1)

plan = planner.plan_part(
    part_index=0,
    routing_strategy="greedy_nearest",
    turn_model="reeds_shepp",
    connector_mode="auto",
    min_turning_radius=2.0,
    step_size=0.2,
    machine_length=6.0,
    machine_width=3.0,
    swath_width=10.0,
)

dubins = planner.plan_dubins((0.0, 0.0, 0.0), (12.0, 8.0, math.pi / 2.0), 2.5)
reeds_shepp = planner.plan_reeds_shepp((0.0, 0.0, 0.0), (0.0, 18.0, math.pi), 4.0)

rr.init("maptrax_python_demo", spawn=True)

rr.log(
    "field/border",
    rr.LineStrips2D([[(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0), (0.0, 0.0)]]),
)
rr.log(
    "planner/ordered_swaths",
    rr.LineStrips2D([line_strip(swath) for swath in plan["ordered_swaths"]]),
)
rr.log(
    "planner/tour",
    rr.LineStrips2D([line_strip(swath) for swath in plan["tour"]]),
)
rr.log(
    "turners/dubins",
    rr.LineStrips2D([[list(point[:2]) for point in dubins["waypoints"]]]),
)
rr.log(
    "turners/reeds_shepp",
    rr.LineStrips2D([[list(point[:2]) for point in reeds_shepp["waypoints"]]]),
)

print("logged ordered_swaths:", len(plan["ordered_swaths"]))
print("logged tour_segments:", len(plan["tour"]))
print("dubins_waypoints:", len(dubins["waypoints"]))
print("reeds_shepp_waypoints:", len(reeds_shepp["waypoints"]))
