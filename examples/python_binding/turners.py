import math

import maptrax


planner = maptrax.Maptrax()

dubins = planner.plan_dubins((0.0, 0.0, 0.0), (12.0, 8.0, math.pi / 2.0), 2.5)
print("dubins:", dubins["name"], len(dubins["waypoints"]), dubins["total_length"])

rs = planner.plan_reeds_shepp((0.0, 0.0, 0.0), (-6.0, 10.0, math.pi), 3.0)
print("reeds_shepp:", rs["name"], len(rs["waypoints"]), rs["total_length"])

sharp = planner.plan_sharp_turn(
    (0.0, 0.0, 0.0),
    (0.0, 0.0, math.pi),
    2.0,
    machine_length=6.0,
    machine_width=3.0,
    pattern="auto",
)
print("sharp_turn:", sharp["name"], len(sharp["waypoints"]), sharp["total_length"])
