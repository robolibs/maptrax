import math

import maptrax


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

print("ordered_swaths:", len(plan["ordered_swaths"]))
print("tour_segments:", len(plan["tour"]))

rs = planner.plan_reeds_shepp(
    (0.0, 0.0, 0.0),
    (0.0, 18.0, math.pi),
    4.0,
)
print("reeds_shepp_name:", rs["name"])
print("reeds_shepp_waypoints:", len(rs["waypoints"]))
print("reeds_shepp_length:", rs["total_length"])
