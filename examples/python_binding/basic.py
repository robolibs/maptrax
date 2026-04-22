import math

import maptrax


planner = maptrax.Maptrax()
planner.set_field(
    [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)],
    (51.0, 5.0, 0.0),
)
planner.generate_field(10.0, 90.0, 1)
print("part_count:", planner.part_count())
print("total_area:", planner.total_area())

part = planner.get_part(0)
print("headlands:", len(part["headlands"]))
print("generated_swaths:", len(part["swaths"]))

stages = planner.plan_stages(
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
print("staged_generated_swaths:", len(stages["generated_swaths"]))
print("staged_tour_segments:", len(stages["tour"]))

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

routed = planner.route_part(part_index=0, routing_strategy="greedy_nearest")
print("routed_swaths:", len(routed))

rs = planner.plan_reeds_shepp(
    (0.0, 0.0, 0.0),
    (0.0, 18.0, math.pi),
    4.0,
)
print("reeds_shepp_name:", rs["name"])
print("reeds_shepp_waypoints:", len(rs["waypoints"]))
print("reeds_shepp_length:", rs["total_length"])

dubins_paths = planner.plan_all_dubins((0.0, 0.0, 0.0), (15.0, 0.0, 0.0), 2.0)
print("dubins_path_families:", len(dubins_paths))
