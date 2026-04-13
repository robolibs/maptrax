import rerun as rr

import maptrax


DATUM = (51.98954034749562, 5.6584737410504715, 53.801823)
FIELD = [
    (85.7, -209.6),
    (201.6, -152.9),
    (109.3, 34.4),
    (230.0, 97.4),
    (121.5, 170.9),
    (-249.8, -6.7),
    (-173.5, -153.4),
]


def centered_obstacle(border, half_size):
    xs = [point[0] for point in border]
    ys = [point[1] for point in border]
    center_x = (min(xs) + max(xs)) * 0.5
    center_y = (min(ys) + max(ys)) * 0.5
    return [
        (center_x - half_size, center_y - half_size),
        (center_x + half_size, center_y - half_size),
        (center_x + half_size, center_y + half_size),
        (center_x - half_size, center_y + half_size),
    ]


def line_strip(points):
    return [[float(x), float(y)] for x, y in points]


def swath_line(swath):
    return line_strip(swath["points"])


def ring_line(ring):
    points = ring["points"]
    if not points:
        return []
    return line_strip(points + [points[0]])


def rgb(index):
    palette = [
        (220, 90, 90),
        (70, 170, 120),
        (70, 130, 210),
        (210, 170, 60),
    ]
    return palette[index % len(palette)]


planner = maptrax.Maptrax()
planner.set_field(FIELD, DATUM)
planner.generate_field(4.0, 0.0, 3)

obstacle = centered_obstacle(FIELD, 25.0)
part = planner.get_part(0)
stages = planner.plan_stages(
    part_index=0,
    obstacle_polygons=[obstacle],
    inflation_distance=2.0,
    routing_strategy="greedy_nearest",
    local_improvement_passes=0,
    turn_model="reeds_shepp",
    connector_mode="auto",
    min_turning_radius=2.0,
    step_size=0.2,
    machine_length=6.0,
    machine_width=3.0,
    swath_width=4.0,
)
routed = planner.route_part(
    part_index=0,
    obstacle_polygons=[obstacle],
    inflation_distance=2.0,
    routing_strategy="greedy_nearest",
    local_improvement_passes=0,
)
machines = planner.plan_machines(
    part_index=0,
    machines=4,
    division_type="alternate",
    obstacle_polygons=[obstacle],
    inflation_distance=2.0,
    routing_strategy="greedy_nearest",
    turn_model="reeds_shepp",
    min_turning_radius=2.0,
    swath_width=4.0,
)

rr.init("maptrax_python_main_extended", spawn=True)

rr.log("field/border", rr.LineStrips2D([FIELD + [FIELD[0]]]))
rr.log("field/obstacle", rr.LineStrips2D([obstacle + [obstacle[0]]], colors=[(220, 70, 70)]))

rr.log("part/boundary", rr.LineStrips2D([ring_line(part["boundary"])]))
rr.log(
    "part/headlands",
    rr.LineStrips2D([ring_line(ring) for ring in part["headlands"]], colors=[(180, 180, 180)]),
)
rr.log(
    "stages/transit_rings",
    rr.LineStrips2D(
        [ring_line(ring) for ring in stages["transit_rings"]],
        colors=[(255, 140, 0)],
    ),
)
rr.log(
    "stages/generated",
    rr.LineStrips2D([swath_line(swath) for swath in stages["generated_swaths"]], colors=[(140, 140, 140)]),
)
rr.log(
    "stages/avoided",
    rr.LineStrips2D([swath_line(swath) for swath in stages["avoided_swaths"]], colors=[(230, 60, 60)]),
)
rr.log(
    "stages/ordered",
    rr.LineStrips2D([swath_line(swath) for swath in routed], colors=[(70, 130, 210)]),
)
rr.log(
    "stages/tour",
    rr.LineStrips2D([swath_line(swath) for swath in stages["tour"]], colors=[(50, 200, 120)]),
)

for machine in machines["machines"]:
    color = rgb(machine["machine_index"])
    rr.log(
        f"machines/{machine['machine_index']}/tour",
        rr.LineStrips2D([swath_line(swath) for swath in machine["tour"]], colors=[color]),
    )

print(f"Field area: {planner.total_area():.1f} m^2")
print(f"Part count: {planner.part_count()}")
print(f"Headlands: {len(part['headlands'])}")
print(f"Generated swaths: {len(part['swaths'])}")
print(f"Avoided swaths: {len(stages['avoided_swaths'])}")
print(f"Ordered swaths: {len(routed)}")
print(f"Tour segments: {len(stages['tour'])}")
for machine in machines["machines"]:
    print(
        f"Machine {machine['machine_index']}: "
        f"assigned={len(machine['assigned_swaths'])}, "
        f"avoided={len(machine['avoided_swaths'])}, "
        f"ordered={len(machine['ordered_swaths'])}, "
        f"tour={len(machine['tour'])}"
    )
