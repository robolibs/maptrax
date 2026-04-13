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


def swath_line(swath):
    return [[float(x), float(y)] for x, y in swath["points"]]


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

machine_plan = planner.plan_machines(
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

rr.init("maptrax_python_main", spawn=True)

rr.log("field/border", rr.LineStrips2D([FIELD + [FIELD[0]]]))
rr.log("field/obstacle", rr.LineStrips2D([obstacle + [obstacle[0]]]))

for machine in machine_plan["machines"]:
    color = rgb(machine["machine_index"])
    rr.log(
        f"machines/{machine['machine_index']}/avoided",
        rr.LineStrips2D([swath_line(swath) for swath in machine["avoided_swaths"]], colors=[color]),
    )
    rr.log(
        f"machines/{machine['machine_index']}/ordered",
        rr.LineStrips2D([swath_line(swath) for swath in machine["ordered_swaths"]], colors=[color]),
    )
    rr.log(
        f"machines/{machine['machine_index']}/tour",
        rr.LineStrips2D([swath_line(swath) for swath in machine["tour"]], colors=[color]),
    )

print(f"Field area: {planner.total_area():.1f} m^2")
for machine in machine_plan["machines"]:
    print(
        f"Machine {machine['machine_index']}: "
        f"assigned={len(machine['assigned_swaths'])}, "
        f"assigned_avoided={len(machine['avoided_swaths'])}, "
        f"nety={len(machine['ordered_swaths'])}"
    )
