# maptrax

`maptrax` is the Rust port of `farmtrax`.

The public spatial interface now uses the sibling `datapod` geometry types:

- `datapod::Point`
- `datapod::Segment`
- `datapod::Polygon`
- `datapod::Aabb`

Current scope:

- field geometry with headlands and swaths
- work division strategies
- graph-backed swath traversal on top of `graphix`
- obstacle avoidance
- Dubins, Reeds-Shepp, and sharp-turn planners
- tour building
- high-level `Maptrax` facade

Planner stages exposed by the facade:

1. field generation
2. decomposition
3. obstacle avoidance
4. routing / swath ordering
5. tour building
6. multi-machine planning

Sibling local dependencies:

- `../graphix`
- `../concord`
- `../datapod`

## Install

```toml
[dependencies]
maptrax = { path = "../maptrax" }
```

## Example

```rust
use concord::Geo;
use datapod::Point;
use maptrax::{Field, polygon_from_points};

let field_polygon = polygon_from_points(vec![
    Point::new(0.0, 0.0),
    Point::new(100.0, 0.0),
    Point::new(100.0, 50.0),
    Point::new(0.0, 50.0),
]);

let mut field = Field::new(field_polygon, Geo::new(51.0, 5.0, 0.0)).expect("field");
field.gen_field(10.0, 90.0, 1).expect("generated");

assert!(!field.get_parts()[0].swaths.is_empty());
```

Runnable workflows live in [`examples/`](examples):

- `field_generation.rs`
- `planner_stages.rs`
- `division.rs`
- `traversal.rs`
- `obstacle_avoidance.rs`
- `facade_end_to_end.rs`
- `farmtrax_rerun.rs`
- `main_multi_obstacle.rs`
- `main_decomposed.rs`
- `main_machine_modes.rs`
- `c_abi/demo.c`
- `python_binding/basic.py`
- `python_binding/turners.py`
- `python_binding/rerun_demo.py`
- `python_binding/main.py`

## Bindings

C ABI surface:

- header: [`include/maptrax.h`](include/maptrax.h)
- Rust implementation: [`src/ffi.rs`](src/ffi.rs)
- example: [`examples/c_abi/demo.c`](examples/c_abi/demo.c)
- local makefile: [`examples/c_abi/Makefile`](examples/c_abi/Makefile)

Build and run the C example:

```sh
cargo build
cd examples/c_abi
make
```

### Python feature split

Two features, because pyo3 cannot serve both jobs at once:

| feature | pyo3 link mode | use |
|---|---|---|
| `python` | links `libpython` | `cargo test --features python` runs the bindings in-process |
| `python-extension` | `extension-module` — host interpreter resolves symbols | what maturin builds and ships |

`python-extension` implies `python`. A test binary cannot link an
`extension-module` build, so `tests/python_api.rs` and
`tests/python_geojson.rs` compile out when it is enabled — which keeps
`cargo test --all-features` working. `pyproject.toml` and both makefiles
already select `python-extension`; the dev shell exports `PYO3_PYTHON` and puts
libpython on `LD_LIBRARY_PATH` so the linked tests run.

Python surface:

- Python module implementation: [`src/python/`](src/python)
- packaging config: [`pyproject.toml`](pyproject.toml)
- examples: [`examples/python_binding/basic.py`](examples/python_binding/basic.py), [`examples/python_binding/turners.py`](examples/python_binding/turners.py), [`examples/python_binding/main.py`](examples/python_binding/main.py)
- local makefile: [`examples/python_binding/Makefile`](examples/python_binding/Makefile)

Build and install the Python module with `maturin`:

```sh
nix develop
cd examples/python_binding
make basic
make turners
make rerun
make main
```

The Python example makefile uses `PYO3_PYTHON` from the flake shell, creates a local `.venv`, and installs the extension into that environment so `pyo3`, `maturin`, and the runtime interpreter stay aligned.

## GeoJSON Export

Behind the optional `geojson` feature, plans are written as GeoJSON through the
sibling [`vectory`](https://codeberg.org/robolibs/vectory) crate. Maptrax plans
in local ENU metres; `vectory` handles the encoding and the ENU to WGS84
conversion through the field datum.

```toml
[dependencies]
maptrax = { path = "../maptrax", features = ["geojson"] }
```

```rust
use maptrax::{GeoJsonOptions, Maptrax, PlannerOptions};

# let mut planner = Maptrax::new();
# let options = PlannerOptions::default();
let planned = planner.plan_all(&options)?;

// Longitude/latitude by default.
planner.export_planned_geojson(&planned, "plan.geojson", &GeoJsonOptions::default())?;

// Border and headlands only, in raw local metres.
planner.export_geojson("field.geojson", &GeoJsonOptions::geometry_only().in_enu())?;
# Ok::<(), maptrax::MaptraxError>(())
```

Facade methods:

- `to_vector` — build a `vectory::Vector` without writing it
- `export_geojson` — field border, parts, headlands and rows
- `export_planned_geojson` — adds ordered rows and the drive path
- `export_machines_geojson` — adds per-machine rows, headland arcs and tours

Every feature carries a `type` property naming its layer:

| `type`          | geometry     | emitted for                            |
|-----------------|--------------|----------------------------------------|
| `field`         | `Polygon`    | the field border                       |
| `part_boundary` | `Polygon`    | each decomposed part                   |
| `headland`      | `Polygon`    | each headland ring                     |
| `swath`         | `LineString` | work rows, with an `order` property    |
| `tour`          | `LineString` | a full drive path, connectors included |
| `headland_arc`  | `LineString` | per-machine headland assignments       |

Machine-owned features additionally carry a `machine` property. Run the demo
with `cargo run --features geojson --example geojson_export`.

Each `export_*` method has a `*_to_geojson` counterpart returning the document
as a `String` instead of writing it — `to_geojson`, `planned_to_geojson`,
`machines_to_geojson`.

### From C

The C declarations are guarded by `MAPTRAX_GEOJSON`, since they only exist in a
library built with the feature. Build with `make -C examples/c_abi run
GEOJSON=1`, or by hand with `cargo build --features geojson` and
`cc -DMAPTRAX_GEOJSON ...`.

```c
MaptraxGeoJsonOptions geo = maptrax_geojson_options_default();

if (!maptrax_planner_export_geojson(planner, "field.geojson", geo)) {
  fprintf(stderr, "%s\n", maptrax_last_error_message());
}

char* text = maptrax_planner_to_geojson(planner, geo);   /* NULL on failure */
printf("%s\n", text);
maptrax_string_free(text);                               /* caller owns it */
```

`maptrax_planner_export_planned_geojson` and
`maptrax_planner_planned_to_geojson` take routing and turn options, plan the
part, and include the ordered rows and drive path.

### From Python

The extension is built with `--features python,geojson` (already wired into
`pyproject.toml` and both makefiles).

```python
import maptrax

mt = maptrax.Maptrax()
mt.set_field([(0, 0), (200, 0), (200, 100), (0, 100)], (51.0, 5.0, 0.0))
mt.generate_field(10.0, 90.0, 2)

text = mt.to_geojson()                      # str, WGS84
mt.export_geojson("field.geojson")
mt.export_geojson("local.geojson", crs="enu", swaths=False)

# Planned and multi-machine variants take a PlannerOptions.
plan = maptrax.DivisionPlan.uniform(3)
options = maptrax.PlannerOptions(machines=maptrax.MachinePlanningOptions(plan=plan))
mt.export_machines_geojson("machines.geojson", options=options)
```

`crs` accepts `"wgs"`/`"wgs84"`/`"epsg:4326"` or `"enu"`/`"local"`; anything
else raises `ValueError`. The layer toggles are the keyword arguments
`part_boundaries`, `headlands`, `swaths` and `tours`.

## Staged Planning

Use `Maptrax::plan_stages(...)` when you want explicit outputs from each planner stage instead of only the final ordered swaths and tour:

```rust
use concord::Geo;
use datapod::Point;
use maptrax::{
    FieldGenerationMode, FieldGenerationOptions, Maptrax, PlannerOptions, polygon_from_points,
};

let polygon = polygon_from_points(vec![
    Point::new(0.0, 0.0),
    Point::new(100.0, 0.0),
    Point::new(100.0, 50.0),
    Point::new(0.0, 50.0),
]);

let mut planner = Maptrax::new();
planner.set_field(polygon, Geo::new(51.0, 5.0, 0.0)).expect("field");

let planned = planner
    .plan_stages(&PlannerOptions {
        field: FieldGenerationOptions {
            swath_width: 10.0,
            headland_count: 1,
            mode: FieldGenerationMode::ExplicitAngle(90.0),
            ..FieldGenerationOptions::default()
        },
        ..PlannerOptions::default()
    })
    .expect("plan");

assert_eq!(planned.parts[0].headlands.len(), 1);
assert!(!planned.parts[0].generated_swaths.is_empty());
assert!(!planned.parts[0].ordered_swaths.is_empty());
assert!(!planned.parts[0].tour.is_empty());
```
