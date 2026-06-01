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
