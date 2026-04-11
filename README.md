# maptrax

`maptrax` is the Rust port of `farmtrax`.

The public spatial interface now uses [`geo`](https://github.com/georust/geo):

- `geo::Point<f64>`
- `geo::Line<f64>`
- `geo::Polygon<f64>`
- `geo::Rect<f64>`

Current scope:

- field geometry with headlands and swaths
- work division strategies
- graph-backed swath traversal on top of `graphix`
- obstacle avoidance
- Dubins, Reeds-Shepp, and sharp-turn planners
- tour building
- high-level `Maptrax` facade

Sibling local dependencies:

- `../graphix_rs`
- `../concord_rs`

## Install

```toml
[dependencies]
maptrax = { path = "../maptrax_rs" }
```

## Example

```rust
use concord::Geo;
use geo::Point;
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
- `division.rs`
- `traversal.rs`
- `obstacle_avoidance.rs`
- `facade_end_to_end.rs`
