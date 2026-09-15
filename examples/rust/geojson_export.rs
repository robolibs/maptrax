//! GeoJSON export through the sibling `vectory` crate.
//!
//! Plans a field, then writes three documents to `target/`:
//!
//!   * `field.geojson`    — border, headlands and rows, in WGS84
//!   * `planned.geojson`  — the same plus the ordered rows and drive path
//!   * `machines.geojson` — a three-machine split, each element tagged
//!
//! Run:
//!
//!   cargo run --features geojson --example geojson_export

use std::fs::create_dir_all;

use maptrax::{
    Balance, DivisionPattern, DivisionPlan, FieldGenerationMode, FieldGenerationOptions, Geo,
    GeoJsonOptions, MachinePlanningOptions, Maptrax, PlannerOptions, point_xy, polygon_from_points,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    create_dir_all("target")?;

    // A 300x160 m field somewhere in the Netherlands.
    let border = polygon_from_points(vec![
        point_xy(0.0, 0.0),
        point_xy(300.0, 0.0),
        point_xy(300.0, 160.0),
        point_xy(0.0, 160.0),
    ]);
    let datum = Geo::new(51.9851, 5.6639, 0.0);

    let mut planner = Maptrax::new();
    planner.set_field(border, datum)?;

    let field_options = FieldGenerationOptions {
        swath_width: 12.0,
        headland_count: 2,
        mode: FieldGenerationMode::ExplicitAngle(90.0),
        ..FieldGenerationOptions::default()
    };

    // 1. Field geometry only.
    planner.generate_field(
        field_options.swath_width,
        90.0,
        field_options.headland_count,
    )?;
    planner.export_geojson("target/field.geojson", &GeoJsonOptions::default())?;
    println!("wrote target/field.geojson");

    // 2. The full plan: ordered rows plus the drive path.
    let planned = planner.plan_all(&PlannerOptions {
        field: field_options.clone(),
        ..PlannerOptions::default()
    })?;
    planner.export_planned_geojson(
        &planned,
        "target/planned.geojson",
        &GeoJsonOptions::default(),
    )?;
    let rows: usize = planned.parts.iter().map(|p| p.ordered_swaths.len()).sum();
    println!("wrote target/planned.geojson ({rows} rows)");

    // 3. Three machines splitting the same part.
    let machines = planner.plan_machines_for_part(
        &MachinePlanningOptions {
            plan: DivisionPlan::uniform(3, DivisionPattern::Block, Balance::ByLength),
            part_index: 0,
        },
        Default::default(),
        &Default::default(),
    )?;
    planner.export_machines_geojson(
        &machines,
        "target/machines.geojson",
        &GeoJsonOptions::default(),
    )?;
    for machine in &machines.machines {
        println!(
            "  machine {}: {} rows, {} tour segments",
            machine.machine_index,
            machine.ordered_swaths.len(),
            machine.tour.len()
        );
    }
    println!("wrote target/machines.geojson");

    // Local metres instead of degrees, when the consumer is not a map.
    planner.export_geojson(
        "target/field_enu.geojson",
        &GeoJsonOptions::default().in_enu(),
    )?;
    println!("wrote target/field_enu.geojson (local ENU metres)");

    Ok(())
}
