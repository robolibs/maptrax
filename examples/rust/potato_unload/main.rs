//! Chaser tractor for a potato harvest on Koen's parcel, stepped in real time
//! into Rerun with a map underneath, then written out as GeoJSON and KML.
//!
//! Inputs are the three shapefiles in `koen.rs`: the boundary, the first
//! ridge as an AB line 0.75 m in from the south edge, and the headland line
//! the tractor centres on, 2 m from the west ditch. Entry is the south-west
//! corner.
//!
//! The route planning here is the *tractor's*. The harvester is a dot: it
//! runs the rows, drives the west lane between them, turns however it likes,
//! and all we take from it is where it is and which rows it has finished.
//! The harvested block is the only ground wide enough to turn a 12 m rig
//! with an 8 m radius, and the rig cannot reverse, so every turn is a
//! forward Dubins loop inside that block.
//!
//! One call, as sketched:
//!   1. up the lane, loop round inside the block,
//!   2. sit in the bay beside the lane facing south, clear of the lane,
//!   3. the harvester comes down the lane and stops alongside; take the
//!      load, merge back onto the lane, south to the entry to tip.
//!
//! `HARVEST=middle` (default) opens the field middle-out, alternating sides.
//! `HARVEST=edge` eats in from the first ridge at the south edge instead.
//!
//! Start a viewer, then run:
//!
//!   rerun
//!   cargo run --features geojson --example potato_unload
//!
//! `SPEEDUP=40` to slow it down, `SPEEDUP=0` to run flat out,
//! `RERUN_SAVE=x.rrd` to record instead of streaming. The drive log lands in
//! `target/potato_unload.{geojson,kml}`.

mod viz;

mod kml;
mod koen;
mod rig;

use std::collections::HashMap;
use std::error::Error;
use std::time::Duration;

use concord::{Wgs, to_enu};
use maptrax::core::{Segment, segment_end, segment_new, segment_start};
use maptrax::{
    Crs, Dubins, Field, Geo, GeoJsonOptions, Point, Point2Ext, Polygon, Pose2D, field_to_vector,
    point_xy, polygon_exterior_points, polygon_from_points, write_vector,
};

use rerun::{RecordingStream, RecordingStreamBuilder};

use koen::RIDGE_WIDTH;

const HEADLAND_WIDTH: f64 = 4.0;

const HARVESTER_SPEED: f64 = 1.4;
const HARVESTER_LANE_SPEED: f64 = 2.0;
const HARVESTER_BODY: f64 = 3.0;

const TRACTOR_SPEED: f64 = 4.0;
const TRACTOR_RADIUS: f64 = 8.0;
const TRACTOR_LENGTH: f64 = 12.0;
const TRACTOR_BODY: f64 = 2.55;

/// Both rigs are a tractor towing a unit: tractor, drawbar to the towed
/// unit's centre, and the unit itself add up to the 12 m.
const TRACTOR_UNIT: f64 = 4.5;
const DRAWBAR: f64 = 4.5;
const TOWED_UNIT: f64 = 6.0;
/// Rig extent ahead of and behind the tractor's centre, which is the pose
/// the planner drives.
const RIG_NOSE: f64 = TRACTOR_UNIT * 0.5;
const RIG_TAIL: f64 = TRACTOR_UNIT * 0.5 + DRAWBAR + TOWED_UNIT * 0.5;
/// Tractor centre to trailer centre; the harvester stops level with this.
const TRAILER_BEHIND: f64 = TRACTOR_UNIT * 0.5 + DRAWBAR;

/// The tractor is in the bay this long before the harvester comes off its
/// row, so the lane is clear when the harvester turns onto it.
const LEAD_SECONDS: f64 = 30.0;

/// How far into its row the harvester must be before the tractor pulls out
/// onto the lane: its own towed unit is still on the lane until then.
const CLEARANCE: f64 = RIG_TAIL + TOWED_UNIT * 0.5 + 4.0;

/// Rig centre sits this far east of the lane in the bay: half a rig, half a
/// harvester, and daylight between them.
const BAY_OFFSET: f64 = TRACTOR_BODY * 0.5 + HARVESTER_BODY * 0.5 + 0.6;

/// 50 t/ha over a 1.5 m ridge is 7.5 kg per metre driven.
const YIELD_T_PER_HA: f64 = 50.0;
const BUNKER_T: f64 = 6.0;
const TRANSFER_SECONDS: f64 = 95.0;

const STEP: f64 = 0.5;
const TICK: f64 = 2.0;
/// Rows already off when we pick up the story; `PRE_OPENED=n` overrides.
const PRE_OPENED: usize = 28;
const RUN_FOR: f64 = 4.0 * 3600.0;

const C_CROP: (u8, u8, u8) = (0x4e, 0x7a, 0x35);
const C_DONE: (u8, u8, u8) = (0xa8, 0x94, 0x68);
const C_BORDER: (u8, u8, u8) = (0xe6, 0xe9, 0xdd);
const C_HEADLAND: (u8, u8, u8) = (0x6c, 0x72, 0x64);
const C_LANE: (u8, u8, u8) = (0xf2, 0xc1, 0x4e);
const C_HARVESTER: (u8, u8, u8) = (0xb8, 0x3b, 0x2e);
const C_DIGGER: (u8, u8, u8) = (0xe0, 0x7a, 0x3a);
const C_TRACTOR: (u8, u8, u8) = (0x2f, 0x7f, 0xb8);
const C_TRAILER: (u8, u8, u8) = (0x6f, 0xb7, 0xe6);
const C_PLAN: (u8, u8, u8) = (0xf2, 0xc1, 0x4e);

/// The field's own axes: `along` runs east up a ridge, `cross` runs north
/// across them. Everything is expressed in these, so the parcel's skew off
/// north costs nothing.
#[derive(Clone, Copy)]
struct Frame {
    origin: Point,
    dir: (f64, f64),
    nrm: (f64, f64),
}

impl Frame {
    fn new(origin: Point, dir: (f64, f64)) -> Self {
        let len = dir.0.hypot(dir.1);
        let dir = (dir.0 / len, dir.1 / len);
        Self {
            origin,
            dir,
            nrm: (-dir.1, dir.0),
        }
    }

    fn along(&self, p: Point) -> f64 {
        (p.x() - self.origin.x()) * self.dir.0 + (p.y() - self.origin.y()) * self.dir.1
    }

    fn cross(&self, p: Point) -> f64 {
        (p.x() - self.origin.x()) * self.nrm.0 + (p.y() - self.origin.y()) * self.nrm.1
    }

    fn point(&self, along: f64, cross: f64) -> Point {
        point_xy(
            self.origin.x() + self.dir.0 * along + self.nrm.0 * cross,
            self.origin.y() + self.dir.1 * along + self.nrm.1 * cross,
        )
    }
}

#[derive(Clone, Copy)]
struct Row {
    id: i32,
    cross: f64,
    west: Point,
    east: Point,
    length: f64,
}

/// Ground the tractor may drive on inside the ring: what the harvester has
/// already taken off, as a `cross` interval.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Block {
    min: f64,
    max: f64,
}

impl Block {
    fn width(&self) -> f64 {
        (self.max - self.min).max(0.0)
    }

    fn covers(&self, cross: f64) -> bool {
        cross >= self.min - 1e-6 && cross <= self.max + 1e-6
    }

    /// In the headland ring, or on ground the harvester has already cleared.
    fn contains(&self, point: Point, world: &World) -> bool {
        if !inside(&world.border, point) {
            return false;
        }
        !inside(&world.ring, point) || self.covers(world.frame.cross(point))
    }

    /// Drag the rig along the waypoints, starting straight, and check the
    /// tractor's nose and centre and the towed unit's centre and tail at
    /// every step. The towed unit cuts inside the curve, so a rigid stick
    /// would reject loops that a real trailer takes.
    fn rig_fits(&self, waypoints: &[Pose2D], world: &World) -> bool {
        let Some(first) = waypoints.first() else {
            return true;
        };
        let mut trailer = Trailer::new(first);
        for pose in waypoints {
            let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
            let nose = point_xy(pose.point.x() + c * RIG_NOSE, pose.point.y() + s * RIG_NOSE);
            if !self.contains(pose.point, world) || !self.contains(nose, world) {
                return false;
            }
            let (centre, tail) = trailer.advance(pose);
            if !self.contains(centre, world) || !self.contains(tail, world) {
                return false;
            }
        }
        true
    }
}

/// The towed unit as the planner sees it: its axle stays put each step and
/// the unit re-aims at the hitch, the kinematic trailer for small steps.
struct Trailer {
    yaw: f64,
    hitch: Point,
}

impl Trailer {
    fn new(pose: &Pose2D) -> Self {
        Self {
            yaw: pose.yaw,
            hitch: hitch_of(pose),
        }
    }

    /// Move the tractor to `pose`; returns the unit's centre and tail.
    fn advance(&mut self, pose: &Pose2D) -> (Point, Point) {
        let axle = point_xy(
            self.hitch.x() - self.yaw.cos() * DRAWBAR,
            self.hitch.y() - self.yaw.sin() * DRAWBAR,
        );
        self.hitch = hitch_of(pose);
        let (dx, dy) = (self.hitch.x() - axle.x(), self.hitch.y() - axle.y());
        if dx.hypot(dy) > 1e-6 {
            self.yaw = dy.atan2(dx);
        }
        let (c, s) = (self.yaw.cos(), self.yaw.sin());
        let centre = point_xy(self.hitch.x() - c * DRAWBAR, self.hitch.y() - s * DRAWBAR);
        let tail = point_xy(
            centre.x() - c * TOWED_UNIT * 0.5,
            centre.y() - s * TOWED_UNIT * 0.5,
        );
        (centre, tail)
    }
}

fn hitch_of(pose: &Pose2D) -> Point {
    let back = TRACTOR_UNIT * 0.5;
    point_xy(
        pose.point.x() - pose.yaw.cos() * back,
        pose.point.y() - pose.yaw.sin() * back,
    )
}

/// Koen's headland line, parametrised by `cross` so the tractor can be put
/// anywhere along it.
#[derive(Clone, Copy)]
struct Lane {
    a: Point,
    b: Point,
    ca: f64,
    cb: f64,
}

impl Lane {
    fn at(&self, cross: f64) -> Point {
        let t = (cross - self.ca) / (self.cb - self.ca);
        point_xy(
            self.a.x() + (self.b.x() - self.a.x()) * t,
            self.a.y() + (self.b.y() - self.a.y()) * t,
        )
    }

    fn heading(&self, northward: bool) -> f64 {
        let (dx, dy) = (self.b.x() - self.a.x(), self.b.y() - self.a.y());
        if (self.cb > self.ca) == northward {
            dy.atan2(dx)
        } else {
            (-dy).atan2(-dx)
        }
    }

    fn south_end(&self) -> f64 {
        self.ca.min(self.cb)
    }
}

struct World {
    frame: Frame,
    lane: Lane,
    east_along: f64,
    border: Polygon,
    ring: Polygon,
}

impl World {
    fn bay_point(&self, cross: f64) -> Point {
        let on_lane = self.lane.at(cross);
        point_xy(
            on_lane.x() + self.frame.dir.0 * BAY_OFFSET,
            on_lane.y() + self.frame.dir.1 * BAY_OFFSET,
        )
    }

    /// Lane on the far side, for the harvester's east transits.
    fn east_lane(&self, cross: f64) -> Point {
        self.frame.point(self.east_along, cross)
    }
}

/// A polyline being followed at constant speed.
#[derive(Default, Clone)]
struct Path {
    points: Vec<Point>,
    travelled: f64,
    length: f64,
}

impl Path {
    fn new(points: Vec<Point>) -> Self {
        let length = points
            .windows(2)
            .map(|pair| distance(pair[0], pair[1]))
            .sum();
        Self {
            points,
            travelled: 0.0,
            length,
        }
    }

    fn done(&self) -> bool {
        self.points.len() < 2 || self.travelled >= self.length - 1e-9
    }

    fn advance(&mut self, metres: f64) {
        self.travelled = (self.travelled + metres).min(self.length);
    }

    fn pose(&self, fallback: Pose2D) -> Pose2D {
        if self.points.len() < 2 {
            return fallback;
        }
        let mut walked = 0.0;
        for pair in self.points.windows(2) {
            let span = distance(pair[0], pair[1]);
            if span < 1e-9 {
                continue;
            }
            if walked + span >= self.travelled {
                let f = (self.travelled - walked) / span;
                return Pose2D::from_point(
                    point_xy(
                        pair[0].x() + (pair[1].x() - pair[0].x()) * f,
                        pair[0].y() + (pair[1].y() - pair[0].y()) * f,
                    ),
                    (pair[1].y() - pair[0].y()).atan2(pair[1].x() - pair[0].x()),
                );
            }
            walked += span;
        }
        let last = self.points[self.points.len() - 1];
        let prev = self.points[self.points.len() - 2];
        Pose2D::from_point(last, (last.y() - prev.y()).atan2(last.x() - prev.x()))
    }
}

/// Where the next unload happens: the rig in the bay facing south, the
/// harvester stopped on the lane beside its trailer.
#[derive(Clone)]
struct Meeting {
    bay: Pose2D,
    stop_cross: f64,
    block: Block,
}

/// A planned drive: the polyline plus the words for the log.
#[derive(Clone)]
struct Trip {
    path: Path,
    plan: String,
}

#[derive(Clone, Copy, PartialEq)]
enum Leg {
    Row,
    Transit,
    Stopped,
    Done,
}

/// Not planned, just observed. It follows its own polylines; the only
/// decision it makes is whether to stop for the tractor on a west transit.
struct Harvester {
    pass: usize,
    leg: Leg,
    path: Path,
    stop_at: Option<f64>,
    load: f64,
    unload_pending: bool,
}

impl Harvester {
    fn row<'a>(&self, rows: &'a [Row], order: &[usize]) -> &'a Row {
        &rows[order[self.pass]]
    }

    fn pose(&self, rows: &[Row], order: &[usize]) -> Pose2D {
        let row = self.row(rows, order);
        self.path.pose(Pose2D::from_point(row.west, 0.0))
    }

    /// Seconds until it comes off the current row onto the lane.
    fn seconds_to_row_end(&self) -> f64 {
        match self.leg {
            Leg::Row => (self.path.length - self.path.travelled).max(0.0) / HARVESTER_SPEED,
            _ => 0.0,
        }
    }
}

#[derive(Clone, PartialEq)]
enum Job {
    Parked,
    Driving,
    Waiting,
    Transferring,
    Holding,
    Returning,
}

impl Job {
    fn label(&self) -> &'static str {
        match self {
            Job::Parked => "parked at entry",
            Job::Driving => "driving out to the bay",
            Job::Waiting => "in the bay, waiting",
            Job::Transferring => "taking the load",
            Job::Holding => "loaded, lane still busy",
            Job::Returning => "loaded, returning to tip",
        }
    }
}

/// The planned machine.
struct Tractor {
    job: Job,
    at: Pose2D,
    path: Path,
    countdown: f64,
    plan: String,
}

impl Tractor {
    fn pose(&self) -> Pose2D {
        self.path.pose(self.at)
    }
}

/// Which order the harvester takes the rows in, and which way it runs them.
#[derive(Clone, Copy, PartialEq)]
enum Harvest {
    MiddleOut,
    EdgeIn,
}

impl Harvest {
    fn from_env() -> Self {
        match std::env::var("HARVEST").as_deref() {
            Ok("edge") => Harvest::EdgeIn,
            _ => Harvest::MiddleOut,
        }
    }

    fn order(&self, count: usize) -> Vec<usize> {
        match self {
            Harvest::MiddleOut => middle_out_order(count),
            Harvest::EdgeIn => (0..count).collect(),
        }
    }

    /// Middle-out starts with a westbound pass so the first transit is on
    /// the west lane; edge-in enters at the south-west corner heading east.
    fn westbound(&self, pass: usize) -> bool {
        match self {
            Harvest::MiddleOut => pass.is_multiple_of(2),
            Harvest::EdgeIn => !pass.is_multiple_of(2),
        }
    }

    fn label(&self) -> &'static str {
        match self {
            Harvest::MiddleOut => "middle-out, alternating sides",
            Harvest::EdgeIn => "eating in from the south edge",
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let rec = stream()?;
    let harvest = Harvest::from_env();
    let speedup: f64 = std::env::var("SPEEDUP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(24.0);

    // Survey -> local ENU metres, datum at the parcel centroid.
    let datum = Geo::new(
        koen::BOUNDARY.iter().map(|p| p.0).sum::<f64>() / koen::BOUNDARY.len() as f64,
        koen::BOUNDARY.iter().map(|p| p.1).sum::<f64>() / koen::BOUNDARY.len() as f64,
        0.0,
    );
    let enu = |(lat, lon): (f64, f64)| {
        let e = to_enu(datum, Wgs::new(lat, lon, 0.0));
        point_xy(e.east(), e.north())
    };
    let border = polygon_from_points(koen::BOUNDARY.iter().copied().map(enu).collect());
    let ab_line: Segment = segment_new(enu(koen::RIDGE_LINE[0]), enu(koen::RIDGE_LINE[1]));
    let lane_a = enu(koen::HEADLAND_LINE[0]);
    let lane_b = enu(koen::HEADLAND_LINE[1]);

    let a = segment_start(ab_line);
    let b = segment_end(ab_line);
    let frame = Frame::new(a, (b.x() - a.x(), b.y() - a.y()));
    let lane = Lane {
        a: lane_a,
        b: lane_b,
        ca: frame.cross(lane_a),
        cb: frame.cross(lane_b),
    };

    let mut field = Field::new(border.clone(), datum)?;
    field.gen_field_from_line(RIDGE_WIDTH, ab_line, HEADLAND_WIDTH, 1)?;
    let ring = field.part(0)?.headlands[0].polygon.clone();

    let rows = collect_rows(&field, &frame);
    let pre_opened: usize = std::env::var("PRE_OPENED")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(PRE_OPENED);
    if rows.len() < pre_opened + 4 {
        return Err("field too small for this demo".into());
    }
    let tonnes_per_metre = YIELD_T_PER_HA * RIDGE_WIDTH / 10_000.0;
    let order = harvest.order(rows.len());

    let world = World {
        frame,
        lane,
        east_along: rows
            .iter()
            .map(|r| frame.along(r.east))
            .fold(f64::NEG_INFINITY, f64::max)
            + HEADLAND_WIDTH * 0.5,
        border: border.clone(),
        ring: ring.clone(),
    };
    let entry = Pose2D::from_point(lane.at(lane.south_end()), lane.heading(true));

    // Assume there is always turning space: the harvester has been going a
    // while when we pick up the story.
    for step in &order[..pre_opened] {
        field.set_swath_finished(0, rows[*step].id, true)?;
    }

    println!(
        "kavel j109, {:.2} ha, {}",
        area(&border) / 10_000.0,
        harvest.label()
    );
    println!(
        "  {} ridges at {RIDGE_WIDTH} m off the AB line, {:.0}-{:.0} m long",
        rows.len(),
        rows.iter().map(|r| r.length).fold(f64::INFINITY, f64::min),
        rows.iter().map(|r| r.length).fold(0.0_f64, f64::max)
    );
    println!(
        "  lane {:.1} m from the ditch; rig {TRACTOR_LENGTH} m, r={TRACTOR_RADIUS} m, forward only",
        frame.along(lane.at(0.0))
            - polygon_exterior_points(&border)
                .iter()
                .map(|p| frame.along(*p))
                .fold(f64::INFINITY, f64::min)
    );
    println!("  running at {speedup:.0}x real time\n");

    viz::polygon_3d(&rec, "enu/field/boundary", &border, C_BORDER, 0.7)?;
    viz::polygon_3d(&rec, "enu/field/headland", &ring, C_HEADLAND, 0.5)?;
    viz::polygon_geo(&rec, "wgs/field/boundary", &border, datum, C_BORDER)?;
    viz::polygon_geo(&rec, "wgs/field/headland", &ring, datum, C_HEADLAND)?;
    let lane_line = vec![vec![lane_a, lane_b]];
    viz::polylines_3d(&rec, "enu/field/lane", &lane_line, C_LANE, 0.5)?;
    viz::polylines_geo(&rec, "wgs/field/lane", &lane_line, datum, C_LANE, 1.5)?;

    let mut harvester = Harvester {
        pass: pre_opened,
        leg: Leg::Row,
        path: row_path(&rows[order[pre_opened]], harvest.westbound(pre_opened)),
        stop_at: None,
        load: 0.0,
        unload_pending: false,
    };
    let mut tractor = Tractor {
        job: Job::Parked,
        at: entry,
        path: Path::default(),
        countdown: 0.0,
        plan: "waiting for a call".to_string(),
    };

    let mut chaser = rig::Rig::new(
        rig::Unit {
            length: TRACTOR_UNIT,
            width: TRACTOR_BODY,
            colour: C_TRACTOR,
        },
        rig::Unit {
            length: TOWED_UNIT,
            width: TRACTOR_BODY,
            colour: C_TRAILER,
        },
        DRAWBAR,
        entry,
    );
    let mut digger = rig::Rig::new(
        rig::Unit {
            length: TRACTOR_UNIT,
            width: TRACTOR_BODY,
            colour: C_HARVESTER,
        },
        rig::Unit {
            length: TOWED_UNIT,
            width: HARVESTER_BODY,
            colour: C_DIGGER,
        },
        DRAWBAR,
        harvester.pose(&rows, &order),
    );

    let mut meeting: Option<Meeting> = None;
    let mut call: Option<Trip> = None;
    let mut history: Vec<(usize, String, Path)> = Vec::new();
    let mut meetings: Vec<(usize, Point)> = Vec::new();
    let mut logged_block: Option<Block> = None;
    let mut clock = 0.0_f64;
    let mut loads = 0usize;

    while clock < RUN_FOR && harvester.leg != Leg::Done {
        let block = harvested_block(&field, &rows);

        let digger_fallback = harvester.pose(&rows, &order);

        // --- harvester: runs its polylines, marks rows off, stops for us
        match harvester.leg {
            Leg::Row => {
                let metres = HARVESTER_SPEED * TICK;
                drive(&mut harvester.path, &mut digger, metres, digger_fallback);
                harvester.load += metres * tonnes_per_metre;
                if harvester.path.done() {
                    let row = *harvester.row(&rows, &order);
                    field.set_swath_finished(0, row.id, true)?;
                    if harvester.pass + 1 < order.len() {
                        let next = rows[order[harvester.pass + 1]];
                        let (path, stop_at) = transit_path(
                            &row,
                            &next,
                            harvest.westbound(harvester.pass),
                            meeting.as_ref().filter(|_| harvester.unload_pending),
                            &world,
                        );
                        harvester.path = path;
                        harvester.stop_at = stop_at;
                        harvester.leg = Leg::Transit;
                    } else {
                        harvester.leg = Leg::Done;
                    }
                }
            }
            Leg::Transit => {
                let metres = HARVESTER_LANE_SPEED * TICK;
                match harvester.stop_at {
                    Some(stop) if harvester.path.travelled + metres >= stop => {
                        harvester.path.travelled = stop;
                        harvester.leg = Leg::Stopped;
                        println!("  {}  harvester stopped beside the bay", hms(clock));
                    }
                    _ => drive(&mut harvester.path, &mut digger, metres, digger_fallback),
                }
                if harvester.path.done() {
                    harvester.pass += 1;
                    let row = *harvester.row(&rows, &order);
                    let westbound = harvest.westbound(harvester.pass);
                    harvester.path = row_path(&row, westbound);
                    harvester.leg = Leg::Row;

                    // A westbound pass ends on our lane. Ask now if the
                    // bunker will not last until the one after.
                    if westbound && !harvester.unload_pending {
                        let per_row = row.length * tonnes_per_metre;
                        if harvester.load + per_row * 3.0 > BUNKER_T {
                            match plan_meeting(block, entry, &world) {
                                Some((found, trip)) => {
                                    println!(
                                        "  {}  row {} — unload after it; block {:.0}..{:.0} m, bay at cross {:.0} m, {}",
                                        hms(clock),
                                        row.id,
                                        block.min,
                                        block.max,
                                        world.frame.cross(found.bay.point),
                                        trip.plan
                                    );
                                    meeting = Some(found);
                                    call = Some(trip);
                                    harvester.unload_pending = true;
                                }
                                None => println!(
                                    "  {}  row {} — bunker nearly full but no room to turn ({:.0} m open)",
                                    hms(clock),
                                    row.id,
                                    block.width()
                                ),
                            }
                        }
                    }
                }
            }
            Leg::Stopped | Leg::Done => {}
        }

        // --- tractor: the machine we actually plan for
        match tractor.job {
            Job::Parked => {
                if let (Some(_), Some(trip)) = (&meeting, &call) {
                    let eta = trip.path.length / TRACTOR_SPEED;
                    let arrives_in = harvester.seconds_to_row_end() - LEAD_SECONDS;
                    if eta >= arrives_in - TICK {
                        tractor.path = trip.path.clone();
                        tractor.plan = trip.plan.clone();
                        tractor.job = Job::Driving;
                        history.push((loads + 1, "load".to_string(), trip.path.clone()));
                        println!("  {}  called — {}", hms(clock), tractor.plan);
                        call = None;
                    }
                }
            }
            Job::Driving => {
                drive(
                    &mut tractor.path,
                    &mut chaser,
                    TRACTOR_SPEED * TICK,
                    tractor.at,
                );
                if tractor.path.done() {
                    tractor.at = tractor.path.pose(tractor.at);
                    tractor.path = Path::default();
                    tractor.job = Job::Waiting;
                    println!("  {}  in the bay", hms(clock));
                }
            }
            Job::Waiting => {
                if harvester.leg == Leg::Stopped {
                    tractor.job = Job::Transferring;
                    println!("  {}  transfer started", hms(clock));
                    tractor.countdown = TRANSFER_SECONDS;
                }
            }
            Job::Transferring => {
                tractor.countdown -= TICK;
                if tractor.countdown <= 0.0 {
                    loads += 1;
                    meetings.push((loads, harvester.pose(&rows, &order).point));
                    harvester.load = 0.0;
                    harvester.unload_pending = false;
                    harvester.stop_at = None;
                    harvester.leg = Leg::Transit;
                    tractor.job = Job::Holding;
                    println!(
                        "  {}  load {loads} taken — holding until the harvester is in its row",
                        hms(clock)
                    );
                }
            }
            Job::Holding => {
                if harvester.leg == Leg::Row && harvester.path.travelled >= CLEARANCE {
                    let found = meeting.take().expect("meeting");
                    let trip = plan_return(tractor.at, found.block, entry, &world);
                    tractor.path = trip.path.clone();
                    tractor.plan = trip.plan;
                    tractor.job = Job::Returning;
                    history.push((loads, "unload".to_string(), trip.path));
                    println!("  {}  lane clear — {}", hms(clock), tractor.plan);
                }
            }
            Job::Returning => {
                drive(
                    &mut tractor.path,
                    &mut chaser,
                    TRACTOR_SPEED * TICK,
                    tractor.at,
                );
                if tractor.path.done() {
                    // Tipping and the yard turn happen off the field.
                    tractor.at = entry;
                    tractor.path = Path::default();
                    tractor.job = Job::Parked;
                    println!("  {}  back at the entry", hms(clock));
                    tractor.plan = "tipped, waiting for a call".to_string();
                }
            }
        }

        // --- log the tick

        if logged_block != Some(block) {
            log_rows(&rec, &rows, block, datum)?;
            logged_block = Some(block);
        }

        digger.place(harvester.pose(&rows, &order));
        digger.log(&rec, "enu/harvester", "wgs/harvester", datum)?;
        if tractor.job == Job::Parked {
            chaser.reset(entry);
        } else {
            chaser.place(tractor.pose());
        }
        chaser.log(&rec, "enu/tractor", "wgs/tractor", datum)?;

        if tractor.path.points.len() >= 2 {
            viz::polylines_3d(
                &rec,
                "enu/tractor/plan",
                std::slice::from_ref(&tractor.path.points),
                C_PLAN,
                0.8,
            )?;
            viz::polylines_geo(
                &rec,
                "wgs/tractor/plan",
                std::slice::from_ref(&tractor.path.points),
                datum,
                C_PLAN,
                1.5,
            )?;
        }
        let driven: Vec<Vec<Point>> = history.iter().map(|(_, _, p)| p.points.clone()).collect();
        viz::polylines_3d(&rec, "enu/tractor/history", &driven, C_TRACTOR, 0.08)?;
        viz::polylines_geo(&rec, "wgs/tractor/history", &driven, datum, C_TRACTOR, 0.5)?;

        if (clock as u64).is_multiple_of(600) {
            println!(
                "  {}  {:<24} bunker {:.1}/{BUNKER_T:.1} t   block {:.0} m   rows {}/{}",
                hms(clock),
                tractor.job.label(),
                harvester.load,
                block.width(),
                field.finished_swath_count(),
                rows.len()
            );
        }

        clock += TICK;
        if speedup > 0.0 {
            std::thread::sleep(Duration::from_secs_f64(TICK / speedup));
        }
    }

    println!(
        "{} simulated, {loads} loads, {} rows off",
        hms(clock),
        field.finished_swath_count()
    );

    export(&field, &rows, &world, &history, &meetings, harvest)?;
    Ok(())
}

/// Up the lane to the top of the block, a compact loop that comes straight
/// back out beside the lane, then one rig length of straight run so the
/// trailer is lined up, and that is the bay. The loop goes as far up the
/// block as it fits; the way back out from the bay has to be legal too.
fn plan_meeting(block: Block, entry: Pose2D, world: &World) -> Option<(Meeting, Trip)> {
    let entry_cross = world.frame.cross(entry.point);
    let south = world.lane.heading(false);
    let mut turn_in = block.max;
    while turn_in > entry_cross + 1.0 && turn_in - 2.0 * TRACTOR_RADIUS - RIG_NOSE > block.min {
        let start = Pose2D::from_point(world.lane.at(turn_in), world.lane.heading(true));
        let mut drop = 2.0 * TRACTOR_RADIUS;
        while drop <= 3.0 * TRACTOR_RADIUS {
            let out_cross = turn_in - drop;
            let out = Pose2D::from_point(world.bay_point(out_cross), south);
            if let Some((name, waypoints, _)) = dubins_between(start, out, block, world) {
                let run = TRACTOR_LENGTH;
                let bay_cross = out_cross - run;
                let bay = Pose2D::from_point(world.bay_point(bay_cross), south);
                if block.covers(bay_cross - RIG_NOSE)
                    && legal_return(bay, block, entry, world).is_some()
                {
                    let mut all = vec![entry.point];
                    all.extend(points_of(&waypoints));
                    all.push(bay.point);
                    let meeting = Meeting {
                        bay,
                        stop_cross: bay_cross + TRAILER_BEHIND,
                        block,
                    };
                    let trip = Trip {
                        path: Path::new(all),
                        plan: format!(
                            "up the lane to the top of the block, {name} loop, {run:.0} m straight to the bay"
                        ),
                    };
                    return Some((meeting, trip));
                }
            }
            drop += 1.0;
        }
        turn_in -= 2.0;
    }
    None
}
/// Bay -> merge onto the lane heading south -> entry. The merge is a Dubins
/// S-bend; if none stays on legal ground the shortest one is taken anyway
/// and the plan says so.
fn plan_return(bay: Pose2D, block: Block, entry: Pose2D, world: &World) -> Trip {
    if let Some(trip) = legal_return(bay, block, entry, world) {
        return trip;
    }
    let bay_cross = world.frame.cross(bay.point);
    let goal = Pose2D::from_point(world.lane.at(bay_cross - 12.0), world.lane.heading(false));
    let shortest = Dubins::new(TRACTOR_RADIUS)
        .get_all_paths(bay, goal, STEP)
        .into_iter()
        .min_by(|a, b| by_length(a.total_length, b.total_length))
        .expect("dubins always returns a path");
    let mut points = points_of(&shortest.waypoints);
    points.push(entry.point);
    Trip {
        path: Path::new(points),
        plan: format!(
            "(!) {} merge clips standing rows, south to the entry",
            shortest.name
        ),
    }
}

/// Shortest legal way from the bay to the entry over a range of merge
/// points. A goal too close makes Dubins throw in a full circle, so the
/// cheapest overall drive wins, not the nearest goal.
fn legal_return(bay: Pose2D, block: Block, entry: Pose2D, world: &World) -> Option<Trip> {
    let bay_cross = world.frame.cross(bay.point);
    let mut best: Option<(f64, Trip)> = None;
    let mut ahead = 8.0;
    while ahead <= 24.0 {
        let goal = Pose2D::from_point(world.lane.at(bay_cross - ahead), world.lane.heading(false));
        if let Some((name, waypoints, length)) = dubins_between(bay, goal, block, world) {
            let total = length + distance(goal.point, entry.point);
            if best.as_ref().is_none_or(|(len, _)| total < *len) {
                let mut points = points_of(&waypoints);
                points.push(entry.point);
                best = Some((
                    total,
                    Trip {
                        path: Path::new(points),
                        plan: format!("{name} merge onto the lane, south to the entry"),
                    },
                ));
            }
        }
        ahead += 2.0;
    }
    best.map(|(_, trip)| trip)
}

/// Shortest forward-only path whose whole rig stays on legal ground.
fn dubins_between(
    start: Pose2D,
    goal: Pose2D,
    block: Block,
    world: &World,
) -> Option<(String, Vec<Pose2D>, f64)> {
    Dubins::new(TRACTOR_RADIUS)
        .get_all_paths(start, goal, STEP)
        .into_iter()
        .filter(|path| block.rig_fits(&path.waypoints, world))
        .min_by(|a, b| by_length(a.total_length, b.total_length))
        .map(|path| {
            (
                format!("Dubins {}", path.name),
                path.waypoints,
                path.total_length,
            )
        })
}

fn row_path(row: &Row, westbound: bool) -> Path {
    if westbound {
        Path::new(vec![row.east, row.west])
    } else {
        Path::new(vec![row.west, row.east])
    }
}

/// Row end -> side lane -> next row start. On the west lane with an unload
/// due, detour to the stop beside the trailer first; the harvester turns
/// however it likes, so the detour is just more polyline.
fn transit_path(
    from: &Row,
    to: &Row,
    west: bool,
    meeting: Option<&Meeting>,
    world: &World,
) -> (Path, Option<f64>) {
    if west {
        let mut points = vec![from.west, world.lane.at(from.cross)];
        let mut stop_at = None;
        if let Some(meeting) = meeting {
            points.push(world.lane.at(meeting.stop_cross));
            stop_at = Some(distance(points[0], points[1]) + distance(points[1], points[2]));
        }
        points.push(world.lane.at(to.cross));
        points.push(to.west);
        (Path::new(points), stop_at)
    } else {
        let points = vec![
            from.east,
            world.east_lane(from.cross),
            world.east_lane(to.cross),
            to.east,
        ];
        (Path::new(points), None)
    }
}

fn export(
    field: &Field,
    rows: &[Row],
    world: &World,
    history: &[(usize, String, Path)],
    meetings: &[(usize, Point)],
    harvest: Harvest,
) -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all("target")?;
    let options = GeoJsonOptions {
        include_part_boundaries: false,
        include_swaths: false,
        include_tours: false,
        ..GeoJsonOptions::default()
    };
    let mut vector = field_to_vector(field, &options);
    vector.set_global_property("name", "kavel j109 potato unload");
    vector.set_global_property("harvest", harvest.label());

    let part = field.part(0)?;
    for row in rows {
        let finished = part.finished_swaths().any(|s| s.id == row.id);
        let mut props = HashMap::new();
        props.insert("row".to_string(), row.id.to_string());
        props.insert("finished".to_string(), finished.to_string());
        let kind = if finished { "harvested" } else { "swath" };
        vector.add_path(vec![row.west, row.east], kind, props);
    }
    vector.add_path(vec![world.lane.a, world.lane.b], "lane", HashMap::new());
    for (line, leg, path) in history {
        let mut props = HashMap::new();
        props.insert("line".to_string(), line.to_string());

        props.insert("leg".to_string(), leg.clone());
        props.insert("length_m".to_string(), format!("{:.1}", path.length));
        vector.add_path(path.points.clone(), "tour", props);
    }
    for (load, point) in meetings {
        let mut props = HashMap::new();
        props.insert("load".to_string(), load.to_string());
        vector.add_point(*point, "meeting", props);
    }

    write_vector(&vector, "target/potato_unload.geojson", Crs::Wgs)?;
    kml::write_kml(&vector, "target/potato_unload.kml")?;
    println!("wrote target/potato_unload.geojson and target/potato_unload.kml");
    Ok(())
}

fn stream() -> Result<RecordingStream, Box<dyn Error>> {
    match std::env::var("RERUN_SAVE") {
        Ok(path) => Ok(RecordingStreamBuilder::new("maptrax_potato_unload").save(path)?),
        Err(_) => viz::connect("maptrax_potato_unload"),
    }
}

fn hms(seconds: f64) -> String {
    let total = seconds as u64;
    format!(
        "{:02}:{:02}:{:02}",
        total / 3600,
        (total % 3600) / 60,
        total % 60
    )
}

fn log_rows(
    rec: &RecordingStream,
    rows: &[Row],
    block: Block,
    datum: Geo,
) -> Result<(), Box<dyn Error>> {
    let mut standing = Vec::new();
    let mut done = Vec::new();
    for row in rows {
        let line = vec![row.west, row.east];
        if block.covers(row.cross) {
            done.push(line);
        } else {
            standing.push(line);
        }
    }
    viz::polylines_3d(rec, "enu/field/rows/standing", &standing, C_CROP, 0.12)?;
    viz::polylines_3d(rec, "enu/field/rows/harvested", &done, C_DONE, 0.12)?;
    viz::polylines_geo(
        rec,
        "wgs/field/rows/standing",
        &standing,
        datum,
        C_CROP,
        0.5,
    )?;
    viz::polylines_geo(rec, "wgs/field/rows/harvested", &done, datum, C_DONE, 0.5)?;
    Ok(())
}

/// Read the harvested extent straight out of the library's completion state.
fn harvested_block(field: &Field, rows: &[Row]) -> Block {
    let part = field.part(0).expect("part");
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for swath in part.finished_swaths() {
        if let Some(row) = rows.iter().find(|row| row.id == swath.id) {
            min = min.min(row.cross);
            max = max.max(row.cross);
        }
    }
    if !min.is_finite() {
        let middle = rows[rows.len() / 2].cross;
        return Block {
            min: middle,
            max: middle,
        };
    }
    Block {
        min: min - RIDGE_WIDTH * 0.5,
        max: max + RIDGE_WIDTH * 0.5,
    }
}

fn inside(polygon: &Polygon, p: Point) -> bool {
    let vs = polygon_exterior_points(polygon);
    let n = vs.len();
    let mut hit = false;
    for i in 0..n {
        let j = (i + n - 1) % n;
        let (xi, yi) = (vs[i].x(), vs[i].y());
        let (xj, yj) = (vs[j].x(), vs[j].y());
        if (yi > p.y()) != (yj > p.y()) && p.x() < (xj - xi) * (p.y() - yi) / (yj - yi) + xi {
            hit = !hit;
        }
    }
    hit
}

fn points_of(waypoints: &[Pose2D]) -> Vec<Point> {
    waypoints.iter().map(|pose| pose.point).collect()
}

fn by_length(a: f64, b: f64) -> std::cmp::Ordering {
    a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal)
}

fn distance(a: Point, b: Point) -> f64 {
    (b.x() - a.x()).hypot(b.y() - a.y())
}

fn area(polygon: &Polygon) -> f64 {
    let vs = polygon_exterior_points(polygon);
    let n = vs.len();
    let mut sum = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        sum += vs[i].x() * vs[j].y() - vs[j].x() * vs[i].y();
    }
    sum.abs() / 2.0
}

/// Every work row, sorted south to north.
fn collect_rows(field: &Field, frame: &Frame) -> Vec<Row> {
    let part = field.part(0).expect("part");
    let mut rows: Vec<Row> = part
        .work_swaths()
        .map(|swath| {
            let a = segment_start(swath.line);
            let b = segment_end(swath.line);
            let (west, east) = if frame.along(a) <= frame.along(b) {
                (a, b)
            } else {
                (b, a)
            };
            Row {
                id: swath.id,
                cross: frame.cross(west),
                west,
                east,
                length: distance(west, east),
            }
        })
        .collect();
    rows.sort_by(|a, b| by_length(a.cross, b.cross));
    rows
}

/// Middle, then one below, one above, two below, two above. A westbound
/// pass on the upper edge is followed by a south run down the west lane to
/// the lower edge, which is the run the tractor meets.
fn middle_out_order(count: usize) -> Vec<usize> {
    let middle = count / 2;
    let mut order = vec![middle];
    let mut step = 1usize;
    while order.len() < count && step <= count {
        if middle >= step {
            order.push(middle - step);
        }
        if middle + step < count && order.len() < count {
            order.push(middle + step);
        }
        step += 1;
    }
    order
}

/// Advance a path in short steps and drag the drawn rig along each one, so
/// the towed unit follows the curve instead of jumping a whole tick.
fn drive(path: &mut Path, rig: &mut rig::Rig, metres: f64, fallback: Pose2D) {
    let mut left = metres;
    while left > 1e-9 {
        let step = left.min(STEP);
        path.advance(step);
        rig.place(path.pose(fallback));
        left -= step;
    }
}
