//! Articulated machines for the viewer: a tractor with a unit towed behind
//! it, drawn as filled boxes in the ENU view and as two dots on the map.

use std::error::Error;

use maptrax::{Geo, Point, Point2Ext, Pose2D, point_xy};
use rerun::components::{AlbedoFactor, TriangleIndices};
use rerun::datatypes::{Rgba32, UVec3D};
use rerun::{GeoPoints, Mesh3D, RecordingStream};

pub struct Unit {
    pub length: f64,
    pub width: f64,
    pub colour: (u8, u8, u8),
}

/// The tractor pose is driven; the towed unit hangs off a hitch at the
/// tractor's rear and only swings round as the hitch drags it.
pub struct Rig {
    pub tractor: Unit,
    pub towed: Unit,
    drawbar: f64,
    pose: Pose2D,
    towed_yaw: f64,
}

impl Rig {
    pub fn new(tractor: Unit, towed: Unit, drawbar: f64, pose: Pose2D) -> Self {
        Self {
            tractor,
            towed,
            drawbar,
            pose,
            towed_yaw: pose.yaw,
        }
    }

    /// Put the rig down straight, as after a yard turn off the field.
    pub fn reset(&mut self, pose: Pose2D) {
        self.pose = pose;
        self.towed_yaw = pose.yaw;
    }

    /// Move the tractor; the towed axle stays put and the unit re-aims at
    /// the new hitch, which is the kinematic trailer for one small step.
    pub fn place(&mut self, pose: Pose2D) {
        let axle = self.towed_centre();
        self.pose = pose;
        let hitch = self.hitch();
        let (dx, dy) = (hitch.x() - axle.x(), hitch.y() - axle.y());
        if dx.hypot(dy) > 1e-6 {
            self.towed_yaw = dy.atan2(dx);
        }
    }

    fn hitch(&self) -> Point {
        let (c, s) = (self.pose.yaw.cos(), self.pose.yaw.sin());
        let back = self.tractor.length * 0.5;
        point_xy(
            self.pose.point.x() - c * back,
            self.pose.point.y() - s * back,
        )
    }

    pub fn towed_centre(&self) -> Point {
        let hitch = self.hitch();
        let (c, s) = (self.towed_yaw.cos(), self.towed_yaw.sin());
        point_xy(hitch.x() - c * self.drawbar, hitch.y() - s * self.drawbar)
    }

    pub fn log(
        &self,
        rec: &RecordingStream,
        path: &str,
        map_path: &str,
        datum: Geo,
    ) -> Result<(), Box<dyn Error>> {
        let hitch = self.hitch();
        let towed = self.towed_centre();
        log_box(
            rec,
            &format!("{path}/tractor"),
            self.pose,
            self.tractor.length,
            self.tractor.width,
            self.tractor.colour,
            0.6,
        )?;
        log_box(
            rec,
            &format!("{path}/drawbar"),
            Pose2D::from_point(
                point_xy((hitch.x() + towed.x()) * 0.5, (hitch.y() + towed.y()) * 0.5),
                self.towed_yaw,
            ),
            self.drawbar,
            0.3,
            self.towed.colour,
            0.3,
        )?;
        log_box(
            rec,
            &format!("{path}/towed"),
            Pose2D::from_point(towed, self.towed_yaw),
            self.towed.length,
            self.towed.width,
            self.towed.colour,
            0.5,
        )?;

        let (t, w) = (self.tractor.colour, self.towed.colour);
        rec.log_static(
            map_path,
            &GeoPoints::from_lat_lon([lat_lon(self.pose.point, datum), lat_lon(towed, datum)])
                .with_colors([
                    rerun::Color::from_rgb(t.0, t.1, t.2),
                    rerun::Color::from_rgb(w.0, w.1, w.2),
                ])
                .with_radii([3.0, 3.5]),
        )?;
        Ok(())
    }
}

/// A filled rectangle, `length` along the heading, lifted slightly so it
/// draws over the rows.
fn log_box(
    rec: &RecordingStream,
    path: &str,
    pose: Pose2D,
    length: f64,
    width: f64,
    colour: (u8, u8, u8),
    lift: f32,
) -> Result<(), Box<dyn Error>> {
    let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
    let (cx, cy) = (pose.point.x(), pose.point.y());
    let place = |lx: f64, ly: f64| {
        [
            (cx + c * lx - s * ly) as f32,
            (cy + s * lx + c * ly) as f32,
            lift,
        ]
    };
    let (hl, hw) = (length * 0.5, width * 0.5);
    let corners = [
        place(hl, hw),
        place(hl, -hw),
        place(-hl, -hw),
        place(-hl, hw),
    ];
    rec.log_static(
        path,
        &Mesh3D::new(corners)
            .with_triangle_indices([
                TriangleIndices(UVec3D::from((0, 1, 2))),
                TriangleIndices(UVec3D::from((0, 2, 3))),
            ])
            .with_albedo_factor(AlbedoFactor(Rgba32::from_rgb(colour.0, colour.1, colour.2))),
    )?;
    Ok(())
}

fn lat_lon(point: Point, datum: Geo) -> [f64; 2] {
    let wgs = concord::to_wgs_from_enu(concord::Enu::new(point.x(), point.y(), 0.0, datum));
    [wgs.latitude, wgs.longitude]
}
