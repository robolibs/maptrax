use crate::core::{angle_difference, normalize_angle, point_distance, point_lerp};

use super::Pose2D;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DubinsSegmentType {
    Left,
    Straight,
    Right,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DubinsSegment {
    pub r#type: DubinsSegmentType,
    pub length: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DubinsPath {
    pub segments: Vec<DubinsSegment>,
    pub waypoints: Vec<Pose2D>,
    pub total_length: f64,
    pub name: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dubins {
    turning_radius: f64,
}

impl Dubins {
    pub const fn new(turning_radius: f64) -> Self {
        Self { turning_radius }
    }

    pub fn plan_path(&self, start: Pose2D, goal: Pose2D, step_size: f64) -> DubinsPath {
        let turn_delta = angle_difference(start.yaw, goal.yaw);
        let turn_type = if turn_delta >= 0.0 {
            DubinsSegmentType::Left
        } else {
            DubinsSegmentType::Right
        };
        let turn_length = turn_delta.abs() * self.turning_radius.max(1e-6);
        let straight_length = point_distance(start.point, goal.point);

        let mut waypoints = sample_linear_path(start, goal, step_size.max(1e-3));
        if waypoints.is_empty() {
            waypoints.push(start);
            waypoints.push(goal);
        }

        DubinsPath {
            segments: vec![
                DubinsSegment {
                    r#type: turn_type,
                    length: turn_length,
                },
                DubinsSegment {
                    r#type: DubinsSegmentType::Straight,
                    length: straight_length,
                },
            ],
            total_length: turn_length + straight_length,
            waypoints,
            name: match turn_type {
                DubinsSegmentType::Left => "LS".to_string(),
                DubinsSegmentType::Right => "RS".to_string(),
                DubinsSegmentType::Straight => "S".to_string(),
            },
        }
    }
}

pub(crate) fn sample_linear_path(start: Pose2D, goal: Pose2D, step_size: f64) -> Vec<Pose2D> {
    let distance = point_distance(start.point, goal.point);
    let steps = ((distance / step_size).ceil() as usize).max(1);
    let mut waypoints = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let point = point_lerp(start.point, goal.point, t);
        let yaw = normalize_angle(start.yaw + angle_difference(start.yaw, goal.yaw) * t);
        waypoints.push(Pose2D::from_point(point, yaw));
    }
    waypoints
}
