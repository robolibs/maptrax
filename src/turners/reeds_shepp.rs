use crate::core::{angle_difference, point_distance};

use super::{DubinsSegmentType, Pose2D, dubins::sample_linear_path};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReedsSheppSegmentType {
    LeftForward,
    StraightForward,
    RightForward,
    LeftBackward,
    StraightBackward,
    RightBackward,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReedsSheppSegment {
    pub r#type: ReedsSheppSegmentType,
    pub length: f64,
    pub forward: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReedsSheppPath {
    pub segments: Vec<ReedsSheppSegment>,
    pub waypoints: Vec<Pose2D>,
    pub total_length: f64,
    pub name: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReedsShepp {
    turning_radius: f64,
}

impl ReedsShepp {
    pub const fn new(turning_radius: f64) -> Self {
        Self { turning_radius }
    }

    pub fn plan_path(&self, start: Pose2D, goal: Pose2D, step_size: f64) -> ReedsSheppPath {
        let turn_delta = angle_difference(start.yaw, goal.yaw);
        let turn_length = turn_delta.abs() * self.turning_radius.max(1e-6);
        let straight_length = point_distance(start.point, goal.point);
        let forward = straight_length >= self.turning_radius * 0.5;

        let waypoints = sample_linear_path(start, goal, step_size.max(1e-3));
        let turn_type = if forward {
            if turn_delta >= 0.0 {
                ReedsSheppSegmentType::LeftForward
            } else {
                ReedsSheppSegmentType::RightForward
            }
        } else if turn_delta >= 0.0 {
            ReedsSheppSegmentType::LeftBackward
        } else {
            ReedsSheppSegmentType::RightBackward
        };

        let straight_type = if forward {
            ReedsSheppSegmentType::StraightForward
        } else {
            ReedsSheppSegmentType::StraightBackward
        };

        let _ = DubinsSegmentType::Straight;

        ReedsSheppPath {
            segments: vec![
                ReedsSheppSegment {
                    r#type: turn_type,
                    length: turn_length,
                    forward,
                },
                ReedsSheppSegment {
                    r#type: straight_type,
                    length: straight_length,
                    forward,
                },
            ],
            waypoints,
            total_length: turn_length + straight_length,
            name: if forward {
                "forward_transition".to_string()
            } else {
                "reverse_transition".to_string()
            },
        }
    }
}
