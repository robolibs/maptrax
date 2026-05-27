pub mod core;
pub mod division;
pub mod facade;
pub mod ffi;
pub mod field;
pub mod net;
#[cfg(feature = "python")]
pub mod python;
pub mod tour;
pub mod turners;

pub use concord::Geo;
pub use core::{
    MaptraxError, Result, angle_between, angle_difference, are_colinear, float_to_byte,
    float_to_byte_in_range, heading_between, normalize_angle, point_distance,
    point_to_line_distance, points_equal, polygon_area, polygon_from_points,
    remove_colinear_points, segment_length,
};
pub use division::{
    Balance, DivisionPattern, DivisionPlan, DivisionResult, Divy, HeadlandArc, HeadlandMode,
    MachineProfile, OptimizeObjective,
};
pub use facade::{
    FieldGenerationMode, FieldGenerationOptions, MachinePlannedPart, MachinePlanningOptions,
    Maptrax, PlannedField, PlannedFieldStages, PlannedMachines, PlannedPart, PlannedPartStages,
    PlannerOptions, tour_polyline,
};
pub use field::canonical_swath_order;
pub use field::{
    DecompositionMode, Field, Part, Ring, SplitBoundary, Swath, SwathAngleSearchOptions,
    SwathAngleSearchResult, SwathObjective, SwathType, create_ring, create_swath,
    generate_headlands_for_polygon, generate_swaths_for_polygon,
};
pub use net::{ABLine, Nety, RoutingOptions, RoutingStrategy};
pub use tour::{ConnectorMode, TourBuilder, TurnPlannerConfig, TurnPlannerModel};
pub use turners::{
    Dubins, DubinsPath, DubinsSegment, DubinsSegmentType, Pose2D, ReedsShepp, ReedsSheppPath,
    ReedsSheppSegment, ReedsSheppSegmentType, SharpTurnPath, Sharper,
};
