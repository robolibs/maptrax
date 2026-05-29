mod error;
mod geometry;
mod utils;

pub use error::{MaptraxError, Result};
pub use geometry::{
    Aabb, Linestring, Point, Point2Ext, Polygon, Segment, aabb_center, aabb_from_points,
    aabb_height, aabb_width, point_distance, point_lerp, point_xy, polygon_aabb, polygon_area,
    polygon_buffer, polygon_ensure_ccw, polygon_exterior_points, polygon_from_points,
    polygon_intersection, polygon_is_axis_aligned_rectangle, polygon_open_vertices,
    polygon_shrink, polygon_unique_sorted_intersections_with_line, segment_distance_to_point,
    segment_end, segment_length, segment_new, segment_start,
};
pub(crate) use utils::next_id;
pub use utils::{
    angle_between, angle_difference, are_colinear, float_to_byte, float_to_byte_in_range,
    heading_between, normalize_angle, point_to_line_distance, points_equal, remove_colinear_points,
};
