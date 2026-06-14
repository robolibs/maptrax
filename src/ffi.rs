//! C ABI for maptrax.
//!
//! Conventions: opaque Box-backed handles (free with the matching
//! *_free); fallible calls return bool/int with the reason in the
//! thread-local maptrax_last_error_message(); borrowed views are valid
//! only for the lifetime documented by the handle they came from.
//!
//! `include/maptrax.h` is generated from this file by cbindgen.

// extern "C" fns take raw pointers from C and deref them by design.
#![allow(clippy::not_unsafe_ptr_arg_deref)]

use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char};
use std::ptr;

use crate::{
    ConnectorMode, Geo, Maptrax, Point, Point2Ext, Pose2D, RoutingOptions, RoutingStrategy,
    TurnPlannerConfig, TurnPlannerModel, point_xy, polygon_exterior_points, polygon_from_points,
};

thread_local! {
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaptraxCoord2 {
    pub x: f64,
    pub y: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaptraxGeo3 {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MaptraxRoutingStrategy {
    GreedyNearest = 0,
    Snake = 1,
    Spiral = 2,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaptraxRoutingOptions {
    pub strategy: MaptraxRoutingStrategy,
    pub local_improvement_passes: usize,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MaptraxTurnModel {
    Auto = 0,
    Dubins = 1,
    ReedsShepp = 2,
    Sharper = 3,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MaptraxConnectorMode {
    Auto = 0,
    Direct = 1,
    Headland = 2,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaptraxFieldOptions {
    pub swath_width: f64,
    pub angle_degrees: f64,
    pub headland_count: usize,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaptraxTurnOptions {
    pub model: MaptraxTurnModel,
    pub connector_mode: MaptraxConnectorMode,
    pub min_turning_radius: f64,
    pub step_size: f64,
    pub machine_length: f64,
    pub machine_width: f64,
    pub swath_width: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaptraxPose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaptraxSwathKind {
    Swath = 0,
    Connection = 1,
    Around = 2,
    Headland = 3,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaptraxSwathView {
    pub kind: MaptraxSwathKind,
    pub id: i32,
    pub width: f64,
    pub point_offset: usize,
    pub point_len: usize,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MaptraxSwathBufferView {
    pub swaths: *const MaptraxSwathView,
    pub swaths_len: usize,
    pub points: *const MaptraxCoord2,
    pub points_len: usize,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MaptraxPoseBufferView {
    pub poses: *const MaptraxPose2,
    pub poses_len: usize,
    pub total_length: f64,
    pub name: *const c_char,
}

pub struct MaptraxPlanner {
    planner: Maptrax,
}

struct FlatSwathBuffer {
    swaths: Vec<MaptraxSwathView>,
    points: Vec<MaptraxCoord2>,
}

pub struct MaptraxPlanResult {
    ordered: FlatSwathBuffer,
    tour: FlatSwathBuffer,
}

pub struct MaptraxPartSnapshot {
    boundary: FlatRingBuffer,
    headlands: FlatRingBuffer,
    swaths: FlatSwathBuffer,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaptraxRingView {
    pub point_offset: usize,
    pub point_len: usize,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MaptraxRingBufferView {
    pub rings: *const MaptraxRingView,
    pub rings_len: usize,
    pub points: *const MaptraxCoord2,
    pub points_len: usize,
}

struct FlatRingBuffer {
    rings: Vec<MaptraxRingView>,
    points: Vec<MaptraxCoord2>,
}

pub struct MaptraxStagesResult {
    headlands: FlatRingBuffer,
    generated: FlatSwathBuffer,
    ordered: FlatSwathBuffer,
    tour: FlatSwathBuffer,
}

pub struct MaptraxPosePath {
    poses: Vec<MaptraxPose2>,
    total_length: f64,
    name: CString,
}

fn clear_last_error() {
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = None;
    });
}

fn set_last_error(message: impl Into<String>) {
    let message = message.into().replace('\0', " ");
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = Some(
            CString::new(message).unwrap_or_else(|_| CString::new("maptrax ffi error").unwrap()),
        );
    });
}

fn bool_result<T>(result: crate::Result<T>) -> bool {
    match result {
        Ok(_) => {
            clear_last_error();
            true
        }
        Err(err) => {
            set_last_error(err.to_string());
            false
        }
    }
}

fn planner_from_ptr_mut<'a>(planner: *mut MaptraxPlanner) -> crate::Result<&'a mut MaptraxPlanner> {
    if planner.is_null() {
        return Err(crate::MaptraxError::InvalidPolygon("null planner handle"));
    }
    // SAFETY: validated non-null above, caller owns the handle.
    Ok(unsafe { &mut *planner })
}

fn planner_from_ptr<'a>(planner: *const MaptraxPlanner) -> crate::Result<&'a MaptraxPlanner> {
    if planner.is_null() {
        return Err(crate::MaptraxError::InvalidPolygon("null planner handle"));
    }
    // SAFETY: validated non-null above, caller owns the handle.
    Ok(unsafe { &*planner })
}

fn coords_from_raw(ptr_coords: *const MaptraxCoord2, len: usize) -> crate::Result<Vec<Point>> {
    if ptr_coords.is_null() {
        return Err(crate::MaptraxError::InvalidPolygon(
            "null coordinate pointer",
        ));
    }
    if len < 3 {
        return Err(crate::MaptraxError::InvalidPolygon(
            "field border requires at least 3 points",
        ));
    }
    // SAFETY: caller promises valid contiguous memory with len items.
    let slice = unsafe { std::slice::from_raw_parts(ptr_coords, len) };
    Ok(slice
        .iter()
        .map(|coord| point_xy(coord.x, coord.y))
        .collect())
}

fn routing_options_from_ffi(options: MaptraxRoutingOptions) -> RoutingOptions {
    RoutingOptions {
        strategy: match options.strategy {
            MaptraxRoutingStrategy::GreedyNearest => RoutingStrategy::GreedyNearest,
            MaptraxRoutingStrategy::Snake => RoutingStrategy::Snake,
            MaptraxRoutingStrategy::Spiral => RoutingStrategy::Spiral,
        },
        local_improvement_passes: options.local_improvement_passes,
    }
}

fn turn_options_from_ffi(options: MaptraxTurnOptions) -> TurnPlannerConfig {
    TurnPlannerConfig {
        model: match options.model {
            MaptraxTurnModel::Auto => TurnPlannerModel::Auto,
            MaptraxTurnModel::Dubins => TurnPlannerModel::Dubins,
            MaptraxTurnModel::ReedsShepp => TurnPlannerModel::ReedsShepp,
            MaptraxTurnModel::Sharper => TurnPlannerModel::Sharper,
        },
        connector_mode: match options.connector_mode {
            MaptraxConnectorMode::Auto => ConnectorMode::Auto,
            MaptraxConnectorMode::Direct => ConnectorMode::Direct,
            MaptraxConnectorMode::Headland => ConnectorMode::Headland,
        },
        min_turning_radius: options.min_turning_radius,
        step_size: options.step_size,
        machine_length: options.machine_length,
        machine_width: options.machine_width,
        swath_width: options.swath_width,
        ..TurnPlannerConfig::default()
    }
}

fn flatten_swaths(swaths: &[crate::Swath]) -> FlatSwathBuffer {
    let mut point_buffer = Vec::new();
    let mut swath_buffer = Vec::with_capacity(swaths.len());

    for swath in swaths {
        let offset = point_buffer.len();
        let points = if swath.points.is_empty() {
            vec![swath.head(), swath.tail()]
        } else {
            swath.points.clone()
        };
        point_buffer.extend(points.iter().map(|point| MaptraxCoord2 {
            x: point.x(),
            y: point.y(),
        }));
        swath_buffer.push(MaptraxSwathView {
            kind: match swath.r#type {
                crate::SwathType::Swath => MaptraxSwathKind::Swath,
                crate::SwathType::Connection => MaptraxSwathKind::Connection,
                crate::SwathType::Around => MaptraxSwathKind::Around,
                crate::SwathType::Headland => MaptraxSwathKind::Headland,
            },
            id: swath.id,
            width: swath.width,
            point_offset: offset,
            point_len: points.len(),
        });
    }

    FlatSwathBuffer {
        swaths: swath_buffer,
        points: point_buffer,
    }
}

fn flatten_rings(rings: &[crate::Ring]) -> FlatRingBuffer {
    let mut point_buffer = Vec::new();
    let mut ring_buffer = Vec::with_capacity(rings.len());

    for ring in rings {
        let offset = point_buffer.len();
        let points = polygon_exterior_points(&ring.polygon)
            .into_iter()
            .map(|point| MaptraxCoord2 {
                x: point.x(),
                y: point.y(),
            })
            .collect::<Vec<_>>();
        point_buffer.extend(points.iter().copied());
        ring_buffer.push(MaptraxRingView {
            point_offset: offset,
            point_len: points.len(),
        });
    }

    FlatRingBuffer {
        rings: ring_buffer,
        points: point_buffer,
    }
}

fn pose_buffer_view(handle: &MaptraxPosePath) -> MaptraxPoseBufferView {
    MaptraxPoseBufferView {
        poses: handle.poses.as_ptr(),
        poses_len: handle.poses.len(),
        total_length: handle.total_length,
        name: handle.name.as_ptr(),
    }
}

fn ring_buffer_view(buffer: &FlatRingBuffer) -> MaptraxRingBufferView {
    MaptraxRingBufferView {
        rings: buffer.rings.as_ptr(),
        rings_len: buffer.rings.len(),
        points: buffer.points.as_ptr(),
        points_len: buffer.points.len(),
    }
}

fn swath_buffer_view(buffer: &FlatSwathBuffer) -> MaptraxSwathBufferView {
    MaptraxSwathBufferView {
        swaths: buffer.swaths.as_ptr(),
        swaths_len: buffer.swaths.len(),
        points: buffer.points.as_ptr(),
        points_len: buffer.points.len(),
    }
}

fn pose_path_handle_from_path(
    name: String,
    total_length: f64,
    waypoints: Vec<Pose2D>,
) -> *mut MaptraxPosePath {
    let handle = MaptraxPosePath {
        poses: waypoints
            .into_iter()
            .map(|pose| MaptraxPose2 {
                x: pose.point.x(),
                y: pose.point.y(),
                yaw: pose.yaw,
            })
            .collect(),
        total_length,
        name: CString::new(name.replace('\0', " "))
            .unwrap_or_else(|_| CString::new("path").unwrap()),
    };
    clear_last_error();
    Box::into_raw(Box::new(handle))
}

fn read_pattern(ptr_pattern: *const c_char) -> &'static str {
    if ptr_pattern.is_null() {
        return "auto";
    }
    // SAFETY: caller provides a valid NUL-terminated C string or null.
    unsafe { CStr::from_ptr(ptr_pattern) }
        .to_str()
        .ok()
        .unwrap_or("auto")
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_last_error_message() -> *const c_char {
    LAST_ERROR.with(|slot| {
        slot.borrow()
            .as_ref()
            .map(|msg| msg.as_ptr())
            .unwrap_or(ptr::null())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_new() -> *mut MaptraxPlanner {
    clear_last_error();
    Box::into_raw(Box::new(MaptraxPlanner {
        planner: Maptrax::new(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_free(planner: *mut MaptraxPlanner) {
    if planner.is_null() {
        return;
    }
    // SAFETY: pointer originated from Box::into_raw in maptrax_planner_new.
    unsafe {
        drop(Box::from_raw(planner));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_set_field(
    planner: *mut MaptraxPlanner,
    coords: *const MaptraxCoord2,
    coords_len: usize,
    datum: MaptraxGeo3,
) -> bool {
    let result = (|| {
        let planner = planner_from_ptr_mut(planner)?;
        let points = coords_from_raw(coords, coords_len)?;
        let border = polygon_from_points(points);
        planner.planner.set_field(
            border,
            Geo::new(datum.latitude, datum.longitude, datum.altitude),
        )
    })();
    bool_result(result)
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_generate_field(
    planner: *mut MaptraxPlanner,
    options: MaptraxFieldOptions,
) -> bool {
    let result = (|| {
        let planner = planner_from_ptr_mut(planner)?;
        planner.planner.generate_field(
            options.swath_width,
            options.angle_degrees,
            options.headland_count,
        )
    })();
    bool_result(result)
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_part_count(planner: *const MaptraxPlanner) -> usize {
    match planner_from_ptr(planner).and_then(|planner| Ok(planner.planner.field()?.parts().len())) {
        Ok(count) => {
            clear_last_error();
            count
        }
        Err(err) => {
            set_last_error(err.to_string());
            0
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_total_area(planner: *const MaptraxPlanner) -> f64 {
    match planner_from_ptr(planner).and_then(|planner| Ok(planner.planner.field()?.total_area())) {
        Ok(area) => {
            clear_last_error();
            area
        }
        Err(err) => {
            set_last_error(err.to_string());
            0.0
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_part_snapshot(
    planner: *const MaptraxPlanner,
    part_index: usize,
) -> *mut MaptraxPartSnapshot {
    let result = (|| {
        let planner = planner_from_ptr(planner)?;
        let part = planner.planner.field()?.part(part_index)?;
        Ok::<_, crate::MaptraxError>(MaptraxPartSnapshot {
            boundary: flatten_rings(std::slice::from_ref(&part.boundary)),
            headlands: flatten_rings(&part.headlands),
            swaths: flatten_swaths(&part.swaths),
        })
    })();

    match result {
        Ok(handle) => {
            clear_last_error();
            Box::into_raw(Box::new(handle))
        }
        Err(err) => {
            set_last_error(err.to_string());
            ptr::null_mut()
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_plan_part(
    planner: *const MaptraxPlanner,
    part_index: usize,
    routing: MaptraxRoutingOptions,
    turn: MaptraxTurnOptions,
) -> *mut MaptraxPlanResult {
    let result = (|| {
        let planner = planner_from_ptr(planner)?;
        let planned = planner.planner.plan_tour_for_part(
            part_index,
            routing_options_from_ffi(routing),
            &turn_options_from_ffi(turn),
        )?;
        Ok::<_, crate::MaptraxError>(MaptraxPlanResult {
            ordered: flatten_swaths(&planned.ordered_swaths),
            tour: flatten_swaths(&planned.tour),
        })
    })();

    match result {
        Ok(handle) => {
            clear_last_error();
            Box::into_raw(Box::new(handle))
        }
        Err(err) => {
            set_last_error(err.to_string());
            ptr::null_mut()
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_plan_stages_part(
    planner: *const MaptraxPlanner,
    part_index: usize,
    routing: MaptraxRoutingOptions,
    turn: MaptraxTurnOptions,
) -> *mut MaptraxStagesResult {
    let result = (|| {
        let planner = planner_from_ptr(planner)?;
        let staged = planner.planner.plan_stages_for_part(
            part_index,
            routing_options_from_ffi(routing),
            &turn_options_from_ffi(turn),
        )?;
        Ok::<_, crate::MaptraxError>(MaptraxStagesResult {
            headlands: flatten_rings(&staged.headlands),
            generated: flatten_swaths(&staged.generated_swaths),
            ordered: flatten_swaths(&staged.ordered_swaths),
            tour: flatten_swaths(&staged.tour),
        })
    })();

    match result {
        Ok(handle) => {
            clear_last_error();
            Box::into_raw(Box::new(handle))
        }
        Err(err) => {
            set_last_error(err.to_string());
            ptr::null_mut()
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_plan_result_free(result: *mut MaptraxPlanResult) {
    if result.is_null() {
        return;
    }
    // SAFETY: pointer originated from Box::into_raw in maptrax_planner_plan_part.
    unsafe {
        drop(Box::from_raw(result));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_stages_result_free(result: *mut MaptraxStagesResult) {
    if result.is_null() {
        return;
    }
    // SAFETY: pointer originated from Box::into_raw in maptrax_planner_plan_stages_part.
    unsafe {
        drop(Box::from_raw(result));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_part_snapshot_free(snapshot: *mut MaptraxPartSnapshot) {
    if snapshot.is_null() {
        return;
    }
    // SAFETY: pointer originated from Box::into_raw in maptrax_planner_part_snapshot.
    unsafe {
        drop(Box::from_raw(snapshot));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_plan_result_ordered_view(
    result: *const MaptraxPlanResult,
) -> MaptraxSwathBufferView {
    if result.is_null() {
        return MaptraxSwathBufferView {
            swaths: ptr::null(),
            swaths_len: 0,
            points: ptr::null(),
            points_len: 0,
        };
    }
    // SAFETY: validated non-null above.
    swath_buffer_view(unsafe { &(*result).ordered })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_stages_result_headlands_view(
    result: *const MaptraxStagesResult,
) -> MaptraxRingBufferView {
    if result.is_null() {
        return MaptraxRingBufferView {
            rings: ptr::null(),
            rings_len: 0,
            points: ptr::null(),
            points_len: 0,
        };
    }
    ring_buffer_view(unsafe { &(*result).headlands })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_part_snapshot_boundary_view(
    snapshot: *const MaptraxPartSnapshot,
) -> MaptraxRingBufferView {
    if snapshot.is_null() {
        return MaptraxRingBufferView {
            rings: ptr::null(),
            rings_len: 0,
            points: ptr::null(),
            points_len: 0,
        };
    }
    ring_buffer_view(unsafe { &(*snapshot).boundary })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_part_snapshot_headlands_view(
    snapshot: *const MaptraxPartSnapshot,
) -> MaptraxRingBufferView {
    if snapshot.is_null() {
        return MaptraxRingBufferView {
            rings: ptr::null(),
            rings_len: 0,
            points: ptr::null(),
            points_len: 0,
        };
    }
    ring_buffer_view(unsafe { &(*snapshot).headlands })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_part_snapshot_swaths_view(
    snapshot: *const MaptraxPartSnapshot,
) -> MaptraxSwathBufferView {
    if snapshot.is_null() {
        return MaptraxSwathBufferView {
            swaths: ptr::null(),
            swaths_len: 0,
            points: ptr::null(),
            points_len: 0,
        };
    }
    swath_buffer_view(unsafe { &(*snapshot).swaths })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_stages_result_generated_view(
    result: *const MaptraxStagesResult,
) -> MaptraxSwathBufferView {
    if result.is_null() {
        return MaptraxSwathBufferView {
            swaths: ptr::null(),
            swaths_len: 0,
            points: ptr::null(),
            points_len: 0,
        };
    }
    swath_buffer_view(unsafe { &(*result).generated })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_stages_result_ordered_view(
    result: *const MaptraxStagesResult,
) -> MaptraxSwathBufferView {
    if result.is_null() {
        return MaptraxSwathBufferView {
            swaths: ptr::null(),
            swaths_len: 0,
            points: ptr::null(),
            points_len: 0,
        };
    }
    swath_buffer_view(unsafe { &(*result).ordered })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_stages_result_tour_view(
    result: *const MaptraxStagesResult,
) -> MaptraxSwathBufferView {
    if result.is_null() {
        return MaptraxSwathBufferView {
            swaths: ptr::null(),
            swaths_len: 0,
            points: ptr::null(),
            points_len: 0,
        };
    }
    swath_buffer_view(unsafe { &(*result).tour })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_plan_result_tour_view(
    result: *const MaptraxPlanResult,
) -> MaptraxSwathBufferView {
    if result.is_null() {
        return MaptraxSwathBufferView {
            swaths: ptr::null(),
            swaths_len: 0,
            points: ptr::null(),
            points_len: 0,
        };
    }
    // SAFETY: validated non-null above.
    swath_buffer_view(unsafe { &(*result).tour })
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_turning_envelope_radius(options: MaptraxTurnOptions) -> f64 {
    turn_options_from_ffi(options).turning_envelope_radius()
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_required_row_skip_stride(options: MaptraxTurnOptions) -> usize {
    let cfg = turn_options_from_ffi(options);
    cfg.required_row_skip_stride(cfg.swath_width)
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_plan_dubins(
    start: MaptraxPose2,
    goal: MaptraxPose2,
    min_turning_radius: f64,
    step_size: f64,
) -> *mut MaptraxPosePath {
    let path = crate::Dubins::new(min_turning_radius).plan_path(
        Pose2D::new(start.x, start.y, start.yaw),
        Pose2D::new(goal.x, goal.y, goal.yaw),
        step_size,
    );
    pose_path_handle_from_path(path.name, path.total_length, path.waypoints)
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_plan_reeds_shepp(
    start: MaptraxPose2,
    goal: MaptraxPose2,
    min_turning_radius: f64,
    step_size: f64,
) -> *mut MaptraxPosePath {
    let path = crate::ReedsShepp::new(min_turning_radius).plan_path(
        Pose2D::new(start.x, start.y, start.yaw),
        Pose2D::new(goal.x, goal.y, goal.yaw),
        step_size,
    );
    pose_path_handle_from_path(path.name, path.total_length, path.waypoints)
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_plan_sharp_turn(
    start: MaptraxPose2,
    goal: MaptraxPose2,
    min_turning_radius: f64,
    machine_length: f64,
    machine_width: f64,
    pattern: *const c_char,
) -> *mut MaptraxPosePath {
    let path = crate::Sharper::new(min_turning_radius, machine_length, machine_width)
        .plan_sharp_turn(
            Pose2D::new(start.x, start.y, start.yaw),
            Pose2D::new(goal.x, goal.y, goal.yaw),
            read_pattern(pattern),
        );
    pose_path_handle_from_path(path.pattern_name, path.total_length, path.waypoints)
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_pose_path_free(handle: *mut MaptraxPosePath) {
    if handle.is_null() {
        return;
    }
    // SAFETY: pointer originated from Box::into_raw in maptrax_plan_*.
    unsafe {
        drop(Box::from_raw(handle));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_pose_path_view(handle: *const MaptraxPosePath) -> MaptraxPoseBufferView {
    if handle.is_null() {
        return MaptraxPoseBufferView {
            poses: ptr::null(),
            poses_len: 0,
            total_length: 0.0,
            name: ptr::null(),
        };
    }
    // SAFETY: validated non-null above.
    pose_buffer_view(unsafe { &*handle })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn c_abi_plans_first_part_and_flattens_buffers() {
        let planner = maptrax_planner_new();
        let border = [
            MaptraxCoord2 { x: 0.0, y: 0.0 },
            MaptraxCoord2 { x: 100.0, y: 0.0 },
            MaptraxCoord2 { x: 100.0, y: 50.0 },
            MaptraxCoord2 { x: 0.0, y: 50.0 },
        ];

        assert!(maptrax_planner_set_field(
            planner,
            border.as_ptr(),
            border.len(),
            MaptraxGeo3 {
                latitude: 51.0,
                longitude: 5.0,
                altitude: 0.0,
            },
        ));
        assert!(maptrax_planner_generate_field(
            planner,
            MaptraxFieldOptions {
                swath_width: 10.0,
                angle_degrees: 90.0,
                headland_count: 1,
            },
        ));

        let result = maptrax_planner_plan_part(
            planner,
            0,
            MaptraxRoutingOptions {
                strategy: MaptraxRoutingStrategy::GreedyNearest,
                local_improvement_passes: 0,
            },
            MaptraxTurnOptions {
                model: MaptraxTurnModel::ReedsShepp,
                connector_mode: MaptraxConnectorMode::Auto,
                min_turning_radius: 2.0,
                step_size: 0.2,
                machine_length: 6.0,
                machine_width: 3.0,
                swath_width: 10.0,
            },
        );

        assert!(!result.is_null());
        let ordered = maptrax_plan_result_ordered_view(result);
        let tour = maptrax_plan_result_tour_view(result);
        assert!(ordered.swaths_len > 0);
        assert!(ordered.points_len > 0);
        assert!(tour.swaths_len >= ordered.swaths_len);
        assert!(tour.points_len >= ordered.points_len);

        maptrax_plan_result_free(result);
        maptrax_planner_free(planner);
    }

    #[test]
    fn c_abi_turners_return_pose_buffers() {
        let envelope = maptrax_turning_envelope_radius(MaptraxTurnOptions {
            model: MaptraxTurnModel::ReedsShepp,
            connector_mode: MaptraxConnectorMode::Auto,
            min_turning_radius: 2.0,
            step_size: 0.2,
            machine_length: 6.0,
            machine_width: 8.0,
            swath_width: 10.0,
        });
        assert!((envelope - 5.0).abs() < 1e-9);
        let stride = maptrax_required_row_skip_stride(MaptraxTurnOptions {
            model: MaptraxTurnModel::Dubins,
            connector_mode: MaptraxConnectorMode::Headland,
            min_turning_radius: 8.0,
            step_size: 0.2,
            machine_length: 9.0,
            machine_width: 4.0,
            swath_width: 6.0,
        });
        assert_eq!(stride, 3);

        let handle = maptrax_plan_reeds_shepp(
            MaptraxPose2 {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
            MaptraxPose2 {
                x: 0.0,
                y: 18.0,
                yaw: std::f64::consts::PI,
            },
            4.0,
            0.2,
        );
        assert!(!handle.is_null());
        let view = maptrax_pose_path_view(handle);
        assert!(view.poses_len > 2);
        assert!(view.total_length > 0.0);
        assert!(!view.name.is_null());
        maptrax_pose_path_free(handle);
    }

    #[test]
    fn c_abi_exposes_staged_part_buffers() {
        let planner = maptrax_planner_new();
        let border = [
            MaptraxCoord2 { x: 0.0, y: 0.0 },
            MaptraxCoord2 { x: 100.0, y: 0.0 },
            MaptraxCoord2 { x: 100.0, y: 50.0 },
            MaptraxCoord2 { x: 0.0, y: 50.0 },
        ];

        assert!(maptrax_planner_set_field(
            planner,
            border.as_ptr(),
            border.len(),
            MaptraxGeo3 {
                latitude: 51.0,
                longitude: 5.0,
                altitude: 0.0,
            },
        ));
        assert!(maptrax_planner_generate_field(
            planner,
            MaptraxFieldOptions {
                swath_width: 10.0,
                angle_degrees: 90.0,
                headland_count: 1,
            },
        ));

        let result = maptrax_planner_plan_stages_part(
            planner,
            0,
            MaptraxRoutingOptions {
                strategy: MaptraxRoutingStrategy::GreedyNearest,
                local_improvement_passes: 0,
            },
            MaptraxTurnOptions {
                model: MaptraxTurnModel::ReedsShepp,
                connector_mode: MaptraxConnectorMode::Auto,
                min_turning_radius: 2.0,
                step_size: 0.2,
                machine_length: 6.0,
                machine_width: 3.0,
                swath_width: 10.0,
            },
        );
        assert!(!result.is_null());

        let headlands = maptrax_stages_result_headlands_view(result);
        let generated = maptrax_stages_result_generated_view(result);
        let ordered = maptrax_stages_result_ordered_view(result);
        let tour = maptrax_stages_result_tour_view(result);
        assert!(headlands.rings_len > 0);
        assert!(generated.swaths_len > 0);
        assert!(ordered.swaths_len > 0);
        assert!(tour.swaths_len >= ordered.swaths_len);

        maptrax_stages_result_free(result);
        maptrax_planner_free(planner);
    }

    #[test]
    fn c_abi_exposes_part_snapshot_and_field_info() {
        let planner = maptrax_planner_new();
        let border = [
            MaptraxCoord2 { x: 0.0, y: 0.0 },
            MaptraxCoord2 { x: 100.0, y: 0.0 },
            MaptraxCoord2 { x: 100.0, y: 50.0 },
            MaptraxCoord2 { x: 0.0, y: 50.0 },
        ];

        assert!(maptrax_planner_set_field(
            planner,
            border.as_ptr(),
            border.len(),
            MaptraxGeo3 {
                latitude: 51.0,
                longitude: 5.0,
                altitude: 0.0,
            },
        ));
        assert!(maptrax_planner_generate_field(
            planner,
            MaptraxFieldOptions {
                swath_width: 10.0,
                angle_degrees: 90.0,
                headland_count: 1,
            },
        ));

        assert_eq!(maptrax_planner_part_count(planner), 1);
        assert!(maptrax_planner_total_area(planner) > 0.0);

        let snapshot = maptrax_planner_part_snapshot(planner, 0);
        assert!(!snapshot.is_null());

        let boundary = maptrax_part_snapshot_boundary_view(snapshot);
        let headlands = maptrax_part_snapshot_headlands_view(snapshot);
        let swaths = maptrax_part_snapshot_swaths_view(snapshot);
        assert_eq!(boundary.rings_len, 1);
        assert!(boundary.points_len >= 4);
        assert!(headlands.rings_len > 0);
        assert!(swaths.swaths_len > 0);

        maptrax_part_snapshot_free(snapshot);
        maptrax_planner_free(planner);
    }
}
