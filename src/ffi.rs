use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char};
use std::ptr;

use geo::Point;

use crate::{
    ConnectorMode, FieldGenerationMode, FieldGenerationOptions, Geo, Maptrax, PlannerOptions,
    Pose2D, RoutingOptions, RoutingStrategy, TurnPlannerConfig, TurnPlannerModel,
    polygon_from_points,
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

pub struct MaptraxPlannerHandle {
    planner: Maptrax,
}

struct FlatSwathBuffer {
    swaths: Vec<MaptraxSwathView>,
    points: Vec<MaptraxCoord2>,
}

pub struct MaptraxPlanResultHandle {
    ordered: FlatSwathBuffer,
    tour: FlatSwathBuffer,
}

pub struct MaptraxPosePathHandle {
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

fn planner_from_ptr_mut<'a>(
    planner: *mut MaptraxPlannerHandle,
) -> crate::Result<&'a mut MaptraxPlannerHandle> {
    if planner.is_null() {
        return Err(crate::MaptraxError::InvalidPolygon("null planner handle"));
    }
    // SAFETY: validated non-null above, caller owns the handle.
    Ok(unsafe { &mut *planner })
}

fn planner_from_ptr<'a>(planner: *const MaptraxPlannerHandle) -> crate::Result<&'a MaptraxPlannerHandle> {
    if planner.is_null() {
        return Err(crate::MaptraxError::InvalidPolygon("null planner handle"));
    }
    // SAFETY: validated non-null above, caller owns the handle.
    Ok(unsafe { &*planner })
}

fn coords_from_raw(ptr_coords: *const MaptraxCoord2, len: usize) -> crate::Result<Vec<Point>> {
    if ptr_coords.is_null() {
        return Err(crate::MaptraxError::InvalidPolygon("null coordinate pointer"));
    }
    if len < 3 {
        return Err(crate::MaptraxError::InvalidPolygon(
            "field border requires at least 3 points",
        ));
    }
    // SAFETY: caller promises valid contiguous memory with len items.
    let slice = unsafe { std::slice::from_raw_parts(ptr_coords, len) };
    Ok(slice.iter().map(|coord| Point::new(coord.x, coord.y)).collect())
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

fn pose_buffer_view(handle: &MaptraxPosePathHandle) -> MaptraxPoseBufferView {
    MaptraxPoseBufferView {
        poses: handle.poses.as_ptr(),
        poses_len: handle.poses.len(),
        total_length: handle.total_length,
        name: handle.name.as_ptr(),
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
) -> *mut MaptraxPosePathHandle {
    let handle = MaptraxPosePathHandle {
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
pub extern "C" fn maptrax_planner_new() -> *mut MaptraxPlannerHandle {
    clear_last_error();
    Box::into_raw(Box::new(MaptraxPlannerHandle {
        planner: Maptrax::new(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_free(planner: *mut MaptraxPlannerHandle) {
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
    planner: *mut MaptraxPlannerHandle,
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
    planner: *mut MaptraxPlannerHandle,
    options: MaptraxFieldOptions,
) -> bool {
    let result = (|| {
        let planner = planner_from_ptr_mut(planner)?;
        planner
            .planner
            .generate_field(options.swath_width, options.angle_degrees, options.headland_count)
    })();
    bool_result(result)
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_planner_plan_part(
    planner: *const MaptraxPlannerHandle,
    part_index: usize,
    routing: MaptraxRoutingOptions,
    turn: MaptraxTurnOptions,
) -> *mut MaptraxPlanResultHandle {
    let result = (|| {
        let planner = planner_from_ptr(planner)?;
        let planned = planner.planner.plan_tour_for_part(
            part_index,
            routing_options_from_ffi(routing),
            &crate::ObstaclePlanningOptions::default(),
            &turn_options_from_ffi(turn),
        )?;
        Ok::<_, crate::MaptraxError>(MaptraxPlanResultHandle {
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
pub extern "C" fn maptrax_plan_result_free(result: *mut MaptraxPlanResultHandle) {
    if result.is_null() {
        return;
    }
    // SAFETY: pointer originated from Box::into_raw in maptrax_planner_plan_part.
    unsafe {
        drop(Box::from_raw(result));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_plan_result_ordered_view(
    result: *const MaptraxPlanResultHandle,
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
pub extern "C" fn maptrax_plan_result_tour_view(
    result: *const MaptraxPlanResultHandle,
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
pub extern "C" fn maptrax_plan_dubins(
    start: MaptraxPose2,
    goal: MaptraxPose2,
    min_turning_radius: f64,
    step_size: f64,
) -> *mut MaptraxPosePathHandle {
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
) -> *mut MaptraxPosePathHandle {
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
) -> *mut MaptraxPosePathHandle {
    let path = crate::Sharper::new(min_turning_radius, machine_length, machine_width)
        .plan_sharp_turn(
            Pose2D::new(start.x, start.y, start.yaw),
            Pose2D::new(goal.x, goal.y, goal.yaw),
            read_pattern(pattern),
        );
    pose_path_handle_from_path(path.pattern_name, path.total_length, path.waypoints)
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_pose_path_free(handle: *mut MaptraxPosePathHandle) {
    if handle.is_null() {
        return;
    }
    // SAFETY: pointer originated from Box::into_raw in maptrax_plan_*.
    unsafe {
        drop(Box::from_raw(handle));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn maptrax_pose_path_view(handle: *const MaptraxPosePathHandle) -> MaptraxPoseBufferView {
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
}
