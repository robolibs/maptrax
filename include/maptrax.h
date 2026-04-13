#ifndef MAPTRAX_H
#define MAPTRAX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MaptraxPlannerHandle MaptraxPlannerHandle;
typedef struct MaptraxPlanResultHandle MaptraxPlanResultHandle;
typedef struct MaptraxStagesResultHandle MaptraxStagesResultHandle;
typedef struct MaptraxPartSnapshotHandle MaptraxPartSnapshotHandle;
typedef struct MaptraxPosePathHandle MaptraxPosePathHandle;

typedef struct {
  double x;
  double y;
} MaptraxCoord2;

typedef struct {
  double latitude;
  double longitude;
  double altitude;
} MaptraxGeo3;

typedef enum {
  MAPTRAX_ROUTING_GREEDY_NEAREST = 0,
  MAPTRAX_ROUTING_SNAKE = 1,
  MAPTRAX_ROUTING_SPIRAL = 2,
} MaptraxRoutingStrategy;

typedef struct {
  MaptraxRoutingStrategy strategy;
  size_t local_improvement_passes;
} MaptraxRoutingOptions;

typedef enum {
  MAPTRAX_TURN_AUTO = 0,
  MAPTRAX_TURN_DUBINS = 1,
  MAPTRAX_TURN_REEDS_SHEPP = 2,
  MAPTRAX_TURN_SHARPER = 3,
} MaptraxTurnModel;

typedef enum {
  MAPTRAX_CONNECTOR_AUTO = 0,
  MAPTRAX_CONNECTOR_DIRECT = 1,
  MAPTRAX_CONNECTOR_HEADLAND = 2,
} MaptraxConnectorMode;

typedef struct {
  double swath_width;
  double angle_degrees;
  size_t headland_count;
} MaptraxFieldOptions;

typedef struct {
  MaptraxTurnModel model;
  MaptraxConnectorMode connector_mode;
  double min_turning_radius;
  double step_size;
  double machine_length;
  double machine_width;
  double swath_width;
} MaptraxTurnOptions;

typedef struct {
  double x;
  double y;
  double yaw;
} MaptraxPose2;

typedef enum {
  MAPTRAX_SWATH = 0,
  MAPTRAX_CONNECTION = 1,
  MAPTRAX_AROUND = 2,
  MAPTRAX_HEADLAND = 3,
} MaptraxSwathKind;

typedef struct {
  MaptraxSwathKind kind;
  int32_t id;
  double width;
  size_t point_offset;
  size_t point_len;
} MaptraxSwathView;

typedef struct {
  const MaptraxSwathView* swaths;
  size_t swaths_len;
  const MaptraxCoord2* points;
  size_t points_len;
} MaptraxSwathBufferView;

typedef struct {
  size_t point_offset;
  size_t point_len;
} MaptraxRingView;

typedef struct {
  const MaptraxRingView* rings;
  size_t rings_len;
  const MaptraxCoord2* points;
  size_t points_len;
} MaptraxRingBufferView;

typedef struct {
  const MaptraxPose2* poses;
  size_t poses_len;
  double total_length;
  const char* name;
} MaptraxPoseBufferView;

const char* maptrax_last_error_message(void);

MaptraxPlannerHandle* maptrax_planner_new(void);
void maptrax_planner_free(MaptraxPlannerHandle* planner);

bool maptrax_planner_set_field(
    MaptraxPlannerHandle* planner,
    const MaptraxCoord2* coords,
    size_t coords_len,
    MaptraxGeo3 datum);

bool maptrax_planner_generate_field(
    MaptraxPlannerHandle* planner,
    MaptraxFieldOptions options);

size_t maptrax_planner_part_count(const MaptraxPlannerHandle* planner);
double maptrax_planner_total_area(const MaptraxPlannerHandle* planner);

MaptraxPartSnapshotHandle* maptrax_planner_part_snapshot(
    const MaptraxPlannerHandle* planner,
    size_t part_index);

void maptrax_part_snapshot_free(MaptraxPartSnapshotHandle* snapshot);
MaptraxRingBufferView maptrax_part_snapshot_boundary_view(
    const MaptraxPartSnapshotHandle* snapshot);
MaptraxRingBufferView maptrax_part_snapshot_headlands_view(
    const MaptraxPartSnapshotHandle* snapshot);
MaptraxRingBufferView maptrax_part_snapshot_transit_rings_view(
    const MaptraxPartSnapshotHandle* snapshot);
MaptraxSwathBufferView maptrax_part_snapshot_swaths_view(
    const MaptraxPartSnapshotHandle* snapshot);

MaptraxPlanResultHandle* maptrax_planner_plan_part(
    const MaptraxPlannerHandle* planner,
    size_t part_index,
    MaptraxRoutingOptions routing,
    MaptraxTurnOptions turn);

MaptraxStagesResultHandle* maptrax_planner_plan_stages_part(
    const MaptraxPlannerHandle* planner,
    size_t part_index,
    MaptraxRoutingOptions routing,
    MaptraxTurnOptions turn);

void maptrax_plan_result_free(MaptraxPlanResultHandle* result);
MaptraxSwathBufferView maptrax_plan_result_ordered_view(
    const MaptraxPlanResultHandle* result);
MaptraxSwathBufferView maptrax_plan_result_tour_view(
    const MaptraxPlanResultHandle* result);

void maptrax_stages_result_free(MaptraxStagesResultHandle* result);
MaptraxRingBufferView maptrax_stages_result_headlands_view(
    const MaptraxStagesResultHandle* result);
MaptraxRingBufferView maptrax_stages_result_transit_rings_view(
    const MaptraxStagesResultHandle* result);
MaptraxSwathBufferView maptrax_stages_result_generated_view(
    const MaptraxStagesResultHandle* result);
MaptraxSwathBufferView maptrax_stages_result_avoided_view(
    const MaptraxStagesResultHandle* result);
MaptraxSwathBufferView maptrax_stages_result_ordered_view(
    const MaptraxStagesResultHandle* result);
MaptraxSwathBufferView maptrax_stages_result_tour_view(
    const MaptraxStagesResultHandle* result);

MaptraxPosePathHandle* maptrax_plan_dubins(
    MaptraxPose2 start,
    MaptraxPose2 goal,
    double min_turning_radius,
    double step_size);

MaptraxPosePathHandle* maptrax_plan_reeds_shepp(
    MaptraxPose2 start,
    MaptraxPose2 goal,
    double min_turning_radius,
    double step_size);

MaptraxPosePathHandle* maptrax_plan_sharp_turn(
    MaptraxPose2 start,
    MaptraxPose2 goal,
    double min_turning_radius,
    double machine_length,
    double machine_width,
    const char* pattern);

void maptrax_pose_path_free(MaptraxPosePathHandle* handle);
MaptraxPoseBufferView maptrax_pose_path_view(const MaptraxPosePathHandle* handle);

#ifdef __cplusplus
}
#endif

#endif
