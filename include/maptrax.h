#ifndef MAPTRAX_H
#define MAPTRAX_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  MAPTRAX_ROUTING_STRATEGY_MAPTRAX_ROUTING_STRATEGY_GREEDY_NEAREST = 0,
  MAPTRAX_ROUTING_STRATEGY_MAPTRAX_ROUTING_STRATEGY_SNAKE = 1,
  MAPTRAX_ROUTING_STRATEGY_MAPTRAX_ROUTING_STRATEGY_SPIRAL = 2,
} MaptraxRoutingStrategy;

typedef enum {
  MAPTRAX_TURN_MODEL_MAPTRAX_TURN_MODEL_AUTO = 0,
  MAPTRAX_TURN_MODEL_MAPTRAX_TURN_MODEL_DUBINS = 1,
  MAPTRAX_TURN_MODEL_MAPTRAX_TURN_MODEL_REEDS_SHEPP = 2,
  MAPTRAX_TURN_MODEL_MAPTRAX_TURN_MODEL_SHARPER = 3,
} MaptraxTurnModel;

typedef enum {
  MAPTRAX_CONNECTOR_MODE_MAPTRAX_CONNECTOR_MODE_AUTO = 0,
  MAPTRAX_CONNECTOR_MODE_MAPTRAX_CONNECTOR_MODE_DIRECT = 1,
  MAPTRAX_CONNECTOR_MODE_MAPTRAX_CONNECTOR_MODE_HEADLAND = 2,
} MaptraxConnectorMode;

typedef enum {
  MAPTRAX_SWATH_KIND_MAPTRAX_SWATH_KIND_SWATH = 0,
  MAPTRAX_SWATH_KIND_MAPTRAX_SWATH_KIND_CONNECTION = 1,
  MAPTRAX_SWATH_KIND_MAPTRAX_SWATH_KIND_AROUND = 2,
  MAPTRAX_SWATH_KIND_MAPTRAX_SWATH_KIND_HEADLAND = 3,
} MaptraxSwathKind;

#if defined(MAPTRAX_GEOJSON)
/**
 * Coordinate reference system for GeoJSON output.
 */
typedef enum {
  /**
   * Longitude/latitude, converted through the field datum.
   */
  MAPTRAX_CRS_MAPTRAX_CRS_WGS = 0,
  /**
   * Raw local ENU metres.
   */
  MAPTRAX_CRS_MAPTRAX_CRS_ENU = 1,
} MaptraxCrs;
#endif

typedef struct MaptraxPartSnapshot MaptraxPartSnapshot;

typedef struct MaptraxPlanResult MaptraxPlanResult;

typedef struct MaptraxPlanner MaptraxPlanner;

typedef struct MaptraxPosePath MaptraxPosePath;

typedef struct MaptraxStagesResult MaptraxStagesResult;

typedef struct {
  double x;
  double y;
} MaptraxCoord2;

typedef struct {
  double latitude;
  double longitude;
  double altitude;
} MaptraxGeo3;

typedef struct {
  double swath_width;
  double angle_degrees;
  uintptr_t headland_count;
} MaptraxFieldOptions;

typedef struct {
  MaptraxRoutingStrategy strategy;
  uintptr_t local_improvement_passes;
} MaptraxRoutingOptions;

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
  MaptraxSwathKind kind;
  int32_t id;
  double width;
  uintptr_t point_offset;
  uintptr_t point_len;
} MaptraxSwathView;

typedef struct {
  const MaptraxSwathView *swaths;
  uintptr_t swaths_len;
  const MaptraxCoord2 *points;
  uintptr_t points_len;
} MaptraxSwathBufferView;

typedef struct {
  uintptr_t point_offset;
  uintptr_t point_len;
} MaptraxRingView;

typedef struct {
  const MaptraxRingView *rings;
  uintptr_t rings_len;
  const MaptraxCoord2 *points;
  uintptr_t points_len;
} MaptraxRingBufferView;

#if defined(MAPTRAX_GEOJSON)
/**
 * Which layers a GeoJSON export contains. Build one with
 * `maptrax_geojson_options_default()` and override what you need.
 */
typedef struct {
  bool include_part_boundaries;
  bool include_headlands;
  bool include_swaths;
  bool include_tours;
  MaptraxCrs crs;
} MaptraxGeoJsonOptions;
#endif

typedef struct {
  double x;
  double y;
  double yaw;
} MaptraxPose2;

typedef struct {
  const MaptraxPose2 *poses;
  uintptr_t poses_len;
  double total_length;
  const char *name;
} MaptraxPoseBufferView;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

const char *maptrax_last_error_message(void);

MaptraxPlanner *maptrax_planner_new(void);

void maptrax_planner_free(MaptraxPlanner *planner);

bool maptrax_planner_set_field(MaptraxPlanner *planner,
                               const MaptraxCoord2 *coords,
                               uintptr_t coords_len,
                               MaptraxGeo3 datum);

bool maptrax_planner_generate_field(MaptraxPlanner *planner, MaptraxFieldOptions options);

uintptr_t maptrax_planner_part_count(const MaptraxPlanner *planner);

double maptrax_planner_total_area(const MaptraxPlanner *planner);

MaptraxPartSnapshot *maptrax_planner_part_snapshot(const MaptraxPlanner *planner,
                                                   uintptr_t part_index);

MaptraxPlanResult *maptrax_planner_plan_part(const MaptraxPlanner *planner,
                                             uintptr_t part_index,
                                             MaptraxRoutingOptions routing,
                                             MaptraxTurnOptions turn);

MaptraxStagesResult *maptrax_planner_plan_stages_part(const MaptraxPlanner *planner,
                                                      uintptr_t part_index,
                                                      MaptraxRoutingOptions routing,
                                                      MaptraxTurnOptions turn);

void maptrax_plan_result_free(MaptraxPlanResult *result);

void maptrax_stages_result_free(MaptraxStagesResult *result);

void maptrax_part_snapshot_free(MaptraxPartSnapshot *snapshot);

MaptraxSwathBufferView maptrax_plan_result_ordered_view(const MaptraxPlanResult *result);

MaptraxRingBufferView maptrax_stages_result_headlands_view(const MaptraxStagesResult *result);

MaptraxRingBufferView maptrax_part_snapshot_boundary_view(const MaptraxPartSnapshot *snapshot);

MaptraxRingBufferView maptrax_part_snapshot_headlands_view(const MaptraxPartSnapshot *snapshot);

MaptraxSwathBufferView maptrax_part_snapshot_swaths_view(const MaptraxPartSnapshot *snapshot);

MaptraxSwathBufferView maptrax_stages_result_generated_view(const MaptraxStagesResult *result);

MaptraxSwathBufferView maptrax_stages_result_ordered_view(const MaptraxStagesResult *result);

MaptraxSwathBufferView maptrax_stages_result_tour_view(const MaptraxStagesResult *result);

MaptraxSwathBufferView maptrax_plan_result_tour_view(const MaptraxPlanResult *result);

#if defined(MAPTRAX_GEOJSON)
/**
 * Every layer on, WGS84 output.
 */
MaptraxGeoJsonOptions maptrax_geojson_options_default(void);
#endif

#if defined(MAPTRAX_GEOJSON)
/**
 * Write the field geometry to `path` as GeoJSON. Returns false and sets the
 * last-error message on failure.
 */
bool maptrax_planner_export_geojson(const MaptraxPlanner *planner,
                                    const char *path,
                                    MaptraxGeoJsonOptions options);
#endif

#if defined(MAPTRAX_GEOJSON)
/**
 * Plan `part_index`, then write the field geometry plus the ordered rows and
 * drive path to `path`.
 */
bool maptrax_planner_export_planned_geojson(const MaptraxPlanner *planner,
                                            uintptr_t part_index,
                                            MaptraxRoutingOptions routing,
                                            MaptraxTurnOptions turn,
                                            const char *path,
                                            MaptraxGeoJsonOptions options);
#endif

#if defined(MAPTRAX_GEOJSON)
/**
 * The field geometry as a GeoJSON string. Returns NULL on failure. The caller
 * owns the returned buffer and must release it with `maptrax_string_free`.
 */
char *maptrax_planner_to_geojson(const MaptraxPlanner *planner, MaptraxGeoJsonOptions options);
#endif

#if defined(MAPTRAX_GEOJSON)
/**
 * Plan `part_index` and return the whole plan as a GeoJSON string. Returns
 * NULL on failure; release with `maptrax_string_free`.
 */
char *maptrax_planner_planned_to_geojson(const MaptraxPlanner *planner,
                                         uintptr_t part_index,
                                         MaptraxRoutingOptions routing,
                                         MaptraxTurnOptions turn,
                                         MaptraxGeoJsonOptions options);
#endif

#if defined(MAPTRAX_GEOJSON)
/**
 * Release a string returned by one of the `*_to_geojson` calls.
 */
void maptrax_string_free(char *text);
#endif

double maptrax_turning_envelope_radius(MaptraxTurnOptions options);

uintptr_t maptrax_required_row_skip_stride(MaptraxTurnOptions options);

MaptraxPosePath *maptrax_plan_dubins(MaptraxPose2 start,
                                     MaptraxPose2 goal,
                                     double min_turning_radius,
                                     double step_size);

MaptraxPosePath *maptrax_plan_reeds_shepp(MaptraxPose2 start,
                                          MaptraxPose2 goal,
                                          double min_turning_radius,
                                          double step_size);

MaptraxPosePath *maptrax_plan_sharp_turn(MaptraxPose2 start,
                                         MaptraxPose2 goal,
                                         double min_turning_radius,
                                         double machine_length,
                                         double machine_width,
                                         const char *pattern);

void maptrax_pose_path_free(MaptraxPosePath *handle);

MaptraxPoseBufferView maptrax_pose_path_view(const MaptraxPosePath *handle);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* MAPTRAX_H */
