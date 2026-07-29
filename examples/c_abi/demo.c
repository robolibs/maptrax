#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../include/maptrax.h"

int main(void) {
  MaptraxPlanner* planner = maptrax_planner_new();
  MaptraxCoord2 border[] = {
      {0.0, 0.0},
      {100.0, 0.0},
      {100.0, 50.0},
      {0.0, 50.0},
  };

  if (!maptrax_planner_set_field(
          planner,
          border,
          sizeof(border) / sizeof(border[0]),
          (MaptraxGeo3){51.0, 5.0, 0.0})) {
    fprintf(stderr, "set_field failed: %s\n", maptrax_last_error_message());
    return 1;
  }

  if (!maptrax_planner_generate_field(
          planner,
          (MaptraxFieldOptions){10.0, 90.0, 1})) {
    fprintf(stderr, "generate_field failed: %s\n", maptrax_last_error_message());
    return 1;
  }

  printf("part_count=%zu\n", maptrax_planner_part_count(planner));
  printf("total_area=%.3f\n", maptrax_planner_total_area(planner));

  MaptraxPartSnapshot* snapshot = maptrax_planner_part_snapshot(planner, 0);
  if (snapshot == NULL) {
    fprintf(stderr, "part_snapshot failed: %s\n", maptrax_last_error_message());
    return 1;
  }

  MaptraxRingBufferView boundary = maptrax_part_snapshot_boundary_view(snapshot);
  MaptraxRingBufferView headlands = maptrax_part_snapshot_headlands_view(snapshot);
  MaptraxSwathBufferView generated = maptrax_part_snapshot_swaths_view(snapshot);
  printf("boundary_rings=%zu\n", boundary.rings_len);
  printf("headlands=%zu\n", headlands.rings_len);
  printf("generated_swaths=%zu\n", generated.swaths_len);

  MaptraxPlanResult* result = maptrax_planner_plan_part(
      planner,
      0,
      (MaptraxRoutingOptions){MAPTRAX_ROUTING_STRATEGY_MAPTRAX_ROUTING_STRATEGY_GREEDY_NEAREST, 0},
      (MaptraxTurnOptions){
          MAPTRAX_TURN_MODEL_MAPTRAX_TURN_MODEL_REEDS_SHEPP,
          MAPTRAX_CONNECTOR_MODE_MAPTRAX_CONNECTOR_MODE_AUTO,
          2.0,
          0.2,
          6.0,
          3.0,
          10.0});

  if (result == NULL) {
    fprintf(stderr, "plan_part failed: %s\n", maptrax_last_error_message());
    return 1;
  }

  MaptraxSwathBufferView ordered = maptrax_plan_result_ordered_view(result);
  MaptraxSwathBufferView tour = maptrax_plan_result_tour_view(result);
  printf("ordered_swaths=%zu\n", ordered.swaths_len);
  printf("tour_swaths=%zu\n", tour.swaths_len);

  MaptraxStagesResult* staged = maptrax_planner_plan_stages_part(
      planner,
      0,
      (MaptraxRoutingOptions){MAPTRAX_ROUTING_STRATEGY_MAPTRAX_ROUTING_STRATEGY_GREEDY_NEAREST, 0},
      (MaptraxTurnOptions){
          MAPTRAX_TURN_MODEL_MAPTRAX_TURN_MODEL_REEDS_SHEPP,
          MAPTRAX_CONNECTOR_MODE_MAPTRAX_CONNECTOR_MODE_AUTO,
          2.0,
          0.2,
          6.0,
          3.0,
          10.0});

  if (staged == NULL) {
    fprintf(stderr, "plan_stages_part failed: %s\n", maptrax_last_error_message());
    return 1;
  }

  MaptraxSwathBufferView staged_generated = maptrax_stages_result_generated_view(staged);
  MaptraxSwathBufferView staged_ordered = maptrax_stages_result_ordered_view(staged);
  MaptraxSwathBufferView staged_tour = maptrax_stages_result_tour_view(staged);
  printf("staged_generated_swaths=%zu\n", staged_generated.swaths_len);
  printf("staged_ordered_swaths=%zu\n", staged_ordered.swaths_len);
  printf("staged_tour_swaths=%zu\n", staged_tour.swaths_len);

  MaptraxPosePath* rs = maptrax_plan_reeds_shepp(
      (MaptraxPose2){0.0, 0.0, 0.0},
      (MaptraxPose2){0.0, 18.0, M_PI},
      4.0,
      0.2);
  if (rs == NULL) {
    fprintf(stderr, "plan_reeds_shepp failed: %s\n", maptrax_last_error_message());
    return 1;
  }

  MaptraxPoseBufferView rs_view = maptrax_pose_path_view(rs);
  printf("reeds_shepp_name=%s\n", rs_view.name);
  printf("reeds_shepp_waypoints=%zu\n", rs_view.poses_len);
  printf("reeds_shepp_length=%.3f\n", rs_view.total_length);

  maptrax_pose_path_free(rs);

#if defined(MAPTRAX_GEOJSON)
  /* Only available when the library was built with --features geojson. */
  MaptraxGeoJsonOptions geo = maptrax_geojson_options_default();

  if (!maptrax_planner_export_geojson(planner, "target/c_abi_field.geojson", geo)) {
    fprintf(stderr, "export_geojson failed: %s\n", maptrax_last_error_message());
    return 1;
  }
  printf("geojson_file=target/c_abi_field.geojson\n");

  char* text = maptrax_planner_to_geojson(planner, geo);
  if (text == NULL) {
    fprintf(stderr, "to_geojson failed: %s\n", maptrax_last_error_message());
    return 1;
  }
  printf("geojson_bytes=%zu\n", strlen(text));
  printf("geojson_is_collection=%d\n", strstr(text, "\"FeatureCollection\"") != NULL);
  maptrax_string_free(text);
#endif

  maptrax_stages_result_free(staged);
  maptrax_part_snapshot_free(snapshot);
  maptrax_plan_result_free(result);
  maptrax_planner_free(planner);
  return 0;
}
