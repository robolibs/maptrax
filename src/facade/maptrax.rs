use concord::Geo;
use geo::Polygon;

use crate::avoid::ObstacleAvoider;
use crate::core::{MaptraxError, Result};
use crate::division::{DivisionPlan, Divy};
use crate::field::{
    DecompositionMode, Field, Part, Ring, Swath, SwathAngleSearchOptions, SwathAngleSearchResult,
    SwathObjective, create_ring,
};
use crate::net::{Nety, RoutingOptions};
use crate::tour::{TourBuilder, TurnPlannerConfig};
use crate::turners::{
    Dubins, DubinsPath, Pose2D, ReedsShepp, ReedsSheppPath, SharpTurnPath, Sharper,
};

#[derive(Debug, Clone, Default)]
pub struct Maptrax {
    field: Option<Field>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum FieldGenerationMode {
    ExplicitAngle(f64),
    Objective {
        objective: SwathObjective,
        options: SwathAngleSearchOptions,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct FieldGenerationOptions {
    pub swath_width: f64,
    pub headland_count: usize,
    pub decomposition: DecompositionMode,
    pub mode: FieldGenerationMode,
}

impl Default for FieldGenerationOptions {
    fn default() -> Self {
        Self {
            swath_width: 10.0,
            headland_count: 0,
            decomposition: DecompositionMode::None,
            mode: FieldGenerationMode::ExplicitAngle(0.0),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ObstaclePlanningOptions {
    pub obstacles: Vec<Polygon>,
    pub inflation_distance: f64,
}

impl Default for ObstaclePlanningOptions {
    fn default() -> Self {
        Self {
            obstacles: Vec::new(),
            inflation_distance: 0.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlannerOptions {
    pub field: FieldGenerationOptions,
    pub routing: RoutingOptions,
    pub turn: TurnPlannerConfig,
    pub obstacles: ObstaclePlanningOptions,
    pub machines: MachinePlanningOptions,
}

impl Default for PlannerOptions {
    fn default() -> Self {
        Self {
            field: FieldGenerationOptions::default(),
            routing: RoutingOptions::default(),
            turn: TurnPlannerConfig::default(),
            obstacles: ObstaclePlanningOptions::default(),
            machines: MachinePlanningOptions::default(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlannedPart {
    pub part_index: usize,
    pub ordered_swaths: Vec<Swath>,
    pub tour: Vec<Swath>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlannedField {
    pub objective_results: Vec<SwathAngleSearchResult>,
    pub parts: Vec<PlannedPart>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlannedPartStages {
    pub part_index: usize,
    pub headlands: Vec<crate::field::Ring>,
    pub transit_rings: Vec<crate::field::Ring>,
    pub generated_swaths: Vec<Swath>,
    pub avoided_swaths: Vec<Swath>,
    pub ordered_swaths: Vec<Swath>,
    pub tour: Vec<Swath>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlannedFieldStages {
    pub objective_results: Vec<SwathAngleSearchResult>,
    pub parts: Vec<PlannedPartStages>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MachinePlanningOptions {
    pub plan: DivisionPlan,
    pub part_index: usize,
}

impl Default for MachinePlanningOptions {
    fn default() -> Self {
        Self {
            plan: DivisionPlan::uniform(
                1,
                crate::division::DivisionPattern::Block,
                crate::division::Balance::ByCount,
            ),
            part_index: 0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct MachinePlannedPart {
    pub machine_index: usize,
    pub assigned_swaths: Vec<Swath>,
    pub assigned_headlands: Vec<Ring>,
    pub avoided_swaths: Vec<Swath>,
    pub ordered_swaths: Vec<Swath>,
    pub tour: Vec<Swath>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlannedMachines {
    pub part_index: usize,
    pub division: crate::division::DivisionResult,
    pub machines: Vec<MachinePlannedPart>,
}

impl Maptrax {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_field(&mut self, border: Polygon, datum: Geo) -> Result<()> {
        self.field = Some(Field::new(border, datum)?);
        Ok(())
    }

    pub fn set_field_object(&mut self, field: Field) {
        self.field = Some(field);
    }

    pub fn has_field(&self) -> bool {
        self.field.is_some()
    }

    pub fn field(&self) -> Result<&Field> {
        self.field.as_ref().ok_or(MaptraxError::MissingField)
    }

    pub fn field_mut(&mut self) -> Result<&mut Field> {
        self.field.as_mut().ok_or(MaptraxError::MissingField)
    }

    pub fn generate_field(
        &mut self,
        swath_width: f64,
        angle_degrees: f64,
        headland_count: usize,
    ) -> Result<()> {
        self.field_mut()?
            .gen_field(swath_width, angle_degrees, headland_count)
    }

    pub fn generate_field_with_objective(
        &mut self,
        swath_width: f64,
        objective: SwathObjective,
        options: SwathAngleSearchOptions,
        headland_count: usize,
    ) -> Result<Vec<SwathAngleSearchResult>> {
        self.field_mut()?
            .generate_with_objective(swath_width, objective, options, headland_count)
    }

    pub fn decompose_field(&mut self, mode: DecompositionMode) -> Result<usize> {
        self.field_mut()?.decompose(mode)
    }

    /// Run the division planner on the given part and return the raw result.
    /// For a full per-machine plan (routing + tour), use `plan_machines_for_part`.
    pub fn divide_part(
        &self,
        part_index: usize,
        plan: &DivisionPlan,
    ) -> Result<crate::division::DivisionResult> {
        let part = self.field()?.part(part_index)?;
        Divy::plan(part, plan)
    }

    pub fn make_nety_from_part(&self, part_index: usize) -> Result<Nety> {
        let part = self.field()?.part(part_index)?;
        Ok(Nety::new(&part.swaths))
    }

    pub fn avoid_obstacles_for_part(
        &self,
        obstacles: Vec<Polygon>,
        inflation_distance: f64,
        part_index: usize,
    ) -> Result<Vec<Swath>> {
        let field = self.field()?;
        let part = field.part(part_index)?;
        let mut avoider = ObstacleAvoider::new(obstacles, field.datum());
        avoider.set_field_boundary(part.boundary.polygon.clone());
        Ok(avoider.avoid(&part.swaths, inflation_distance))
    }

    pub fn plan_dubins(
        &self,
        start: Pose2D,
        end: Pose2D,
        min_turning_radius: f64,
        step_size: f64,
    ) -> DubinsPath {
        Dubins::new(min_turning_radius).plan_path(start, end, step_size)
    }

    pub fn plan_reeds_shepp(
        &self,
        start: Pose2D,
        end: Pose2D,
        min_turning_radius: f64,
        step_size: f64,
    ) -> ReedsSheppPath {
        ReedsShepp::new(min_turning_radius).plan_path(start, end, step_size)
    }

    pub fn plan_sharp_turn(
        &self,
        start: Pose2D,
        end: Pose2D,
        min_turning_radius: f64,
        machine_length: f64,
        machine_width: f64,
        pattern: &str,
    ) -> SharpTurnPath {
        Sharper::new(min_turning_radius, machine_length, machine_width)
            .plan_sharp_turn(start, end, pattern)
    }

    pub fn build_tour(
        &self,
        part_index: usize,
        ordered_swaths: &[Swath],
        cfg: &TurnPlannerConfig,
    ) -> Result<Vec<Swath>> {
        let part: &Part = self.field()?.part(part_index)?;
        Ok(TourBuilder::build(part, ordered_swaths, cfg))
    }

    pub fn plan_field(
        &mut self,
        options: &FieldGenerationOptions,
    ) -> Result<Vec<SwathAngleSearchResult>> {
        let field = self.field_mut()?;
        field.decompose(options.decomposition)?;
        match &options.mode {
            FieldGenerationMode::ExplicitAngle(angle) => {
                field.gen_field(options.swath_width, *angle, options.headland_count)?;
                Ok(Vec::new())
            }
            FieldGenerationMode::Objective {
                objective,
                options: search,
            } => field.generate_with_objective(
                options.swath_width,
                *objective,
                *search,
                options.headland_count,
            ),
        }
    }

    pub fn plan_ordered_swaths_for_part(
        &self,
        part_index: usize,
        routing: RoutingOptions,
        obstacle_options: &ObstaclePlanningOptions,
    ) -> Result<Vec<Swath>> {
        let field = self.field()?;
        let part = field.part(part_index)?;
        let (working_swaths, _) = obstacle_adjusted_swaths_and_rings(
            part,
            field.datum(),
            obstacle_options,
            obstacle_clearance_distance(
                part.swaths.first().map(|swath| swath.width).unwrap_or(0.0),
                obstacle_options.inflation_distance,
            ),
        )?;
        let mut nety = Nety::new(&working_swaths);
        nety.field_traversal_with_options(None, routing);
        Ok(ordered_work_swaths(nety.get_swaths()))
    }

    pub fn plan_tour_for_part(
        &self,
        part_index: usize,
        routing: RoutingOptions,
        obstacle_options: &ObstaclePlanningOptions,
        turn: &TurnPlannerConfig,
    ) -> Result<PlannedPart> {
        let ordered_swaths =
            self.plan_ordered_swaths_for_part(part_index, routing, obstacle_options)?;
        let tour = self.build_tour(part_index, &ordered_swaths, turn)?;
        Ok(PlannedPart {
            part_index,
            ordered_swaths,
            tour,
        })
    }

    pub fn plan_stages_for_part(
        &self,
        part_index: usize,
        routing: RoutingOptions,
        obstacle_options: &ObstaclePlanningOptions,
        turn: &TurnPlannerConfig,
    ) -> Result<PlannedPartStages> {
        let field = self.field()?;
        let part = field.part(part_index)?;
        let generated_swaths = part.swaths.clone();
        let (avoided_swaths, transit_rings) =
            obstacle_adjusted_swaths_and_rings(
                part,
                field.datum(),
                obstacle_options,
                obstacle_clearance_distance(
                    part.swaths.first().map(|swath| swath.width).unwrap_or(0.0),
                    obstacle_options.inflation_distance,
                ),
            )?;
        let mut nety = Nety::new(&avoided_swaths);
        nety.field_traversal_with_options(None, routing);
        let ordered_swaths = ordered_work_swaths(nety.get_swaths());
        let mut transit_part = part.clone();
        transit_part.transit_rings = transit_rings.clone();
        let tour = TourBuilder::build(&transit_part, &ordered_swaths, turn);
        Ok(PlannedPartStages {
            part_index,
            headlands: part.headlands.clone(),
            transit_rings,
            generated_swaths,
            avoided_swaths,
            ordered_swaths,
            tour,
        })
    }

    pub fn plan_stages(&mut self, options: &PlannerOptions) -> Result<PlannedFieldStages> {
        let objective_results = self.plan_field(&options.field)?;
        let part_count = self.field()?.get_parts().len();
        let mut parts = Vec::with_capacity(part_count);
        for part_index in 0..part_count {
            parts.push(self.plan_stages_for_part(
                part_index,
                options.routing,
                &options.obstacles,
                &options.turn,
            )?);
        }
        Ok(PlannedFieldStages {
            objective_results,
            parts,
        })
    }

    pub fn plan_all(&mut self, options: &PlannerOptions) -> Result<PlannedField> {
        let staged = self.plan_stages(options)?;
        let parts = staged
            .parts
            .into_iter()
            .map(|part| PlannedPart {
                part_index: part.part_index,
                ordered_swaths: part.ordered_swaths,
                tour: part.tour,
            })
            .collect();
        Ok(PlannedField {
            objective_results: staged.objective_results,
            parts,
        })
    }

    pub fn plan_machines_for_part(
        &self,
        options: &MachinePlanningOptions,
        obstacle_options: &ObstaclePlanningOptions,
        routing: RoutingOptions,
        turn: &TurnPlannerConfig,
    ) -> Result<PlannedMachines> {
        let field = self.field()?;
        let part = field.part(options.part_index)?.clone();
        let division = Divy::plan(&part, &options.plan)?;

        let machine_count = options.plan.machine_count();
        let mut machines = Vec::with_capacity(machine_count);
        let empty_headlands: Vec<Ring> = Vec::new();
        for machine_index in 0..machine_count {
            let assigned_swaths = division
                .swaths_per_machine
                .get(machine_index)
                .cloned()
                .unwrap_or_default();
            let assigned_headlands = division
                .headlands_per_machine
                .get(machine_index)
                .cloned()
                .unwrap_or_else(|| empty_headlands.clone());

            let avoided_swaths = if obstacle_options.obstacles.is_empty() {
                assigned_swaths.clone()
            } else {
                let mut avoider =
                    ObstacleAvoider::new(obstacle_options.obstacles.clone(), field.datum());
                avoider.set_field_boundary(part.boundary.polygon.clone());
                avoider.avoid(
                    &assigned_swaths,
                    obstacle_clearance_distance(
                        part.swaths.first().map(|swath| swath.width).unwrap_or(turn.swath_width),
                        obstacle_options.inflation_distance,
                    ),
                )
            };
            let transit_rings = obstacle_transit_rings(
                &part,
                field.datum(),
                obstacle_options,
                obstacle_clearance_distance(
                    part.swaths.first().map(|swath| swath.width).unwrap_or(turn.swath_width),
                    obstacle_options.inflation_distance,
                ),
            )?;

            let mut nety = Nety::new(&avoided_swaths);
            nety.field_traversal_with_options(None, routing);
            let ordered_swaths = ordered_work_swaths(nety.get_swaths());
            let mut transit_part = part.clone();
            transit_part.transit_rings = transit_rings;
            let tour = TourBuilder::build(&transit_part, &ordered_swaths, turn);

            machines.push(MachinePlannedPart {
                machine_index,
                assigned_swaths,
                assigned_headlands,
                avoided_swaths,
                ordered_swaths,
                tour,
            });
        }

        Ok(PlannedMachines {
            part_index: options.part_index,
            division,
            machines,
        })
    }
}

fn ordered_work_swaths(swaths: &[Swath]) -> Vec<Swath> {
    swaths
        .iter()
        .filter(|swath| swath.r#type != crate::field::SwathType::Connection)
        .cloned()
        .collect()
}

fn obstacle_adjusted_swaths_and_rings(
    part: &Part,
    datum: Geo,
    obstacle_options: &ObstaclePlanningOptions,
    clearance_distance: f64,
) -> Result<(Vec<Swath>, Vec<Ring>)> {
    if obstacle_options.obstacles.is_empty() {
        return Ok((part.swaths.clone(), Vec::new()));
    }

    let mut avoider = ObstacleAvoider::new(obstacle_options.obstacles.clone(), datum);
    avoider.set_field_boundary(part.boundary.polygon.clone());
    let avoided = avoider.avoid(&part.swaths, clearance_distance);
    let rings = avoider
        .transit_obstacles()
        .iter()
        .enumerate()
        .map(|(index, polygon)| create_ring(polygon.clone(), format!("obstacle_transit_{}", index)))
        .collect::<Result<Vec<_>>>()?;
    Ok((avoided, rings))
}

fn obstacle_transit_rings(
    part: &Part,
    datum: Geo,
    obstacle_options: &ObstaclePlanningOptions,
    clearance_distance: f64,
) -> Result<Vec<Ring>> {
    obstacle_adjusted_swaths_and_rings(part, datum, obstacle_options, clearance_distance)
        .map(|(_, rings)| rings)
}

fn obstacle_clearance_distance(swath_width: f64, inflation_distance: f64) -> f64 {
    swath_width.max(inflation_distance)
}
