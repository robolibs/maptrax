use concord::Geo;
use geo::Polygon;

use crate::core::{MaptraxError, Result};
use crate::division::{DivisionPlan, Divy};
use crate::field::{
    DecompositionMode, Field, Part, Swath, SwathAngleSearchOptions, SwathAngleSearchResult,
    SwathObjective,
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
pub struct PlannerOptions {
    pub field: FieldGenerationOptions,
    pub routing: RoutingOptions,
    pub turn: TurnPlannerConfig,
    pub machines: MachinePlanningOptions,
}

impl Default for PlannerOptions {
    fn default() -> Self {
        Self {
            field: FieldGenerationOptions::default(),
            routing: RoutingOptions::default(),
            turn: TurnPlannerConfig::default(),
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
    pub generated_swaths: Vec<Swath>,
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
    /// Arcs of headland rings assigned to this machine (open polylines).
    pub assigned_headland_arcs: Vec<crate::division::HeadlandArc>,
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
    ) -> Result<Vec<Swath>> {
        let field = self.field()?;
        let part = field.part(part_index)?;
        let mut nety = Nety::new(&part.swaths);
        nety.field_traversal_with_options(None, routing);
        Ok(ordered_work_swaths(nety.get_swaths()))
    }

    pub fn plan_tour_for_part(
        &self,
        part_index: usize,
        routing: RoutingOptions,
        turn: &TurnPlannerConfig,
    ) -> Result<PlannedPart> {
        let ordered_swaths = self.plan_ordered_swaths_for_part(part_index, routing)?;
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
        turn: &TurnPlannerConfig,
    ) -> Result<PlannedPartStages> {
        let field = self.field()?;
        let part = field.part(part_index)?;
        let generated_swaths = part.swaths.clone();
        let mut nety = Nety::new(&part.swaths);
        nety.field_traversal_with_options(None, routing);
        let ordered_swaths = ordered_work_swaths(nety.get_swaths());
        let tour = TourBuilder::build(part, &ordered_swaths, turn);
        Ok(PlannedPartStages {
            part_index,
            headlands: part.headlands.clone(),
            generated_swaths,
            ordered_swaths,
            tour,
        })
    }

    pub fn plan_stages(&mut self, options: &PlannerOptions) -> Result<PlannedFieldStages> {
        let objective_results = self.plan_field(&options.field)?;
        let part_count = self.field()?.get_parts().len();
        let mut parts = Vec::with_capacity(part_count);
        for part_index in 0..part_count {
            parts.push(self.plan_stages_for_part(part_index, options.routing, &options.turn)?);
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

    /// Plan machines for every Part in the current field, reusing the same
    /// DivisionPlan for each. Useful after AutoSplit decomposition when the
    /// field has been bisected into multiple sub-fields.
    pub fn plan_machines_for_all_parts(
        &self,
        plan: &DivisionPlan,
        routing: RoutingOptions,
        turn: &TurnPlannerConfig,
    ) -> Result<Vec<PlannedMachines>> {
        let part_count = self.field()?.get_parts().len();
        let mut out = Vec::with_capacity(part_count);
        for part_index in 0..part_count {
            let plan = plan.clone();
            out.push(self.plan_machines_for_part(
                &MachinePlanningOptions { plan, part_index },
                routing,
                turn,
            )?);
        }
        Ok(out)
    }

    pub fn plan_machines_for_part(
        &self,
        options: &MachinePlanningOptions,
        routing: RoutingOptions,
        turn: &TurnPlannerConfig,
    ) -> Result<PlannedMachines> {
        let field = self.field()?;
        let part = field.part(options.part_index)?.clone();
        let division = Divy::plan(&part, &options.plan)?;

        let machine_count = options.plan.machine_count();
        let mut machines = Vec::with_capacity(machine_count);
        for machine_index in 0..machine_count {
            let assigned_swaths = division
                .swaths_per_machine
                .get(machine_index)
                .cloned()
                .unwrap_or_default();
            let assigned_headland_arcs = division
                .headland_arcs_per_machine
                .get(machine_index)
                .cloned()
                .unwrap_or_default();

            let mut nety = Nety::new(&assigned_swaths);
            nety.field_traversal_with_options(None, routing);
            let ordered_swaths = ordered_work_swaths(nety.get_swaths());
            // Include the machine's assigned headland rings as driven
            // work at the start of the tour.
            let tour = TourBuilder::build_with_headlands(
                &part,
                &assigned_headland_arcs,
                &ordered_swaths,
                turn,
            );

            machines.push(MachinePlannedPart {
                machine_index,
                assigned_swaths,
                assigned_headland_arcs,
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

/// Flatten a tour (list of swath/connector segments) into a single continuous
/// polyline of (x, y) points. Adjacent segments meet at shared endpoints, so
/// we skip duplicate points at the joins. The returned Vec<Point> is the
/// full drive path you can feed to a controller, plot, or export.
pub fn tour_polyline(tour: &[Swath]) -> Vec<geo::Point<f64>> {
    let mut out: Vec<geo::Point<f64>> = Vec::new();
    for swath in tour {
        let segment_points: Vec<geo::Point<f64>> = if swath.points.len() >= 2 {
            swath.points.clone()
        } else {
            vec![swath.head(), swath.tail()]
        };
        for point in segment_points {
            match out.last() {
                Some(last) => {
                    let dx = point.x() - last.x();
                    let dy = point.y() - last.y();
                    if dx * dx + dy * dy > 1e-12 {
                        out.push(point);
                    }
                }
                None => out.push(point),
            }
        }
    }
    out
}

fn ordered_work_swaths(swaths: &[Swath]) -> Vec<Swath> {
    swaths
        .iter()
        .filter(|swath| swath.r#type != crate::field::SwathType::Connection)
        .cloned()
        .collect()
}
