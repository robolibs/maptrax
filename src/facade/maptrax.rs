use crate::Geo;
use crate::core::{MaptraxError, Point, Point2Ext, Polygon, Result};
use crate::division::{DivisionPlan, Divy};
use crate::field::{
    DecompositionMode, Field, Part, Swath, SwathAngleSearchOptions, SwathAngleSearchResult,
    SwathObjective,
};
use crate::net::{Nety, RoutingOptions, RoutingStrategy};
use crate::tour::{
    HeadlandSizingPolicy, TourBuilder, TourValidation, TurnFeasibilityReport, TurnPlannerConfig,
    TurnPlannerModel, required_row_skip_stride, turn_feasibility_report, validate_tour,
};
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

    /// Inspect turn feasibility for a candidate plan without mutating the field.
    /// Reports the headland count the turn model needs, the count that would be
    /// used under `policy`, and the row-skip stride to recover any shortfall.
    pub fn turn_feasibility(
        &self,
        swath_width: f64,
        requested_headland_count: usize,
        turn: &TurnPlannerConfig,
        policy: HeadlandSizingPolicy,
    ) -> TurnFeasibilityReport {
        turn_feasibility_report(swath_width, requested_headland_count, turn, policy)
    }

    /// Generate the field with a turn-feasibility-aware headland count.
    ///
    /// - `StrictUser`: errors if the requested count is below what the turn
    ///   model needs.
    /// - `WarnOnly`: keeps the requested count (warnings live in the report).
    /// - `AutoIncrease`: raises the count to `max(requested, required)`.
    ///
    /// Returns the [`TurnFeasibilityReport`] describing the decision. The
    /// low-level [`Maptrax::generate_field`] is left untouched for callers that
    /// want exact control.
    pub fn generate_field_feasible(
        &mut self,
        swath_width: f64,
        angle_degrees: f64,
        requested_headland_count: usize,
        turn: &TurnPlannerConfig,
        policy: HeadlandSizingPolicy,
    ) -> Result<TurnFeasibilityReport> {
        let report = turn_feasibility_report(swath_width, requested_headland_count, turn, policy);
        if policy == HeadlandSizingPolicy::StrictUser
            && report.requested_headland_count < report.required_headland_count
        {
            return Err(MaptraxError::InfeasiblePlan(format!(
                "{:?} turns need {} headlands at swath_width={swath_width:.2}m \
                 (min_turning_radius={:.2}m) but only {} were requested",
                turn.model,
                report.required_headland_count,
                turn.min_turning_radius,
                report.requested_headland_count,
            )));
        }
        self.field_mut()?
            .gen_field(swath_width, angle_degrees, report.effective_headland_count)?;
        Ok(report)
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
        let part = self.field()?.part(part_index)?;
        let routing = resolve_routing(routing, turn, effective_swath_width(turn, part));
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
        let routing = resolve_routing(routing, turn, effective_swath_width(turn, part));
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
        let routing = resolve_routing(routing, turn, effective_swath_width(turn, &part));

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

    /// Validate that a built tour's connectors stay inside this part's safe
    /// turn corridor and outside its work area. With headlands present, the
    /// outer safe boundary is the first headland ring — not the field border —
    /// so the border-to-first-headland strip remains a no-turn danger zone.
    pub fn validate_part_tour(&self, part_index: usize, tour: &[Swath]) -> Result<TourValidation> {
        let part = self.field()?.part(part_index)?;
        let work_area = (part.headlands.len() >= 2)
            .then(|| part.headlands.last().map(|ring| &ring.polygon))
            .flatten();
        let outer_boundary = part
            .headlands
            .first()
            .map(|ring| &ring.polygon)
            .unwrap_or(&part.boundary.polygon);
        Ok(validate_tour(tour, outer_boundary, work_area))
    }

    /// Build a [`vectory::Vector`] holding the field geometry — border,
    /// parts, headlands and generated rows. Coordinates stay in local ENU;
    /// the conversion to longitude/latitude happens on write.
    #[cfg(feature = "geojson")]
    pub fn to_vector(
        &self,
        options: &crate::export::GeoJsonOptions,
    ) -> Result<crate::export::Vector> {
        Ok(crate::export::field_to_vector(self.field()?, options))
    }

    /// Write the field geometry to a GeoJSON file.
    #[cfg(feature = "geojson")]
    pub fn export_geojson(
        &self,
        path: impl AsRef<std::path::Path>,
        options: &crate::export::GeoJsonOptions,
    ) -> Result<()> {
        let vector = self.to_vector(options)?;
        crate::export::write_vector(&vector, path, options.crs)
    }

    /// Write the field geometry plus a planned result — ordered rows and the
    /// drive path of every part.
    #[cfg(feature = "geojson")]
    pub fn export_planned_geojson(
        &self,
        planned: &PlannedField,
        path: impl AsRef<std::path::Path>,
        options: &crate::export::GeoJsonOptions,
    ) -> Result<()> {
        let vector = crate::export::planned_field_to_vector(self.field()?, planned, options);
        crate::export::write_vector(&vector, path, options.crs)
    }

    /// Write the field geometry plus a multi-machine plan. Machine-owned rows,
    /// headland arcs and tours carry a `machine` property.
    #[cfg(feature = "geojson")]
    pub fn export_machines_geojson(
        &self,
        planned: &PlannedMachines,
        path: impl AsRef<std::path::Path>,
        options: &crate::export::GeoJsonOptions,
    ) -> Result<()> {
        let vector = crate::export::planned_machines_to_vector(self.field()?, planned, options);
        crate::export::write_vector(&vector, path, options.crs)
    }

    /// The field geometry as a GeoJSON string, for callers that want to hand
    /// the document to a socket or an HTTP response instead of a file.
    #[cfg(feature = "geojson")]
    pub fn to_geojson(&self, options: &crate::export::GeoJsonOptions) -> Result<String> {
        let vector = self.to_vector(options)?;
        crate::export::to_json_string(&vector, options.crs)
    }

    /// A planned result as a GeoJSON string.
    #[cfg(feature = "geojson")]
    pub fn planned_to_geojson(
        &self,
        planned: &PlannedField,
        options: &crate::export::GeoJsonOptions,
    ) -> Result<String> {
        let vector = crate::export::planned_field_to_vector(self.field()?, planned, options);
        crate::export::to_json_string(&vector, options.crs)
    }

    /// A multi-machine plan as a GeoJSON string.
    #[cfg(feature = "geojson")]
    pub fn machines_to_geojson(
        &self,
        planned: &PlannedMachines,
        options: &crate::export::GeoJsonOptions,
    ) -> Result<String> {
        let vector = crate::export::planned_machines_to_vector(self.field()?, planned, options);
        crate::export::to_json_string(&vector, options.crs)
    }

    /// Plan machines, then validate each machine's tour against the allowed
    /// corridor. If any connector leaves the corridor, escalate the fallback
    /// ladder from PLAN.md §"Turner Validation": widen the row-skip stride,
    /// then relax the turn model (→ Reeds-Shepp → Sharper). Returns the first
    /// fully-feasible plan, or the least-bad attempt with a diagnostic warning.
    /// The returned warnings describe any escalation that was applied.
    pub fn plan_machines_auto(
        &self,
        options: &MachinePlanningOptions,
        routing: RoutingOptions,
        turn: &TurnPlannerConfig,
    ) -> Result<(PlannedMachines, Vec<String>)> {
        let swath_width = {
            let part = self.field()?.part(options.part_index)?;
            effective_swath_width(turn, part)
        };

        let base_stride = match routing.strategy {
            RoutingStrategy::SkipRows { stride } => stride.max(1),
            RoutingStrategy::TurnRadiusAware => required_row_skip_stride(swath_width, turn),
            _ => 1,
        };
        let max_stride = base_stride + 3;
        // Try the requested model first, then progressively more flexible ones.
        let mut models = vec![turn.model];
        for fallback in [TurnPlannerModel::ReedsShepp, TurnPlannerModel::Sharper] {
            if !models.contains(&fallback) {
                models.push(fallback);
            }
        }

        let mut warnings = Vec::new();
        let mut best: Option<(PlannedMachines, usize)> = None;

        for model in models {
            for stride in base_stride..=max_stride {
                let mut attempt_turn = turn.clone();
                attempt_turn.model = model;
                let attempt_routing = RoutingOptions {
                    strategy: RoutingStrategy::SkipRows { stride },
                    ..routing
                };
                let plan = self.plan_machines_for_part(options, attempt_routing, &attempt_turn)?;

                let mut violations = 0;
                for machine in &plan.machines {
                    violations += self
                        .validate_part_tour(options.part_index, &machine.tour)?
                        .violations();
                }

                if violations == 0 {
                    if model != turn.model || stride != base_stride {
                        warnings.push(format!(
                            "escalated to turn_model={model:?}, row_skip_stride={stride} \
                             for connector feasibility"
                        ));
                    }
                    return Ok((plan, warnings));
                }
                if best.as_ref().is_none_or(|(_, b)| violations < *b) {
                    best = Some((plan, violations));
                }
            }
        }

        let (plan, violations) = best.expect("at least one attempt is always made");
        warnings.push(format!(
            "no fully feasible connector plan found after stride/model escalation; \
             best attempt leaves {violations} corridor violation(s)"
        ));
        Ok((plan, warnings))
    }
}

/// Swath width to size turn feasibility against: the explicit `turn.swath_width`
/// if set, otherwise inferred from the part's generated swaths.
fn effective_swath_width(turn: &TurnPlannerConfig, part: &Part) -> f64 {
    if turn.swath_width > 0.0 {
        turn.swath_width
    } else {
        part.swaths.first().map(|s| s.width).unwrap_or(0.0)
    }
}

/// Resolve a `TurnRadiusAware` routing request into a concrete `SkipRows`
/// stride derived from the turn model. Other strategies pass through.
fn resolve_routing(
    routing: RoutingOptions,
    turn: &TurnPlannerConfig,
    swath_width: f64,
) -> RoutingOptions {
    match routing.strategy {
        RoutingStrategy::TurnRadiusAware => RoutingOptions {
            strategy: RoutingStrategy::SkipRows {
                stride: required_row_skip_stride(swath_width, turn),
            },
            ..routing
        },
        _ => routing,
    }
}

/// Flatten a tour (list of swath/connector segments) into a single continuous
/// polyline of (x, y) points. Adjacent segments meet at shared endpoints, so
/// we skip duplicate points at the joins. The returned Vec<Point> is the
/// full drive path you can feed to a controller, plot, or export.
pub fn tour_polyline(tour: &[Swath]) -> Vec<Point> {
    let mut out: Vec<Point> = Vec::new();
    for swath in tour {
        let segment_points: Vec<Point> = if swath.points.len() >= 2 {
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
