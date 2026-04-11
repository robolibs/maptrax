use concord::Geo;
use geo::Polygon;

use crate::avoid::ObstacleAvoider;
use crate::core::{MaptraxError, Result};
use crate::division::{DivisionType, Divy};
use crate::field::{Field, Part, Swath};
use crate::net::Nety;
use crate::tour::{TourBuilder, TurnPlannerConfig};
use crate::turners::{
    Dubins, DubinsPath, Pose2D, ReedsShepp, ReedsSheppPath, SharpTurnPath, Sharper,
};

#[derive(Debug, Clone, Default)]
pub struct Maptrax {
    field: Option<Field>,
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

    pub fn make_divy(
        &self,
        division_type: DivisionType,
        machines: usize,
        part_index: usize,
    ) -> Result<Divy> {
        let part = self.field()?.part(part_index)?.clone();
        Divy::from_part(part, division_type, machines)
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
}
